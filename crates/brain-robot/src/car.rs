//! 汽车身体适配：把一辆“阿克曼前轮转向”的汽车包装成 `RobotBody`。
//!
//! 与 `DroneBody`（无人机）平行：上层大脑通过统一接口下发 `NavigateTo`，
//! 本适配把任务翻译成**纵向速度 + 前轮转角**并沿自行车运动学模型积分，
//! 同时在 `BodyState` 中暴露转向/车轮关节与四个接地点，供决策层观察。
//! 也提供 `drive(speed, steering)` 低层接口，可直接消费 `AckermannDwa` 输出。

use brain_core::error::Result;
use brain_core::time::instant_now;
use brain_core::{Pose, Vec3};
use brain_kinematics::{norm_angle, BicycleModel, BicycleState};

use crate::body::RobotBody;
use crate::command::{EffectorCommand, Task, TaskTarget};
use crate::state::{BasePose, BodyState, ContactState, JointState, RobotKind};

/// 一辆仿真汽车。
pub struct CarBody {
    model: BicycleModel,
    st: BicycleState,
    /// 当前导航目标（后轴中心系世界坐标）。
    goal: Option<Vec3>,
    /// 最大行驶速度（m/s）。
    max_speed: f32,
    /// 转向比例增益（rad 航向误差 → rad 转角）。
    steer_gain: f32,
    /// 累计车轮转角（用于展示车轮滚动关节）。
    wheel_travel: f32,
    dt: f32,
    state: BodyState,
}

impl CarBody {
    pub fn new(model: BicycleModel, x: f32, y: f32, theta: f32) -> Self {
        let mut state = BodyState::new(RobotKind::Car);
        state.joints = vec![
            JointState {
                name: "steer_fl".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
            JointState {
                name: "steer_fr".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
            JointState {
                name: "wheel_fl".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
            JointState {
                name: "wheel_fr".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
            JointState {
                name: "wheel_rl".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
            JointState {
                name: "wheel_rr".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
        ];
        state.contacts = vec![
            ContactState {
                frame: "wheel_fl".into(),
                in_contact: true,
                force: 0.0,
            },
            ContactState {
                frame: "wheel_fr".into(),
                in_contact: true,
                force: 0.0,
            },
            ContactState {
                frame: "wheel_rl".into(),
                in_contact: true,
                force: 0.0,
            },
            ContactState {
                frame: "wheel_rr".into(),
                in_contact: true,
                force: 0.0,
            },
        ];
        Self {
            model,
            st: BicycleState::new(x, y, theta),
            goal: None,
            max_speed: 5.0,
            steer_gain: 2.0,
            wheel_travel: 0.0,
            dt: 0.1,
            state,
        }
    }

    /// 位置与朝向。
    pub fn pose(&self) -> (f32, f32, f32) {
        (self.st.x, self.st.y, self.st.theta)
    }

    /// 低层接口：直接下发速度与前轮转角（例如来自 Ackermann DWA）。
    pub fn drive(&mut self, speed: f32, steering: f32) {
        self.st = self.model.step(&self.st, speed, steering, self.dt);
        self.wheel_travel += speed.abs() * self.dt;
    }

    /// 高层接口：设定导航目标（后轴中心要到达的点）。
    pub fn set_goal(&mut self, goal: Vec3) {
        self.goal = Some(goal);
    }

    /// 推进一步（若设置了目标，则用比例转向朝目标行驶）。
    pub fn step(&mut self) {
        if let Some(g) = self.goal {
            let to_goal = Vec3::new(g.x - self.st.x, g.y - self.st.y, 0.0);
            let dist = to_goal.norm();
            if dist < 0.3 {
                self.drive(0.0, 0.0);
                self.goal = None;
                return;
            }
            let goal_angle = to_goal.y.atan2(to_goal.x);
            let heading_err = norm_angle(goal_angle - self.st.theta);
            let steering = (self.steer_gain * heading_err)
                .clamp(-self.model.max_steering, self.model.max_steering);
            // 近目标减速，避免过冲。
            let speed = (self.max_speed * (dist / 3.0).min(1.0)).max(0.5);
            self.drive(speed, steering);
        } else {
            self.drive(0.0, 0.0);
        }
    }
}
impl RobotBody for CarBody {
    fn kind(&self) -> RobotKind {
        RobotKind::Car
    }

    fn read_state(&mut self) -> Result<BodyState> {
        self.state.timestamp = instant_now();
        self.state.base =
            BasePose::new(Pose::from_translation(Vec3::new(self.st.x, self.st.y, 0.0)));
        self.state.base.pose.rotation =
            brain_core::Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), self.st.theta);
        self.state.base.linear_vel = Vec3::new(
            self.st.speed * self.st.theta.cos(),
            self.st.speed * self.st.theta.sin(),
            0.0,
        );
        self.state.base.angular_vel = Vec3::new(
            0.0,
            0.0,
            self.model.angular_velocity(self.st.speed, self.st.steering),
        );
        let st_steer = self.st.steering;
        for j in self.state.joints.iter_mut() {
            match j.name.as_str() {
                "steer_fl" | "steer_fr" => j.position = st_steer,
                "wheel_fl" | "wheel_fr" | "wheel_rl" | "wheel_rr" => j.position = self.wheel_travel,
                _ => {}
            }
        }
        Ok(self.state.clone())
    }

    fn send_command(&mut self, cmd: &EffectorCommand) -> Result<()> {
        match &cmd.task {
            Task::NavigateTo(TaskTarget::Point(p)) | Task::Reach(TaskTarget::Point(p)) => {
                self.set_goal(*p);
            }
            Task::Stop | Task::Hold | Task::Release => {
                self.goal = None;
                self.drive(0.0, 0.0);
            }
            _ => {}
        }
        Ok(())
    }

    fn shutdown(&mut self) {
        // 真实"断电"语义：清除目标并停止行驶。
        self.goal = None;
        self.drive(0.0, 0.0);
        self.wheel_travel = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::LocomotionMode;
    use brain_core::time::Timestamp;

    #[test]
    fn car_drives_toward_goal() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let mut car = CarBody::new(model, 0.0, 0.0, 0.0);
        car.set_goal(Vec3::new(20.0, 0.0, 0.0));
        for _ in 0..200 {
            car.step();
        }
        let (x, _, _) = car.pose();
        assert!(x > 15.0, "car should advance toward goal, x={x}");
        // 车速非零的累计位移已由 x>15 证明（车确实移动并靠近目标）。
    }

    #[test]
    fn car_steers_toward_offset_goal() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let mut car = CarBody::new(model, 0.0, 0.0, 0.0);
        // 目标在 +y（车右侧）→ 应产生正的前轮转角。
        car.set_goal(Vec3::new(0.0, 20.0, 0.0));
        for _ in 0..5 {
            car.step();
        }
        assert!(
            car.st.steering > 0.01,
            "should steer right, got {}",
            car.st.steering
        );
    }

    #[test]
    fn stop_clears_goal_and_halts() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let mut car = CarBody::new(model, 0.0, 0.0, 0.0);
        car.send_command(&EffectorCommand {
            timestamp: 0 as Timestamp,
            locomotion: LocomotionMode::Navigate,
            task: Task::NavigateTo(TaskTarget::Point(Vec3::new(10.0, 0.0, 0.0))),
        })
        .unwrap();
        for _ in 0..10 {
            car.step();
        }
        car.send_command(&EffectorCommand {
            timestamp: 0 as Timestamp,
            locomotion: LocomotionMode::Idle,
            task: Task::Stop,
        })
        .unwrap();
        for _ in 0..5 {
            car.step();
        }
        assert!(
            car.st.speed == 0.0,
            "car should be halted, speed={}",
            car.st.speed
        );
    }

    #[test]
    fn pose_reports_initial() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let car = CarBody::new(model, 1.0, -2.0, 0.5);
        let (x, y, th) = car.pose();
        assert!((x - 1.0).abs() < 1e-4);
        assert!((y + 2.0).abs() < 1e-4);
        assert!((th - 0.5).abs() < 1e-4);
    }

    #[test]
    fn kind_is_car() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let car = CarBody::new(model, 0.0, 0.0, 0.0);
        assert_eq!(car.kind(), RobotKind::Car);
    }

    #[test]
    fn direct_drive_moves_and_turns() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let mut car = CarBody::new(model, 0.0, 0.0, 0.0);
        let p0 = car.pose();
        // 直线加速 5 步。
        for _ in 0..5 {
            car.drive(2.0, 0.0);
        }
        let (x, _, _) = car.pose();
        assert!(x > p0.0 + 0.5, "should move forward, x={x}");
        // 打满转向应产生横移与转向。
        let (x1, y1, _) = car.pose();
        car.drive(2.0, 0.5);
        let (x2, y2, _) = car.pose();
        assert!(y2 != y1, "turning should change lateral pos");
        assert!(x2 >= x1);
        // 车轮行程累计非零。
        assert!(car.wheel_travel > 0.0);
    }

    #[test]
    fn reach_goal_sets_goal() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let mut car = CarBody::new(model, 0.0, 0.0, 0.0);
        car.send_command(&EffectorCommand {
            timestamp: 0,
            locomotion: LocomotionMode::Navigate,
            task: Task::Reach(TaskTarget::Point(Vec3::new(5.0, 0.0, 0.0))),
        })
        .unwrap();
        assert!(car.goal.is_some());
        for _ in 0..50 {
            car.step();
        }
        let (x, _, _) = car.pose();
        assert!(x > 3.0, "should approach reach goal, x={x}");
    }

    #[test]
    fn read_state_exposes_steering_joints_and_angular_vel() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let mut car = CarBody::new(model, 0.0, 0.0, 0.0);
        // 给一个转向，然后读状态。
        car.set_goal(Vec3::new(0.0, 20.0, 0.0));
        for _ in 0..5 {
            car.step();
        }
        let st = car.read_state().unwrap();
        assert_eq!(st.kind, RobotKind::Car);
        // 转向关节跟随当前前轮转角。
        let steer = st
            .joints
            .iter()
            .find(|j| j.name == "steer_fl")
            .unwrap()
            .position;
        assert!((steer - car.st.steering).abs() < 1e-4, "steer={steer}");
        // 角速度 = 自行车模型角速度。
        let wz = st.base.angular_vel.z;
        let expect = model.angular_velocity(car.st.speed, car.st.steering);
        assert!((wz - expect).abs() < 1e-4);
        // 四个接地点。
        assert_eq!(st.contacts.len(), 4);
        assert!(st.contacts.iter().all(|c| c.in_contact));
    }

    #[test]
    fn shutdown_stops_car_and_clears_goal() {
        let model = BicycleModel::new(2.6, 0.6, 0.8);
        let mut car = CarBody::new(model, 0.0, 0.0, 0.0);
        car.set_goal(Vec3::new(10.0, 0.0, 0.0));
        for _ in 0..5 {
            car.step();
        }
        assert!(car.st.speed > 0.0, "car should be moving");
        // shutdown = 断电：停止并清除目标。
        car.shutdown();
        assert_eq!(car.st.speed, 0.0);
        assert!(car.goal.is_none());
        assert_eq!(car.wheel_travel, 0.0);
    }
}
