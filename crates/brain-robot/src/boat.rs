//! 水面自动驾驶艇（ASV/USV）身体适配 —— 双差速推进。
//!
//! 与 `CarBody`（阿克曼）不同，水面艇通常用**左右双推进器差速**转向（`v, ω`），
//! 无法“刹车”，靠水流与自身惯性滑行。本适配把高层 `NavigateTo` 翻译成
//! `(纵向速度, 偏航角速度)` 并沿差速模型积分，同时在 `BodyState` 中暴露
//! 左右推进器、尾舵与吃水线接触点。

use brain_core::error::Result;
use brain_core::time::instant_now;
use brain_core::{Pose, Vec3};
use brain_kinematics::norm_angle;

use crate::body::RobotBody;
use crate::command::{EffectorCommand, Task, TaskTarget};
use crate::state::{BasePose, BodyState, ContactState, JointState, RobotKind};

/// 一艘仿真水面艇（双差速推进）。
pub struct BoatBody {
    x: f32,
    y: f32,
    theta: f32,
    /// 纵向速度（m/s）与偏航角速度（rad/s）。
    v: f32,
    omega: f32,
    max_speed: f32,
    max_turn: f32,
    steer_gain: f32,
    goal: Option<Vec3>,
    dt: f32,
    state: BodyState,
}

impl BoatBody {
    pub fn new(x: f32, y: f32, theta: f32) -> Self {
        let mut state = BodyState::new(RobotKind::SurfaceVessel);
        state.joints = vec![
            JointState {
                name: "thruster_port".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
            JointState {
                name: "thruster_stbd".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
            JointState {
                name: "rudder".into(),
                position: 0.0,
                velocity: 0.0,
                effort: 0.0,
            },
        ];
        state.contacts = vec![
            ContactState {
                frame: "waterline_port".into(),
                in_contact: true,
                force: 0.0,
            },
            ContactState {
                frame: "waterline_stbd".into(),
                in_contact: true,
                force: 0.0,
            },
        ];
        Self {
            x,
            y,
            theta,
            v: 0.0,
            omega: 0.0,
            max_speed: 5.0,
            max_turn: 1.0,
            steer_gain: 2.0,
            goal: None,
            dt: 0.1,
            state,
        }
    }

    /// 位置与朝向。
    pub fn pose(&self) -> (f32, f32, f32) {
        (self.x, self.y, self.theta)
    }

    /// 低层接口：直接下发 `(纵向速度, 偏航角速度)`（来自差分 DWA 等）。
    pub fn drive(&mut self, v: f32, omega: f32) {
        self.v = v;
        self.omega = omega;
        self.theta += omega * self.dt;
        self.x += v * self.theta.cos() * self.dt;
        self.y += v * self.theta.sin() * self.dt;
    }

    /// 高层接口：设定导航目标。
    pub fn set_goal(&mut self, goal: Vec3) {
        self.goal = Some(goal);
    }

    /// 推进一步（若设定了目标，则差速纯追踪朝目标行驶）。
    pub fn step(&mut self) {
        if let Some(g) = self.goal {
            let dx = g.x - self.x;
            let dy = g.y - self.y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist < 0.3 {
                self.drive(0.0, 0.0);
                self.goal = None;
                return;
            }
            let goal_angle = dy.atan2(dx);
            let heading_err = norm_angle(goal_angle - self.theta);
            let omega = (self.steer_gain * heading_err).clamp(-self.max_turn, self.max_turn);
            // 转向越猛航速越低（先摆正船头）。
            let turn_penalty = 1.0 - (heading_err.abs() / std::f32::consts::PI).min(0.8);
            let speed = self.max_speed * turn_penalty * (dist / 3.0).clamp(0.3, 1.0);
            self.drive(speed, omega);
        } else {
            self.drive(0.0, 0.0);
        }
    }
}
impl RobotBody for BoatBody {
    fn kind(&self) -> RobotKind {
        RobotKind::SurfaceVessel
    }

    fn read_state(&mut self) -> Result<BodyState> {
        self.state.timestamp = instant_now();
        self.state.base = BasePose::new(Pose::from_translation(Vec3::new(self.x, self.y, 0.0)));
        self.state.base.pose.rotation =
            brain_core::Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), self.theta);
        self.state.base.linear_vel =
            Vec3::new(self.v * self.theta.cos(), self.v * self.theta.sin(), 0.0);
        self.state.base.angular_vel = Vec3::new(0.0, 0.0, self.omega);
        for j in self.state.joints.iter_mut() {
            match j.name.as_str() {
                "thruster_port" | "thruster_stbd" => j.velocity = self.v,
                "rudder" => j.position = (self.omega / self.max_turn).clamp(-1.0, 1.0),
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
        // 真实"断电"语义：清除目标并停止推进。
        self.goal = None;
        self.drive(0.0, 0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::LocomotionMode;
    use brain_core::time::Timestamp;

    #[test]
    fn boat_drives_toward_goal() {
        let mut boat = BoatBody::new(0.0, 0.0, 0.0);
        boat.set_goal(Vec3::new(20.0, 0.0, 0.0));
        for _ in 0..200 {
            boat.step();
        }
        let (x, _, _) = boat.pose();
        assert!(x > 15.0, "boat should advance toward goal, x={x}");
    }

    #[test]
    fn boat_turns_toward_goal() {
        let mut boat = BoatBody::new(0.0, 0.0, 0.0);
        boat.set_goal(Vec3::new(0.0, 20.0, 0.0)); // 目标在左侧(+y)
        for _ in 0..10 {
            boat.step();
        }
        assert!(boat.theta > 0.0, "should turn left, theta={}", boat.theta);
    }

    #[test]
    fn stop_halts() {
        let mut boat = BoatBody::new(0.0, 0.0, 0.0);
        boat.send_command(&EffectorCommand {
            timestamp: 0 as Timestamp,
            locomotion: LocomotionMode::Navigate,
            task: Task::NavigateTo(TaskTarget::Point(Vec3::new(10.0, 0.0, 0.0))),
        })
        .unwrap();
        for _ in 0..10 {
            boat.step();
        }
        boat.send_command(&EffectorCommand {
            timestamp: 0 as Timestamp,
            locomotion: LocomotionMode::Idle,
            task: Task::Stop,
        })
        .unwrap();
        for _ in 0..5 {
            boat.step();
        }
        assert!(boat.v == 0.0, "boat should stop, v={}", boat.v);
    }

    #[test]
    fn pose_reports_initial() {
        let boat = BoatBody::new(2.0, -1.0, 0.3);
        let (x, y, th) = boat.pose();
        assert!((x - 2.0).abs() < 1e-4);
        assert!((y + 1.0).abs() < 1e-4);
        assert!((th - 0.3).abs() < 1e-4);
    }

    #[test]
    fn kind_is_surface_vessel() {
        let boat = BoatBody::new(0.0, 0.0, 0.0);
        assert_eq!(boat.kind(), RobotKind::SurfaceVessel);
    }

    #[test]
    fn direct_drive_moves_and_rotates() {
        let mut boat = BoatBody::new(0.0, 0.0, 0.0);
        // 直线前进。
        for _ in 0..5 {
            boat.drive(2.0, 0.0);
        }
        let (x, _, _) = boat.pose();
        assert!(x > 0.5, "should move forward, x={x}");
        // 差速转向（omega>0）应改变朝向并产生横向位移。
        let before = boat.theta;
        boat.drive(1.0, 0.5);
        assert!(boat.theta > before, "omega should turn the boat");
    }

    #[test]
    fn hold_clears_goal_and_stops() {
        let mut boat = BoatBody::new(0.0, 0.0, 0.0);
        boat.set_goal(Vec3::new(10.0, 0.0, 0.0));
        boat.step();
        assert!(boat.v > 0.0, "should be moving toward goal");
        boat.send_command(&EffectorCommand {
            timestamp: 0,
            locomotion: LocomotionMode::Idle,
            task: Task::Hold,
        })
        .unwrap();
        boat.step();
        assert!(boat.v == 0.0, "hold should stop thrusters, v={}", boat.v);
        assert!(boat.goal.is_none());
    }

    #[test]
    fn read_state_exposes_thrusters_and_rudder() {
        let mut boat = BoatBody::new(0.0, 0.0, 0.0);
        // 设一个偏航目标，让尾舵有转角、推进器有速度。
        boat.set_goal(Vec3::new(0.0, 20.0, 0.0));
        for _ in 0..5 {
            boat.step();
        }
        let st = boat.read_state().unwrap();
        assert_eq!(st.kind, RobotKind::SurfaceVessel);
        // 推进器速度 = v。
        let thr = st
            .joints
            .iter()
            .find(|j| j.name == "thruster_port")
            .unwrap()
            .velocity;
        assert!((thr - boat.v).abs() < 1e-4);
        // 尾舵位置在 [-1,1]。
        let rudder = st
            .joints
            .iter()
            .find(|j| j.name == "rudder")
            .unwrap()
            .position;
        assert!(rudder.abs() <= 1.0);
        // 角速度 = omega。
        assert!((st.base.angular_vel.z - boat.omega).abs() < 1e-4);
        // 两个吃水线接触点。
        assert_eq!(st.contacts.len(), 2);
        assert!(st.contacts.iter().all(|c| c.in_contact));
    }

    #[test]
    fn shutdown_stops_boat_and_clears_goal() {
        let mut boat = BoatBody::new(0.0, 0.0, 0.0);
        boat.set_goal(Vec3::new(10.0, 0.0, 0.0));
        boat.step();
        assert!(boat.v > 0.0, "boat should be moving");
        // shutdown = 断电：停止推进并清除目标。
        boat.shutdown();
        assert_eq!(boat.v, 0.0);
        assert_eq!(boat.omega, 0.0);
        assert!(boat.goal.is_none());
    }
}
