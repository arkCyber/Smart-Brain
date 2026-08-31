//! 具身适配示例：把无人机（`FcuTransport`）包装成一个 `RobotBody`。
//!
//! 这演示了核心设计——**无人机只是众多“身体”之一**。上层大脑通过统一的
//! `RobotBody` 接口驱动它，与驱动四足/机械臂的方式完全一致。

use brain_core::error::Result;
use brain_core::time::Timestamp;
use brain_core::Pose;
use brain_message::{Command, CommandTarget, Mode};
use brain_robot::body::RobotBody;
use brain_robot::command::{EffectorCommand, LocomotionMode, Task, TaskTarget};
use brain_robot::state::{BasePose, BodyState, RobotKind};
use brain_transport::FcuTransport;

/// 把任意 `FcuTransport`（飞控链路）适配为 `RobotBody`。
pub struct DroneBody {
    fcu: Box<dyn FcuTransport>,
    state: BodyState,
}

impl DroneBody {
    pub fn new(fcu: Box<dyn FcuTransport>) -> Self {
        Self {
            fcu,
            state: BodyState::new(RobotKind::Aerial),
        }
    }

    fn apply_mode(&self, loco: LocomotionMode) -> Mode {
        match loco {
            LocomotionMode::Idle | LocomotionMode::Stand | LocomotionMode::Hold => Mode::Loiter,
            LocomotionMode::Navigate => Mode::Cruise,
            LocomotionMode::Walk | LocomotionMode::Run | LocomotionMode::Jump => Mode::Cruise,
        }
    }
}

impl RobotBody for DroneBody {
    fn kind(&self) -> RobotKind {
        RobotKind::Aerial
    }

    fn read_state(&mut self) -> Result<BodyState> {
        let telem = match self.fcu.try_recv_telemetry()? {
            Some(t) => t,
            None => return Ok(self.state.clone()),
        };
        self.state.timestamp = telem.timestamp;
        self.state.battery_pct = telem.battery.remaining_pct;
        self.state.base = BasePose::new(Pose::new_euler(
            brain_core::Vec3::new(0.0, 0.0, telem.gps.alt),
            telem.attitude.roll,
            telem.attitude.pitch,
            telem.attitude.yaw,
        ));
        self.state.base.linear_vel =
            brain_core::Vec3::new(telem.velocity.x, telem.velocity.y, telem.velocity.z);
        Ok(self.state.clone())
    }

    fn send_command(&mut self, cmd: &EffectorCommand) -> Result<()> {
        let mode = self.apply_mode(cmd.locomotion);
        let target = match &cmd.task {
            Task::NavigateTo(TaskTarget::Point(p)) | Task::Reach(TaskTarget::Point(p)) => {
                CommandTarget::Position {
                    north: p.x,
                    east: p.y,
                    down: -p.z,
                }
            }
            _ => CommandTarget::None,
        };
        self.fcu.send_command(&Command {
            timestamp: cmd.timestamp,
            mode,
            target,
        })
    }

    fn shutdown(&mut self) {
        self.fcu.shutdown();
    }
}

/// 演示：用统一 `RobotBody` 接口驱动无人机（与驱动四足/机械臂方式一致）。
pub fn demonstrate(_now: Timestamp) {
    use brain_transport::MockTransport;

    let mut drone: Box<dyn RobotBody> = Box::new(DroneBody::new(Box::new(MockTransport::new())));
    println!("\n=== RobotBody 具身抽象演示 ===");
    println!("kind = {:?}", drone.kind());

    // 通过统一接口下发“导航到目标”，再读取身体状态。
    let nav = EffectorCommand {
        timestamp: 0,
        locomotion: LocomotionMode::Navigate,
        task: Task::NavigateTo(TaskTarget::Point(brain_core::Vec3::new(50.0, 0.0, 30.0))),
    };
    drone.send_command(&nav).expect("navigate");
    let s = drone.read_state().expect("read state");
    println!(
        "after NavigateTo -> base alt={:.1} battery={:.1}%",
        s.base.pose.position.z, s.battery_pct
    );
    println!("(四足/机械臂/人形只需各自实现 RobotBody，上层逻辑无需改动)");
}
