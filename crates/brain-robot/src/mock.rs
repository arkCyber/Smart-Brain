//! 通用仿真身体：不依赖任何传输，供 SITL 与单元测试演示任意形态机器人。

use brain_core::time::instant_now;
use brain_core::{Result, Vec3};

use crate::body::RobotBody;
use crate::command::{EffectorCommand, Task, TaskTarget};
use crate::state::{BodyState, ContactState, JointState, RobotKind};

/// 通用的“仿真身体”，可配置为任意 `RobotKind`。
///
/// 收到 `NavigateTo`/`Reach` 会移动基座或末端；收到 `Grasp` 会建立接触；
/// 用于在没有真实身体时验证“大脑”的行为树/决策逻辑。
pub struct MockRobotBody {
    kind: RobotKind,
    state: BodyState,
    /// 单步移动速度（每帧位移，m）。
    step: f32,
}

impl MockRobotBody {
    pub fn new(kind: RobotKind) -> Self {
        let mut state = BodyState::new(kind);
        // 给每种形态预置一些关节与接触，便于观察。
        match kind {
            RobotKind::Quadruped | RobotKind::Humanoid => {
                state.joints = (0..12)
                    .map(|i| JointState {
                        name: format!("leg_{i}"),
                        position: 0.0,
                        velocity: 0.0,
                        effort: 0.0,
                    })
                    .collect();
                state.contacts = vec![
                    ContactState {
                        frame: "foot_fl".into(),
                        in_contact: true,
                        force: 0.0,
                    },
                    ContactState {
                        frame: "foot_fr".into(),
                        in_contact: true,
                        force: 0.0,
                    },
                ];
            }
            RobotKind::Manipulator => {
                state.joints = (0..6)
                    .map(|i| JointState {
                        name: format!("joint_{i}"),
                        position: 0.0,
                        velocity: 0.0,
                        effort: 0.0,
                    })
                    .collect();
            }
            RobotKind::SurfaceVessel => {
                // 水面艇：双差速推进（左右推进器）+ 尾舵，以及与水的“接触”（浮于水面）。
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
            }
            RobotKind::Car | RobotKind::Wheeled => {
                // 汽车：2 个前轮转向关节 + 4 个车轮，以及 4 个接地接触点。
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
            }
            _ => {}
        }
        Self {
            kind,
            state,
            step: 1.0,
        }
    }
}

impl RobotBody for MockRobotBody {
    fn kind(&self) -> RobotKind {
        self.kind
    }

    fn read_state(&mut self) -> Result<BodyState> {
        self.state.timestamp = instant_now();
        Ok(self.state.clone())
    }

    fn send_command(&mut self, cmd: &EffectorCommand) -> Result<()> {
        match &cmd.task {
            Task::NavigateTo(TaskTarget::Point(p)) | Task::Reach(TaskTarget::Point(p)) => {
                let delta = p.sub(self.state.base.pose.position).normalized() * self.step;
                self.state.base.pose.position = self.state.base.pose.position.add(delta);
                self.state.base.linear_vel = delta;
            }
            Task::Grasp(_) => {
                for c in self.state.contacts.iter_mut() {
                    if c.frame.starts_with("foot") || c.frame.starts_with("gripper") {
                        c.in_contact = true;
                        c.force = 10.0;
                    }
                }
            }
            Task::Hold | Task::Stop => {
                self.state.base.linear_vel = Vec3::ZERO;
                self.state.base.angular_vel = Vec3::ZERO;
            }
            Task::NavigateTo(TaskTarget::Joint { index, target }) => {
                if let Some(j) = self.state.joints.get_mut(*index) {
                    j.position = *target;
                }
            }
            _ => {}
        }
        Ok(())
    }

    fn shutdown(&mut self) {}
}
