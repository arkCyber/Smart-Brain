//! `brain-robot` — 具身（embodiment）抽象层。
//!
//! 这是让“大脑”适用于任意具身机器人的关键 crate。核心思想：
//! **大脑（感知/决策/规划）与身体（无人机/四足/轮式/机械臂/人形）解耦**。
//! 身体通过统一的 `RobotBody` trait 暴露自身状态并接受高层指令，
//! 无人机只是其中的一种实现（见 `brain-node` 中的适配示例）。

pub mod boat;
pub mod body;
pub mod car;
pub mod command;
pub mod mock;
pub mod state;

pub use boat::BoatBody;
pub use body::{BodyCommandSink, RobotBody};
pub use car::CarBody;
pub use command::{EffectorCommand, LocomotionMode, Task, TaskTarget};
pub use mock::MockRobotBody;
pub use state::{BasePose, BodyState, ContactState, JointState, RobotKind};
