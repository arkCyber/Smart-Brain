//! `brain-sim` — 仿真接入层。
//!
//! 对应真机部署第一阶段的“先把 90% 测试跑在仿真里”：提供一个**可插拔的仿真后端
//! 契约**（[`sim::Simulator`]），让同一套“大脑”逻辑无需改动即可接入：
//! - 默认的确定性进程内 [`MockSimulator`]（2D 栅格世界 + 速度积分 + 测距 +
//!   目标检测），开箱即用、离线可测；
//! - 未来真机阶段可实现同一 trait 对接 Gazebo / AirSim / Isaac / 自研物理引擎，
//!   大脑代码零改动。
//!
//! 仿真对象是**身体**（`RobotBody` 的速度指令）而非具体链路，因此天然与
//! `brain-transport`/`brain-zenoh` 解耦——本 crate 只关心“世界怎么动 + 感知
//! 看到什么”。

pub mod mock;
pub mod sim;

pub use mock::MockSimulator;
pub use sim::{SimDetection, SimState, Simulator};
