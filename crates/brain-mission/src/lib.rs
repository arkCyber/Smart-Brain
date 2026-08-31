//! `brain-mission` — 任务与应用层。
//!
//! 对应参考架构第 5 层：航线自主规划（Mission Planning）与蜂群协同通信
//! （Swarm Link）。提供航点任务模型、任务执行器（把航点转成巡航指令），
//! 以及一个面向蜂群的广播链路（仿真实现）。

pub mod mission;
pub mod swarm;

pub use mission::{Mission, MissionExecutor, MissionPhase, MissionProgress, Waypoint};
pub use swarm::{SwarmLink, SwarmRole, SwarmShare};
