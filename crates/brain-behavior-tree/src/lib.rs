//! `brain-behavior-tree` — 决策层：行为树框架 + 飞行节点。
//!
//! 对应参考架构第 3/4 层的决策逻辑。将飞行行为拆解为可组合节点：
//! 组合节点（Sequence / Selector）与装饰器（Inverter / Retry）负责逻辑，
//! 行为节点（起飞/巡航/发现目标/跟踪/返航/降落/fail-safe）负责具体动作。
//! 相比 if-else / FSM，行为树更易维护与复用。

pub mod control;
pub mod core;
pub mod drone_nodes;

pub use control::{Inverter, Retry, Selector, Sequence};
pub use core::{BehaviorContext, BrainOutput, Node, Status, Tree};
pub use drone_nodes::{
    BatteryCheck, Cruise, DetectTarget, Failsafe, GpsFixCheck, Land, LogNode, ReturnHome, Takeoff,
    TrackTarget,
};
