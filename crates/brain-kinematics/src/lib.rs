//! `brain-kinematics` — 运动学基础。
//!
//! 面向四足/机械臂/人形等具身机器人：提供串联关节链的正运动学（FK）、
//! 几何雅可比与基于雅可比转置的逆运动学（IK）。无人机虽用不到关节，
//! 但操作臂与足式机器人必需；且本 crate 仅依赖 `brain-core` 的数学原语。

pub mod bicycle;
pub mod chain;

pub use bicycle::{norm_angle, BicycleModel, BicycleState};
pub use chain::{KinematicChain, Link};
