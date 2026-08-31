//! 大脑下发的控制指令（大脑 → 小脑）与感知层输出。

use serde::{Deserialize, Serialize};

use brain_core::time::Timestamp;

use crate::telemetry::Vec3;

/// 飞控飞行模式（映射到小脑的 flight mode）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Mode {
    /// 手动 / 待命。
    Idle,
    /// 自动起飞。
    Takeoff,
    /// 自动巡航 / 定点飞行。
    Cruise,
    /// 目标跟踪。
    Track,
    /// 自动降落。
    Land,
    /// 返航。
    ReturnHome,
    /// 自动悬停（fail-safe 时由看门狗强制触发）。
    Loiter,
}

/// 大脑下发给飞控的控制指令。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Command {
    pub timestamp: Timestamp,
    pub mode: Mode,
    /// 期望的位置 / 速度目标（按模式解释）。
    pub target: CommandTarget,
}

/// 指令目标：位置航点或速度向量。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CommandTarget {
    /// 北东地（NED）位置航点，米。
    Position { north: f32, east: f32, down: f32 },
    /// 机体速度指令，m/s。
    Velocity(Vec3),
    /// 无目标（例如进入悬停）。
    None,
}

/// 行为树生成的航点指令（任务层内部使用）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WaypointCommand {
    pub sequence: u32,
    pub north: f32,
    pub east: f32,
    pub down: f32,
    /// 到达容差，米。
    pub accept_radius: f32,
}

/// 感知层检测到的一个目标。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Detection {
    /// 目标类别 id。
    pub class_id: u32,
    /// 置信度 0..1。
    pub confidence: f32,
    /// 目标在相机坐标系下的方位（弧度，yaw/pitch）。
    pub bearing_yaw: f32,
    pub bearing_pitch: f32,
    /// 估计距离，米（深度估计输出）。
    pub range_m: f32,
}

/// 感知跟踪状态。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TrackingStatus {
    NoTarget,
    Acquiring,
    Locked { target: Detection, lock_age_ms: u64 },
    Lost { last_seen_ms: u64 },
}
