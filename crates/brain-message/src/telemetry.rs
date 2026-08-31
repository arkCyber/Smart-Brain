//! 飞控遥测消息（小脑 → 大脑）。

use serde::{Deserialize, Serialize};

use brain_core::time::Timestamp;

/// 单个遥测包。携带时间戳便于对齐与延迟测量。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Telemetry {
    pub timestamp: Timestamp,
    pub attitude: Attitude,
    pub gps: GpsFix,
    pub battery: BatteryStatus,
    /// 地面（或机体坐标系）速度，m/s。
    pub velocity: Vec3,
}

/// 机体三轴姿态（欧拉角，弧度）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Attitude {
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// 三轴向量（速度 / 加速度）。复用 `brain-core` 的通用数学类型，避免重复定义。
pub use brain_core::Vec3;

/// GPS 定位信息。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsFix {
    pub lat: f64,
    pub lon: f64,
    pub alt: f32,
    pub fix_type: FixType,
    /// 卫星数。
    pub satellites: u8,
}

/// GPS 定位质量。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FixType {
    NoFix,
    Fix2D,
    Fix3D,
}

/// 电池状态。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BatteryStatus {
    /// 剩余电量百分比 0..100。
    pub remaining_pct: f32,
    pub voltage: f32,
    pub current: f32,
}

impl Telemetry {
    /// 构造一份默认遥测（用于仿真与测试）。
    pub fn default_at(timestamp: Timestamp) -> Self {
        Self {
            timestamp,
            attitude: Attitude {
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
            },
            gps: GpsFix {
                lat: 0.0,
                lon: 0.0,
                alt: 0.0,
                fix_type: FixType::NoFix,
                satellites: 0,
            },
            battery: BatteryStatus {
                remaining_pct: 100.0,
                voltage: 16.8,
                current: 0.0,
            },
            velocity: Vec3::default(),
        }
    }
}
