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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_at_values() {
        let t = Telemetry::default_at(123);
        assert_eq!(t.timestamp, 123);
        assert_eq!(t.attitude.yaw, 0.0);
        assert_eq!(t.gps.fix_type, FixType::NoFix);
        assert_eq!(t.gps.satellites, 0);
        assert_eq!(t.battery.remaining_pct, 100.0);
        assert_eq!(t.battery.voltage, 16.8);
    }

    #[test]
    fn telemetry_serde_round_trip() {
        let mut t = Telemetry::default_at(42);
        t.attitude.roll = 0.1;
        t.attitude.pitch = -0.2;
        t.attitude.yaw = std::f32::consts::PI;
        t.gps.lat = 39.9;
        t.gps.lon = 116.4;
        t.gps.alt = 30.0;
        t.gps.fix_type = FixType::Fix3D;
        t.gps.satellites = 12;
        t.battery.remaining_pct = 67.5;
        t.battery.current = 3.2;
        t.velocity = Vec3::new(1.0, -1.0, 0.0);

        let json = serde_json::to_string(&t).unwrap();
        let back: Telemetry = serde_json::from_str(&json).unwrap();
        assert_eq!(back, t);
    }

    #[test]
    fn fix_type_serde_round_trip() {
        for f in [FixType::NoFix, FixType::Fix2D, FixType::Fix3D] {
            let json = serde_json::to_string(&f).unwrap();
            let back: FixType = serde_json::from_str(&json).unwrap();
            assert_eq!(back, f);
        }
    }

    #[test]
    fn telemetry_json_shape_stable() {
        let t = Telemetry::default_at(9);
        let json = serde_json::to_value(&t).unwrap();
        assert_eq!(json["timestamp"], 9);
        assert_eq!(json["gps"]["fix_type"], "NoFix");
        assert_eq!(json["battery"]["remaining_pct"], 100.0);
    }
}
