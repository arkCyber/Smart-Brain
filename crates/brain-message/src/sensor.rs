//! 通用传感器消息（可 `serde` 序列化，供数据总线 / 串口 / UDP 传输）。
//!
//! 与 `brain-odometry` 内部的**计算型**样本不同，这里定义的是**消息层**的通用传感器
//! 样本，身体无关、可跨层传输：IMU、里程计、测距扫描、接触力。配合
//! `brain-middleware` 的话题常量即可在总线上发布/订阅。

use serde::{Deserialize, Serialize};

use brain_core::time::Timestamp;
use brain_core::Vec3;

/// 四元数（用于里程计姿态），复导出自 `brain-core`。
pub use brain_core::Quat;

/// 通用 IMU 样本（机体系）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ImuSample {
    pub timestamp: Timestamp,
    /// 加速度（m/s²）。
    pub accel: Vec3,
    /// 角速度（rad/s）。
    pub gyro: Vec3,
}

impl ImuSample {
    pub fn new(timestamp: Timestamp, accel: Vec3, gyro: Vec3) -> Self {
        Self {
            timestamp,
            accel,
            gyro,
        }
    }
}

/// 通用里程计样本（位姿 + 速度）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OdometrySample {
    pub timestamp: Timestamp,
    /// 世界系位置（米）。
    pub position: Vec3,
    /// 世界系姿态（四元数）。
    pub orientation: Quat,
    /// 线速度（m/s）。
    pub linear_vel: Vec3,
    /// 角速度（rad/s）。
    pub angular_vel: Vec3,
}

impl OdometrySample {
    pub fn new(
        timestamp: Timestamp,
        position: Vec3,
        orientation: Quat,
        linear_vel: Vec3,
        angular_vel: Vec3,
    ) -> Self {
        Self {
            timestamp,
            position,
            orientation,
            linear_vel,
            angular_vel,
        }
    }
}

/// 一束测距扫描（如激光雷达 / 多束超声波）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RangeScan {
    pub timestamp: Timestamp,
    /// 每个方位角（弧度，相对机体前方，正值偏左）。
    pub bearings: Vec<f32>,
    /// 对应距离（米）。
    pub ranges: Vec<f32>,
    /// 最大量程（米）。
    pub max_range: f32,
}

impl RangeScan {
    pub fn new(timestamp: Timestamp, bearings: Vec<f32>, ranges: Vec<f32>, max_range: f32) -> Self {
        Self {
            timestamp,
            bearings,
            ranges,
            max_range,
        }
    }

    /// 光线数量。
    pub fn len(&self) -> usize {
        self.bearings.len().min(self.ranges.len())
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 取第 `i` 束的距离（越界返回 None）。
    pub fn beam_range(&self, i: usize) -> Option<f32> {
        if i >= self.len() {
            return None;
        }
        Some(self.ranges[i])
    }

    /// 返回钳制到 `[0, max_range]` 的距离副本（对坏数据鲁棒）。
    pub fn clamped_ranges(&self) -> Vec<f32> {
        self.ranges
            .iter()
            .map(|&r| r.clamp(0.0, self.max_range))
            .collect()
    }
}

/// 接触力样本（足端 / 轮 / 末端）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactSample {
    pub timestamp: Timestamp,
    /// 接触坐标系（如 "foot_fl"/"wheel_fl"）。
    pub frame: String,
    pub in_contact: bool,
    /// 接触力幅值（N）。
    pub force: f32,
}

impl ContactSample {
    pub fn new(
        timestamp: Timestamp,
        frame: impl Into<String>,
        in_contact: bool,
        force: f32,
    ) -> Self {
        Self {
            timestamp,
            frame: frame.into(),
            in_contact,
            force,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip<T: Serialize + for<'a> Deserialize<'a> + PartialEq + std::fmt::Debug>(v: &T) {
        let json = serde_json::to_string(v).unwrap();
        let back: T = serde_json::from_str(&json).unwrap();
        assert_eq!(v, &back, "serde round-trip failed for {json}");
    }

    #[test]
    fn imu_sample_round_trip() {
        round_trip(&ImuSample::new(
            10,
            Vec3::new(0.0, 0.0, -9.81),
            Vec3::new(0.01, 0.0, 0.0),
        ));
    }

    #[test]
    fn odometry_sample_round_trip() {
        round_trip(&OdometrySample::new(
            20,
            Vec3::new(1.0, 2.0, 0.0),
            Quat::IDENTITY,
            Vec3::new(0.5, 0.0, 0.0),
            Vec3::ZERO,
        ));
    }

    #[test]
    fn range_scan_len_and_round_trip() {
        let s = RangeScan::new(0, vec![0.0, 0.5], vec![1.0, 2.0], 10.0);
        assert_eq!(s.len(), 2);
        assert!(!s.is_empty());
        round_trip(&s);
    }

    #[test]
    fn contact_sample_round_trip() {
        round_trip(&ContactSample::new(5, "foot_fl", true, 12.5));
    }

    #[test]
    fn range_scan_beam_and_clamp() {
        let s = RangeScan::new(0, vec![0.0, 0.5], vec![1.5, 25.0], 10.0);
        assert_eq!(s.beam_range(0), Some(1.5));
        assert_eq!(s.beam_range(1), Some(25.0));
        assert_eq!(s.beam_range(5), None);
        // 25.0 被钳到 max_range=10.0。
        assert_eq!(s.clamped_ranges(), vec![1.5, 10.0]);
    }

    #[test]
    fn sensor_json_shape_stable() {
        // 锁定线上序列化形状，避免字段重排/更名破坏协议。
        let imu = ImuSample::new(1, Vec3::new(1.0, 2.0, 3.0), Vec3::new(0.25, 0.5, 0.75));
        let v = serde_json::to_value(&imu).unwrap();
        assert_eq!(v["timestamp"], 1);
        assert_eq!(v["accel"]["x"], 1.0);
        assert_eq!(v["gyro"]["z"], 0.75);

        let range = RangeScan::new(2, vec![0.0], vec![3.0], 10.0);
        let rv = serde_json::to_value(&range).unwrap();
        assert_eq!(rv["timestamp"], 2);
        assert_eq!(rv["max_range"], 10.0);
        assert_eq!(rv["ranges"][0], 3.0);
    }
}
