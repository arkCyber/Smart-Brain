//! 通用机器人状态定义（身体 → 大脑）。

use brain_core::time::Timestamp;
use brain_core::{Pose, Vec3};
use serde::{Deserialize, Serialize};

/// 机器人“形态”（embodiment 类别）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum RobotKind {
    /// 无人机（旋翼/固定翼）。
    Aerial,
    /// 四足机器人。
    Quadruped,
    /// 双足人形。
    Humanoid,
    /// 轮式/履带底盘。
    Wheeled,
    /// 汽车（阿克曼/前轮转向，含乘用车、卡车）。
    Car,
    /// 机械臂（含移动操作臂）。
    Manipulator,
    /// 水下机器人（ROV/潜艇）。
    Underwater,
    /// 水面自动驾驶艇（ASV/USV，双差速推进）。
    SurfaceVessel,
}

impl RobotKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            RobotKind::Aerial => "aerial",
            RobotKind::Quadruped => "quadruped",
            RobotKind::Humanoid => "humanoid",
            RobotKind::Wheeled => "wheeled",
            RobotKind::Car => "car",
            RobotKind::Manipulator => "manipulator",
            RobotKind::Underwater => "underwater",
            RobotKind::SurfaceVessel => "surface_vessel",
        }
    }
}

/// 机体基座位姿（世界系）。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct BasePose {
    pub pose: Pose,
    /// 线速度（世界系，m/s）。
    pub linear_vel: Vec3,
    /// 角速度（世界系，rad/s）。
    pub angular_vel: Vec3,
}

impl BasePose {
    pub fn new(pose: Pose) -> Self {
        Self {
            pose,
            linear_vel: Vec3::ZERO,
            angular_vel: Vec3::ZERO,
        }
    }
}

/// 单个关节状态。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointState {
    pub name: String,
    /// 关节角（rad）或位移（m，对移动关节）。
    pub position: f32,
    /// 关节速度。
    pub velocity: f32,
    /// 关节力矩 / 力。
    pub effort: f32,
}

/// 接触状态（脚/轮/末端）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactState {
    pub frame: String,
    /// 是否接触地面/物体。
    pub in_contact: bool,
    /// 接触力幅值（N）。
    pub force: f32,
}

/// 一帧通用身体状态（身体 → 大脑）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyState {
    pub timestamp: Timestamp,
    pub kind: RobotKind,
    pub base: BasePose,
    pub joints: Vec<JointState>,
    pub contacts: Vec<ContactState>,
    /// 剩余电量百分比 0..100。
    pub battery_pct: f32,
}

impl BodyState {
    pub fn new(kind: RobotKind) -> Self {
        Self {
            timestamp: 0,
            kind,
            base: BasePose::new(Pose::IDENTITY),
            joints: Vec::new(),
            contacts: Vec::new(),
            battery_pct: 100.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kinds_are_serializable() {
        let j = serde_json::to_string(&RobotKind::Quadruped).unwrap();
        assert_eq!(j, "\"Quadruped\"");
        assert_eq!(RobotKind::Manipulator.as_str(), "manipulator");
    }
}
