//! 仿真后端契约：世界状态、感知输出与 `Simulator` trait。

use brain_core::time::Timestamp;
use brain_core::{Pose, Vec3};

/// 仿真世界里的一次目标检测（感知输出，供大脑决策）。
#[derive(Debug, Clone, PartialEq)]
pub struct SimDetection {
    /// 类别 id。
    pub class_id: u32,
    /// 置信度 0..1。
    pub confidence: f32,
    /// 距离（m）。
    pub range_m: f32,
    /// 相对机体前向的方位角（rad，正值偏左）。
    pub bearing_rad: f32,
}

/// 仿真世界状态快照。
#[derive(Debug, Clone)]
pub struct SimState {
    pub timestamp: Timestamp,
    pub robot_pose: Pose,
    pub robot_linear_vel: Vec3,
    pub robot_angular_vel: Vec3,
    /// 累计碰撞次数（用于安全评估）。
    pub collisions: u64,
    /// 已完成的仿真步数。
    pub steps: u64,
}

/// 可插拔的仿真后端。
///
/// 实现者只需提供“推进世界 + 读状态 + 下发速度指令 + 感知查询”。真机阶段
/// （Gazebo/AirSim/Isaac）实现同一 trait 即可无缝替换 `MockSimulator`。
pub trait Simulator: Send {
    /// 推进 `dt` 秒。返回 `Err` 表示仿真失败（需上层兜底）。
    fn step(&mut self, dt: f32) -> brain_core::Result<()>;

    /// 当前世界状态。
    fn state(&self) -> SimState;

    /// 下发身体速度指令（机体系线速度 + 角速度）。
    fn set_velocity_command(&mut self, linear: Vec3, angular: Vec3) -> brain_core::Result<()>;

    /// 沿指定方位角的测距（m）。方位角相对机体前向，正值偏左。
    fn range(&self, bearing_rad: f32) -> f32;

    /// 当前帧感知到的目标列表。
    fn detections(&self) -> Vec<SimDetection>;

    /// 把机器人与世界重置到指定位姿。
    fn reset(&mut self, pose: Pose);
}
