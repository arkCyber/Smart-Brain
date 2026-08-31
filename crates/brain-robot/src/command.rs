//! 通用高层指令定义（大脑 → 身体）。

use brain_core::time::Timestamp;
use brain_core::{Pose, Vec3};
use serde::{Deserialize, Serialize};

/// 通用运动/移动模式。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LocomotionMode {
    /// 待命 / 停住。
    Idle,
    /// 站立（保持平衡）。
    Stand,
    /// 行走（四足/双足）。
    Walk,
    /// 奔跑。
    Run,
    /// 导航到某地。
    Navigate,
    /// 跳跃/跨越障碍。
    Jump,
    /// 悬停/原地稳定。
    Hold,
}

/// 任务目标（位置或位姿）。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TaskTarget {
    /// 只指定目标点（移动类任务）。
    Point(Vec3),
    /// 指定完整位姿（操作/末端类任务）。
    Pose(Pose),
    /// 关节空间目标（joint index, target value）。
    Joint { index: usize, target: f32 },
    /// 无目标（停、悬停等）。
    None,
}

/// 高层任务。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Task {
    /// 移动身体到目标。
    NavigateTo(TaskTarget),
    /// 末端到达位姿（机械臂/灵巧操作）。
    Reach(TaskTarget),
    /// 抓取（在目标位姿处闭合夹爪）。
    Grasp(TaskTarget),
    /// 释放/松开。
    Release,
    /// 原地保持/悬停。
    Hold,
    /// 停止。
    Stop,
}

/// 发给身体的一帧高层指令。
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EffectorCommand {
    pub timestamp: Timestamp,
    pub locomotion: LocomotionMode,
    pub task: Task,
}

impl EffectorCommand {
    pub fn stop(timestamp: Timestamp) -> Self {
        Self {
            timestamp,
            locomotion: LocomotionMode::Idle,
            task: Task::Stop,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_core::{Pose, Vec3};

    #[test]
    fn command_serializes() {
        let cmd = EffectorCommand {
            timestamp: 1,
            locomotion: LocomotionMode::Navigate,
            task: Task::NavigateTo(TaskTarget::Point(Vec3::new(1.0, 2.0, 0.0))),
        };
        let s = serde_json::to_string(&cmd).unwrap();
        assert!(s.contains("NavigateTo"));
        let back: EffectorCommand = serde_json::from_str(&s).unwrap();
        assert_eq!(back.locomotion, LocomotionMode::Navigate);
        let _ = Pose::IDENTITY;
    }
}
