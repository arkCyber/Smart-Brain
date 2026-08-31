//! 任务行为树装配：把飞行节点组合成一个可执行任务树。

use brain_behavior_tree::control::{Selector, Sequence};
use brain_behavior_tree::core::Node;
use brain_behavior_tree::drone_nodes::*;
use brain_message::telemetry::Vec3;
use brain_message::CommandTarget;

/// 构建一条“巡检 + 目标发现/跟踪 + 返航降落”的任务行为树。
///
/// 结构（根为选择器）：
///   ├─ A) 正常任务序列
///   │     LogNode → Takeoff → Cruise → [检测并跟踪 | 返航] → Land
///   └─ B) Fail-safe 兜底（若上层触发，强制 Loiter）
pub fn build_mission_tree() -> Box<dyn Node> {
    Box::new(Selector::new(vec![
        // ---- A) 正常任务 ----
        Box::new(Sequence::new(vec![
            Box::new(LogNode::new("mission armed, executing survey")),
            Box::new(GpsFixCheck::new(8)),
            Box::new(Takeoff::new(30.0)),
            Box::new(Cruise::new(vec![
                CommandTarget::Position {
                    north: 80.0,
                    east: 0.0,
                    down: -30.0,
                },
                CommandTarget::Position {
                    north: 80.0,
                    east: 80.0,
                    down: -30.0,
                },
                CommandTarget::Position {
                    north: 0.0,
                    east: 80.0,
                    down: -30.0,
                },
            ])),
            // 目标处理：优先跟踪目标，否则返航。
            Box::new(Selector::new(vec![
                Box::new(Sequence::new(vec![
                    Box::new(DetectTarget::new(0.5)),
                    Box::new(TrackTarget::new(2.0)),
                ])),
                Box::new(ReturnHome::new()),
            ])),
            Box::new(Land::new()),
        ])),
        // ---- B) Fail-safe 兜底 ----
        Box::new(Failsafe::new("brain watchdog timeout -> loiter")),
    ]))
}

/// 示例：构造一个简单的速度指令（供文档展示）。
#[allow(dead_code)]
fn sample_velocity() -> CommandTarget {
    CommandTarget::Velocity(Vec3 {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    })
}
