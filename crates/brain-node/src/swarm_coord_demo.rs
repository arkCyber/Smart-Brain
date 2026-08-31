//! 蜂群协同演示：Leader 选举 + 任务分配（多机确定性一致）。
//!
//! 展示 `brain-mission::swarm_coord` 如何基于全网一致的态势收敛出唯一 leader，
//! 再由 leader 把巡检/航点任务分发给全体成员；分配表可经 SwarmLink 的 JSON
//! 接口在多机（Zenoh peer 模式）间传递。

use brain_core::Vec3;
use brain_mission::swarm::SwarmShare;
use brain_mission::swarm_coord::{SwarmCoordinator, SwarmTask, TaskAllocator};

/// 运行蜂群协同演示。
pub fn run() {
    println!("\n=== 蜂群协同（Leader 选举 + 任务分配）===");

    // 三机编队，各自广播态势。
    let shares = vec![
        SwarmShare::new("node-a", 1, Vec3::new(0.0, 0.0, -30.0), 0.0, false, 80.0),
        SwarmShare::new("node-b", 2, Vec3::new(50.0, 0.0, -30.0), 0.0, true, 60.0),
        SwarmShare::new("node-c", 3, Vec3::new(0.0, 50.0, -30.0), 0.0, false, 95.0),
    ];

    // 每台机器各自独立做选举，结果应一致。
    let coord_a = SwarmCoordinator::new("node-a");
    let coord_b = SwarmCoordinator::new("node-b");
    let coord_c = SwarmCoordinator::new("node-c");
    let leaders = [
        coord_a.is_leader(&shares),
        coord_b.is_leader(&shares),
        coord_c.is_leader(&shares),
    ];
    let leader = leaders
        .iter()
        .position(|&b| b)
        .map(|i| shares[i].node_id.as_str())
        .unwrap_or("?");
    println!("  选举结果（三机一致）: leader = {leader}");
    println!(
        "  各机 is_leader: a={} b={} c={}",
        leaders[0], leaders[1], leaders[2]
    );

    // leader 负责分发任务。
    let tasks = vec![
        SwarmTask::new("survey_nw", 3),
        SwarmTask::new("survey_ne", 2),
        SwarmTask::new("survey_sw", 2),
        SwarmTask::new("survey_se", 1),
    ];
    let plan = coord_a.plan(&shares, &tasks).expect("leader should plan");
    for (agent, assigned) in &plan {
        let ids: Vec<&str> = assigned.iter().map(|t| t.id.as_str()).collect();
        println!("  {agent} 承担: {ids:?}");
    }

    // 展示把分配表经 SwarmLink 的 JSON 接口序列化（供 Zenoh/UDP 传递）。
    let json = serde_json::to_string(&plan).expect("serialize plan");
    let back: Vec<(String, Vec<SwarmTask>)> = serde_json::from_str(&json).expect("deserialize");
    let _ = TaskAllocator::tasks_for(&back, "node-a");
    println!("  分配表 JSON 往返成功（{} 字节）", json.len());
}
