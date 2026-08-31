//! 闭环自主导航演示：感知→建图→DWA 局部避障→探索/回溯。
//!
//! 把 `brain-mapping`（占据网格/光线投射）、`brain-nav`（前沿探索/面包屑回溯）、
//! `brain-planning`（DWA）串成一个可运行的自主导航闭环，跑在一个 2D 世界里。

use brain_autopilot::{Autopilot, AutopilotConfig, World};

/// 顶层入口。
pub fn run() {
    println!("\n=== 闭环自主导航（室内探索 / 避障 / 回溯）===");

    // 一个带障碍与死胡同的世界。
    let mut world = World::new(30, 30);
    world.wall(8..9, 0..30); // 左墙
    world.wall(20..21, 0..30); // 右墙
    world.wall(0..30, 26..27); // 口袋底
    world.set_obstacle(14, 14); // 中间立柱
    world.set_obstacle(15, 14);
    world.set_obstacle(14, 15);

    let cfg = AutopilotConfig {
        use_rrt: true, // 用 RRT 做全局路径引导（连续空间绕障）
        ..AutopilotConfig::default()
    };
    let mut ap = Autopilot::new(cfg, world.width(), world.height(), (11.0, 3.0, 0.0));

    let stats = ap.run(&world, 800);
    println!(
        "  运行 {} 步：距离 {:.1}m，探索 {:.0}%，被障碍阻断 {:.0} 次，回溯触发 {:.0} 次，全局规划 {} 次",
        stats.steps,
        stats.distance,
        stats.explored_ratio * 100.0,
        stats.blocked,
        stats.backtrack_events,
        stats.planned
    );
    println!(
        "  结束位姿 ({:.1}, {:.1})，安全={}",
        stats.end_pose.0, stats.end_pose.1, stats.safe
    );
    println!("  （全局引导：RRT 连续空间采样绕障；局部避障：DWA）");
}
