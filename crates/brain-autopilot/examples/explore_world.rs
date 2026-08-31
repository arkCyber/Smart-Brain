//! `brain-autopilot` 最小示例：在 2D 世界里闭环自主探索（安全）。
//!
//! 运行：`cargo run -p brain-autopilot --example explore_world`

use brain_autopilot::{Autopilot, AutopilotConfig, World};

fn main() {
    let mut world = World::new(20, 20);
    world.wall(5..6, 0..10); // 一堵竖墙
    world.wall(0..20, 12..13); // 一条横墙

    let mut pilot = Autopilot::new(
        AutopilotConfig::default(),
        world.width(),
        world.height(),
        (1.0, 1.0, 0.0),
    );

    let stats = pilot.run(&world, 600);
    println!(
        "safe = {} | distance = {:.1} m | explored = {:.0}% | A* planned = {}",
        stats.safe,
        stats.distance,
        stats.explored_ratio * 100.0,
        stats.planned
    );
}
