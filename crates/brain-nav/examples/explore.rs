//! `brain-nav` 最小示例：前沿探索——寻找下一个未知区域边界。
//!
//! 运行：`cargo run -p brain-nav --example explore`

use brain_core::Vec3;
use brain_mapping::{GridConfig, Index3, OccupancyGrid3D};
use brain_nav::Explorer;

fn main() {
    let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 8.0, 8.0, 8.0));
    // 把左下角一块标为空闲，使未知边界只在前方
    for x in 0..3 {
        for y in 0..8 {
            for z in 0..4 {
                grid.set_log_odds(Index3::new(x, y, z), -2.0);
            }
        }
    }

    let explorer = Explorer::new(0);
    if let Some(target) = explorer.next_target(&grid, Vec3::new(1.0, 1.0, 0.5)) {
        println!(
            "explore toward ({:.1}, {:.1}) — coverage {}",
            target.position.x, target.position.y, target.coverage
        );
    } else {
        println!("area fully explored");
    }
}
