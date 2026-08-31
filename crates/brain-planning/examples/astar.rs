//! `brain-planning` 最小示例：A* 在带墙网格上规划无碰撞路径。
//!
//! 运行：`cargo run -p brain-planning --example astar`

use brain_mapping::{GridConfig, Index3, OccupancyGrid3D};
use brain_planning::{AStar2D, GridPoint};

fn main() {
    let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 10.0, 10.0, 10.0));
    for y in 2..8 {
        grid.set_log_odds(Index3::new(3, y, 0), 2.0); // 在 x=3 竖一堵墙
    }

    let astar = AStar2D::new(0, 0.1);
    if let Some(path) = astar.plan(&grid, GridPoint { x: 0, y: 4 }, GridPoint { x: 6, y: 4 }) {
        print!("path ({} points):", path.len());
        for p in &path {
            print!(" ({},{})", p.x, p.y);
        }
        println!();
    } else {
        println!("no path found");
    }
}
