//! `brain-mapping` 最小示例：光线投射把“空闲/占据”写入 3D 占据网格。
//!
//! 运行：`cargo run -p brain-mapping --example raycast`

use brain_core::Vec3;
use brain_mapping::{GridConfig, Index3, OccupancyGrid3D, RaycastUpdater};

fn main() {
    let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 10.0, 10.0, 10.0));
    let updater = RaycastUpdater::default();

    // 从 (0.5,0.5,0.5) 沿 +X 打一束光，命中 3.3m 处的障碍
    updater.update_ray(
        &mut grid,
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(1.0, 0.0, 0.0),
        6.0,
        Some(3.3),
    );

    println!("hit cell  = {:?}", grid.state(Index3::new(3, 0, 0)));
    println!("free cell = {:?}", grid.state(Index3::new(1, 0, 0)));
}
