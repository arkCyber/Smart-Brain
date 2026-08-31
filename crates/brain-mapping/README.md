# brain-mapping

> Smart-Brain spatial perception: 3D occupancy/voxel grid with ray-casting update

**所属层**：第 4 层 · 避障核心 —— 3D 占据网格（Voxel Grid）。

## 职责

室内无 GPS、障碍细密，需要把点云转成"已占据 / 空闲 / 未知"三种状态的 3D 概率网格。

- 概率占据网格 `OccupancyGrid3D`（log-odds 存储）
- 光线投射（3D DDA）更新 `RaycastUpdater`：沿传感器光束写入空闲/占据
- 碰撞查询、前沿（未知区）探测，供避障与探索使用

## 核心 API

```rust
pub use grid::{CellState, GridConfig, Index3, OccupancyGrid3D};
pub use raycast::RaycastUpdater;
```

## 用法

```rust
use brain_core::Vec3;
use brain_mapping::{GridConfig, Index3, OccupancyGrid3D, RaycastUpdater};

fn main() {
    let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 10.0, 10.0, 10.0));
    let updater = RaycastUpdater::default();
    // 从原点沿 +X 打一束光，命中 3.3m 处障碍
    updater.update_ray(&mut grid, Vec3::new(0.5, 0.5, 0.5), Vec3::new(1.0, 0.0, 0.0), 6.0, Some(3.3));
    println!("hit = {:?}", grid.state(Index3::new(3, 0, 0)));
    println!("free = {:?}", grid.state(Index3::new(1, 0, 0)));
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`

> **应用案例**：`brain-autopilot` 用 `OccupancyGrid3D` 建图 → `brain-nav` 探索 → `brain-planning` 避障。
