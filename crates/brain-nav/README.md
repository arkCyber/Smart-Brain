# brain-nav

> Smart-Brain indoor task layer: frontier exploration and breadcrumb backtracking

**所属层**：第 5 层 · 室内任务层 —— 未知区域探索 + 原路回溯。

## 职责

对应参考架构第 5 层：

- `explorer`：基于前沿（frontier）的未知区域探索，自动挑选最近的未知边界
- `backtrack`：基于历史安全轨迹的"面包屑"原路返回，用于全盲/故障时脱困

## 核心 API

```rust
pub use explorer::{Explorer, ExploreTarget};
pub use backtrack::{Backtracker, BacktrackMode};
```

## 用法

```rust
use brain_core::Vec3;
use brain_mapping::{GridConfig, Index3, OccupancyGrid3D};
use brain_nav::Explorer;

fn main() {
    let grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 8.0, 8.0, 8.0));
    let explorer = Explorer::new(0); // 期望高度层
    if let Some(target) = explorer.next_target(&grid, Vec3::new(1.0, 1.0, 0.5)) {
        println!("explore toward {:?}", target.position);
    }
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-mapping`

> **应用案例**：`brain-autopilot::Autopilot` 串起"建图 → 前沿探索 → DWA 避障 → 面包屑回溯"的完整自主闭环。
