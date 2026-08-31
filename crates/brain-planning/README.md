# brain-planning

> Smart-Brain path planning: grid A* and dynamic-window local obstacle avoidance

**所属层**：第 4 层 · 规划核心 —— 在占据网格之上做图形搜索与运动学可行规划。

## 职责

在 3D 占据网格之上做全局与局部路径规划：

- `astar`：二维 A*，在占据网格某一高度层找无碰撞航点路径
- `rrt`：快速探索随机树（连续空间采样，确定性种子）
- `dwa`：动态窗口法，结合无人机运动学极限做毫秒级局部速度避障，输出 `(v, ω)`
- `ackermann_dwa`：阿克曼（汽车）局部避障
- `dubins` / `reeds_shepp`：汽车圆弧/直线最短路径与可倒车的掉头/泊车路径

## 核心 API

```rust
pub use astar::{AStar2D, GridPoint, Path};
pub use rrt::{RrtConfig, RrtPath, RrtPlanner};
pub use dwa::{DwaConfig, DwaPlanner, VelocityCommand};
pub use ackermann_dwa::{AckermannCommand, AckermannDwaConfig, AckermannDwaPlanner};
pub use dubins::{DubinsConfig, DubinsPath, DubinsPlanner};
pub use reeds_shepp::{ReedsSheppConfig, ReedsSheppPath, ReedsSheppPlanner};
```

## 用法

```rust
use brain_mapping::{GridConfig, Index3, OccupancyGrid3D};
use brain_planning::{AStar2D, GridPoint};

fn main() {
    let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 10.0, 10.0, 10.0));
    for y in 2..8 {
        grid.set_log_odds(Index3::new(3, y, 0), 2.0); // 一堵墙
    }
    let astar = AStar2D::new(0, 0.1); // 高度层 z、安全半径
    if let Some(path) = astar.plan(&grid, GridPoint { x: 0, y: 4 }, GridPoint { x: 6, y: 4 }) {
        println!("path length = {}", path.len());
    }
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-mapping`、`brain-kinematics`

> **应用案例**：`brain-autopilot`（DWA 避障 + 回溯）、`CarAutopilot`（A*/RRT → Dubins 平滑 → Ackermann DWA → 倒车闭环）。
