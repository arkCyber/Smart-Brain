//! `brain-planning` — 室内路径规划与局部避障。
//!
//! 对应参考架构第 4 层“规划核心”：在 3D 占据网格之上做图形搜索与
//! 运动学可行规划。
//! - `astar`：二维 A*，在占据网格的某一高度层找无碰撞航点路径
//! - `rrt`：快速探索随机树（RRT），连续空间采样，绕开障碍（确定性种子）
//! - `dwa`：动态窗口法，结合无人机运动学极限（最大加速度/转弯半径）做
//!   毫秒级局部速度避障，输出 `(v, ω)` 指令

pub mod ackermann_dwa;
pub mod astar;
pub mod dubins;
pub mod dwa;
pub mod reeds_shepp;
pub mod rrt;

pub use ackermann_dwa::{AckermannCommand, AckermannDwaConfig, AckermannDwaPlanner};
pub use astar::{AStar2D, GridPoint, Path};
pub use dubins::{DubinsConfig, DubinsPath, DubinsPlanner};
pub use dwa::{DwaConfig, DwaPlanner, VelocityCommand};
pub use reeds_shepp::{ReedsSheppConfig, ReedsSheppPath, ReedsSheppPlanner};
pub use rrt::{RrtConfig, RrtPath, RrtPlanner};
