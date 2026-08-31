//! `brain-mapping` — 空间感知与避障核心：3D 占据网格（Voxel Grid）。
//!
//! 对应参考架构第 4 层“避障核心”。室内无 GPS、障碍细密，需要把点云转成
//! “已占据 / 空闲 / 未知”三种状态的 3D 概率网格。本模块提供：
//! - 概率占据网格 `OccupancyGrid3D`（log-odds 存储）
//! - 光线投射（3D DDA）更新：沿传感器光束把空闲/占据写入网格
//! - 碰撞查询、前沿（未知区）探测，供避障与探索使用

pub mod grid;
pub mod raycast;

pub use grid::{CellState, GridConfig, Index3, OccupancyGrid3D};
pub use raycast::RaycastUpdater;
