//! `brain-nav` — 室内任务层。
//!
//! 对应参考架构第 5 层：
//! - `explorer`：基于前沿（frontier）的未知区域探索，自动挑选最近的未知边界
//! - `backtrack`：基于历史安全轨迹的“面包屑”原路返回，用于全盲/故障时脱困

pub mod backtrack;
pub mod explorer;

pub use backtrack::{BacktrackMode, Backtracker};
pub use explorer::{ExploreTarget, Explorer};
