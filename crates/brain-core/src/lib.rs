//! `brain-core` — Smart-Brain 基础层。
//!
//! 提供所有 crate 共用的最小基础设施：统一错误类型、配置结构、时间戳与
//! 通用数学原语。保持零外部系统依赖，作为 workspace 的“契约底座”。

pub mod config;
pub mod error;
pub mod math;
pub mod time;

pub use config::BrainConfig;
pub use error::{BrainError, Result};
pub use math::{Pose, Quat, Vec3};
pub use time::{instant_now, Timestamp};
