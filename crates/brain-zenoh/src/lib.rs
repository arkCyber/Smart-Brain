//! `brain-zenoh` — 统一通信中间件层（Zenoh 风格）。
//!
//! 对应参考架构第 3 层。把 Zenoh 的三大支柱糅合为一个统一 API：
//! 1. **发布/订阅（Pub/Sub）**：`put` / `subscribe`
//! 2. **地理分布式存储与查询（Store/Query）**：`put` 写入存储，`get` 全网查询
//! 3. **边缘计算（Compute）**：`declare_queryable` 在查询路径上触发计算/服务
//!
//! 默认使用纯 Rust 的进程内实现 `LocalZenoh`（无需任何网络依赖，可离线测试，
//! 语义与 Zenoh 一致：键表达式、保留最近值、分布式存储聚合、按需计算）。
//! 启用 `real-zenoh` feature 时可切换到真实 `zenoh` crate（见 `zenoh_impl`）。
//!
//! 生产特性：
//! - 键表达式支持 `*`（单段）、`**`（任意深度）通配，并经 [`core::valid_key_expr`]
//!   校验（拒绝空段/部分通配/保留字符，段数上限 [`core::MAX_KEY_DEPTH`]）；
//! - `get` 在**锁外**执行用户 `QueryHandler`（避免 handler 内回调本后端导致死锁），
//!   并**隔离 handler 的失败与 panic**——单个计算/服务异常（返回 `Err` 或 panic）
//!   会被记录并跳过，不会击穿进程；
//! - `LocalZenoh` 可 `Clone`（克隆体共享同一份存储/订阅，便于多模块持有）；
//! - 订阅支持主动 `unsubscribe`（`Drop` 自动退订）；
//! - 存储管理：`value`/`contains`/`remove`/`clear`，并有 `store_count`/
//!   `subscription_count`/`queryable_count` 监控指标。

pub mod backend;
pub mod core;
pub mod local;
#[cfg(feature = "real-zenoh")]
pub mod zenoh_impl;

pub use backend::{CommBackend, QueryHandler, QueryableHandle, Subscription};
pub use core::{
    decode, encode, is_concrete_key, key_matches, valid_key_expr, Reply, Sample, Value,
};
pub use local::{LocalZenoh, LocalZenohError};
#[cfg(feature = "real-zenoh")]
pub use zenoh_impl::ZenohBackend;
