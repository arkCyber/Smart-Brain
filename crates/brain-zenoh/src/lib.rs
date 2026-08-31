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

pub mod backend;
pub mod core;
pub mod local;
#[cfg(feature = "real-zenoh")]
pub mod zenoh_impl;

pub use backend::{CommBackend, QueryHandler, QueryableHandle, Subscription};
pub use core::{decode, encode, key_matches, Reply, Sample, Value};
pub use local::{LocalZenoh, LocalZenohError};
#[cfg(feature = "real-zenoh")]
pub use zenoh_impl::ZenohBackend;
