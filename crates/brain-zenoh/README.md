# brain-zenoh

> Smart-Brain unified communication: Pub/Sub + Store/Query + Compute (Zenoh-style)

**所属层**：第 3 层 · 统一通信层 —— Zenoh 风格中间件。

## 职责

把 Zenoh 的三大支柱糅合为一个统一 API：

1. **发布/订阅（Pub/Sub）**：`put` / `subscribe`
2. **分布式存储与查询（Store/Query）**：`put` 写入存储，`get` 全网查询
3. **边缘计算（Compute）**：`declare_queryable` 在查询路径上触发计算/服务

默认使用纯 Rust 进程内实现 `LocalZenoh`（无网络依赖、离线可测，语义与 Zenoh 一致：键表达式、保留最近值、分布式存储聚合、按需计算）。启用 `real-zenoh` feature 可切换到真实 `zenoh` crate。

## 核心 API

```rust
pub use core::{Sample, Value, Reply, key_matches, encode, decode};
pub use backend::{CommBackend, Subscription, QueryableHandle, QueryHandler};
pub use local::{LocalZenoh, LocalZenohError};
pub use zenoh_impl::ZenohBackend;
```

## 用法

```rust
use brain_zenoh::{LocalZenoh, CommBackend};

fn main() {
    let zenoh = LocalZenoh::new();
    let mut sub = zenoh.subscribe("demo/telemetry");
    zenoh.put("demo/telemetry", b"30.5".to_vec());
    if let Some(s) = sub.recv() {
        println!("received on {:?}: {:?}", s.key_expr, String::from_utf8_lossy(&s.value));
    }
}
```

## 依赖

- 外部：`log`、`serde`、`serde_json`
- 内部：`brain-core`

> **应用案例**：`brain-node/zenoh_demo.rs`（Pub/Sub + Store/Query + Compute）、`zenoh_fcu_demo.rs`（飞控链路走 Zenoh）。
