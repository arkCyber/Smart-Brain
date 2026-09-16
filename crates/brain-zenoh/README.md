# brain-zenoh

> Smart-Brain unified communication: Pub/Sub + Store/Query + Compute (Zenoh-style)

**所属层**：第 3 层 · 统一通信层 —— Zenoh 风格中间件。

## 职责

把 Zenoh 的三大支柱糅合为一个统一 API：

1. **发布/订阅（Pub/Sub）**：`put` / `subscribe`
2. **分布式存储与查询（Store/Query）**：`put` 写入存储，`get` 全网查询
3. **边缘计算（Compute）**：`declare_queryable` 在查询路径上触发计算/服务

默认使用纯 Rust 进程内实现 `LocalZenoh`（无网络依赖、离线可测，语义与 Zenoh 一致：键表达式、保留最近值、分布式存储聚合、按需计算）。启用 `real-zenoh` feature 可切换到真实 `zenoh` crate。

**生产特性**：
- 键表达式支持 `*`（单段）/`**`（任意深度）通配，并经 `valid_key_expr` 校验（拒绝空段/部分通配/保留字符）；
- `get` 在锁外执行用户 `QueryHandler`（避免死锁），并**隔离 handler 失败/panic**（handler 可返回 `Result`，异常不击穿进程）；
- `LocalZenoh` 可 `Clone`（克隆体共享同一份存储/订阅）；
- 订阅支持主动 `unsubscribe`（`Drop` 自动退订）；
- 存储管理：`value`/`contains`/`remove`/`clear`，以及 `store_count`/`subscription_count`/`queryable_count` 监控指标。

## 核心 API

```rust
pub use core::{Sample, Value, Reply, key_matches, valid_key_expr, is_concrete_key, encode, decode};
pub use backend::{CommBackend, Subscription, QueryableHandle, QueryHandler};
pub use local::{LocalZenoh, LocalZenohError};
pub use zenoh_impl::ZenohBackend;
```

## 用法

```rust
use brain_zenoh::{CommBackend, LocalZenoh};

fn main() {
    let zenoh = LocalZenoh::new();
    // 通配订阅：`*` 单段、`**` 任意深度
    let sub = zenoh.subscribe("sensor/**").unwrap();
    zenoh.put("sensor/temp", b"30.5".to_vec()).unwrap(); // 保留语义：订阅即收到最近值
    if let Ok(s) = sub.try_recv() {
        println!("received on {}: {}", s.key, String::from_utf8_lossy(&s.value));
    }
    // 主动退订（Drop 时也会自动退订）
    sub.unsubscribe();

    // 存储管理
    let v = zenoh.value("sensor/temp");
    println!("stored = {v:?}, count = {}", zenoh.store_count());
}
```

> 说明：所有键/键表达式均经 `valid_key_expr` 校验；`get` 会在锁外执行用户
> `QueryHandler`（避免 handler 内回调本后端导致死锁）。

## 依赖

- 外部：`log`、`serde`、`serde_json`
- 内部：`brain-core`

> **应用案例**：`brain-node/zenoh_demo.rs`（Pub/Sub + Store/Query + Compute）、`zenoh_fcu_demo.rs`（飞控链路走 Zenoh）。
