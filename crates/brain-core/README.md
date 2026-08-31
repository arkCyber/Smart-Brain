# brain-core

> Smart-Brain foundation: shared error, config, and time primitives

**所属层**：基础层 —— workspace 的"契约底座"，零外部系统依赖。

## 职责

供所有 crate 共用的最小基础设施：统一错误类型、配置结构、时间戳与通用数学原语。

- 统一错误类型 `BrainError` / `Result`
- 可 JSON 序列化的 `BrainConfig`（环境变量 → `config.json` → `config.example.json` → 内置默认）
- 可注入时钟 `Clock`（`SystemClock` / `ManualClock`）与真正单调计时器 `Stopwatch`
- **NTP 风格时间同步** `TimeSync` / `SyncDriver`（四时间戳握手估计偏移与往返时延，滑动窗口中位数滤波，带 `is_synced` / `is_stale` 健康判定，`SyncExchange` trait 可对接 UDP/串口/Zenoh）
- 通用数学原语 `Vec3` / `Quat` / `Pose`

## 核心 API

```rust
pub use error::{BrainError, Result};
pub use config::BrainConfig;
pub use math::{Pose, Quat, Vec3};
pub use time::{Clock, ManualClock, Stopwatch, SystemClock, TimeSync, SyncedClock, SyncDriver, SyncExchange, SyncSample, Timestamp, instant_now};
```

## 用法

```rust
use brain_core::{BrainConfig, Result, Vec3};

fn main() -> Result<()> {
    // 依次尝试候选路径加载配置，缺失时回退到内置默认值
    let cfg = BrainConfig::load_candidates(&["config.json", "config.example.json"]);
    println!("node_id = {}", cfg.node_id);

    let v = Vec3::new(1.0, 2.0, 3.0);
    println!("|v| = {}", v.norm());
    Ok(())
}
```

## 依赖

- 外部：`thiserror`、`serde`、`serde_json`、`log`
- 内部：无（不依赖其它 workspace crate）
