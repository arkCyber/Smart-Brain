# brain-middleware

> Smart-Brain data bus: topic publish/subscribe and module registry

**所属层**：第 3 层 · 决策/中间件层 —— 大脑内部的"神经网"（数据总线）。

## 职责

提供**类型安全的话题发布/订阅**（类似 ROS2 topic / Zenoh key-expression）以及按名字索引的**模块注册表**。各模块通过总线解耦：感知模块发布检测结果，决策模块订阅之，无需互相知道对方的存在。

## 核心 API

```rust
pub use bus::{DataBus, Topic};
```

## 用法

```rust
use brain_middleware::{DataBus, Topic};
use std::sync::Arc;

fn main() {
    let bus = DataBus::new();

    // 注册一个话题（类型安全），订阅/发布两端各自取到同一实例
    let alt: Arc<Topic<f64>> = bus.register("altitude");
    alt.publish(30.5, 1); // 发布消息 + 时间戳

    let t: Arc<Topic<f64>> = bus.topic("altitude").unwrap();
    println!("altitude = {}", t.peek().unwrap()); // 读取最近值
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-message`

> **应用案例**：`brain-node/parallel.rs` 用 `DataBus` 演示多线程并行流水线（感知线程 + 决策线程共享总线）。
