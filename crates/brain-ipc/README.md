# brain-ipc

> Smart-Brain low-latency IPC: zero-alloc ring buffers for high-rate sensor data

**所属层**：第 3 层 · 统一通信层 —— 本地极低延迟传输。

## 职责

用于在模块间传输百万级点云与高帧率图像：提供**预分配、零内存分配**的环形缓冲（覆盖式写入），以及一个线程安全的共享封装，适合在感知/避障主循环间搬运高频传感器帧。

## 核心 API

```rust
pub use ring::{FixedRingBuffer, RingError, SharedRing};
```

## 用法

```rust
use brain_ipc::{FixedRingBuffer, SharedRing};

fn main() {
    let mut ring = FixedRingBuffer::<f32>::new(4).unwrap();
    ring.push(3.14);
    if let Some(v) = ring.pop_oldest() {
        println!("ring value = {}", v);
    }

    let shared = SharedRing::new(64).unwrap();
    shared.push(42);
    println!("shared oldest = {:?}", shared.pop_oldest());
}
```

## 依赖

- 外部：无
- 内部：`brain-core`

> **应用案例**：`brain-odometry` 用环形缓冲缓存高频 IMU 帧；`brain-node/indoor.rs` 演示室内感知闭环。
