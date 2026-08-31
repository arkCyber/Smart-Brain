# brain-transport

> Smart-Brain hardware transport: serial/CAN/UDP link to the FCU

**所属层**：第 2 层 · 硬件接口层 —— 大脑与小脑（飞控）之间的物理链路抽象。

## 职责

定义统一的 `FcuTransport` trait，屏蔽底层**串口 / UDP / CAN** 的差异。默认提供 `MockTransport` 用于 SITL 仿真与单元测试；真机联调时启用 `serial` feature 使用真实串口（对应 Jetson/RK3588 上的 UART）。

- `FcuTransport` trait：大脑访问飞控的唯一入口，只下发高层意图
- 后端：`MockTransport`（默认）、`SerialTransport`（`serial` feature）、`UdpTransport`、`CanTransport`（`can`）、`ZenohFcuTransport`
- MAVLink 编解码 `MavLinkTransport` / `MavMessage` / `decode_stream`

## 核心 API

```rust
pub use can::{CanFrame, CanTransport, decode_telemetry, encode_command, encode_telemetry};
pub use mavlink::{MavLinkTransport, MavMessage, decode_stream};
pub use mock::MockTransport;
pub use serial_backend::{SerialConfig, SerialTransport};
pub use udp::UdpTransport;
pub use zenoh_fcu::{MockFcuZenoh, ZenohFcuTransport};
```

## 用法

```rust
use brain_message::{Command, CommandTarget, Mode};
use brain_transport::{FcuTransport, MockTransport};

fn main() {
    let mut fcu = MockTransport::new();
    let cmd = Command { timestamp: 0, mode: Mode::Takeoff, target: CommandTarget::None };
    fcu.send_command(&cmd).unwrap(); // 下发指令（需 trait 在作用域内）
    if let Some(telemetry) = fcu.try_recv_telemetry().unwrap() {
        println!("altitude = {}", telemetry.gps.alt);
    }
}
```

## 依赖

- 外部：`log`、`serde_json`
- 内部：`brain-core`、`brain-message`、`brain-zenoh`

> **应用案例**：`brain-node/zenoh_fcu_demo.rs` 演示通过 Zenoh 链路收发飞控遥测/指令。
