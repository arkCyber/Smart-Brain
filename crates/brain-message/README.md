# brain-message

> Smart-Brain message/telemetry protocol definitions (MAVLink-like)

**所属层**：通信协议层 —— 大脑与飞控、各模块之间交换的消息类型。

## 职责

设计上模仿 MAVLink：定义明确的**遥测**（来自小脑）与**指令**（大脑下发）消息类型。所有消息可被 `serde` 序列化，便于通过数据总线 / 串口 / UDP / Zenoh 传输与持久化。

- 遥测 `Telemetry`：`Attitude`、`GpsFix`、`BatteryStatus`
- 指令 `Command`：`WaypointCommand`、`CommandTarget`、`Mode`、`Detection`、`TrackingStatus`
- 传感器帧 `ImuSample` / `OdometrySample` / `RangeScan` / `ContactSample`
- 帧封装 `FrameReader` / `encode_frame` / `verify_frame` / `crc16`（带 CRC 校验）

## 核心 API

```rust
pub use command::{Command, CommandTarget, Detection, Mode, TrackingStatus, WaypointCommand};
pub use frame::{FrameReader, crc16, encode_frame, verify_frame, MAX_FRAME_PAYLOAD, FRAME_OVERHEAD};
pub use sensor::{ContactSample, ImuSample, OdometrySample, Quat, RangeScan};
pub use telemetry::{Attitude, BatteryStatus, GpsFix, Telemetry, Vec3};
```

## 用法

```rust
use brain_message::{crc16, encode_frame, verify_frame, FrameReader, Command, CommandTarget, Mode};

fn main() {
    // 大脑下发航点指令
    let cmd = Command {
        timestamp: 7,
        mode: Mode::Takeoff,
        target: CommandTarget::Position { north: 0.0, east: 0.0, down: -30.0 },
    };
    // 先序列化负载，再封装成带 CRC 的传输帧
    let payload = serde_json::to_vec(&cmd).unwrap();
    let bytes = encode_frame(&payload);
    assert_eq!(verify_frame(&bytes).unwrap(), payload.as_slice());
    println!("framed {} bytes, crc = {:#06x}", bytes.len(), crc16(&payload));
}
```

> 说明：示例与单元测试默认不依赖 `serde_json`（序列化在调用方进行）；
> 直接演示帧封装/分帧见 `examples/frame.rs`。

## 依赖

- 外部：`serde`
- 内部：`brain-core`
