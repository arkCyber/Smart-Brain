//! `brain-message` — 大脑与飞控、模块之间的消息协议定义。
//!
//! 设计上模仿 MAVLink：定义明确的遥测（来自小脑）与指令（大脑下发）
//! 消息类型，所有消息可被 `serde` 序列化，便于通过数据总线/串口/UDP
//! 传输与持久化。

pub mod command;
pub mod frame;
pub mod sensor;
pub mod telemetry;

pub use command::{Command, CommandTarget, Detection, Mode, TrackingStatus, WaypointCommand};
pub use frame::{
    crc16, encode_frame, verify_frame, FrameReader, FRAME_OVERHEAD, MAX_FRAME_PAYLOAD,
};
pub use sensor::{ContactSample, ImuSample, OdometrySample, Quat, RangeScan};
pub use telemetry::{Attitude, BatteryStatus, GpsFix, Telemetry, Vec3};
