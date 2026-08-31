//! `brain-transport` — 大脑与小脑（飞控）之间的物理链路抽象。
//!
//! 定义统一的 `FcuTransport` trait，屏蔽底层串口/UDP/CAN 的差异。
//! 默认提供 `MockTransport` 用于 SITL 仿真与单元测试；真机联调时启用
//! `serial` feature 使用真实串口（对应 Jetson/RK3588 上的 UART）。

pub mod can;
pub mod mavlink;
pub mod mock;
pub mod serial_backend;
pub mod udp;
pub mod zenoh_fcu;

pub use can::{decode_telemetry, encode_command, encode_telemetry, CanFrame, CanTransport};
pub use mavlink::{decode_stream, MavLinkTransport, MavMessage};
pub use mock::MockTransport;
pub use serial_backend::{SerialConfig, SerialTransport};
pub use udp::UdpTransport;
pub use zenoh_fcu::{MockFcuZenoh, ZenohFcuTransport};

use brain_core::Result;
use brain_message::{Command, Telemetry};

/// 与飞控（小脑）的传输通道抽象。
pub trait FcuTransport: Send {
    /// 向飞控下发一条控制指令。
    fn send_command(&mut self, cmd: &Command) -> Result<()>;

    /// 读取最新一帧飞控遥测（若可用）。
    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>>;

    /// 关闭连接并释放资源。
    fn shutdown(&mut self) {}
}

/// 根据配置字符串创建传输实例。用于 demo 与接线。
pub fn open_transport(kind: &str) -> Result<Box<dyn FcuTransport>> {
    match kind {
        "mock" => Ok(Box::new(MockTransport::new())),
        "udp" => Ok(Box::new(UdpTransport::connect(
            "127.0.0.1:14550",
            "127.0.0.1:14555",
        )?)),
        "serial" => {
            let cfg = SerialConfig {
                port: "/dev/ttyS0".into(),
                baud_rate: 921_600,
            };
            Ok(Box::new(SerialTransport::open(cfg)?))
        }
        "can" => Ok(Box::new(CanTransport::open("can0")?)),
        other => Err(brain_core::BrainError::Transport(format!(
            "unknown transport kind: {other}"
        ))),
    }
}
