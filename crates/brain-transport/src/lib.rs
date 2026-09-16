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

    /// 关闭连接并释放资源（默认空实现：多数后端在 `Drop` 时释放 OS 句柄）。
    /// 需要立即释放/停止的后端（串口/CAN/UDP）应覆盖此方法。
    fn shutdown(&mut self) {}
}

/// 根据配置字符串创建传输实例。用于 demo 与接线。
pub fn open_transport(kind: &str) -> Result<Box<dyn FcuTransport>> {
    match kind {
        "mock" => Ok(Box::new(MockTransport::new())),
        "mavlink" => Ok(Box::new(MavLinkTransport::new())),
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

#[cfg(test)]
mod tests {
    use super::*;
    use brain_message::{Command, CommandTarget, Mode};

    #[test]
    fn open_transport_mock_roundtrip() {
        let mut t = open_transport("mock").unwrap();
        t.send_command(&Command {
            timestamp: 1,
            mode: Mode::Takeoff,
            target: CommandTarget::None,
        })
        .unwrap();
        // mock 传输可立即返回一帧遥测。
        assert!(t.try_recv_telemetry().unwrap().is_some());
        t.shutdown(); // 默认实现，不 panic
    }

    #[test]
    fn open_transport_udp_connects() {
        let mut t = open_transport("udp").unwrap();
        // UDP 双向通道就绪：发指令不报错，无遥测时返回 None。
        t.send_command(&Command {
            timestamp: 1,
            mode: Mode::Loiter,
            target: CommandTarget::None,
        })
        .unwrap();
        // 尚未收到对端遥测 -> None（不 panic）。
        let _ = t.try_recv_telemetry();
        t.shutdown();
    }

    #[test]
    fn open_transport_mavlink_roundtrip() {
        let mut t = open_transport("mavlink").unwrap();
        t.send_command(&Command {
            timestamp: 1,
            mode: Mode::Loiter,
            target: CommandTarget::None,
        })
        .unwrap();
        // MAVLink 内存桥：发送后无对端遥测 -> None（不 panic）。
        assert!(t.try_recv_telemetry().unwrap().is_none());
        t.shutdown();
    }

    #[test]
    fn open_transport_serial_requires_feature() {
        // 未启用 `serial` feature 时应优雅报错，而非 panic。
        let r = open_transport("serial");
        #[cfg(feature = "serial")]
        assert!(
            r.is_err(),
            "serial open needs a real port; here it should fail"
        );
        #[cfg(not(feature = "serial"))]
        assert!(r.is_err(), "serial feature not enabled should error");
    }

    #[test]
    fn open_transport_unknown_kind_errors() {
        // Box<dyn FcuTransport> 无 Debug，用 match 而非 unwrap_err。
        match open_transport("bluetooth") {
            Err(e) => assert!(e.to_string().contains("unknown transport kind")),
            Ok(_) => panic!("unknown kind should error"),
        }
    }

    #[test]
    fn fcu_transport_shutdown_default_is_noop() {
        // 自定义空实现验证 trait 的默认 shutdown()。
        struct Noop;
        impl FcuTransport for Noop {
            fn send_command(&mut self, _: &Command) -> Result<()> {
                Ok(())
            }
            fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
                Ok(None)
            }
        }
        let mut t = Noop;
        t.send_command(&Command {
            timestamp: 0,
            mode: Mode::Idle,
            target: CommandTarget::None,
        })
        .unwrap();
        assert!(t.try_recv_telemetry().unwrap().is_none());
        t.shutdown(); // 默认 no-op
    }
}
