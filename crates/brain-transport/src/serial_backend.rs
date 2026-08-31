//! 真实串口传输后端（可选，`serial` feature）。
//!
//! 在 Jetson / RK3588 上通过 UART 连接 STM32/Pixhawk（小脑）。
//! 使用 JSON 行协议封装 `Command` 与 `Telemetry`，便于调试与解析。

use brain_core::error::BrainError;
use brain_core::Result;
#[cfg(feature = "serial")]
use brain_message::{encode_frame, FrameReader};
use brain_message::{Command, Telemetry};

use crate::FcuTransport;

/// 串口连接配置。
#[derive(Debug, Clone)]
pub struct SerialConfig {
    pub port: String,
    pub baud_rate: u32,
}

/// 串口传输后端。
///
/// 未启用 `serial` feature 时构造会返回错误，避免在桌面环境引入依赖。
pub struct SerialTransport {
    #[cfg(feature = "serial")]
    port: Option<Box<dyn serialport::SerialPort>>,
    #[cfg(feature = "serial")]
    reader: FrameReader,
    #[cfg(feature = "serial")]
    pending: std::collections::VecDeque<Telemetry>,
    #[cfg(not(feature = "serial"))]
    #[allow(dead_code)]
    config: SerialConfig,
}

impl SerialTransport {
    /// 打开串口。
    pub fn open(config: SerialConfig) -> Result<Self> {
        #[cfg(feature = "serial")]
        {
            let port = serialport::new(&config.port, config.baud_rate)
                .timeout(std::time::Duration::from_millis(10))
                .open()
                .map_err(|e| BrainError::Transport(format!("open {}: {e}", config.port)))?;
            Ok(Self {
                port: Some(port),
                reader: FrameReader::new(),
                pending: std::collections::VecDeque::new(),
            })
        }
        #[cfg(not(feature = "serial"))]
        {
            let _ = config.baud_rate;
            Err(BrainError::Transport(
                "serial feature not enabled; build with --features serial".into(),
            ))
        }
    }
}

impl FcuTransport for SerialTransport {
    fn send_command(&mut self, cmd: &Command) -> Result<()> {
        let payload = serde_json::to_vec(cmd).map_err(|e| BrainError::Transport(e.to_string()))?;
        #[cfg(feature = "serial")]
        {
            use std::io::Write;
            let port = self
                .port
                .as_mut()
                .ok_or_else(|| BrainError::Transport("serial port closed".into()))?;
            let frame = encode_frame(&payload);
            port.write_all(&frame)
                .map_err(|e| BrainError::Transport(format!("serial write: {e}")))?;
            // 立即刷新发送缓冲，确保字节按时出队到 UART。
            port.flush()
                .map_err(|e| BrainError::Transport(format!("serial flush: {e}")))?;
        }
        let _ = payload;
        Ok(())
    }

    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
        #[cfg(feature = "serial")]
        {
            use std::io::Read;
            let Some(port) = self.port.as_mut() else {
                return Ok(None); // 已 shutdown
            };
            let mut buf = [0u8; 512];
            // 先把已解码的遥测弹出一帧。
            if let Some(t) = self.pending.pop_front() {
                return Ok(Some(t));
            }
            // 读取原始字节并累积到 FrameReader，正确处理半包/粘包。
            match port.read(&mut buf) {
                Ok(0) => Ok(None),
                Ok(n) => {
                    for frame in self.reader.push(&buf[..n]) {
                        if let Ok(t) = serde_json::from_slice::<Telemetry>(&frame) {
                            self.pending.push_back(t);
                        }
                    }
                    Ok(self.pending.pop_front())
                }
                Err(_) => Ok(None),
            }
        }
        #[cfg(not(feature = "serial"))]
        {
            Ok(None)
        }
    }

    fn shutdown(&mut self) {
        // 真实释放串口句柄（关闭 UART 端口），并清空解码缓冲。
        #[cfg(feature = "serial")]
        {
            if let Some(mut port) = self.port.take() {
                let _ = port.flush();
                drop(port); // 关闭串口
            }
            self.pending.clear();
            self.reader = FrameReader::new();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(not(feature = "serial"))]
    use brain_message::{Command, CommandTarget, Mode};

    #[cfg(not(feature = "serial"))]
    fn cmd() -> Command {
        Command {
            timestamp: 1,
            mode: Mode::Cruise,
            target: CommandTarget::None,
        }
    }

    #[test]
    fn open_without_serial_feature_errors() {
        // 未启用 `serial` feature 时应优雅报错（真机需 `--features serial`）。
        let r = SerialTransport::open(SerialConfig {
            port: "/dev/ttyS0".into(),
            baud_rate: 921_600,
        });
        #[cfg(feature = "serial")]
        assert!(r.is_err(), "needs a real port here; should fail");
        #[cfg(not(feature = "serial"))]
        assert!(r.is_err(), "serial feature not enabled should error");
    }

    #[cfg(not(feature = "serial"))]
    #[test]
    fn non_serial_paths_are_noop_safe() {
        // 仅当未启用 `serial` feature 时编译：走 #[cfg(not)] 分支，send/recv/shutdown 必须安全。
        let mut t = SerialTransport {
            config: SerialConfig {
                port: "x".into(),
                baud_rate: 9600,
            },
        };
        t.send_command(&cmd()).unwrap();
        assert!(t.try_recv_telemetry().unwrap().is_none());
        t.shutdown();
    }
}
