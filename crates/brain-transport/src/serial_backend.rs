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
    port: Box<dyn serialport::SerialPort>,
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
                port,
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
            let frame = encode_frame(&payload);
            self.port
                .write_all(&frame)
                .map_err(|e| BrainError::Transport(format!("serial write: {e}")))?;
        }
        let _ = payload;
        Ok(())
    }

    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
        #[cfg(feature = "serial")]
        {
            use std::io::Read;
            let mut buf = [0u8; 512];
            // 先把已解码的遥测弹出一帧。
            if let Some(t) = self.pending.pop_front() {
                return Ok(Some(t));
            }
            // 读取原始字节并累积到 FrameReader，正确处理半包/粘包。
            match self.port.read(&mut buf) {
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

    fn shutdown(&mut self) {}
}
