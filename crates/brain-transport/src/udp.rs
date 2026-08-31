//! UDP 传输后端：用于 SITL 仿真或与地面站/仿真器（Gazebo、AirSim）通信。
//!
//! 使用 `brain_message` 的帧编解码（长度 + CRC-16）进行可靠分帧，正确处理
//! 半包/粘包，避免"一条 UDP 报文 = 一条消息"的脆弱假设。

use std::collections::VecDeque;
use std::net::UdpSocket;

use brain_core::error::BrainError;
use brain_core::Result;
use brain_message::{encode_frame, Command, FrameReader, Telemetry};

use crate::FcuTransport;

/// UDP 传输后端。默认用于本地 SITL 仿真。
pub struct UdpTransport {
    socket: Option<UdpSocket>,
    peer: String,
    reader: FrameReader,
    pending: VecDeque<Telemetry>,
}

impl UdpTransport {
    /// 绑定本地地址并连接到对端。
    pub fn connect(bind_addr: &str, peer_addr: &str) -> Result<Self> {
        let socket = UdpSocket::bind(bind_addr)
            .map_err(|e| BrainError::Transport(format!("bind {bind_addr}: {e}")))?;
        socket
            .set_read_timeout(Some(std::time::Duration::from_millis(5)))
            .ok();
        Ok(Self {
            socket: Some(socket),
            peer: peer_addr.to_string(),
            reader: FrameReader::new(),
            pending: VecDeque::new(),
        })
    }

    /// 读取并解码最多 `max_rounds` 个 UDP 报文，填充待处理遥测队列。
    fn drain(&mut self, max_rounds: usize) {
        let Some(socket) = self.socket.as_ref() else {
            return; // 已 shutdown
        };
        let mut buf = [0u8; 4096];
        for _ in 0..max_rounds {
            match socket.recv_from(&mut buf) {
                Ok((n, _)) => {
                    for frame in self.reader.push(&buf[..n]) {
                        if let Ok(t) = serde_json::from_slice::<Telemetry>(&frame) {
                            self.pending.push_back(t);
                        }
                    }
                }
                Err(_) => break, // 超时/无数据
            }
        }
    }
}

impl FcuTransport for UdpTransport {
    fn send_command(&mut self, cmd: &Command) -> Result<()> {
        let socket = self
            .socket
            .as_ref()
            .ok_or_else(|| BrainError::Transport("udp transport closed".into()))?;
        let payload = serde_json::to_vec(cmd).map_err(|e| BrainError::Transport(e.to_string()))?;
        let frame = encode_frame(&payload);
        socket
            .send_to(&frame, &self.peer)
            .map_err(|e| BrainError::Transport(format!("udp send: {e}")))?;
        Ok(())
    }

    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
        self.drain(4);
        Ok(self.pending.pop_front())
    }

    fn shutdown(&mut self) {
        // 真实释放底层 UDP 套接字（关闭端口），并清空待处理遥测。
        self.socket = None;
        self.pending.clear();
        self.reader = FrameReader::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    /// 用两个本地 UDP 套接字做端到端帧收发测试。
    #[test]
    fn udp_frame_roundtrip() {
        let recv = UdpTransport::connect("127.0.0.1:0", "127.0.0.1:0").unwrap();
        let recv_addr = recv.socket.as_ref().unwrap().local_addr().unwrap();
        let send = UdpTransport::connect("127.0.0.1:0", &recv_addr.to_string()).unwrap();
        let mut recv = recv;

        // 发送一帧遥测（用帧编解码验证接收端可靠分帧）。
        let telem = Telemetry::default_at(42);
        let payload = serde_json::to_vec(&telem).unwrap();
        send.socket
            .as_ref()
            .unwrap()
            .send_to(&encode_frame(&payload), recv_addr)
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));
        let got = recv.try_recv_telemetry().unwrap();
        assert_eq!(got.unwrap().timestamp, 42);
    }

    #[test]
    fn connect_invalid_addr_errors() {
        // 无效的绑定地址应优雅报错，而非 panic。
        assert!(UdpTransport::connect("999.999.999.999:0", "127.0.0.1:1").is_err());
    }

    #[test]
    fn empty_recv_returns_none() {
        let mut t = UdpTransport::connect("127.0.0.1:0", "127.0.0.1:0").unwrap();
        // 无数据时 try_recv_telemetry 应返回 None（不 panic）。
        assert!(t.try_recv_telemetry().unwrap().is_none());
        t.shutdown();
    }

    #[test]
    fn send_command_returns_ok() {
        let mut t = UdpTransport::connect("127.0.0.1:0", "127.0.0.1:0").unwrap();
        let cmd = Command {
            timestamp: 1,
            mode: brain_message::Mode::Cruise,
            target: brain_message::CommandTarget::None,
        };
        t.send_command(&cmd).unwrap();
        t.shutdown();
    }

    #[test]
    fn shutdown_releases_socket_and_blocks_send() {
        let mut t = UdpTransport::connect("127.0.0.1:0", "127.0.0.1:0").unwrap();
        t.shutdown();
        // shutdown 释放套接字后，发送指令应报"closed"错误。
        let cmd = Command {
            timestamp: 1,
            mode: brain_message::Mode::Idle,
            target: brain_message::CommandTarget::None,
        };
        assert!(t.send_command(&cmd).is_err());
        assert!(t.try_recv_telemetry().unwrap().is_none());
    }
}
