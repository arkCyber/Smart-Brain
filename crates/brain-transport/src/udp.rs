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
    socket: UdpSocket,
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
            socket,
            peer: peer_addr.to_string(),
            reader: FrameReader::new(),
            pending: VecDeque::new(),
        })
    }

    /// 读取并解码最多 `max_rounds` 个 UDP 报文，填充待处理遥测队列。
    fn drain(&mut self, max_rounds: usize) {
        let mut buf = [0u8; 4096];
        for _ in 0..max_rounds {
            match self.socket.recv_from(&mut buf) {
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
        let payload = serde_json::to_vec(cmd).map_err(|e| BrainError::Transport(e.to_string()))?;
        let frame = encode_frame(&payload);
        self.socket
            .send_to(&frame, &self.peer)
            .map_err(|e| BrainError::Transport(format!("udp send: {e}")))?;
        Ok(())
    }

    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
        self.drain(4);
        Ok(self.pending.pop_front())
    }

    fn shutdown(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    /// 用两个本地 UDP 套接字做端到端帧收发测试。
    #[test]
    fn udp_frame_roundtrip() {
        let recv = UdpTransport::connect("127.0.0.1:0", "127.0.0.1:0").unwrap();
        let recv_addr = recv.socket.local_addr().unwrap();
        let send = UdpTransport::connect("127.0.0.1:0", &recv_addr.to_string()).unwrap();
        let mut recv = recv;

        // 发送一帧遥测（用帧编解码验证接收端可靠分帧）。
        let telem = Telemetry::default_at(42);
        let payload = serde_json::to_vec(&telem).unwrap();
        send.socket
            .send_to(&encode_frame(&payload), recv_addr)
            .unwrap();

        std::thread::sleep(Duration::from_millis(20));
        let got = recv.try_recv_telemetry().unwrap();
        assert_eq!(got.unwrap().timestamp, 42);
    }
}
