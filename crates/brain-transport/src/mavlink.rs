//! MAVLink 风格消息二进制编解码。
//!
//! 在 `brain_message::frame`（长度 + CRC-16）之上，把 `Command`/`Telemetry`
//! 封装成 MAVLink 式消息：`[msgid, 二进制负载]`，再经帧编解码走串口/UDP。
//! 负载为小端定长布局，体积小、无序列化开销、便于在单片机(zenoh-pico/MAVLink)
//! 上解析。

use std::collections::VecDeque;

use brain_core::error::BrainError;
use brain_core::Result;
use brain_message::telemetry::FixType;
use brain_message::{encode_frame, Command, CommandTarget, FrameReader, Mode, Telemetry};

/// 消息 ID。
pub const MSG_TELEMETRY: u8 = 1;
pub const MSG_COMMAND: u8 = 2;

/// 一条 MAVLink 风格消息。
#[derive(Debug, Clone, PartialEq)]
pub enum MavMessage {
    Telemetry(Telemetry),
    Command(Command),
}

// ---- 小端打包/拆包辅助 ----

fn put_u8(b: &mut Vec<u8>, v: u8) {
    b.push(v);
}
fn put_u64(b: &mut Vec<u8>, v: u64) {
    b.extend_from_slice(&v.to_le_bytes());
}
fn put_f32(b: &mut Vec<u8>, v: f32) {
    b.extend_from_slice(&v.to_le_bytes());
}
fn put_f64(b: &mut Vec<u8>, v: f64) {
    b.extend_from_slice(&v.to_le_bytes());
}

fn get_u8(data: &[u8], off: usize) -> Result<u8> {
    data.get(off)
        .copied()
        .ok_or_else(|| BrainError::Transport("mav: short u8".into()))
}
fn get_u32(data: &[u8], off: usize) -> Result<u32> {
    let s = data
        .get(off..off + 4)
        .ok_or_else(|| BrainError::Transport("mav: short u32".into()))?;
    Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}
fn get_u64(data: &[u8], off: usize) -> Result<u64> {
    let s = data
        .get(off..off + 8)
        .ok_or_else(|| BrainError::Transport("mav: short u64".into()))?;
    let mut a = [0u8; 8];
    a.copy_from_slice(s);
    Ok(u64::from_le_bytes(a))
}
fn get_f32(data: &[u8], off: usize) -> Result<f32> {
    Ok(f32::from_bits(get_u32(data, off)?))
}
fn get_f64(data: &[u8], off: usize) -> Result<f64> {
    Ok(f64::from_bits(get_u64(data, off)?))
}

// ---- Mode / FixType 与字节的映射 ----

fn mode_to_u8(m: Mode) -> u8 {
    match m {
        Mode::Idle => 0,
        Mode::Takeoff => 1,
        Mode::Cruise => 2,
        Mode::Track => 3,
        Mode::ReturnHome => 4,
        Mode::Land => 5,
        Mode::Loiter => 6,
    }
}
fn mode_from_u8(v: u8) -> Result<Mode> {
    Ok(match v {
        0 => Mode::Idle,
        1 => Mode::Takeoff,
        2 => Mode::Cruise,
        3 => Mode::Track,
        4 => Mode::ReturnHome,
        5 => Mode::Land,
        6 => Mode::Loiter,
        _ => return Err(BrainError::Transport(format!("mav: bad mode {v}"))),
    })
}
fn fix_to_u8(f: FixType) -> u8 {
    match f {
        FixType::NoFix => 0,
        FixType::Fix2D => 1,
        FixType::Fix3D => 2,
    }
}
fn fix_from_u8(v: u8) -> FixType {
    match v {
        1 => FixType::Fix2D,
        2 => FixType::Fix3D,
        _ => FixType::NoFix,
    }
}

// ---- Telemetry（msgid=1，66 字节）----

const TELEMETRY_LEN: usize = 66;

fn encode_telemetry(t: &Telemetry) -> Vec<u8> {
    let mut b = Vec::with_capacity(TELEMETRY_LEN);
    put_u64(&mut b, t.timestamp);
    put_f32(&mut b, t.attitude.roll);
    put_f32(&mut b, t.attitude.pitch);
    put_f32(&mut b, t.attitude.yaw);
    put_f64(&mut b, t.gps.lat);
    put_f64(&mut b, t.gps.lon);
    put_f32(&mut b, t.gps.alt);
    put_u8(&mut b, fix_to_u8(t.gps.fix_type));
    put_u8(&mut b, t.gps.satellites);
    put_f32(&mut b, t.battery.remaining_pct);
    put_f32(&mut b, t.battery.voltage);
    put_f32(&mut b, t.battery.current);
    put_f32(&mut b, t.velocity.x);
    put_f32(&mut b, t.velocity.y);
    put_f32(&mut b, t.velocity.z);
    b
}

fn decode_telemetry(data: &[u8]) -> Result<Telemetry> {
    if data.len() < TELEMETRY_LEN {
        return Err(BrainError::Transport(format!(
            "mav: telemetry too short {}",
            data.len()
        )));
    }
    let mut o = 0usize;
    let timestamp = get_u64(data, o)?;
    o += 8;
    let (roll, pitch, yaw) = (
        get_f32(data, o)?,
        get_f32(data, o + 4)?,
        get_f32(data, o + 8)?,
    );
    o += 12;
    let (lat, lon) = (get_f64(data, o)?, get_f64(data, o + 8)?);
    o += 16;
    let alt = get_f32(data, o)?;
    o += 4;
    let fix_type = fix_from_u8(get_u8(data, o)?);
    let satellites = get_u8(data, o + 1)?;
    o += 2;
    let battery_pct = get_f32(data, o)?;
    let voltage = get_f32(data, o + 4)?;
    let current = get_f32(data, o + 8)?;
    o += 12;
    let vx = get_f32(data, o)?;
    let vy = get_f32(data, o + 4)?;
    let vz = get_f32(data, o + 8)?;

    Ok(Telemetry {
        timestamp,
        attitude: brain_message::Attitude { roll, pitch, yaw },
        gps: brain_message::GpsFix {
            lat,
            lon,
            alt,
            fix_type,
            satellites,
        },
        battery: brain_message::BatteryStatus {
            remaining_pct: battery_pct,
            voltage,
            current,
        },
        velocity: brain_core::Vec3::new(vx, vy, vz),
    })
}

// ---- Command（msgid=2，22 字节定长）----

const COMMAND_LEN: usize = 22;

fn encode_command(c: &Command) -> Vec<u8> {
    let mut b = Vec::with_capacity(COMMAND_LEN);
    put_u64(&mut b, c.timestamp);
    put_u8(&mut b, mode_to_u8(c.mode));
    let (kind, x, y, z) = match &c.target {
        CommandTarget::None => (0u8, 0.0f32, 0.0f32, 0.0f32),
        CommandTarget::Position { north, east, down } => (1, *north, *east, *down),
        CommandTarget::Velocity(v) => (2, v.x, v.y, v.z),
    };
    put_u8(&mut b, kind);
    put_f32(&mut b, x);
    put_f32(&mut b, y);
    put_f32(&mut b, z);
    b
}

fn decode_command(data: &[u8]) -> Result<Command> {
    if data.len() < COMMAND_LEN {
        return Err(BrainError::Transport(format!(
            "mav: command too short {}",
            data.len()
        )));
    }
    let timestamp = get_u64(data, 0)?;
    let mode = mode_from_u8(get_u8(data, 8)?)?;
    let kind = get_u8(data, 9)?;
    let x = get_f32(data, 10)?;
    let y = get_f32(data, 14)?;
    let z = get_f32(data, 18)?;
    let target = match kind {
        0 => CommandTarget::None,
        1 => CommandTarget::Position {
            north: x,
            east: y,
            down: z,
        },
        2 => CommandTarget::Velocity(brain_core::Vec3::new(x, y, z)),
        _ => {
            return Err(BrainError::Transport(format!(
                "mav: bad target kind {kind}"
            )))
        }
    };
    Ok(Command {
        timestamp,
        mode,
        target,
    })
}

// ---- 公共 API ----

impl MavMessage {
    /// 编码为 `[msgid, payload]`。
    pub fn encode(&self) -> Vec<u8> {
        let mut b = Vec::new();
        match self {
            MavMessage::Telemetry(t) => {
                b.push(MSG_TELEMETRY);
                b.extend_from_slice(&encode_telemetry(t));
            }
            MavMessage::Command(c) => {
                b.push(MSG_COMMAND);
                b.extend_from_slice(&encode_command(c));
            }
        }
        b
    }

    /// 从 `[msgid, payload]` 解码。
    pub fn decode(data: &[u8]) -> Result<Self> {
        let (msgid, rest) = data
            .split_first()
            .ok_or_else(|| BrainError::Transport("mav: empty message".into()))?;
        match *msgid {
            MSG_TELEMETRY => Ok(MavMessage::Telemetry(decode_telemetry(rest)?)),
            MSG_COMMAND => Ok(MavMessage::Command(decode_command(rest)?)),
            other => Err(BrainError::Transport(format!("mav: unknown msgid {other}"))),
        }
    }

    /// 编码并加上帧头/CRC（走串口/UDP）。
    pub fn to_frame(&self) -> Vec<u8> {
        encode_frame(&self.encode())
    }
}

/// 从字节流中解析出若干 MAVLink 消息（内部用 `FrameReader` 处理半包/粘包）。
pub fn decode_stream(reader: &mut FrameReader, bytes: &[u8]) -> Result<Vec<MavMessage>> {
    let mut out = Vec::new();
    for frame in reader.push(bytes) {
        out.push(MavMessage::decode(&frame)?);
    }
    Ok(out)
}

/// 基于 MAVLink 帧的飞控链路：发送 `Command`、接收 `Telemetry`。
///
/// 通过 `tx_bytes()`/`ingest()` 与底层字节流（串口/UDP）对接。
pub struct MavLinkTransport {
    tx: Vec<u8>,
    reader: FrameReader,
    pending: VecDeque<Telemetry>,
}

impl MavLinkTransport {
    pub fn new() -> Self {
        Self {
            tx: Vec::new(),
            reader: FrameReader::new(),
            pending: VecDeque::new(),
        }
    }

    /// 待发送的帧字节（交给底层写）。
    pub fn tx_bytes(&self) -> &[u8] {
        &self.tx
    }

    /// 注入收到的字节并解析。
    pub fn ingest(&mut self, bytes: &[u8]) -> Result<()> {
        for msg in decode_stream(&mut self.reader, bytes)? {
            if let MavMessage::Telemetry(t) = msg {
                self.pending.push_back(t);
            }
        }
        Ok(())
    }
}

impl Default for MavLinkTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl crate::FcuTransport for MavLinkTransport {
    fn send_command(&mut self, cmd: &Command) -> Result<()> {
        self.tx = MavMessage::Command(cmd.clone()).to_frame();
        Ok(())
    }

    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
        Ok(self.pending.pop_front())
    }

    fn shutdown(&mut self) {
        // 清空待发送帧与已解码遥测队列，重置分帧器（纯内存桥接，无 OS 资源）。
        self.tx.clear();
        self.pending.clear();
        self.reader = FrameReader::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::FcuTransport;

    fn sample_telemetry() -> Telemetry {
        let mut t = Telemetry::default_at(1234);
        t.attitude.roll = 0.1;
        t.attitude.pitch = -0.2;
        t.attitude.yaw = 1.5;
        t.gps.lat = 39.9;
        t.gps.lon = 116.4;
        t.gps.alt = 42.5;
        t.gps.fix_type = FixType::Fix3D;
        t.gps.satellites = 12;
        t.battery.remaining_pct = 77.0;
        t.velocity = brain_core::Vec3::new(1.0, -2.0, 0.5);
        t
    }

    #[test]
    fn telemetry_roundtrip() {
        let t = sample_telemetry();
        let msg = MavMessage::Telemetry(t.clone());
        let decoded = MavMessage::decode(&msg.encode()).unwrap();
        assert_eq!(decoded, msg);
    }

    #[test]
    fn command_roundtrip_all_targets() {
        let commands = vec![
            Command {
                timestamp: 7,
                mode: Mode::Takeoff,
                target: CommandTarget::Position {
                    north: 1.0,
                    east: 2.0,
                    down: -30.0,
                },
            },
            Command {
                timestamp: 8,
                mode: Mode::Cruise,
                target: CommandTarget::Velocity(brain_core::Vec3::new(1.0, 0.0, 0.0)),
            },
            Command {
                timestamp: 9,
                mode: Mode::Land,
                target: CommandTarget::None,
            },
        ];
        for c in commands {
            let msg = MavMessage::Command(c.clone());
            let decoded = MavMessage::decode(&msg.encode()).unwrap();
            assert_eq!(decoded, msg);
        }
    }

    #[test]
    fn full_frame_roundtrip() {
        let msg = MavMessage::Telemetry(sample_telemetry());
        let frame = msg.to_frame();
        let mut reader = FrameReader::new();
        let msgs = decode_stream(&mut reader, &frame).unwrap();
        assert_eq!(msgs.len(), 1);
        assert_eq!(msgs[0], msg);
    }

    #[test]
    fn corrupt_frame_rejected() {
        let msg = MavMessage::Telemetry(sample_telemetry());
        let mut frame = msg.to_frame();
        let last = frame.len() - 1;
        frame[last] ^= 0xFF; // 篡改 CRC
        let mut reader = FrameReader::new();
        // 坏帧被 FrameReader 丢弃，不产生消息。
        let msgs = reader.push(&frame);
        assert!(msgs.is_empty());
    }

    #[test]
    fn mavlink_transport_roundtrip() {
        let mut tx = MavLinkTransport::new();
        let mut rx = MavLinkTransport::new();

        // 发送方把命令编码成帧。
        let cmd = Command {
            timestamp: 1,
            mode: Mode::Takeoff,
            target: CommandTarget::Position {
                north: 0.0,
                east: 0.0,
                down: -30.0,
            },
        };
        tx.send_command(&cmd).unwrap();
        let bytes = tx.tx_bytes().to_vec();
        assert!(bytes.len() > 10);

        // 接收方解析命令帧（这里验证字节流可被解码）。
        rx.ingest(&bytes).unwrap();
        let _ = rx.try_recv_telemetry().unwrap();
    }

    #[test]
    fn shutdown_clears_buffers() {
        let mut tx = MavLinkTransport::new();
        tx.send_command(&Command {
            timestamp: 1,
            mode: Mode::Cruise,
            target: CommandTarget::None,
        })
        .unwrap();
        assert!(!tx.tx_bytes().is_empty());
        tx.shutdown();
        // shutdown 清空待发送帧与解码缓冲。
        assert!(tx.tx_bytes().is_empty());
        assert!(tx.try_recv_telemetry().unwrap().is_none());
    }
}
