//! CAN 总线传输后端（真机接线，可选 `can` feature）。
//!
//! 包含两部分：
//! 1. **纯函数 CAN 编解码**：把 `Command`/`Telemetry` 序列化并分片到若干 8 字节
//!    经典 CAN 帧（`CanFrame`），以及反向重组。这部分不依赖任何外部库，可离线测试。
//! 2. **`CanTransport`**：实现 `FcuTransport`，通过 `socketcan`（Linux SocketCAN，
//!    常见于 Jetson/RK3588 与车规网关）读写真实 CAN 总线。
//!
//! 协议约定（经典 CAN，DLC<=8）：
//! - 仲裁 ID 高位标识消息方向：`0x1xx`=Command（大脑→飞控），`0x2xx`=Telemetry（飞控→大脑）。
//! - 每帧 `data[0]` 为分片序号 `seq`；`seq==0` 的首帧在 `data[1..3]` 携带整条负载长度（u16 LE）。
//! - 非首帧 `data[1..8]` 为 7 字节负载；末帧由“已收字节数达到总长”判定。

use brain_core::{BrainError, Result};
use brain_message::{Command, Telemetry};

/// Command 消息的仲裁 ID 基址（低字节为分片序号）。
pub const CAN_CMD_BASE_ID: u32 = 0x100;
/// Telemetry 消息的仲裁 ID 基址。
pub const CAN_TELEM_BASE_ID: u32 = 0x200;

/// 一帧经典 CAN 帧（最多 8 字节数据）。
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CanFrame {
    /// 仲裁 ID。
    pub id: u32,
    /// 数据负载（最多 8 字节）。
    pub data: [u8; 8],
    /// 有效数据长度（0..=8）。
    pub len: u8,
}

impl CanFrame {
    /// 构造一帧，`data` 长度不得超过 8。
    pub fn new(id: u32, data: &[u8]) -> Result<Self> {
        if data.len() > 8 {
            return Err(brain_core::BrainError::Transport(format!(
                "CAN frame too long: {} bytes (max 8)",
                data.len()
            )));
        }
        let mut buf = [0u8; 8];
        buf[..data.len()].copy_from_slice(data);
        Ok(Self {
            id,
            data: buf,
            len: data.len() as u8,
        })
    }

    /// 负载切片（按 `len`）。
    pub fn payload(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }
}

/// 把一段负载编码成若干 CAN 帧（以 `base_id` 为基址，低字节为分片序号）。
pub fn encode_frames(base_id: u32, payload: &[u8]) -> Result<Vec<CanFrame>> {
    let mut frames = Vec::new();
    let total = payload.len();
    if total == 0 {
        return Err(brain_core::BrainError::Transport(
            "empty CAN payload".into(),
        ));
    }
    let mut seq = 0u8;
    let mut offset = 0usize;
    while offset < total {
        let mut frame_data = [0u8; 8];
        frame_data[0] = seq;
        if seq == 0 {
            // 首帧：data[1..3] 携带总长（u16 LE），data[3..8] 为 5 字节负载。
            frame_data[1] = (total & 0xff) as u8;
            frame_data[2] = ((total >> 8) & 0xff) as u8;
            let take = (total - offset).min(5);
            frame_data[3..3 + take].copy_from_slice(&payload[offset..offset + take]);
            frames.push(CanFrame::new(
                base_id | (seq as u32),
                &frame_data[..3 + take],
            )?);
            offset += take;
        } else {
            // 后续帧：data[1..8] 为 7 字节负载。
            let take = (total - offset).min(7);
            frame_data[1..1 + take].copy_from_slice(&payload[offset..offset + take]);
            frames.push(CanFrame::new(
                base_id | (seq as u32),
                &frame_data[..1 + take],
            )?);
            offset += take;
        }
        seq += 1;
    }
    Ok(frames)
}

/// 把若干同基址的 CAN 帧重组成原始负载。
pub fn decode_frames(base_id: u32, frames: &[CanFrame]) -> Option<Vec<u8>> {
    if frames.is_empty() {
        return None;
    }
    // 取出首帧（seq==0）。
    let first = frames.iter().find(|f| f.id == base_id)?;
    if first.len < 3 {
        return None; // 首帧至少要有总长字段
    }
    let total = u16::from_le_bytes([first.data[1], first.data[2]]) as usize;
    let mut out = Vec::with_capacity(total);
    // 首帧负载在 data[3..len]
    out.extend_from_slice(&first.payload()[3..]);
    // 后续帧按 seq 升序补齐。
    let mut frames_sorted: Vec<&CanFrame> = frames
        .iter()
        .filter(|f| f.id > base_id && (f.id - base_id) <= 0xff && (f.id & 0xff) != 0)
        .collect();
    frames_sorted.sort_by_key(|f| (f.id & 0xff) as u8);
    // 校验分片序号必须连续（seq 1,2,3,...）：检测缺段/重复段，避免重排出损坏数据。
    let mut expect = 1u8;
    for f in &frames_sorted {
        if (f.id & 0xff) as u8 != expect {
            return None; // 缺段或重复段
        }
        expect = expect.wrapping_add(1);
    }
    for f in frames_sorted {
        out.extend_from_slice(&f.payload()[1..]);
        if out.len() >= total {
            break;
        }
    }
    if out.len() < total {
        return None;
    }
    out.truncate(total);
    Some(out)
}

/// 编码一条 Command 为 CAN 帧序列。
pub fn encode_command(cmd: &Command) -> Result<Vec<CanFrame>> {
    let payload =
        serde_json::to_vec(cmd).map_err(|e| brain_core::BrainError::Transport(e.to_string()))?;
    encode_frames(CAN_CMD_BASE_ID, &payload)
}

/// 从 CAN 帧序列重组并解析一条 Telemetry。
pub fn decode_telemetry(frames: &[CanFrame]) -> Result<Option<Telemetry>> {
    let Some(bytes) = decode_frames(CAN_TELEM_BASE_ID, frames) else {
        return Ok(None);
    };
    let t: Telemetry = serde_json::from_slice(&bytes)
        .map_err(|e| brain_core::BrainError::Transport(e.to_string()))?;
    Ok(Some(t))
}

/// 编码一条 Telemetry 为 CAN 帧序列（便于回环测试与对端上报）。
pub fn encode_telemetry(t: &Telemetry) -> Result<Vec<CanFrame>> {
    let payload =
        serde_json::to_vec(t).map_err(|e| brain_core::BrainError::Transport(e.to_string()))?;
    encode_frames(CAN_TELEM_BASE_ID, &payload)
}

/// 单条遥测可占用的最大 CAN 分片数（防御重组缓冲无界增长）。
///
/// 一帧承载 7 字节负载，64 帧 ≈ 448 字节，远超一条遥测 JSON（通常 < 300 字节）；
/// 该值只用于防御恶意/损坏数据导致的分片无限堆积，正常消息远不会触及。
pub const MAX_CAN_FRAMES: usize = 64;

/// 增量重组：把一帧 CAN 遥测分片累积到 `pending`，若由此得到一条完整遥测则返回 `Some`。
///
/// 生产防护：
/// - 收到**新消息首帧**（`seq==0`）时若仍有未完成的旧分片，先丢弃旧分片重新对齐，
///   避免把两条不同消息的分片混在一起导致死等/错乱；
/// - 重组缓冲超过 [`MAX_CAN_FRAMES`] 上限时清空重来，避免垃圾数据无限堆积。
pub fn ingest_telemetry_frame(
    pending: &mut Vec<CanFrame>,
    frame: CanFrame,
) -> Result<Option<Telemetry>> {
    // 新消息开始：丢弃上一段未完成的分片。
    if (frame.id & 0xff) == 0 && !pending.is_empty() {
        pending.clear();
    }
    pending.push(frame);
    // 上限保护：太多分片仍未凑成一条消息 -> 清空重来。
    if pending.len() > MAX_CAN_FRAMES {
        pending.clear();
        return Ok(None);
    }
    if let Some(t) = decode_telemetry(pending)? {
        pending.clear();
        return Ok(Some(t));
    }
    Ok(None)
}

/// 基于 Linux SocketCAN 的 `FcuTransport`（可选 `can` feature，仅 Linux）。
pub struct CanTransport {
    #[cfg(all(feature = "can", target_os = "linux"))]
    socket: Option<socketcan::CanSocket>,
    /// 累积收到的 Telemetry 分片，用于重组。
    #[cfg(all(feature = "can", target_os = "linux"))]
    pending_telem: Vec<CanFrame>,
    #[cfg(not(feature = "can"))]
    _iface: String,
}

impl CanTransport {
    /// 打开一个 SocketCAN 接口（例如 `"can0"`）。
    pub fn open(iface: &str) -> Result<Self> {
        #[cfg(all(feature = "can", target_os = "linux"))]
        {
            let socket = socketcan::CanSocket::open(iface)
                .map_err(|e| BrainError::Transport(format!("open {iface}: {e}")))?;
            let _ = socket.set_nonblocking(true);
            Ok(Self {
                socket: Some(socket),
                pending_telem: Vec::new(),
            })
        }
        #[cfg(not(all(feature = "can", target_os = "linux")))]
        {
            Err(BrainError::Transport(format!(
                "can transport unavailable: feature 'can' + Linux required (iface={iface})"
            )))
        }
    }
}

impl crate::FcuTransport for CanTransport {
    fn send_command(&mut self, cmd: &Command) -> Result<()> {
        #[cfg(all(feature = "can", target_os = "linux"))]
        {
            let socket = self
                .socket
                .as_ref()
                .ok_or_else(|| BrainError::Transport("can transport closed".into()))?;
            for f in encode_command(cmd)? {
                let frame = socketcan::CanFrame::new(f.id, f.payload())
                    .ok_or_else(|| BrainError::Transport("invalid can id".into()))?;
                socket
                    .write_frame(&frame)
                    .map_err(|e| BrainError::Transport(format!("can write: {e}")))?;
            }
            Ok(())
        }
        #[cfg(not(all(feature = "can", target_os = "linux")))]
        {
            let _ = cmd;
            Err(BrainError::Transport(
                "can transport unavailable: feature 'can' + Linux required".into(),
            ))
        }
    }

    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
        #[cfg(all(feature = "can", target_os = "linux"))]
        {
            // 非阻塞读取所有已就绪的帧，增量重组（遇新首帧自动对齐、超上限自动清空）。
            if let Some(socket) = self.socket.as_ref() {
                loop {
                    match socket.read_frame() {
                        Ok(f) => {
                            if (f.id() & 0x300) == CAN_TELEM_BASE_ID {
                                let frame = CanFrame::new(f.id(), f.data())?;
                                if let Some(t) =
                                    ingest_telemetry_frame(&mut self.pending_telem, frame)?
                                {
                                    return Ok(Some(t));
                                }
                            }
                        }
                        Err(_) => break, // 无更多数据（含 WouldBlock）
                    }
                }
            }
            Ok(None)
        }
        #[cfg(not(all(feature = "can", target_os = "linux")))]
        {
            Ok(None)
        }
    }

    fn shutdown(&mut self) {
        // 真实释放 SocketCAN 套接字，并清空重组缓冲。
        #[cfg(all(feature = "can", target_os = "linux"))]
        {
            self.socket = None;
            self.pending_telem.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_message::{Command, CommandTarget, Mode};

    #[test]
    fn can_frame_limits_length() {
        assert!(CanFrame::new(0x100, &[0u8; 8]).is_ok());
        assert!(CanFrame::new(0x100, &[0u8; 9]).is_err());
    }

    #[test]
    fn single_frame_roundtrip() {
        let payload = b"hello";
        let frames = encode_frames(CAN_CMD_BASE_ID, payload).unwrap();
        assert_eq!(frames.len(), 1);
        assert_eq!(decode_frames(CAN_CMD_BASE_ID, &frames).unwrap(), payload);
    }

    #[test]
    fn multi_frame_roundtrip() {
        // 20 字节负载：1 首帧(5) + 3 后续帧(7,7,1)。
        let payload: Vec<u8> = (0..20u8).collect();
        let frames = encode_frames(CAN_TELEM_BASE_ID, &payload).unwrap();
        assert!(frames.len() >= 2);
        assert_eq!(decode_frames(CAN_TELEM_BASE_ID, &frames).unwrap(), payload);
    }

    #[test]
    fn command_roundtrip_over_can() {
        let cmd = Command {
            timestamp: 42,
            mode: Mode::Cruise,
            target: CommandTarget::Position {
                north: 10.0,
                east: 20.0,
                down: -30.0,
            },
        };
        let frames = encode_command(&cmd).unwrap();
        // 命令帧方向为 CAN_CMD_BASE_ID。
        assert!(frames.iter().all(|f| (f.id & 0x300) == CAN_CMD_BASE_ID));
        // 用命令方向重组后应得到同样的 JSON（验证编解码不丢数据）。
        let bytes = decode_frames(CAN_CMD_BASE_ID, &frames).unwrap();
        let back: Command = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(back, cmd);
    }

    #[test]
    fn telemetry_roundtrip_over_can() {
        let t = Telemetry::default_at(7);
        let frames = encode_telemetry(&t).unwrap();
        let got = decode_telemetry(&frames).unwrap().unwrap();
        assert_eq!(got, t);
    }

    #[test]
    fn decode_frames_requires_first_segment() {
        // 缺少首帧（seq==0）时无法重组。
        let payload = b"0123456789abcdef";
        let frames = encode_frames(CAN_TELEM_BASE_ID, payload).unwrap();
        let without_first: Vec<CanFrame> = frames
            .iter()
            .filter(|f| f.id != CAN_TELEM_BASE_ID)
            .cloned()
            .collect();
        assert_eq!(decode_frames(CAN_TELEM_BASE_ID, &without_first), None);
    }

    #[test]
    fn decode_frames_rejects_duplicate_segment() {
        // 重复段（seq==1 出现两次）会被序号连续性校验拒绝，避免重排出损坏数据。
        let payload: Vec<u8> = (0..16u8).collect(); // seq0(5)+seq1(7)+seq2(4) = 3 帧
        let mut frames = encode_frames(CAN_TELEM_BASE_ID, &payload).unwrap();
        let dup = frames.iter().find(|f| (f.id & 0xff) == 1).unwrap().clone();
        frames.push(dup);
        assert_eq!(decode_frames(CAN_TELEM_BASE_ID, &frames), None);
    }

    #[test]
    fn ingest_reassembles_full_message_incrementally() {
        let t = Telemetry::default_at(3);
        let frames = encode_telemetry(&t).unwrap();
        let mut pending = Vec::new();
        // 逐帧喂入：只有喂满后才产出。
        for f in frames.iter().take(frames.len() - 1) {
            assert_eq!(
                ingest_telemetry_frame(&mut pending, f.clone()).unwrap(),
                None
            );
        }
        let got = ingest_telemetry_frame(&mut pending, frames.last().unwrap().clone()).unwrap();
        assert_eq!(got, Some(t));
        // 重组成功后缓冲应被清空。
        assert!(pending.is_empty());
    }

    #[test]
    fn ingest_resyncs_on_new_first_frame() {
        // 第一条消息只发首帧（未完成），随后来了第二条消息的首帧 -> 应丢弃旧分片重新对齐。
        let mut pending = Vec::new();
        let a = encode_telemetry(&Telemetry::default_at(1)).unwrap();
        // 只喂第一条消息的首帧，未完成。
        let _ = ingest_telemetry_frame(&mut pending, a[0].clone()).unwrap();
        assert_eq!(pending.len(), 1);
        // 第二条消息完整喂入（首帧触发清空旧分片）。
        let b = encode_telemetry(&Telemetry::default_at(2)).unwrap();
        let mut got = None;
        for f in &b {
            if let Some(t) = ingest_telemetry_frame(&mut pending, f.clone()).unwrap() {
                got = Some(t);
            }
        }
        assert_eq!(got, Some(Telemetry::default_at(2)));
        assert!(pending.is_empty());
    }

    #[test]
    fn ingest_caps_pending_at_max_frames() {
        // 大量非首帧（seq!=0）且凑不成一条消息 -> 超过上限后清空重来。
        let mut pending = Vec::new();
        let mut fed = 0;
        for seq in 1..=MAX_CAN_FRAMES as u8 + 5 {
            let f = CanFrame::new(CAN_TELEM_BASE_ID | seq as u32, &[0u8; 8]).unwrap();
            let _ = ingest_telemetry_frame(&mut pending, f).unwrap();
            if pending.is_empty() {
                fed = seq as usize;
            }
        }
        // 上限被触发过一次：缓冲不再增长。
        assert!(fed < MAX_CAN_FRAMES + 5, "should have hit the cap");
        assert!(pending.is_empty() || pending.len() <= MAX_CAN_FRAMES);
    }
}
