//! 线缆帧编解码：长度前缀 + CRC-16，用于串口/UDP 等字节流的可靠分帧。
//!
//! 对应参考里 mavio/MAVLink 的帧封装：把任意负载（遥测/指令/点云）封装成
//! `[len_lo, len_hi, payload..., crc_lo, crc_hi]`，接收端用 `FrameReader` 从
//! 字节流中正确切出完整帧（解决"半包/粘包"问题）。

/// 帧头与尾固定开销字节数（2 长度 + 2 CRC）。
pub const FRAME_OVERHEAD: usize = 4;

/// 单帧负载长度上限（16 KiB）。遥测/指令等消息远小于此；用于抵御损坏的
/// 长度前缀（u16 最大 65535）导致的无界缓冲/失步。
pub const MAX_FRAME_PAYLOAD: usize = 0x4000;

/// CRC-16/CCITT（多项式 0x1021，初值 0xFFFF）。
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// 把负载封装成一帧字节（小端长度 + 负载 + CRC）。
pub fn encode_frame(payload: &[u8]) -> Vec<u8> {
    let len = payload.len() as u16;
    let mut frame = Vec::with_capacity(payload.len() + FRAME_OVERHEAD);
    frame.extend_from_slice(&len.to_le_bytes());
    frame.extend_from_slice(payload);
    frame.extend_from_slice(&crc16(payload).to_le_bytes());
    frame
}

/// 校验一帧：长度与 CRC 是否匹配。
pub fn verify_frame(frame: &[u8]) -> Option<&[u8]> {
    if frame.len() < FRAME_OVERHEAD {
        return None;
    }
    let len = u16::from_le_bytes([frame[0], frame[1]]) as usize;
    if frame.len() != len + FRAME_OVERHEAD {
        return None;
    }
    let payload = &frame[2..2 + len];
    let crc = u16::from_le_bytes([frame[2 + len], frame[3 + len]]);
    if crc16(payload) == crc {
        Some(payload)
    } else {
        None
    }
}

/// 字节流 → 完整帧解析器。
///
/// 内部维护累积缓冲，正确处理半包（数据不足）与粘包（一次多帧）。
pub struct FrameReader {
    buf: Vec<u8>,
}

impl Default for FrameReader {
    fn default() -> Self {
        Self::new()
    }
}

impl FrameReader {
    pub fn new() -> Self {
        Self { buf: Vec::new() }
    }

    /// 追加一段字节，返回解析出的全部完整帧负载。
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        self.buf.extend_from_slice(bytes);
        let mut out = Vec::new();
        loop {
            if self.buf.len() < FRAME_OVERHEAD {
                break;
            }
            let len = u16::from_le_bytes([self.buf[0], self.buf[1]]) as usize;
            // 防御：损坏/异常的长度前缀（超过上限）视为失步，丢弃该 2 字节长度头
            // 重新对齐，避免声称巨大长度导致缓冲无界增长/长时间等待。
            if len > MAX_FRAME_PAYLOAD {
                self.buf.drain(..2);
                continue;
            }
            let total = len + FRAME_OVERHEAD;
            if self.buf.len() < total {
                break; // 半包，等待更多数据
            }
            // 取出并校验一帧。
            let frame: Vec<u8> = self.buf.drain(..total).collect();
            if let Some(payload) = verify_frame(&frame) {
                out.push(payload.to_vec());
            }
            // 校验失败：丢弃该帧，继续找下一帧（简单滑动）。
        }
        out
    }

    /// 是否还有未消费的缓冲字节。
    pub fn pending(&self) -> usize {
        self.buf.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_is_deterministic_and_sensitive() {
        let a = crc16(b"hello");
        let b = crc16(b"hello");
        let c = crc16(b"hellp"); // 一位改动
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn crc16_ccitt_known_answer() {
        // CRC-16/CCITT-FALSE（初值 0xFFFF，多项式 0x1021）对 "123456789" 的标准值。
        assert_eq!(crc16(b"123456789"), 0x29B1);
        assert_eq!(crc16(b""), 0xFFFF);
    }

    #[test]
    fn reader_resyncs_on_absurd_length_prefix() {
        // 伪造一个声称 1MiB 以上的长度前缀（失步），随后应能跳过并继续解析后续真帧。
        let good = encode_frame(b"ok");
        let mut corrupt = vec![0xFF, 0xFF]; // 长度前缀 = 65535（> MAX）
        corrupt.extend_from_slice(&good);
        let mut reader = FrameReader::new();
        let frames = reader.push(&corrupt);
        assert_eq!(
            frames.len(),
            1,
            "should skip garbage and parse the good frame"
        );
        assert_eq!(frames[0], b"ok");
    }

    #[test]
    fn frame_roundtrip() {
        let payload = b"telemetry payload".to_vec();
        let frame = encode_frame(&payload);
        assert_eq!(verify_frame(&frame), Some(payload.as_slice()));
    }

    #[test]
    fn corrupt_frame_rejected() {
        let payload = b"important".to_vec();
        let mut frame = encode_frame(&payload);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF; // 篡改 CRC
        assert!(verify_frame(&frame).is_none());
    }

    #[test]
    fn reader_handles_split_and_concatenated() {
        let p1 = b"frame-one".to_vec();
        let p2 = b"frame-two".to_vec();
        let f1 = encode_frame(&p1);
        let f2 = encode_frame(&p2);

        let mut reader = FrameReader::new();
        // 半包：只给一半。
        assert!(reader.push(&f1[..3]).is_empty());
        assert_eq!(reader.pending(), 3);
        // 剩余一半 + 另一整帧（粘包）。
        let mut rest = f1[3..].to_vec();
        rest.extend_from_slice(&f2);
        let frames = reader.push(&rest);
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0], p1);
        assert_eq!(frames[1], p2);
    }

    #[test]
    fn reader_rejects_corrupt_but_keeps_valid() {
        let good = encode_frame(b"ok");
        let mut bad = encode_frame(b"bad-data");
        bad[3] ^= 0x01; // 破坏负载
        let mut reader = FrameReader::new();
        let frames = reader.push(&bad);
        assert!(frames.is_empty()); // 坏帧被丢弃
        let frames = reader.push(&good);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0], b"ok");
    }
}
