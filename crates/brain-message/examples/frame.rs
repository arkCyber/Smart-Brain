//! `brain-message` 最小示例：帧封装（带 CRC）+ `FrameReader` 处理半包/粘包。
//!
//! 运行：`cargo run -p brain-message --example frame`

use brain_message::{crc16, encode_frame, verify_frame, FrameReader, FRAME_OVERHEAD};

fn main() {
    // 把任意字节负载（遥测/指令序列化结果）封装成带 CRC 的传输帧
    let payload = b"telemetry payload".to_vec();
    let frame = encode_frame(&payload);
    println!(
        "framed {} bytes (payload {} bytes): {:?}…",
        frame.len(),
        frame.len() - FRAME_OVERHEAD,
        &frame[..6]
    );
    println!("crc16 = {:#06x}", crc16(&payload));

    // 校验整帧并取出负载
    assert_eq!(verify_frame(&frame), Some(payload.as_slice()));

    // FrameReader：把字节流切成完整帧（正确处理半包/粘包）
    let mut reader = FrameReader::new();
    reader.push(&frame[..3]); // 先给半个包
    let got = reader.push(&frame[3..]); // 再给剩余部分
    assert_eq!(got, vec![payload]);
    println!(
        "reader recovered {} frame(s), pending = {}",
        got.len(),
        reader.pending()
    );
}
