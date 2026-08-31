//! `brain-autopilot` 示例：AIS 报文 → 局部坐标 → COLREGS 会遇避让（能见度受限）。
//!
//! 运行：`cargo run -p brain-autopilot --example ais_colregs`
//!
//! 展示水面艇"多艇会遇感知 → 规则避让"完整链路：
//! 1. 解码一帧 A 类位置报告（`!AIVDM`，6-bit 负载）；
//! 2. 把目标经纬度换算成本船局部东/北偏移（米）；
//! 3. 交给 COLREGS 引擎在能见度受限（Rule 19）下给出避让动作。

use brain_autopilot::{
    Colregs, ColregsAction, ColregsParams, EncounterType, VesselPose, Visibility, decode_ais,
    to_local_offset,
};
use brain_core::Vec3;

/// 测试用编码：把一组 `(值, 位宽)` 打包成 AIVDM 6-bit 负载（与解码器对称）。
fn encode_payload(fields: &[(u32, u32)]) -> String {
    let mut bits: Vec<u8> = Vec::new();
    for (val, nbits) in fields {
        for i in (0..*nbits).rev() {
            bits.push(((val >> i) & 1) as u8);
        }
    }
    while bits.len() % 6 != 0 {
        bits.push(0);
    }
    bits.chunks(6)
        .map(|chunk| {
            let mut v = 0u8;
            for &b in chunk {
                v = (v << 1) | b;
            }
            let c = if v <= 39 { 48 + v } else { 96 + (v - 40) };
            c as char
        })
        .collect()
}

fn main() {
    // 构造一帧类型 1（A 类位置报告）：MMSI、SOG、经度、纬度、COG、船艏向。
    let mmsi = 366_999_000u32;
    let lon = 10.0001f32; // ≈ 本船东侧约 10m
    let lat = 20.0000f32; // 与本船同纬度
    let payload = encode_payload(&[
        (1, 6),                            // type
        (0, 2),                            // repeat
        (mmsi, 30),
        (0, 4),                            // nav status（在航）
        (0, 8),                            // ROT
        (123, 10),                         // SOG = 12.3 kn
        (0, 1),                            // accuracy
        ((lon * 600_000.0) as i32 as u32, 28),
        ((lat * 600_000.0) as i32 as u32, 27),
        (456, 12),                         // COG = 45.6°
        (90, 9),                           // heading
        (0, 6),                            // ts
    ]);
    let sentence = format!("!AIVDM,1,1,,B,{payload},0*00");

    // 1) 解码
    let msg = decode_ais(&sentence).expect("decode AIS");
    let (target_lon, target_lat) = msg.position().expect("target has position");
    println!("decoded MMSI {} @ ({target_lon}, {target_lat})", msg.mmsi());

    // 2) 相对本船（参考点 10°E, 20°N）的局部偏移
    let off: Vec3 = to_local_offset(10.0, 20.0, target_lon, target_lat);
    println!("relative to own: east={:.0} m, north={:.0} m", off.x, off.y);

    // 3) COLREGS：本船机动船朝 +x，目标在其正前方相向驶来，能见度受限。
    let own = VesselPose::new(0.0, 0.0, 0.0); // 机动船
    let other = VesselPose::new(off.x, off.y, std::f32::consts::PI); // 相向
    let params = ColregsParams {
        visibility: Visibility::Restricted,
        ..ColregsParams::default()
    };
    let (kind, action) = Colregs::classify(own, other, &params);
    println!("COLREGS: {kind:?} → {action:?}");
    debug_assert_eq!(kind, EncounterType::RestrictedVisibility);
    debug_assert_eq!(action, ColregsAction::GiveWayStarboard);

    // 能见度良好：对方若是帆船在本船左舷，机动船仍须让路（帆船优先通行权）。
    let other_sail = VesselPose::sailing(5.0, -5.0, 1.1); // 左舷帆船
    let (kind2, a2) = Colregs::classify(own, other_sail, &ColregsParams::default());
    println!("sailing on port, power-driven: {kind2:?} → {a2:?}");
    debug_assert_eq!(kind2, EncounterType::Crossing);
    debug_assert_eq!(a2, ColregsAction::GiveWayStarboard);
}
