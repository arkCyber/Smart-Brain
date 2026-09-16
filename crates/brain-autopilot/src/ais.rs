//! AIS（自动识别系统）报文解析 —— 水面艇会遇感知的"输入层"。
//!
//! 解析 NMEA 0183 `!AIVDM / !AIVDO` 语句的 6-bit 负载，解码常见报文类型：
//! - 类型 1/2/3：**A 类位置报告**（船舶位置、对地航速/航向、真船艏向、航行状态）；
//! - 类型 18：**B 类位置报告**（小型/非 SOLAS 船舶）；
//! - 类型 5：**静态与航次数据**（船名、呼号、船型、IMO、尺寸）。
//!
//! 与 [`crate::colregs`] 配合：解析出的目标经纬度可换算成局部坐标喂给避碰引擎，
//! 构成"多艇会遇感知 → 规则避让"的完整链路。

use brain_core::Vec3;

use crate::colregs::{Propulsion, VesselPose};

/// AIS 6-bit 字符字母表（索引即 6-bit 值，0 为 `@` 填充/终止符）。
const SIXBIT_ALPHABET: &[u8] =
    b"@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_ !\"#$%&'()*+,-./0123456789:;<=>?";

/// 地球平均半径（米），用于经纬度 → 局部东/北偏移的等距圆柱近似。
const EARTH_RADIUS_M: f32 = 6_371_000.0;

/// AIS 解析错误。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AisError {
    /// 非 AIVDM/AIVDO 语句。
    NotAisSentence,
    /// 多句（分片）报文暂不支持（type 5 在部分电台被分片）。
    MultipartNotSupported { total: u32 },
    /// 负载字段缺失或包含非法 6-bit 字符。
    InvalidPayload(String),
    /// 数据不足以容纳该报文类型所需字段。
    Truncated { msg_type: u8, have_bits: usize },
}

impl core::fmt::Display for AisError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AisError::NotAisSentence => write!(f, "not an AIVDM/AIVDO sentence"),
            AisError::MultipartNotSupported { total } => {
                write!(f, "multipart AIS message (fragments={total}) not supported")
            }
            AisError::InvalidPayload(msg) => write!(f, "invalid AIS payload: {msg}"),
            AisError::Truncated {
                msg_type,
                have_bits,
            } => {
                write!(f, "AIS type {msg_type} truncated: have {have_bits} bits")
            }
        }
    }
}

impl std::error::Error for AisError {}

/// 航行状态（类型 1/2/3 的字段，仅 A 类有效；B 类无此字段）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NavStatus {
    UnderWayUsingEngine = 0,
    AtAnchor = 1,
    NotUnderCommand = 2,
    RestrictedManoeuverability = 3,
    ConstrainedByDraught = 4,
    Moored = 5,
    Aground = 6,
    EngagedInFishing = 7,
    UnderWaySailing = 8,
    /// 未指定/其他/未知。
    Other = 15,
}

impl NavStatus {
    pub fn from_bits(bits: u32) -> Self {
        match bits {
            0 => NavStatus::UnderWayUsingEngine,
            1 => NavStatus::AtAnchor,
            2 => NavStatus::NotUnderCommand,
            3 => NavStatus::RestrictedManoeuverability,
            4 => NavStatus::ConstrainedByDraught,
            5 => NavStatus::Moored,
            6 => NavStatus::Aground,
            7 => NavStatus::EngagedInFishing,
            8 => NavStatus::UnderWaySailing,
            _ => NavStatus::Other,
        }
    }
}

/// 位置报告（类型 1/2/3 或 18）解码结果。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PositionReport {
    /// 原始报文类型（1/2/3/18）。
    pub msg_type: u8,
    /// 是否 B 类（类型 18）。
    pub class_b: bool,
    /// MMSI（船舶识别码）。
    pub mmsi: u32,
    /// 航行状态（仅 A 类；B 类恒为 [`NavStatus::Other`]）。
    pub nav_status: NavStatus,
    /// 对地航速（节）。不可用时为负值。
    pub sog_knots: f32,
    /// 对地航向（度）。不可用为负值。
    pub cog_deg: f32,
    /// 真船艏向（度）。不可用为 `None`。
    pub heading_deg: Option<f32>,
    /// 经度（度，东为正）。不可用为 `None`。
    pub longitude_deg: Option<f32>,
    /// 纬度（度，北为正）。不可用为 `None`。
    pub latitude_deg: Option<f32>,
    /// 位置更新时间戳（秒 0-59；60+ 表示不可用/推算）。
    pub timestamp_s: u8,
}

/// 静态与航次数据（类型 5）。
#[derive(Debug, Clone, PartialEq)]
pub struct StaticVoyageData {
    pub mmsi: u32,
    /// IMO 编号（0 表示不可用）。
    pub imo: u32,
    /// 呼号（6-bit 字符，去除尾部空格）。
    pub callsign: String,
    /// 船名（6-bit 字符，去除尾部空格）。
    pub ship_name: String,
    /// 船型（类型 5 字段 19）。
    pub ship_type: u8,
    /// 船舶尺寸（米）：后端 / 前端 / 左舷 / 右舷。
    pub dim_b: u16,
    pub dim_a: u16,
    pub dim_c: u16,
    pub dim_d: u16,
}

/// 一条解析后的 AIS 消息。
#[derive(Debug, Clone, PartialEq)]
pub enum AisMessage {
    PositionReport(PositionReport),
    StaticVoyage(StaticVoyageData),
    /// 其他暂不解析的报文类型（保留类型号）。
    Other {
        msg_type: u8,
    },
}

impl AisMessage {
    /// 若为位置报告，返回 `(经度, 纬度)`（度）；否则/不可用返回 `None`。
    pub fn position(&self) -> Option<(f32, f32)> {
        match self {
            AisMessage::PositionReport(p) => match (p.longitude_deg, p.latitude_deg) {
                (Some(lon), Some(lat)) => Some((lon, lat)),
                _ => None,
            },
            _ => None,
        }
    }

    /// 该报文对应的 MMSI。
    pub fn mmsi(&self) -> u32 {
        match self {
            AisMessage::PositionReport(p) => p.mmsi,
            AisMessage::StaticVoyage(s) => s.mmsi,
            AisMessage::Other { .. } => 0,
        }
    }

    /// 位置报告的节选航速/航向（若为位置报告）。
    pub fn motion(&self) -> Option<(f32, f32)> {
        match self {
            AisMessage::PositionReport(p) => Some((p.sog_knots, p.cog_deg)),
            _ => None,
        }
    }
}

/// 把 (参考经度, 参考纬度, 目标经度, 目标纬度) 换算为局部 `(东向, 北向)` 米偏移
/// （等距圆柱近似，适合短距离局部避碰）。`Vec3` 的 `x`=东、`y`=北、`z`=0。
pub fn to_local_offset(ref_lon: f32, ref_lat: f32, lon: f32, lat: f32) -> Vec3 {
    let dlon_deg = lon - ref_lon;
    let dlat_deg = lat - ref_lat;
    let m_per_deg_lat = std::f32::consts::PI * EARTH_RADIUS_M / 180.0;
    let m_per_deg_lon = m_per_deg_lat * ref_lat.to_radians().cos();
    Vec3::new(dlon_deg * m_per_deg_lon, dlat_deg * m_per_deg_lat, 0.0)
}

/// 把一条 AIS 位置报告相对本船（参考经纬度）转为局部 `VesselPose`（x=东, y=北），
/// 供 [`crate::colregs::Colregs::classify_many`] 做多目标协同避让。
///
/// AIS 的 COG 是相对真北、顺时针的度数（0=北、90=东）；本框架 `VesselPose` 的航向
/// 是 `atan2(y=北, x=东)` 弧度，故局部航向 = `π/2 − COG`。无法获得位置时返回 `None`。
pub fn ais_to_vessel_pose(
    own_lon: f32,
    own_lat: f32,
    msg: &AisMessage,
    propulsion: Propulsion,
) -> Option<VesselPose> {
    let (lon, lat) = msg.position()?;
    let off = to_local_offset(own_lon, own_lat, lon, lat);
    let cog_deg = msg.motion().map(|(_, c)| c).unwrap_or(0.0);
    let heading = std::f32::consts::FRAC_PI_2 - cog_deg.to_radians();
    Some(VesselPose {
        x: off.x,
        y: off.y,
        heading,
        propulsion,
    })
}

/// 从一句 `!AIVDM` 语句中提取 6-bit 负载字符串。
fn payload_of(sentence: &str) -> Result<String, AisError> {
    let trimmed = sentence.trim();
    if !(trimmed.starts_with("!AIVDM") || trimmed.starts_with("!AIVDO")) {
        return Err(AisError::NotAisSentence);
    }
    let fields: Vec<&str> = trimmed.split(',').collect();
    if fields.len() < 7 {
        return Err(AisError::InvalidPayload("missing fields".into()));
    }
    let total: u32 = fields[1]
        .parse()
        .map_err(|_| AisError::InvalidPayload("bad fragment count".into()))?;
    if total != 1 {
        return Err(AisError::MultipartNotSupported { total });
    }
    Ok(fields[5].to_string())
}

/// 6-bit 负载 → 字节流（MSB-first，末尾不足 8 位按零补齐）。
fn decode_6bit(payload: &str) -> Result<Vec<u8>, AisError> {
    let mut bits = Vec::with_capacity(payload.len() * 6);
    for c in payload.chars() {
        if !c.is_ascii() {
            return Err(AisError::InvalidPayload(format!("non-ascii char: {c}")));
        }
        let mut v = c as u8;
        if (48..=87).contains(&v) {
            v -= 48;
        } else if (96..=119).contains(&v) {
            v -= 56;
        } else {
            return Err(AisError::InvalidPayload(format!("invalid 6-bit char: {c}")));
        }
        for shift in (0..6).rev() {
            bits.push((v >> shift) & 1);
        }
    }
    // 补零到整字节（末尾若干 6-bit 之后的余位对齐到字节边界）。
    while bits.len() % 8 != 0 {
        bits.push(0);
    }
    Ok(bits
        .chunks(8)
        .map(|chunk| {
            let mut b = 0u8;
            for &bit in chunk {
                b = (b << 1) | bit;
            }
            b
        })
        .collect())
}

/// 读取 `bytes` 中从 `start` 位起的 `len` 位（MSB-first）为无符号整数。
fn bits(bytes: &[u8], start: u32, len: u32) -> u32 {
    let mut out = 0u32;
    for i in 0..len {
        let bit = start + i;
        let byte = bytes[(bit / 8) as usize];
        let inbyte = 7 - (bit % 8);
        out = (out << 1) | (((byte >> inbyte) & 1) as u32);
    }
    out
}

/// 有符号版本（二进制补码）。
fn bits_signed(bytes: &[u8], start: u32, len: u32) -> i32 {
    let raw = bits(bytes, start, len);
    let sign = 1u32 << (len - 1);
    if raw & sign != 0 {
        (raw as i32) - (1 << len)
    } else {
        raw as i32
    }
}

/// 校验 `start+len` 不超过可用位数，否则返回截断错误。
fn need(bytes: &[u8], msg_type: u8, start: u32, len: u32) -> Result<(), AisError> {
    if (start + len) as usize > bytes.len() * 8 {
        Err(AisError::Truncated {
            msg_type,
            have_bits: bytes.len() * 8,
        })
    } else {
        Ok(())
    }
}

/// 6-bit 字段 → 字符串（`@`=终止，跳过非法；去除尾部空格）。
fn sixbit_str(bytes: &[u8], start_bits: u32, nchars: u32) -> String {
    let mut s = String::with_capacity(nchars as usize);
    for i in 0..nchars {
        let v = bits(bytes, start_bits + i * 6, 6) as usize;
        if v == 0 || v >= SIXBIT_ALPHABET.len() {
            break;
        }
        let ch = SIXBIT_ALPHABET[v] as char;
        s.push(ch);
    }
    s.trim_end().to_string()
}

/// 解码位置报告（类型 1/2/3 或 18）。
fn decode_pos_report(
    bytes: &[u8],
    msg_type: u8,
    class_b: bool,
) -> Result<PositionReport, AisError> {
    // 先校验 MMSI 字段（位 8..37）可达，避免短负载越界。
    need(bytes, msg_type, 8, 30)?;
    let mmsi = bits(bytes, 8, 30);
    if class_b {
        // 类型 18：B 类位置报告（无航行状态字段）。
        need(bytes, msg_type, 46, 92)?; // 46..137
        let sog_raw = bits(bytes, 46, 10);
        let lon = bits_signed(bytes, 57, 28);
        let lat = bits_signed(bytes, 85, 27);
        let cog_raw = bits(bytes, 112, 12);
        let heading_raw = bits(bytes, 124, 9);
        let ts = bits(bytes, 133, 6) as u8;
        Ok(PositionReport {
            msg_type,
            class_b: true,
            mmsi,
            nav_status: NavStatus::Other,
            sog_knots: if sog_raw == 1023 {
                -1.0
            } else {
                sog_raw as f32 / 10.0
            },
            cog_deg: if cog_raw == 3600 {
                -1.0
            } else {
                cog_raw as f32 / 10.0
            },
            heading_deg: if heading_raw == 511 {
                None
            } else {
                Some(heading_raw as f32)
            },
            longitude_deg: if lon == 181 * 600_000 {
                None
            } else {
                Some(lon as f32 / 600_000.0)
            },
            latitude_deg: if lat == 91 * 600_000 {
                None
            } else {
                Some(lat as f32 / 600_000.0)
            },
            timestamp_s: ts,
        })
    } else {
        // 类型 1/2/3：A 类位置报告。
        need(bytes, msg_type, 38, 4)?;
        need(bytes, msg_type, 50, 10)?;
        need(bytes, msg_type, 61, 81)?; // 61..141
        let nav = NavStatus::from_bits(bits(bytes, 38, 4));
        let sog_raw = bits(bytes, 50, 10);
        let lon = bits_signed(bytes, 61, 28);
        let lat = bits_signed(bytes, 89, 27);
        let cog_raw = bits(bytes, 116, 12);
        let heading_raw = bits(bytes, 128, 9);
        let ts = bits(bytes, 137, 6) as u8;
        Ok(PositionReport {
            msg_type,
            class_b: false,
            mmsi,
            nav_status: nav,
            sog_knots: if sog_raw == 1023 {
                -1.0
            } else {
                sog_raw as f32 / 10.0
            },
            cog_deg: if cog_raw == 3600 {
                -1.0
            } else {
                cog_raw as f32 / 10.0
            },
            heading_deg: if heading_raw == 511 {
                None
            } else {
                Some(heading_raw as f32)
            },
            longitude_deg: if lon == 181 * 600_000 {
                None
            } else {
                Some(lon as f32 / 600_000.0)
            },
            latitude_deg: if lat == 91 * 600_000 {
                None
            } else {
                Some(lat as f32 / 600_000.0)
            },
            timestamp_s: ts,
        })
    }
}

/// 解码类型 5 静态/航次数据（424 bits）。
fn decode_static(bytes: &[u8]) -> Result<StaticVoyageData, AisError> {
    need(bytes, 5, 8 + 30, 5)?;
    need(bytes, 5, 40, 231)?; // 40..270
    let mmsi = bits(bytes, 8, 30);
    let imo = bits(bytes, 40, 30);
    let callsign = sixbit_str(bytes, 70, 7);
    let ship_name = sixbit_str(bytes, 112, 20);
    let ship_type = bits(bytes, 232, 8) as u8;
    let dim_b = bits(bytes, 240, 9) as u16;
    let dim_a = bits(bytes, 249, 9) as u16;
    let dim_c = bits(bytes, 258, 6) as u16;
    let dim_d = bits(bytes, 265, 6) as u16;
    Ok(StaticVoyageData {
        mmsi,
        imo,
        callsign,
        ship_name,
        ship_type,
        dim_b,
        dim_a,
        dim_c,
        dim_d,
    })
}

/// 解码一句 AIS 语句为一条消息。
pub fn decode(sentence: &str) -> Result<AisMessage, AisError> {
    let payload = payload_of(sentence)?;
    if payload.is_empty() {
        return Err(AisError::InvalidPayload("empty payload".into()));
    }
    let bytes = decode_6bit(&payload)?;
    let msg_type = bits(&bytes, 0, 6) as u8;
    match msg_type {
        1..=3 => Ok(AisMessage::PositionReport(decode_pos_report(
            &bytes, msg_type, false,
        )?)),
        18 => Ok(AisMessage::PositionReport(decode_pos_report(
            &bytes, msg_type, true,
        )?)),
        5 => Ok(AisMessage::StaticVoyage(decode_static(&bytes)?)),
        other => Ok(AisMessage::Other { msg_type: other }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- 测试用编码辅助：把字段打包成 AIVDM 负载 ----

    fn push_bits(out: &mut Vec<u8>, value: u32, nbits: u32) {
        for i in (0..nbits).rev() {
            out.push(((value >> i) & 1) as u8);
        }
    }

    fn encode_payload(fields: &[(u32, u32)]) -> String {
        let mut bits_buf: Vec<u8> = Vec::new();
        for (val, nbits) in fields {
            push_bits(&mut bits_buf, *val, *nbits);
        }
        while !bits_buf.len().is_multiple_of(6) {
            bits_buf.push(0);
        }
        bits_buf
            .chunks(6)
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

    fn mk_type1(mmsi: u32, sog: u32, lon: i32, lat: i32, cog: u32, heading: u32) -> String {
        encode_payload(&[
            (1, 6), // type
            (0, 2), // repeat
            (mmsi, 30),
            (0, 4), // nav status
            (0, 8), // ROT
            (sog, 10),
            (0, 1), // acc
            (lon as u32, 28),
            (lat as u32, 27),
            (cog, 12),
            (heading, 9),
            (0, 6), // ts
        ])
    }

    #[test]
    fn decode_6bit_single_char() {
        // '1' = ASCII 49 → 6-bit 值 1 → 位串 000001 → 补齐字节 0x04（首 6 位在高位）。
        assert_eq!(decode_6bit("1").unwrap(), vec![0x04]);
        // '`' = ASCII 96 → 6-bit 值 40 → 位串 101000 → 补齐字节 0xA0。
        assert_eq!(decode_6bit("`").unwrap(), vec![0xA0]);
    }

    #[test]
    fn bits_reader_is_msb_first() {
        let bytes = [0b10110000, 0b00001111];
        assert_eq!(bits(&bytes, 0, 4), 0b1011);
        assert_eq!(bits(&bytes, 1, 3), 0b011);
        assert_eq!(bits(&bytes, 12, 4), 0b1111);
        // 有符号：0b1111（4 位）= -1。
        assert_eq!(bits_signed(&bytes, 12, 4), -1);
    }

    #[test]
    fn round_trip_type1_position_report() {
        let mmsi = 366_999_000u32;
        let lon_raw = (10.0f32 * 600_000.0) as i32; // 10°E
        let lat_raw = (20.0f32 * 600_000.0) as i32; // 20°N
        let payload = mk_type1(mmsi, 123, lon_raw, lat_raw, 456, 90);
        let sentence = format!("!AIVDM,1,1,,B,{payload},0*00");

        let msg = decode(&sentence).unwrap();
        let AisMessage::PositionReport(p) = msg else {
            panic!("expected position report");
        };
        assert_eq!(p.msg_type, 1);
        assert!(!p.class_b);
        assert_eq!(p.mmsi, mmsi);
        assert_eq!(p.nav_status, NavStatus::UnderWayUsingEngine);
        assert!((p.sog_knots - 12.3).abs() < 1e-4);
        assert!((p.cog_deg - 45.6).abs() < 1e-3);
        assert_eq!(p.heading_deg, Some(90.0));
        assert!((p.longitude_deg.unwrap() - 10.0).abs() < 1e-4);
        assert!((p.latitude_deg.unwrap() - 20.0).abs() < 1e-4);
    }

    #[test]
    fn negative_longitude_decodes() {
        let lon_raw = (-73.98765f32 * 600_000.0) as i32;
        let lat_raw = (40.01234f32 * 600_000.0) as i32;
        let payload = mk_type1(123_456_789, 0, lon_raw, lat_raw, 0, 511);
        let msg = decode(&format!("!AIVDM,1,1,,A,{payload},0")).unwrap();
        let AisMessage::PositionReport(p) = msg else {
            panic!()
        };
        assert!((p.longitude_deg.unwrap() - (-73.98765)).abs() < 1e-4);
        assert!((p.latitude_deg.unwrap() - 40.01234).abs() < 1e-4);
        assert_eq!(p.heading_deg, None); // 511 = 不可用
    }

    #[test]
    fn not_available_sog_cog() {
        let payload = mk_type1(111_111_111, 1023, 0, 0, 3600, 511);
        let msg = decode(&format!("!AIVDM,1,1,,B,{payload},0")).unwrap();
        let AisMessage::PositionReport(p) = msg else {
            panic!()
        };
        assert!(p.sog_knots < 0.0);
        assert!(p.cog_deg < 0.0);
        assert_eq!(p.heading_deg, None);
    }

    #[test]
    fn class_b_type18() {
        // 类型 18：B 类位置报告（字段布局不同于 A 类）。
        let mmsi = 338_000_000u32;
        let lon_raw = (-80.123f32 * 600_000.0) as i32;
        let lat_raw = (26.5f32 * 600_000.0) as i32;
        let payload = encode_payload(&[
            (18, 6), // type
            (0, 2),  // repeat
            (mmsi, 30),
            (0, 8),    // reserved
            (250, 10), // SOG = 25.0 kn
            (0, 1),    // acc
            (lon_raw as u32, 28),
            (lat_raw as u32, 27),
            (1200, 12), // COG = 120.0°
            (0, 9),     // heading（511 不可用）
            (0, 6),     // ts
        ]);
        let msg = decode(&format!("!AIVDM,1,1,,A,{payload},0")).unwrap();
        let AisMessage::PositionReport(p) = msg else {
            panic!()
        };
        assert_eq!(p.msg_type, 18);
        assert!(p.class_b);
        assert_eq!(p.nav_status, NavStatus::Other);
        assert!((p.sog_knots - 25.0).abs() < 1e-4);
        assert!((p.cog_deg - 120.0).abs() < 1e-3);
        assert!((p.longitude_deg.unwrap() - (-80.123)).abs() < 1e-4);
        assert!((p.latitude_deg.unwrap() - 26.5).abs() < 1e-4);
    }

    #[test]
    fn rejects_non_ais_sentence() {
        assert!(matches!(
            decode("$GPGGA,123"),
            Err(AisError::NotAisSentence)
        ));
    }

    #[test]
    fn rejects_multipart() {
        assert!(matches!(
            decode("!AIVDM,2,1,3,B,ABC,0*00"),
            Err(AisError::MultipartNotSupported { total: 2 })
        ));
    }

    #[test]
    fn truncated_payload_reports_truncated() {
        // 类型 1 需要 ≥141 位；只给类型号字段（6 位）应报截断。
        let payload = encode_payload(&[(1, 6)]);
        let got = decode(&format!("!AIVDM,1,1,,B,{payload},0"));
        assert!(matches!(got, Err(AisError::Truncated { msg_type: 1, .. })));
    }

    #[test]
    fn sixbit_string_trims_and_stops_at_at() {
        // 3 个 6-bit 字符：'A'(1), 'B'(2), 填充(0/@) → 共 18 位 → 3 字节。
        let bytes = vec![0b00000100, 0b00100000, 0b00000000]; // 000001 000010 000000
        let s = sixbit_str(&bytes, 0, 3);
        assert_eq!(s, "AB");
    }

    #[test]
    fn to_local_offset_returns_meters() {
        // 参考点 (10°E, 20°N)，目标向东 1°、向北 0.5°。
        let off = to_local_offset(10.0, 20.0, 11.0, 20.5);
        // 1° 经度 ≈ 111.3km·cos(20°) ≈ 104.6km；0.5° 纬度 ≈ 55.7km。
        assert!(off.x > 100_000.0 && off.x < 110_000.0, "east={}", off.x);
        assert!(off.y > 50_000.0 && off.y < 60_000.0, "north={}", off.y);
        assert_eq!(off.z, 0.0);
    }

    #[test]
    fn ais_to_vessel_pose_converts_position_and_heading() {
        // 本船 (20°N, 20°E)，目标在其正北约 10m、朝北航行（COG=0°）。
        let mmsi = 366_999_000u32;
        let lat_raw = ((20.0 + 10.0 / 111_320.0) * 600_000.0) as i32; // 北移 10m
        let lon_raw = (20.0f32 * 600_000.0) as i32; // 经度与本船相同
        let payload = mk_type1(mmsi, 0, lon_raw, lat_raw, 0, 0); // COG=0（朝北）
        let msg = decode(&format!("!AIVDM,1,1,,B,{payload},0")).unwrap();

        let vp = ais_to_vessel_pose(20.0, 20.0, &msg, Propulsion::PowerDriven).unwrap();
        // 目标在正北方 ~10m → x≈0、y≈10。
        assert!(vp.x.abs() < 1.0, "x={}", vp.x);
        assert!(vp.y > 9.0 && vp.y < 11.0, "y={}", vp.y);
        // COG=0（北）→ 局部航向 π/2（atan2 坐标系指北）。
        assert!((vp.heading - std::f32::consts::FRAC_PI_2).abs() < 1e-3);
    }
}
