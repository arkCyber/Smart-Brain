//! 核心数据类型：键表达式、值、样本、回复，以及 serde 编解码。

use brain_core::time::Timestamp;
use serde::{de::DeserializeOwned, Serialize};

/// 负载类型：Zenoh 传输任意字节。这里用 `Vec<u8>` 承载序列化后的消息。
pub type Value = Vec<u8>;

/// 一条发布/订阅样本。
#[derive(Debug, Clone)]
pub struct Sample {
    pub key: String,
    pub value: Value,
    pub timestamp: Timestamp,
}

impl Sample {
    pub fn new(key: impl Into<String>, value: Value, timestamp: Timestamp) -> Self {
        Self {
            key: key.into(),
            value,
            timestamp,
        }
    }
}

/// 一次查询/回复的应答。
#[derive(Debug, Clone)]
pub struct Reply {
    pub key: String,
    pub value: Value,
}

impl Reply {
    pub fn new(key: impl Into<String>, value: Value) -> Self {
        Self {
            key: key.into(),
            value,
        }
    }
}

/// 键表达式匹配：支持精确匹配，以及末尾 `/*` 的前缀通配。
///
/// 例：`query="sensor/*"` 匹配 `stored="sensor/temp"`；
///     `query="fcu/telemetry"` 仅匹配同名。
pub fn key_matches(query: &str, stored: &str) -> bool {
    if let Some(prefix) = query.strip_suffix("/*") {
        stored.len() > prefix.len()
            && stored.starts_with(prefix)
            && stored.as_bytes()[prefix.len()] == b'/'
    } else {
        query == stored
    }
}

/// 用 JSON 把消息序列化为负载。
pub fn encode<T: Serialize>(v: &T) -> brain_core::Result<Value> {
    serde_json::to_vec(v).map_err(|e| brain_core::BrainError::Bus(format!("encode: {e}")))
}

/// 从负载反序列化为消息。
pub fn decode<T: DeserializeOwned>(value: &[u8]) -> brain_core::Result<T> {
    serde_json::from_slice(value).map_err(|e| brain_core::BrainError::Bus(format!("decode: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::{Deserialize, Serialize};

    #[derive(Debug, Serialize, Deserialize, PartialEq)]
    struct Msg {
        id: u32,
        text: String,
    }

    #[test]
    fn encode_decode_roundtrip() {
        let m = Msg {
            id: 7,
            text: "hello".into(),
        };
        let value = encode(&m).unwrap();
        let back: Msg = decode(&value).unwrap();
        assert_eq!(back, m);
    }

    #[test]
    fn key_matching() {
        assert!(key_matches("fcu/telemetry", "fcu/telemetry"));
        assert!(key_matches("sensor/*", "sensor/temp"));
        assert!(key_matches("sensor/*", "sensor/imu"));
        assert!(!key_matches("sensor/*", "sensor")); // 前缀必须含分隔符
        assert!(!key_matches("sensor/temp", "sensor/imu"));
    }
}
