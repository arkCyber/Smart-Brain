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

/// 键表达式匹配：支持精确匹配、单段通配 `*` 与任意深度通配 `**`。
///
/// 段以 `/` 分隔；`*` 匹配恰好一个非空段，`**` 匹配零个或多个段。
///
/// 例：
/// - `query="sensor/*"` 匹配 `stored="sensor/temp"`、`"sensor/imu"`，不匹配 `"sensor"`；
/// - `query="sensor/**"` 匹配 `"sensor/temp"`、`"sensor/nav/gps"`，也匹配 `"sensor"`；
/// - `query="fcu/telemetry"` 仅匹配同名（精确）。
///
/// 实现为**动态规划**（`O(|query| × |stored|)`），避免 `**` 的指数回溯。
pub fn key_matches(query: &str, stored: &str) -> bool {
    let q: Vec<&str> = query.split('/').collect();
    let s: Vec<&str> = stored.split('/').collect();
    // dp[i][j]：query 前 i 段能否匹配 stored 前 j 段。
    let mut dp = vec![vec![false; s.len() + 1]; q.len() + 1];
    dp[0][0] = true;
    for i in 1..=q.len() {
        // 空 stored 侧：仅 `**` 能消费 0 段。
        if q[i - 1] == "**" {
            dp[i][0] = dp[i - 1][0];
        }
        for j in 1..=s.len() {
            dp[i][j] = match q[i - 1] {
                "**" => dp[i - 1][j] || dp[i][j - 1], // 消费 0 段 或 1+ 段
                "*" => dp[i - 1][j - 1],              // 恰好一段
                seg => dp[i - 1][j - 1] && seg == s[j - 1],
            };
        }
    }
    dp[q.len()][s.len()]
}

/// 键/键表达式允许的最大段数（防止恶意/病态深键导致 `**` 匹配指数爆炸或栈过深）。
pub const MAX_KEY_DEPTH: usize = 64;

/// 键/键表达式合法性校验（生产用，拒绝非法输入）。
///
/// 规则：
/// - 非空；
/// - 段间以单个 `/` 分隔，不允许空段（如 `a//b`、`a/`、`/a`）；
/// - 通配符 `*`/`**` 必须**独立成段**（不支持 `sen*or` 之类部分通配）；
/// - 普通段不允许出现 Zenoh 保留字符 `.` `#` `+` `$`；
/// - 段数不超过 [`MAX_KEY_DEPTH`]。
pub fn valid_key_expr(expr: &str) -> bool {
    if expr.is_empty() {
        return false;
    }
    let mut depth = 0usize;
    for seg in expr.split('/') {
        depth += 1;
        if depth > MAX_KEY_DEPTH {
            return false; // 段数超限
        }
        if seg.is_empty() {
            return false; // 空段
        }
        if seg.contains('*') {
            // 通配符必须独立成段：只能是 `*` 或 `**`。
            if seg != "*" && seg != "**" {
                return false;
            }
        } else {
            for ch in seg.chars() {
                if matches!(ch, '.' | '#' | '+' | '$') {
                    return false;
                }
            }
        }
    }
    true
}

/// 是否为**具体键**（不含任何通配符的合法键），用于 `put`/`declare_queryable`。
pub fn is_concrete_key(key: &str) -> bool {
    !key.contains('*') && valid_key_expr(key)
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
        // 精确
        assert!(key_matches("fcu/telemetry", "fcu/telemetry"));
        assert!(!key_matches("fcu/telemetry", "fcu/nav"));
        // 单段通配 *
        assert!(key_matches("sensor/*", "sensor/temp"));
        assert!(key_matches("sensor/*", "sensor/imu"));
        assert!(!key_matches("sensor/*", "sensor")); // * 需恰好一段
        assert!(!key_matches("sensor/*", "sensor/nav/gps")); // 不跨深度
                                                             // 任意深度通配 **
        assert!(key_matches("sensor/**", "sensor/temp"));
        assert!(key_matches("sensor/**", "sensor/nav/gps"));
        assert!(key_matches("sensor/**", "sensor")); // ** 可匹配零段
        assert!(!key_matches("sensor/**", "other/x"));
        // 前缀 + 多段
        assert!(key_matches("**/gps", "sensor/nav/gps"));
        assert!(!key_matches("sensor/temp", "sensor/imu"));
    }

    #[test]
    fn valid_key_expression_validation() {
        assert!(valid_key_expr("fcu/telemetry"));
        assert!(valid_key_expr("sensor/*"));
        assert!(valid_key_expr("sensor/**"));
        assert!(valid_key_expr("a/b/c"));
        // 非法
        assert!(!valid_key_expr("")); // 空
        assert!(!valid_key_expr("a//b")); // 空段
        assert!(!valid_key_expr("a/")); // 尾空段
        assert!(!valid_key_expr("/a")); // 首空段
        assert!(!valid_key_expr("sen*or")); // 部分通配
        assert!(!valid_key_expr("a.b")); // 保留字符
        assert!(!valid_key_expr("a#b"));
    }

    #[test]
    fn concrete_key_check() {
        assert!(is_concrete_key("fcu/telemetry"));
        assert!(is_concrete_key("a/b/c"));
        assert!(!is_concrete_key("sensor/*")); // 含通配不是具体键
        assert!(!is_concrete_key("a//b")); // 非法
    }

    #[test]
    fn key_depth_is_capped() {
        // 恰好 MAX_KEY_DEPTH 段合法；超过则非法（防病态深键/`**` 指数爆炸）。
        let ok = (0..MAX_KEY_DEPTH)
            .map(|i| format!("s{i}"))
            .collect::<Vec<_>>()
            .join("/");
        assert!(valid_key_expr(&ok), "at the cap should be valid");
        let too_deep = (0..=MAX_KEY_DEPTH)
            .map(|i| format!("s{i}"))
            .collect::<Vec<_>>()
            .join("/");
        assert!(
            !valid_key_expr(&too_deep),
            "beyond the cap should be invalid"
        );
    }

    #[test]
    fn double_star_matches_deep_key_and_terminates() {
        // `**` 对深键匹配且能快速终止（配合深度上限，不会栈溢出/指数爆炸）。
        let deep = (0..48)
            .map(|i| format!("s{i}"))
            .collect::<Vec<_>>()
            .join("/");
        assert!(key_matches("**", &deep));
        assert!(key_matches("**/s47", &deep));
        assert!(key_matches(&deep, &deep)); // 精确
        assert!(!key_matches(&format!("{deep}/x"), &deep));
    }

    #[test]
    fn many_double_stars_still_linear() {
        // 多个 `**` 对深键：DP 实现为 O(q×s)，语义上 `**` 可吞任意段。
        let q = (0..8).map(|_| "**").collect::<Vec<_>>().join("/");
        let s = (0..40)
            .map(|i| format!("s{i}"))
            .collect::<Vec<_>>()
            .join("/");
        assert!(key_matches(&q, &s), "many ** should match any key");
        assert!(key_matches(&format!("a/{q}/z"), &format!("a/{s}/z")));
        assert!(!key_matches(&format!("a/{q}/z"), &format!("b/{s}/z")));
    }
}
