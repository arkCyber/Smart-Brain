//! 时间原语。
//!
//! 为方便在无 std 时钟或仿真环境下注入时间，统一使用单调时钟获取
//! 相对时间戳（毫秒），用于心跳与 fail-safe 计时。

use std::time::{SystemTime, UNIX_EPOCH};

/// 单调相对时间戳（毫秒）。所有模块用它作为“心跳时间戳”。
pub type Timestamp = u64;

/// 获取当前相对时间戳（毫秒）。
pub fn instant_now() -> Timestamp {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// 时间戳差值（毫秒），用于计算两次事件间隔。
pub fn elapsed_since(last: Timestamp) -> u64 {
    instant_now().saturating_sub(last)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timestamps_monotonic() {
        let a = instant_now();
        let b = instant_now();
        assert!(b >= a);
    }
}
