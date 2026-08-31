//! 全局配置。

use serde::{Deserialize, Serialize};

use crate::error::{BrainError, Result};

/// 任务计算机（大脑）的顶层配置。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BrainConfig {
    /// 大脑的名字 / 节点 ID。
    pub node_id: String,

    /// 心跳周期（毫秒），用于 fail-safe 看门狗。
    pub heartbeat_period_ms: u64,

    /// Fail-safe 判定阈值（毫秒）：超过则认为大脑卡死。
    pub failsafe_timeout_ms: u64,

    /// 行为树主循环周期（毫秒）。
    pub tick_period_ms: u64,

    /// 飞控连接配置。
    pub fcu: FcuConfig,
}

/// 与飞控（小脑）的连接配置。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FcuConfig {
    /// 传输方式："mock" | "serial" | "udp" | "can"。
    pub transport: String,
    /// 串口设备路径（serial 时使用）。
    pub serial_port: String,
    /// 波特率。
    pub baud_rate: u32,
    /// UDP 目标地址（udp 时使用）。
    pub udp_target: String,
}

impl Default for BrainConfig {
    fn default() -> Self {
        Self {
            node_id: "smart-brain-01".into(),
            heartbeat_period_ms: 10,
            failsafe_timeout_ms: 50,
            tick_period_ms: 20,
            fcu: FcuConfig {
                transport: "mock".into(),
                serial_port: "/dev/ttyS0".into(),
                baud_rate: 921_600,
                udp_target: "127.0.0.1:14550".into(),
            },
        }
    }
}

impl BrainConfig {
    /// 从 JSON 字符串解析配置。
    pub fn from_json(json: &str) -> Result<Self> {
        serde_json::from_str(json).map_err(|e| BrainError::Config(e.to_string()))
    }

    /// 校验配置是否自洽。
    pub fn validate(&self) -> Result<()> {
        if self.failsafe_timeout_ms == 0 {
            return Err(BrainError::Config("failsafe_timeout_ms must be > 0".into()));
        }
        if self.heartbeat_period_ms == 0 {
            return Err(BrainError::Config("heartbeat_period_ms must be > 0".into()));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_valid() {
        let cfg = BrainConfig::default();
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn parses_from_json() {
        let cfg = BrainConfig::from_json(r#"{"node_id":"n1","heartbeat_period_ms":5,"failsafe_timeout_ms":50,"tick_period_ms":20,"fcu":{"transport":"mock","serial_port":"","baud_rate":0,"udp_target":""}}"#)
            .unwrap();
        assert_eq!(cfg.node_id, "n1");
    }
}
