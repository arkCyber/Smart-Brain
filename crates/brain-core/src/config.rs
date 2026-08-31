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

    /// 安全兜底参数（围栏 / 电量 / pre-arm）。缺省时使用内置安全默认值。
    #[serde(default)]
    pub safety: SafetyConfig,
}

/// 安全兜底参数（对应 `brain-state::safety`）。
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SafetyConfig {
    /// 地理围栏最大水平半径（米）。
    pub geofence_radius_m: f32,
    /// 地理围栏最大高度（米，向上为正）。
    pub geofence_max_altitude_m: f32,
    /// 电量：触发自动返航的剩余电量（%）。
    pub battery_rth_pct: f32,
    /// 电量：触发紧急降落的临界阈值（%）。
    pub battery_critical_pct: f32,
    /// 电量：低电提示阈值（%）。
    pub battery_low_pct: f32,
    /// pre-arm：最低可见卫星数。
    pub prearm_min_gps_satellites: u8,
    /// pre-arm：是否强制要求 3D 定位。
    pub prearm_require_fix3d: bool,
    /// pre-arm：最低剩余电量（%）。
    pub prearm_min_battery_pct: f32,
    /// pre-arm：是否要求已设置 home。
    pub prearm_require_home: bool,
}

impl Default for SafetyConfig {
    fn default() -> Self {
        Self {
            geofence_radius_m: 500.0,
            geofence_max_altitude_m: 200.0,
            battery_rth_pct: 30.0,
            battery_critical_pct: 15.0,
            battery_low_pct: 40.0,
            prearm_min_gps_satellites: 10,
            prearm_require_fix3d: true,
            prearm_min_battery_pct: 25.0,
            prearm_require_home: true,
        }
    }
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

impl Default for FcuConfig {
    fn default() -> Self {
        Self {
            transport: "mock".into(),
            serial_port: "/dev/ttyS0".into(),
            baud_rate: 921_600,
            udp_target: "127.0.0.1:14550".into(),
        }
    }
}

impl Default for BrainConfig {
    fn default() -> Self {
        Self {
            node_id: "smart-brain-01".into(),
            heartbeat_period_ms: 10,
            failsafe_timeout_ms: 50,
            tick_period_ms: 20,
            fcu: FcuConfig::default(),
            safety: SafetyConfig::default(),
        }
    }
}

impl BrainConfig {
    /// 从 JSON 字符串解析配置。
    pub fn from_json(json: &str) -> Result<Self> {
        serde_json::from_str(json).map_err(|e| BrainError::Config(e.to_string()))
    }

    /// 从文件加载配置。
    pub fn from_file(path: &str) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| BrainError::Config(format!("read {path}: {e}")))?;
        Self::from_json(&text)
    }

    /// 依次尝试候选路径加载配置；全部失败时回退到 [`Self::default`]。
    ///
    /// 候选路径可来自环境变量（例如 `$SMART_BRAIN_CONFIG`）。生产部署时建议
    /// 显式传入配置文件路径，避免静默回退到默认值。
    pub fn load_candidates(paths: &[impl AsRef<std::path::Path>]) -> Self {
        for p in paths {
            if let Ok(cfg) = Self::from_file(p.as_ref().to_str().unwrap_or_default()) {
                log::info!("loaded config from {}", p.as_ref().display());
                return cfg;
            }
        }
        log::warn!(
            "no config file found at candidates; using defaults: {:?}",
            paths
                .iter()
                .map(|p| p.as_ref().display().to_string())
                .collect::<Vec<_>>()
        );
        Self::default()
    }

    /// 校验配置是否自洽。
    pub fn validate(&self) -> Result<()> {
        if self.failsafe_timeout_ms == 0 {
            return Err(BrainError::Config("failsafe_timeout_ms must be > 0".into()));
        }
        if self.heartbeat_period_ms == 0 {
            return Err(BrainError::Config("heartbeat_period_ms must be > 0".into()));
        }
        if self.tick_period_ms == 0 {
            return Err(BrainError::Config("tick_period_ms must be > 0".into()));
        }
        if self.fcu.serial_port.is_empty() && self.fcu.transport == "serial" {
            return Err(BrainError::Config(
                "serial transport requires a non-empty serial_port".into(),
            ));
        }
        if self.fcu.transport == "serial" && self.fcu.baud_rate == 0 {
            return Err(BrainError::Config(
                "serial transport requires a non-zero baud_rate".into(),
            ));
        }
        if self.fcu.transport == "udp" && self.fcu.udp_target.is_empty() {
            return Err(BrainError::Config(
                "udp transport requires a non-empty udp_target".into(),
            ));
        }
        // 安全参数自洽性：围栏范围为正、电量阈值单调（critical < rth < low）。
        if self.safety.geofence_radius_m <= 0.0 || self.safety.geofence_max_altitude_m <= 0.0 {
            return Err(BrainError::Config(
                "geofence radius/altitude must be positive".into(),
            ));
        }
        if !(self.safety.battery_critical_pct < self.safety.battery_rth_pct
            && self.safety.battery_rth_pct < self.safety.battery_low_pct)
        {
            return Err(BrainError::Config(
                "battery thresholds must satisfy critical < rth < low".into(),
            ));
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

    #[test]
    fn validates_zero_periods() {
        let cfg = BrainConfig {
            tick_period_ms: 0,
            ..BrainConfig::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validates_serial_port_requirement() {
        let cfg = BrainConfig {
            fcu: FcuConfig {
                transport: "serial".into(),
                serial_port: String::new(),
                baud_rate: 0,
                udp_target: String::new(),
            },
            ..BrainConfig::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn validates_udp_target_requirement() {
        let cfg = BrainConfig {
            fcu: FcuConfig {
                transport: "udp".into(),
                serial_port: String::new(),
                baud_rate: 0,
                udp_target: String::new(),
            },
            ..BrainConfig::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn load_candidates_falls_back_to_default() {
        let cfg = BrainConfig::load_candidates(&["/nonexistent/a.json", "/nonexistent/b.json"]);
        assert_eq!(cfg.node_id, "smart-brain-01");
    }

    #[test]
    fn load_candidates_picks_first_existing() {
        let dir = std::env::temp_dir();
        let path = dir.join("smart_brain_cfg_test.json");
        std::fs::write(
            &path,
            r#"{"node_id":"from-file","heartbeat_period_ms":5,"failsafe_timeout_ms":50,"tick_period_ms":20,"fcu":{"transport":"mock","serial_port":"","baud_rate":0,"udp_target":""}}"#,
        )
        .unwrap();
        let cfg = BrainConfig::load_candidates(&[&path]);
        assert_eq!(cfg.node_id, "from-file");
        std::fs::remove_file(&path).unwrap();
    }

    #[test]
    fn old_config_without_safety_field_parses_with_defaults() {
        // 不含 safety 字段的旧配置仍可解析，且回退到默认安全参数。
        let cfg = BrainConfig::from_json(r#"{"node_id":"n1","heartbeat_period_ms":5,"failsafe_timeout_ms":50,"tick_period_ms":20,"fcu":{"transport":"mock","serial_port":"","baud_rate":0,"udp_target":""}}"#)
            .unwrap();
        assert_eq!(
            cfg.safety.geofence_radius_m,
            SafetyConfig::default().geofence_radius_m
        );
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn validates_safety_threshold_order() {
        let cfg = BrainConfig {
            safety: SafetyConfig {
                battery_critical_pct: 50.0, // 错误顺序：critical >= rth
                ..SafetyConfig::default()
            },
            ..BrainConfig::default()
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn safety_config_roundtrips_json() {
        let json = serde_json::to_string(&SafetyConfig::default()).unwrap();
        let back: SafetyConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(back.geofence_radius_m, 500.0);
    }

    #[test]
    fn fcu_config_default_roundtrips() {
        let f = FcuConfig::default();
        assert_eq!(f.transport, "mock");
        assert_eq!(f.baud_rate, 921_600);
        // serde 往返。
        let json = serde_json::to_string(&f).unwrap();
        let back: FcuConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(back.serial_port, "/dev/ttyS0");
    }

    #[test]
    fn validates_serial_baud_rate() {
        let cfg = BrainConfig {
            fcu: FcuConfig {
                transport: "serial".into(),
                serial_port: "/dev/ttyUSB0".into(),
                baud_rate: 0,
                udp_target: String::new(),
            },
            ..BrainConfig::default()
        };
        assert!(cfg.validate().is_err());
    }
}
