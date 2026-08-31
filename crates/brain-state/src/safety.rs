//! 安全兜底原语：地理围栏（geofence）、电池电量告警与起飞前自检（pre-arm）。
//!
//! 与 [`super::failsafe::FailsafeWatchdog`] 不同，本模块关心的是“飞行边界”：
//! 位置是否越界、电量是否不足、以及“允许解锁起飞”的前置条件是否满足。
//! 三者都是纯函数式判定，便于单元测试与集成进决策层。

use brain_core::Vec3;
use brain_message::telemetry::FixType;

/// 地理围栏：约束相对起飞点（home）的水平半径与高度范围。
///
/// 采用北东地（NED）坐标：`x/y` 为北/东，`z` 向下为正，故“高度（向上）”
/// 为 `-(pos.z - home.z)`。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geofence {
    /// 起飞点（NED）。
    pub home: Vec3,
    /// 距 home 的最大水平半径（米）。
    pub max_radius_m: f32,
    /// 最大离地高度（米，向上为正）。
    pub max_altitude_m: f32,
    /// 最小高度（米，向上为正；通常为 0 即禁止入地）。
    pub min_altitude_m: f32,
}

impl Default for Geofence {
    /// 默认：以原点为 home、半径 1000m、高度 [0, 500]m（宽松边界）。
    fn default() -> Self {
        Self {
            home: Vec3::ZERO,
            max_radius_m: 1000.0,
            max_altitude_m: 500.0,
            min_altitude_m: 0.0,
        }
    }
}

impl Geofence {
    pub fn new(home: Vec3, max_radius_m: f32, max_altitude_m: f32, min_altitude_m: f32) -> Self {
        Self {
            home,
            max_radius_m,
            max_altitude_m,
            min_altitude_m,
        }
    }

    /// 检查位置是否越界；越界返回对应的违规类型。
    pub fn check(&self, pos: Vec3) -> Option<GeofenceViolation> {
        let dx = pos.x - self.home.x;
        let dy = pos.y - self.home.y;
        let distance = (dx * dx + dy * dy).sqrt();
        if distance > self.max_radius_m {
            return Some(GeofenceViolation::ExceededRadius {
                distance,
                max: self.max_radius_m,
            });
        }
        let altitude = -(pos.z - self.home.z); // 向上为正
        if altitude > self.max_altitude_m {
            return Some(GeofenceViolation::ExceededAltitude {
                altitude,
                max: self.max_altitude_m,
            });
        }
        if altitude < self.min_altitude_m {
            return Some(GeofenceViolation::BelowMinimumAltitude {
                altitude,
                min: self.min_altitude_m,
            });
        }
        None
    }

    /// 是否在边界内。
    pub fn contains(&self, pos: Vec3) -> bool {
        self.check(pos).is_none()
    }
}

/// 地理围栏违规类型。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeofenceViolation {
    /// 超出最大水平半径。
    ExceededRadius { distance: f32, max: f32 },
    /// 超出最大高度。
    ExceededAltitude { altitude: f32, max: f32 },
    /// 低于最小高度（可能入地）。
    BelowMinimumAltitude { altitude: f32, min: f32 },
}

/// 电池电量告警级别。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryAdvisory {
    /// 电量充足，正常飞行。
    Nominal,
    /// 电量偏低，建议尽快规划返航。
    Low,
    /// 已触发自动返航阈值，应进入 ReturnHome。
    RthTrigger,
    /// 已到临界阈值，应进入紧急降落。
    Critical,
}

/// 电池电量监视器：根据剩余电量输出告警级别。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryMonitor {
    /// 触发自动返航的剩余电量阈值（%）。
    pub rth_threshold_pct: f32,
    /// 触发紧急降落的临界阈值（%）。
    pub critical_threshold_pct: f32,
    /// 提示低电的阈值（%）。
    pub low_warning_pct: f32,
}

impl Default for BatteryMonitor {
    fn default() -> Self {
        Self {
            rth_threshold_pct: 30.0,
            critical_threshold_pct: 15.0,
            low_warning_pct: 40.0,
        }
    }
}

impl BatteryMonitor {
    pub fn new(rth_threshold_pct: f32, critical_threshold_pct: f32, low_warning_pct: f32) -> Self {
        Self {
            rth_threshold_pct,
            critical_threshold_pct,
            low_warning_pct,
        }
    }

    /// 根据剩余电量（%）输出告警。
    pub fn evaluate(&self, remaining_pct: f32) -> BatteryAdvisory {
        if remaining_pct <= self.critical_threshold_pct {
            BatteryAdvisory::Critical
        } else if remaining_pct <= self.rth_threshold_pct {
            BatteryAdvisory::RthTrigger
        } else if remaining_pct <= self.low_warning_pct {
            BatteryAdvisory::Low
        } else {
            BatteryAdvisory::Nominal
        }
    }
}
/// 允许解锁起飞（arm）所需的前置信号。
#[derive(Debug, Clone, Copy)]
pub struct ArmSignals {
    /// GPS 定位质量。
    pub gps_fix: FixType,
    /// 可见卫星数。
    pub gps_satellites: u8,
    /// 剩余电量（%）。
    pub battery_pct: f32,
    /// 是否已设置起飞点（home）。
    pub home_set: bool,
    /// 看门狗是否处于 Armed（未触发）。
    pub watchdog_armed: bool,
    /// 与飞控的链路是否已建立。
    pub link_connected: bool,
}

/// 起飞前自检配置。
#[derive(Debug, Clone, Copy)]
pub struct PreArmConfig {
    /// 最低可见卫星数。
    pub min_gps_satellites: u8,
    /// 是否强制要求 3D 定位。
    pub require_fix3d: bool,
    /// 最低剩余电量（%）。
    pub min_battery_pct: f32,
    /// 是否要求已设置 home。
    pub require_home: bool,
}

impl Default for PreArmConfig {
    fn default() -> Self {
        Self {
            min_gps_satellites: 10,
            require_fix3d: true,
            min_battery_pct: 25.0,
            require_home: true,
        }
    }
}

/// 起飞前自检结果。
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PreArmStatus {
    /// 是否全部通过、允许解锁。
    pub passed: bool,
    /// 未通过的原因列表（可通过校验信息）。
    pub failures: Vec<String>,
}

impl PreArmStatus {
    pub fn is_ok(&self) -> bool {
        self.passed
    }
}

/// 起飞前自检（pre-arm check）。
#[derive(Debug, Clone, Copy)]
pub struct PreArmCheck {
    pub cfg: PreArmConfig,
}

impl PreArmCheck {
    pub fn new(cfg: PreArmConfig) -> Self {
        Self { cfg }
    }

    /// 评估一组前置信号，返回是否允许解锁及失败原因。
    pub fn evaluate(&self, s: &ArmSignals) -> PreArmStatus {
        let mut failures = Vec::new();
        if s.gps_fix != FixType::Fix3D && self.cfg.require_fix3d {
            failures.push("GPS fix is not 3D".into());
        }
        if s.gps_satellites < self.cfg.min_gps_satellites {
            failures.push(format!(
                "not enough GPS satellites: {} < {}",
                s.gps_satellites, self.cfg.min_gps_satellites
            ));
        }
        if s.battery_pct < self.cfg.min_battery_pct {
            failures.push(format!(
                "battery too low: {:.1}% < {:.1}%",
                s.battery_pct, self.cfg.min_battery_pct
            ));
        }
        if !s.home_set && self.cfg.require_home {
            failures.push("home position not set".into());
        }
        if !s.watchdog_armed {
            failures.push("watchdog is not armed (previous trip)".into());
        }
        if !s.link_connected {
            failures.push("FCU link not connected".into());
        }
        let passed = failures.is_empty();
        PreArmStatus { passed, failures }
    }
}

/// 汇总的安全评估：把看门狗、围栏、电量结合成一条“允许飞行”结论。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlightPermission {
    /// 允许正常飞行。
    Allowed,
    /// 应触发自动返航（越界或低电到 RTH 阈值）。
    ReturnHome { reason: String },
    /// 应立即降落（临界电量或严重越界）。
    LandImmediately { reason: String },
}

/// 基于当前位置、电量与看门狗状态，给出统一的飞行权限结论。
pub fn flight_permission(
    geofence: &Geofence,
    battery: &BatteryMonitor,
    pos: Vec3,
    battery_pct: f32,
    watchdog_armed: bool,
) -> FlightPermission {
    if !watchdog_armed {
        return FlightPermission::LandImmediately {
            reason: "watchdog tripped".into(),
        };
    }
    if let Some(v) = geofence.check(pos) {
        return FlightPermission::ReturnHome {
            reason: format!("geofence violation: {v:?}"),
        };
    }
    match battery.evaluate(battery_pct) {
        BatteryAdvisory::Critical => FlightPermission::LandImmediately {
            reason: "critical battery".into(),
        },
        BatteryAdvisory::RthTrigger => FlightPermission::ReturnHome {
            reason: "low battery (RTH)".into(),
        },
        _ => FlightPermission::Allowed,
    }
}

/// 记录一次安全事件（供决策层订阅与日志）。
#[derive(Debug, Clone, PartialEq)]
pub enum SafetyEvent {
    Geofence(GeofenceViolation),
    Battery(BatteryAdvisory),
    PreArmFailed { failures: Vec<String> },
}

/// 便捷：把位置与电量情况归一化为事件。
pub fn classify(
    geofence: &Geofence,
    battery: &BatteryMonitor,
    pos: Vec3,
    battery_pct: f32,
) -> Option<SafetyEvent> {
    if let Some(v) = geofence.check(pos) {
        return Some(SafetyEvent::Geofence(v));
    }
    let a = battery.evaluate(battery_pct);
    if a != BatteryAdvisory::Nominal {
        return Some(SafetyEvent::Battery(a));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn home() -> Vec3 {
        Vec3::ZERO
    }

    #[test]
    fn geofence_accepts_inside() {
        let g = Geofence::new(home(), 100.0, 50.0, 0.0);
        assert_eq!(g.check(Vec3::new(10.0, 5.0, -30.0)), None);
        assert!(g.contains(Vec3::new(10.0, 5.0, -30.0)));
    }

    #[test]
    fn geofence_rejects_radius() {
        let g = Geofence::new(home(), 100.0, 50.0, 0.0);
        let v = g.check(Vec3::new(120.0, 0.0, -10.0)).unwrap();
        assert!(matches!(v, GeofenceViolation::ExceededRadius { .. }));
    }

    #[test]
    fn geofence_rejects_high_altitude() {
        let g = Geofence::new(home(), 100.0, 50.0, 0.0);
        let v = g.check(Vec3::new(0.0, 0.0, -60.0)).unwrap(); // 60m 高
        assert!(matches!(v, GeofenceViolation::ExceededAltitude { .. }));
    }

    #[test]
    fn geofence_rejects_underground() {
        let g = Geofence::new(home(), 100.0, 50.0, 0.0);
        let v = g.check(Vec3::new(0.0, 0.0, 5.0)).unwrap(); // 向下 5m
        assert!(matches!(v, GeofenceViolation::BelowMinimumAltitude { .. }));
    }

    #[test]
    fn battery_levels() {
        let m = BatteryMonitor::default();
        assert_eq!(m.evaluate(80.0), BatteryAdvisory::Nominal);
        assert_eq!(m.evaluate(35.0), BatteryAdvisory::Low);
        assert_eq!(m.evaluate(25.0), BatteryAdvisory::RthTrigger);
        assert_eq!(m.evaluate(10.0), BatteryAdvisory::Critical);
    }

    #[test]
    fn prearm_passes_when_all_ok() {
        let chk = PreArmCheck::new(PreArmConfig::default());
        let s = ArmSignals {
            gps_fix: FixType::Fix3D,
            gps_satellites: 14,
            battery_pct: 90.0,
            home_set: true,
            watchdog_armed: true,
            link_connected: true,
        };
        let st = chk.evaluate(&s);
        assert!(st.is_ok(), "failures: {:?}", st.failures);
    }

    #[test]
    fn prearm_reports_each_failure() {
        let chk = PreArmCheck::new(PreArmConfig::default());
        let s = ArmSignals {
            gps_fix: FixType::NoFix,
            gps_satellites: 3,
            battery_pct: 10.0,
            home_set: false,
            watchdog_armed: false,
            link_connected: false,
        };
        let st = chk.evaluate(&s);
        assert!(!st.passed);
        assert_eq!(st.failures.len(), 6);
    }

    #[test]
    fn flight_permission_prioritizes_safety() {
        let g = Geofence::new(home(), 100.0, 50.0, 0.0);
        let b = BatteryMonitor::default();
        // 看门狗触发 → 立即降落优先。
        let p = flight_permission(&g, &b, Vec3::ZERO, 90.0, false);
        assert!(matches!(p, FlightPermission::LandImmediately { .. }));
        // 越界 → 返航。
        let p = flight_permission(&g, &b, Vec3::new(200.0, 0.0, -10.0), 90.0, true);
        assert!(matches!(p, FlightPermission::ReturnHome { .. }));
        // 临界电量 → 立即降落。
        let p = flight_permission(&g, &b, Vec3::ZERO, 10.0, true);
        assert!(matches!(p, FlightPermission::LandImmediately { .. }));
        // 正常 → 允许。
        let p = flight_permission(&g, &b, Vec3::ZERO, 80.0, true);
        assert_eq!(p, FlightPermission::Allowed);
    }

    #[test]
    fn geofence_default_contains_origin() {
        let g = Geofence::default();
        assert_eq!(g.max_radius_m, 1000.0);
        assert!(g.contains(Vec3::ZERO));
        assert!(g.contains(Vec3::new(100.0, 100.0, -100.0)));
    }

    #[test]
    fn battery_monitor_new_and_boundaries() {
        let m = BatteryMonitor::new(30.0, 15.0, 40.0);
        assert_eq!(m.evaluate(30.0), BatteryAdvisory::RthTrigger);
        assert_eq!(m.evaluate(15.0), BatteryAdvisory::Critical);
        assert_eq!(m.evaluate(40.0), BatteryAdvisory::Low);
        assert_eq!(m.evaluate(40.1), BatteryAdvisory::Nominal);
    }

    #[test]
    fn prearm_custom_config_and_is_ok() {
        // 自定义：不要求 3D、更少卫星、不要求 home。
        let cfg = PreArmConfig {
            min_gps_satellites: 4,
            require_fix3d: false,
            min_battery_pct: 20.0,
            require_home: false,
        };
        let chk = PreArmCheck::new(cfg);
        let s = ArmSignals {
            gps_fix: FixType::Fix2D,
            gps_satellites: 5,
            battery_pct: 50.0,
            home_set: false,
            watchdog_armed: true,
            link_connected: true,
        };
        let st = chk.evaluate(&s);
        assert!(st.is_ok());
        assert!(st.passed);
    }

    #[test]
    fn flight_permission_rth_on_low_battery() {
        let g = Geofence::default();
        let b = BatteryMonitor::default();
        // 低电（RTH 阈值内、未到临界）→ 返航。
        let p = flight_permission(&g, &b, Vec3::ZERO, 25.0, true);
        assert!(matches!(p, FlightPermission::ReturnHome { .. }));
    }

    #[test]
    fn classify_returns_safety_events() {
        let g = Geofence::new(home(), 100.0, 50.0, 0.0);
        let b = BatteryMonitor::default();
        // 越界 → Geofence 事件。
        assert!(matches!(
            classify(&g, &b, Vec3::new(200.0, 0.0, -10.0), 90.0),
            Some(SafetyEvent::Geofence(_))
        ));
        // 低电 → Battery 事件。
        assert!(matches!(
            classify(&g, &b, Vec3::ZERO, 25.0),
            Some(SafetyEvent::Battery(BatteryAdvisory::RthTrigger))
        ));
        // 一切正常 → 无事件。
        assert!(classify(&g, &b, Vec3::ZERO, 90.0).is_none());
    }
}
