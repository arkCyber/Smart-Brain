//! 把 `brain-state::safety` 集成进决策循环：一个可复用的“安全监督器”。
//!
//! `SafetySupervisor` 把地理围栏、电量监视、pre-arm 自检合并成一个统一入口：
//! 决策循环每 tick 调用 [`SafetySupervisor::apply`]，根据当前位姿/电量/看门狗
//! 状态把飞控指令强制覆盖为“返航”或“紧急降落”。纯逻辑、无外部依赖，便于单测。

use brain_core::Vec3;
use brain_message::{Command, CommandTarget, Mode};
use brain_state::safety::{
    flight_permission, ArmSignals, BatteryMonitor, FlightPermission, Geofence, PreArmCheck,
    PreArmStatus,
};

/// 安全监督器给出的指令覆盖决策。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyDirective {
    /// 正常，不覆盖指令。
    Continue,
    /// 强制返航（越界或电量到 RTH 阈值）。
    ReturnHome,
    /// 强制紧急降落（临界电量或看门狗触发）。
    LandImmediately,
}

/// 安全监督器：持有围栏/电量/pre-arm 配置，为决策循环提供安全覆盖。
#[derive(Debug, Clone, Copy)]
pub struct SafetySupervisor {
    pub geofence: Geofence,
    pub battery: BatteryMonitor,
    pub pre_arm: PreArmCheck,
}

impl SafetySupervisor {
    pub fn new(geofence: Geofence, battery: BatteryMonitor, pre_arm: PreArmCheck) -> Self {
        Self {
            geofence,
            battery,
            pre_arm,
        }
    }

    /// 起飞前自检：是否允许解锁。
    pub fn can_arm(&self, signals: &ArmSignals) -> PreArmStatus {
        self.pre_arm.evaluate(signals)
    }

    /// 根据当前位姿/电量/看门狗状态，推导应采取的覆盖指令。
    pub fn directive(&self, pos: Vec3, battery_pct: f32, watchdog_armed: bool) -> SafetyDirective {
        match flight_permission(
            &self.geofence,
            &self.battery,
            pos,
            battery_pct,
            watchdog_armed,
        ) {
            FlightPermission::Allowed => SafetyDirective::Continue,
            FlightPermission::ReturnHome { .. } => SafetyDirective::ReturnHome,
            FlightPermission::LandImmediately { .. } => SafetyDirective::LandImmediately,
        }
    }

    /// 按安全权限覆盖一条指令；返回是否发生了覆盖。
    ///
    /// 当需要返航/降落时，把 `cmd.mode` 置为 `ReturnHome`/`Land` 并清空目标，
    /// 从而在决策层兜底，即使行为树/任务逻辑出现偏差也不会越出安全边界。
    pub fn apply(
        &self,
        cmd: &mut Command,
        pos: Vec3,
        battery_pct: f32,
        watchdog_armed: bool,
    ) -> bool {
        match self.directive(pos, battery_pct, watchdog_armed) {
            SafetyDirective::ReturnHome => {
                cmd.mode = Mode::ReturnHome;
                cmd.target = CommandTarget::None;
                true
            }
            SafetyDirective::LandImmediately => {
                cmd.mode = Mode::Land;
                cmd.target = CommandTarget::None;
                true
            }
            _ => false,
        }
    }
}

/// 便捷演示：展示 pre-arm、围栏越界→返航、低电→紧急降落。
pub fn run_safety_demo() {
    use brain_state::safety::{BatteryMonitor, Geofence};
    println!("\n=== 安全监督器（geofence / battery / pre-arm）集成演示 ===");

    let supervisor = SafetySupervisor::new(
        Geofence::new(Vec3::ZERO, 100.0, 50.0, 0.0),
        BatteryMonitor::default(),
        brain_state::safety::PreArmCheck::new(Default::default()),
    );

    // 1) pre-arm：全部满足才允许解锁。
    let ok = ArmSignals {
        gps_fix: brain_message::telemetry::FixType::Fix3D,
        gps_satellites: 14,
        battery_pct: 90.0,
        home_set: true,
        watchdog_armed: true,
        link_connected: true,
    };
    let st = supervisor.can_arm(&ok);
    println!(
        "  pre-arm 通过: {}（failures={:?}）",
        st.passed, st.failures
    );

    // 2) 位姿越界 → 强制返航。
    let mut cmd = Command {
        timestamp: 0,
        mode: Mode::Cruise,
        target: CommandTarget::None,
    };
    let outside = Vec3::new(200.0, 0.0, -10.0); // 超出 100m 半径
    let overridden = supervisor.apply(&mut cmd, outside, 80.0, true);
    println!("  越界: override={overridden}, mode={:?}", cmd.mode);

    // 3) 临界电量 → 紧急降落。
    let mut cmd = Command {
        timestamp: 0,
        mode: Mode::Cruise,
        target: CommandTarget::None,
    };
    let overridden = supervisor.apply(&mut cmd, Vec3::ZERO, 10.0, true);
    println!("  低电: override={overridden}, mode={:?}", cmd.mode);
}
#[cfg(test)]
mod tests {
    use super::*;
    use brain_state::safety::{BatteryMonitor, Geofence, PreArmConfig};

    fn supervisor() -> SafetySupervisor {
        SafetySupervisor::new(
            Geofence::new(Vec3::ZERO, 100.0, 50.0, 0.0),
            BatteryMonitor::default(),
            brain_state::safety::PreArmCheck::new(PreArmConfig::default()),
        )
    }

    #[test]
    fn directive_allowed_when_safe() {
        let s = supervisor();
        assert_eq!(
            s.directive(Vec3::new(10.0, 5.0, -30.0), 80.0, true),
            SafetyDirective::Continue
        );
    }

    #[test]
    fn directive_return_home_when_outside_geofence() {
        let s = supervisor();
        assert_eq!(
            s.directive(Vec3::new(200.0, 0.0, -10.0), 80.0, true),
            SafetyDirective::ReturnHome
        );
    }

    #[test]
    fn directive_land_when_critical_battery() {
        let s = supervisor();
        assert_eq!(
            s.directive(Vec3::ZERO, 10.0, true),
            SafetyDirective::LandImmediately
        );
    }

    #[test]
    fn directive_land_when_watchdog_tripped() {
        let s = supervisor();
        assert_eq!(
            s.directive(Vec3::ZERO, 90.0, false),
            SafetyDirective::LandImmediately
        );
    }

    #[test]
    fn apply_overrides_command_to_return_home() {
        let s = supervisor();
        let mut cmd = Command {
            timestamp: 1,
            mode: Mode::Cruise,
            target: CommandTarget::None,
        };
        let overridden = s.apply(&mut cmd, Vec3::new(200.0, 0.0, -10.0), 80.0, true);
        assert!(overridden);
        assert_eq!(cmd.mode, Mode::ReturnHome);
    }

    #[test]
    fn apply_leaves_command_untouched_when_safe() {
        let s = supervisor();
        let mut cmd = Command {
            timestamp: 1,
            mode: Mode::Cruise,
            target: CommandTarget::None,
        };
        let overridden = s.apply(&mut cmd, Vec3::ZERO, 90.0, true);
        assert!(!overridden);
        assert_eq!(cmd.mode, Mode::Cruise);
    }

    #[test]
    fn can_arm_reflects_pre_arm_check() {
        let s = supervisor();
        let ok = ArmSignals {
            gps_fix: brain_message::telemetry::FixType::Fix3D,
            gps_satellites: 14,
            battery_pct: 90.0,
            home_set: true,
            watchdog_armed: true,
            link_connected: true,
        };
        assert!(s.can_arm(&ok).passed);
        let bad = ArmSignals {
            gps_satellites: 2,
            ..ok
        };
        assert!(!s.can_arm(&bad).passed);
    }
}
