//! 通用（身体无关）状态机与机器人顶层状态。
//!
//! 提供可复用的有限状态机 `Fsm<S>`（只接受预设迁移，非法迁移返回错误），
//! 以及身体无关的 `RobotState` / `RobotStateMachine`，适用于无人机、四足、
//! 轮式、机械臂、水面艇等任意具身（区别于 `state_machine` 中飞行专用的
//! `FlightState`）。保留了合法的紧急迁移（`EmergencyStop` / `Fault`）。

use brain_core::error::{BrainError, Result};

/// 一条合法状态迁移（`from -> to`）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Transition<S>(pub S, pub S);

/// 通用有限状态机：当前状态 + 允许的迁移集合。
#[derive(Debug, Clone)]
pub struct Fsm<S> {
    current: S,
    allowed: Vec<Transition<S>>,
}

impl<S> Fsm<S>
where
    S: Copy + PartialEq + Eq + std::fmt::Debug,
{
    /// 用初始状态与允许迁移集合创建。
    pub fn new(initial: S, allowed: &[Transition<S>]) -> Self {
        Self {
            current: initial,
            allowed: allowed.to_vec(),
        }
    }

    /// 当前状态。
    pub fn current(&self) -> S {
        self.current
    }

    /// 是否允许 `current -> to`。
    pub fn can(&self, to: S) -> bool {
        self.allowed
            .iter()
            .any(|&Transition(a, b)| a == self.current && b == to)
    }

    /// 尝试迁移；非法返回错误并保持原状态。
    pub fn transition(&mut self, to: S) -> Result<()> {
        if self.can(to) {
            log::debug!("state: {:?} -> {:?}", self.current, to);
            self.current = to;
            Ok(())
        } else {
            Err(BrainError::State(format!(
                "illegal transition {:?} -> {:?}",
                self.current, to
            )))
        }
    }

    /// 尝试迁移并返回**新状态**；非法返回错误并保持原状态。
    pub fn transition_to(&mut self, to: S) -> Result<S> {
        self.transition(to)?;
        Ok(self.current)
    }

    /// 从状态 `from` 出发、允许迁移到的所有目标（供上层做“可选项”决策）。
    pub fn next(&self, from: S) -> Vec<S> {
        self.allowed
            .iter()
            .filter_map(|&Transition(a, b)| if a == from { Some(b) } else { None })
            .collect()
    }

    /// 重置到指定状态（不清空迁移表）。
    pub fn reset(&mut self, initial: S) {
        self.current = initial;
    }

    /// 所有允许的迁移（供测试/文档）。
    pub fn allowed(&self) -> &[Transition<S>] {
        &self.allowed
    }
}

/// 身体无关的机器人顶层状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    /// 待命（上电就绪，可接收指令）。
    Standby,
    /// 启动 / 初始化。
    Starting,
    /// 执行主任务（航行/行走/飞行/抓取…）。
    Active,
    /// 暂停。
    Paused,
    /// 目标跟踪。
    Tracking,
    /// 返回（回巢 / 充电）。
    Returning,
    /// 故障。
    Fault,
    /// 急停（最高优先级安全态）。
    EmergencyStop,
    /// 下电。
    PowerOff,
}

impl RobotState {
    /// 稳定的机器可读名称。
    pub fn as_str(&self) -> &'static str {
        match self {
            RobotState::Standby => "standby",
            RobotState::Starting => "starting",
            RobotState::Active => "active",
            RobotState::Paused => "paused",
            RobotState::Tracking => "tracking",
            RobotState::Returning => "returning",
            RobotState::Fault => "fault",
            RobotState::EmergencyStop => "emergency_stop",
            RobotState::PowerOff => "power_off",
        }
    }

    /// 是否处于安全（非执行/非致命）状态。
    pub fn is_safe(&self) -> bool {
        matches!(
            self,
            RobotState::Standby
                | RobotState::Fault
                | RobotState::EmergencyStop
                | RobotState::PowerOff
        )
    }

    /// 由机器可读名称解析（`as_str` 的逆操作）。
    pub fn from_name(s: &str) -> Option<RobotState> {
        Some(match s {
            "standby" => RobotState::Standby,
            "starting" => RobotState::Starting,
            "active" => RobotState::Active,
            "paused" => RobotState::Paused,
            "tracking" => RobotState::Tracking,
            "returning" => RobotState::Returning,
            "fault" => RobotState::Fault,
            "emergency_stop" => RobotState::EmergencyStop,
            "power_off" => RobotState::PowerOff,
            _ => return None,
        })
    }
}

impl std::fmt::Display for RobotState {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

impl std::str::FromStr for RobotState {
    type Err = ();
    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        Self::from_name(s).ok_or(())
    }
}

/// 基于 `Fsm` 的通用机器人状态机（起始于 `Standby`）。
#[derive(Debug, Clone)]
pub struct RobotStateMachine {
    inner: Fsm<RobotState>,
}

impl RobotStateMachine {
    /// 身体无关的合法迁移集合。
    pub const ALLOWED: &'static [Transition<RobotState>] = &[
        Transition(RobotState::Standby, RobotState::Starting),
        Transition(RobotState::Standby, RobotState::PowerOff),
        Transition(RobotState::Starting, RobotState::Standby),
        Transition(RobotState::Starting, RobotState::Active),
        Transition(RobotState::Starting, RobotState::Fault),
        Transition(RobotState::Active, RobotState::Paused),
        Transition(RobotState::Active, RobotState::Tracking),
        Transition(RobotState::Active, RobotState::Returning),
        Transition(RobotState::Active, RobotState::Fault),
        Transition(RobotState::Active, RobotState::EmergencyStop),
        Transition(RobotState::Paused, RobotState::Active),
        Transition(RobotState::Paused, RobotState::Standby),
        Transition(RobotState::Paused, RobotState::Fault),
        Transition(RobotState::Tracking, RobotState::Active),
        Transition(RobotState::Tracking, RobotState::Returning),
        Transition(RobotState::Tracking, RobotState::Paused),
        Transition(RobotState::Tracking, RobotState::Fault),
        Transition(RobotState::Tracking, RobotState::EmergencyStop),
        Transition(RobotState::Returning, RobotState::Standby),
        Transition(RobotState::Returning, RobotState::Active),
        Transition(RobotState::Returning, RobotState::Fault),
        Transition(RobotState::Returning, RobotState::EmergencyStop),
        Transition(RobotState::Fault, RobotState::Standby),
        Transition(RobotState::Fault, RobotState::EmergencyStop),
        Transition(RobotState::Fault, RobotState::PowerOff),
        Transition(RobotState::EmergencyStop, RobotState::Standby),
        Transition(RobotState::EmergencyStop, RobotState::PowerOff),
        Transition(RobotState::PowerOff, RobotState::Standby),
    ];

    /// 从待命开始。
    pub fn new() -> Self {
        Self {
            inner: Fsm::new(RobotState::Standby, Self::ALLOWED),
        }
    }

    pub fn current(&self) -> RobotState {
        self.inner.current()
    }

    pub fn can(&self, to: RobotState) -> bool {
        self.inner.can(to)
    }

    pub fn transition(&mut self, to: RobotState) -> Result<()> {
        self.inner.transition(to)
    }

    /// 尝试迁移并返回新状态（转发 `Fsm::transition_to`）。
    pub fn transition_to(&mut self, to: RobotState) -> Result<RobotState> {
        self.inner.transition_to(to)
    }

    /// 从状态 `from` 出发允许迁移到的目标（转发 `Fsm::next`）。
    pub fn next(&self, from: RobotState) -> Vec<RobotState> {
        self.inner.next(from)
    }

    /// 重置到指定状态（转发 `Fsm::reset`）。
    pub fn reset(&mut self, initial: RobotState) {
        self.inner.reset(initial);
    }

    pub fn allowed(&self) -> &[Transition<RobotState>] {
        self.inner.allowed()
    }
}

impl Default for RobotStateMachine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robot_valid_chain() {
        let mut sm = RobotStateMachine::new();
        assert_eq!(sm.current(), RobotState::Standby);
        sm.transition(RobotState::Starting).unwrap();
        sm.transition(RobotState::Active).unwrap();
        sm.transition(RobotState::Tracking).unwrap();
        sm.transition(RobotState::Returning).unwrap();
        sm.transition(RobotState::Standby).unwrap();
        assert_eq!(sm.current(), RobotState::Standby);
    }

    #[test]
    fn robot_illegal_transition_rejected() {
        let mut sm = RobotStateMachine::new();
        // Standby -> Tracking 非法。
        assert!(sm.transition(RobotState::Tracking).is_err());
        assert_eq!(sm.current(), RobotState::Standby);
        assert!(sm.can(RobotState::Starting));
        assert!(!sm.can(RobotState::Tracking));
    }

    #[test]
    fn robot_emergency_and_fault_paths() {
        let mut sm = RobotStateMachine::new();
        sm.transition(RobotState::Starting).unwrap();
        sm.transition(RobotState::Active).unwrap();
        // Active -> EmergencyStop（安全兜底）。
        sm.transition(RobotState::EmergencyStop).unwrap();
        assert_eq!(sm.current(), RobotState::EmergencyStop);
        // 急停后只能待命或下电。
        sm.transition(RobotState::Standby).unwrap();
        assert_eq!(sm.current(), RobotState::Standby);
    }

    #[test]
    fn robot_state_safe_and_labels() {
        assert_eq!(RobotState::Active.as_str(), "active");
        assert_eq!(RobotState::EmergencyStop.as_str(), "emergency_stop");
        assert!(RobotState::Standby.is_safe());
        assert!(RobotState::EmergencyStop.is_safe());
        assert!(RobotState::Fault.is_safe());
        assert!(!RobotState::Active.is_safe());
        assert!(!RobotState::Tracking.is_safe());
    }

    /// 证明 `Fsm` 是真正通用的：可用于任意自定义状态枚举。
    #[test]
    fn fsm_is_generic_over_any_enum() {
        #[derive(Debug, Clone, Copy, PartialEq, Eq)]
        enum Traffic {
            Red,
            Yellow,
            Green,
        }
        let allowed = [
            Transition(Traffic::Red, Traffic::Green),
            Transition(Traffic::Green, Traffic::Yellow),
            Transition(Traffic::Yellow, Traffic::Red),
        ];
        let mut fsm = Fsm::new(Traffic::Red, &allowed);
        fsm.transition(Traffic::Green).unwrap();
        fsm.transition(Traffic::Yellow).unwrap();
        fsm.transition(Traffic::Red).unwrap();
        assert_eq!(fsm.current(), Traffic::Red);
        // 非法：Green -> Red（需经过 Yellow）。
        let mut fsm2 = Fsm::new(Traffic::Red, &allowed);
        fsm2.transition(Traffic::Green).unwrap();
        assert!(fsm2.transition(Traffic::Red).is_err());
    }

    #[test]
    fn fsm_transition_to_next_and_reset() {
        let mut sm = RobotStateMachine::new();
        // transition_to 返回新状态。
        let s = sm.transition_to(RobotState::Starting).unwrap();
        assert_eq!(s, RobotState::Starting);
        // next(Active) 给出从 Active 出发的可选项。
        let nexts = sm.next(RobotState::Active);
        assert!(nexts.contains(&RobotState::Tracking));
        assert!(nexts.contains(&RobotState::Returning));
        assert!(nexts.contains(&RobotState::EmergencyStop));
        // reset 回到指定状态。
        sm.reset(RobotState::Standby);
        assert_eq!(sm.current(), RobotState::Standby);
    }

    #[test]
    fn robot_state_display_and_parse_roundtrip() {
        for s in [
            RobotState::Standby,
            RobotState::Starting,
            RobotState::Active,
            RobotState::Paused,
            RobotState::Tracking,
            RobotState::Returning,
            RobotState::Fault,
            RobotState::EmergencyStop,
            RobotState::PowerOff,
        ] {
            assert_eq!(format!("{s}"), s.as_str());
            assert_eq!(RobotState::from_name(&s.to_string()), Some(s));
            let parsed: RobotState = s.to_string().parse().unwrap();
            assert_eq!(parsed, s);
        }
        assert_eq!(RobotState::from_name("nope"), None);
        assert!("nope".parse::<RobotState>().is_err());
    }

    /// 迁移表一致性：每个状态都应有至少一条出边与一条入边（无孤立/死状态）。
    #[test]
    fn allowed_table_consistent_no_dead_states() {
        let states = [
            RobotState::Standby,
            RobotState::Starting,
            RobotState::Active,
            RobotState::Paused,
            RobotState::Tracking,
            RobotState::Returning,
            RobotState::Fault,
            RobotState::EmergencyStop,
            RobotState::PowerOff,
        ];
        for s in states {
            let has_out = RobotStateMachine::ALLOWED
                .iter()
                .any(|&Transition(a, _)| a == s);
            let has_in = RobotStateMachine::ALLOWED
                .iter()
                .any(|&Transition(_, b)| b == s);
            assert!(has_out, "{s:?} 缺少出边（死状态）");
            assert!(has_in, "{s:?} 缺少入边（不可达）");
        }
    }
}
