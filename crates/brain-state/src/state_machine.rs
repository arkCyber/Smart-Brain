//! 大脑控制意图状态机。

use brain_core::error::{BrainError, Result};
use brain_message::Mode;

/// 大脑控制意图的顶层状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightState {
    /// 地面待命。
    Ground,
    /// 起飞中。
    TakingOff,
    /// 巡航。
    Cruising,
    /// 目标跟踪。
    Tracking,
    /// 返航中。
    ReturningHome,
    /// 降落中。
    Landing,
    /// 自动悬停（fail-safe 触发后）。
    Loitering,
}

/// 合法状态迁移。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Transition(pub FlightState, pub FlightState);

impl FlightState {
    /// 该状态对应的飞控飞行模式。
    pub fn flight_mode(&self) -> Mode {
        match self {
            FlightState::Ground => Mode::Idle,
            FlightState::TakingOff => Mode::Takeoff,
            FlightState::Cruising => Mode::Cruise,
            FlightState::Tracking => Mode::Track,
            FlightState::ReturningHome => Mode::ReturnHome,
            FlightState::Landing => Mode::Land,
            FlightState::Loitering => Mode::Loiter,
        }
    }

    /// 是否处于安全（非致命）状态。
    pub fn is_safe(&self) -> bool {
        matches!(
            self,
            FlightState::Ground | FlightState::Loitering | FlightState::Landing
        )
    }
}

/// 状态机：只允许预定义迁移，非法迁移返回错误。
#[derive(Debug, Clone)]
pub struct StateMachine {
    current: FlightState,
}

impl StateMachine {
    /// 从地面待命状态开始。
    pub fn new() -> Self {
        Self {
            current: FlightState::Ground,
        }
    }

    /// 当前状态。
    pub fn current(&self) -> FlightState {
        self.current
    }

    /// 尝试迁移。若非法则返回错误并保持原状态。
    pub fn transition(&mut self, to: FlightState) -> Result<()> {
        let allowed = match self.current {
            FlightState::Ground => matches!(to, FlightState::TakingOff | FlightState::Loitering),
            FlightState::TakingOff => matches!(
                to,
                FlightState::Cruising | FlightState::Landing | FlightState::Loitering
            ),
            FlightState::Cruising => matches!(
                to,
                FlightState::Tracking
                    | FlightState::ReturningHome
                    | FlightState::Landing
                    | FlightState::Loitering
            ),
            FlightState::Tracking => matches!(
                to,
                FlightState::Cruising
                    | FlightState::ReturningHome
                    | FlightState::Landing
                    | FlightState::Loitering
            ),
            FlightState::ReturningHome => {
                matches!(to, FlightState::Landing | FlightState::Loitering)
            }
            FlightState::Landing => matches!(to, FlightState::Ground | FlightState::Loitering),
            FlightState::Loitering => matches!(
                to,
                FlightState::Ground
                    | FlightState::TakingOff
                    | FlightState::Cruising
                    | FlightState::Landing
            ),
        };

        if allowed {
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

    /// 所有合法迁移（供测试/文档使用）。
    pub const ALLOWED: &'static [Transition] = &[
        Transition(FlightState::Ground, FlightState::TakingOff),
        Transition(FlightState::Ground, FlightState::Loitering),
        Transition(FlightState::TakingOff, FlightState::Cruising),
        Transition(FlightState::TakingOff, FlightState::Landing),
        Transition(FlightState::TakingOff, FlightState::Loitering),
        Transition(FlightState::Cruising, FlightState::Tracking),
        Transition(FlightState::Cruising, FlightState::ReturningHome),
        Transition(FlightState::Cruising, FlightState::Landing),
        Transition(FlightState::Cruising, FlightState::Loitering),
        Transition(FlightState::Tracking, FlightState::Cruising),
        Transition(FlightState::Tracking, FlightState::ReturningHome),
        Transition(FlightState::Tracking, FlightState::Landing),
        Transition(FlightState::Tracking, FlightState::Loitering),
        Transition(FlightState::ReturningHome, FlightState::Landing),
        Transition(FlightState::ReturningHome, FlightState::Loitering),
        Transition(FlightState::Landing, FlightState::Ground),
        Transition(FlightState::Landing, FlightState::Loitering),
        Transition(FlightState::Loitering, FlightState::Ground),
        Transition(FlightState::Loitering, FlightState::TakingOff),
        Transition(FlightState::Loitering, FlightState::Cruising),
        Transition(FlightState::Loitering, FlightState::Landing),
    ];
}

impl Default for StateMachine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn valid_chain() {
        let mut sm = StateMachine::new();
        sm.transition(FlightState::TakingOff).unwrap();
        sm.transition(FlightState::Cruising).unwrap();
        sm.transition(FlightState::Tracking).unwrap();
        sm.transition(FlightState::ReturningHome).unwrap();
        sm.transition(FlightState::Landing).unwrap();
        sm.transition(FlightState::Ground).unwrap();
        assert_eq!(sm.current(), FlightState::Ground);
    }

    #[test]
    fn illegal_transition_rejected() {
        let mut sm = StateMachine::new();
        // Ground -> Tracking 非法。
        assert!(sm.transition(FlightState::Tracking).is_err());
        assert_eq!(sm.current(), FlightState::Ground);
    }
}
