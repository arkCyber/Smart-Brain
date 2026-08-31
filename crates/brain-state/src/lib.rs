//! `brain-state` — 飞行状态机与 Fail-safe 看门狗。
//!
//! 状态机描述大脑对飞控的控制意图（待命/起飞/巡航/跟踪/返航/降落/悬停），
//! 看门狗负责监督大脑自身是否“卡死”：若心跳中断超过阈值（默认 50ms），
//! 立即剥夺大脑控制权并强制进入自动悬停（Loiter），实现安全兜底。

pub mod failsafe;
pub mod robot_state;
pub mod safety;
pub mod state_machine;

pub use failsafe::{FailsafeEvent, FailsafeWatchdog, WatchdogStatus};
pub use robot_state::{Fsm, RobotState, RobotStateMachine, Transition};
pub use safety::{
    classify, flight_permission, ArmSignals, BatteryAdvisory, BatteryMonitor, FlightPermission,
    Geofence, GeofenceViolation, PreArmCheck, PreArmConfig, PreArmStatus, SafetyEvent,
};
pub use state_machine::{FlightState, StateMachine, Transition as FlightTransition};
