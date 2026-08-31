# brain-state

> Smart-Brain flight state machine and fail-safe watchdog

**所属层**：第 3 层 · 决策/中间件层 —— 状态机 + Fail-safe 看门狗。

## 职责

状态机描述大脑对飞控的控制意图（待命/起飞/巡航/跟踪/返航/降落/悬停）；看门狗负责监督大脑自身是否"卡死"——若心跳中断超过阈值（默认 50ms），立即剥夺大脑控制权并强制进入自动悬停（Loiter），实现安全兜底。

- `StateMachine` / `FlightState`：飞行状态机
- `FailsafeWatchdog` / `WatchdogStatus` / `FailsafeEvent`：心跳看门狗
- `RobotStateMachine` / `RobotState` / `Fsm`：通用机器人状态机（站立/行走/操作/抓取）
- `SafetyGuard`：安全监督器（围栏 / 电量 / pre-arm 阈值兜底）

## 核心 API

```rust
pub use failsafe::{FailsafeEvent, FailsafeWatchdog, WatchdogStatus};
pub use robot_state::{Fsm, RobotState, RobotStateMachine, Transition};
pub use safety::{SafetyGuard, SafetyLimits, SafetyViolation};
pub use state_machine::{FlightState, StateMachine};
```

## 用法

```rust
use brain_state::{FailsafeEvent, FailsafeWatchdog};

fn main() {
    let mut wd = FailsafeWatchdog::new(50); // 心跳阈值 50ms
    loop {
        wd.feed(now_ms); // 每个心跳周期喂狗
        match wd.check(now_ms) {
            Some(FailsafeEvent::Trip { .. }) => println!("force LOITER"),
            _ => {}
        }
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-message`

> **应用案例**：`brain-node/safety_guard.rs`（安全监督器接入任务循环）、顶层 README 的 Fail-safe 演示段。
