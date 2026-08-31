# brain-robot

> Smart-Brain embodiment abstraction: body-agnostic robot state/command interface

**所属层**：第 3.5 层 · 具身抽象层 —— 身体无关的机器人接口。

## 职责

让"大脑"适用于任意具身机器人的关键 crate：**大脑（感知/决策/规划）与身体（无人机/四足/轮式/机械臂/人形）解耦**。身体通过统一的 `RobotBody` trait 暴露自身状态并接受高层指令。

- `body`：`RobotBody` trait + `BodyCommandSink`
- `state`：`RobotKind` / `BodyState` / `BasePose` / `JointState` / `ContactState`
- `command`：`EffectorCommand` / `Task` / `TaskTarget` / `LocomotionMode`
- `mock`：`MockRobotBody`（任意形态的 SITL 仿真身体）
- `car` / `boat`：`CarBody`（阿克曼汽车）、`BoatBody`（双差速水面艇）

## 核心 API

```rust
pub use body::{BodyCommandSink, RobotBody};
pub use state::{BasePose, BodyState, ContactState, JointState, RobotKind};
pub use command::{EffectorCommand, LocomotionMode, Task, TaskTarget};
pub use mock::MockRobotBody;
pub use car::CarBody;
pub use boat::BoatBody;
```

## 用法

```rust
use brain_core::Vec3;
use brain_robot::{BodyState, EffectorCommand, LocomotionMode, MockRobotBody, RobotBody, RobotKind, Task, TaskTarget};

fn main() {
    let mut body = MockRobotBody::new(RobotKind::Aerial);
    let cmd = EffectorCommand {
        timestamp: 0,
        locomotion: LocomotionMode::Navigate,
        task: Task::NavigateTo(TaskTarget::Point(Vec3::new(10.0, 0.0, 0.0))),
    };
    body.send_command(&cmd).unwrap(); // 高层速度/导航指令
    let st: BodyState = body.read_state().unwrap();
    println!("kind = {:?}, pos = {:?}", st.kind, st.base.pose.position);
}
```

## 依赖

- 外部：`serde`、`serde_json`
- 内部：`brain-core`、`brain-kinematics`

> **应用案例**：`brain-node/embodiment.rs` 把无人机包装成 `RobotBody`；`car_driving_demo.rs` 演示汽车；`boat_demo.rs` 演示水面艇。
