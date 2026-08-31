# brain-kinematics

> Smart-Brain kinematics: pose transforms, forward/inverse kinematics, Jacobian

**所属层**：第 3.5 层 · 运动学层 —— 正/逆运动学、雅可比、车辆运动学。

## 职责

面向四足/机械臂/人形等具身机器人：提供串联关节链的正运动学（FK）、几何雅可比与基于雅可比转置的逆运动学（IK）。无人机虽用不到关节，但操作臂与足式机器人必需。仅依赖 `brain-core` 的数学原语。

- `chain`：`KinematicChain` / `Link`（串联关节链 FK / 雅可比 / IK）
- `bicycle`：`BicycleModel` / `BicycleState`（自行车 / Ackermann 车辆运动学）

## 核心 API

```rust
pub use chain::{KinematicChain, Link};
pub use bicycle::{BicycleModel, BicycleState, norm_angle};
```

## 用法

```rust
use brain_kinematics::{BicycleModel, BicycleState};

fn main() {
    let model = BicycleModel::new(2.6, 0.6, 0.8); // 轴距/最大转向/转向速率
    let mut state = BicycleState::new(0.0, 0.0, 0.0);
    for _ in 0..50 {
        state = model.step(&state, 4.0, 0.3, 0.1); // 线速度 4、转向 0.3、dt 0.1
    }
    println!("pos = ({:.1}, {:.1}) heading = {:.2}", state.x, state.y, state.theta);
}
```

## 依赖

- 外部：无
- 内部：`brain-core`

> **应用案例**：`brain-autopilot::CarAutopilot` 用 `BicycleModel` 做阿克曼汽车的运动学积分；`brain-locomotion` 用 `KinematicChain` 做腿部 IK。
