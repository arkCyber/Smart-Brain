# brain-locomotion

> Smart-Brain locomotion layer: gait phase generation, foot trajectories, and velocity->foot-target control for legged robots

**所属层**：第 3.5 层 · 步态/全身控制层 —— 动作生成。

## 职责

面向四足/双足的**动作生成**层：把"大脑"的高层运动意图（线速度 + 角速度 + 期望身高）转换为**逐腿足端落点**与**躯干姿态目标**，供下游 `brain-kinematics` 的 IK 求解关节角，最终驱动 `brain-robot::RobotBody`。

- `gait`：周期步态相位生成（Stand/Walk/Trot/Run）
- `foot_trajectory`：足端轨迹（支撑相平移 + 摆动相抬升前摆）
- `controller`：速度指令 → 足端落点 + 躯干俯仰/身高目标
- `leg_ik`：腿部逆运动学
- `dynamics`：逆动力学（RNEA，运动学 + 外力 → 髋/膝关节力矩）
- `contact`：**足-地接触动力学**（弹簧-阻尼地面反力，判断支撑相）
- `wbc`：全身控制（`WholeBodyController`，含 `joint_torques` 静力学与
  `dynamic_torques` **全身动力学力矩**：接触门控 + 逐腿逆动力学）

## 核心 API

```rust
pub use controller::{LocomotionCommand, LocomotionController, LocomotionOutput};
pub use dynamics::LegDynamics;
pub use foot_trajectory::FootTrajectory;
pub use gait::{GaitConfig, GaitGenerator, GaitPhase, GaitType};
pub use leg_ik::{LegIK, IkError};
pub use contact::{ContactConfig, ContactModel, FootContact};
pub use wbc::{WholeBodyCommand, WholeBodyController, WholeBodyTarget, WbcError};
```

## 用法

```rust
use brain_locomotion::{GaitConfig, GaitGenerator, GaitType};

fn main() {
    let mut gen = GaitGenerator::new(GaitConfig::quadruped(GaitType::Trot, 2.0));
    gen.advance(0.016); // 推进 16ms
    let phase = gen.snapshot(); // 逐腿相位与摆动标志
    println!("legs = {}, swinging = {}", phase.legs.len(), phase.swing_count());
}
```

**足-地接触 + 全身动力学力矩**（完整演示见 `examples/dynamic_contact.rs`）：

```rust
use brain_core::Vec3;
use brain_locomotion::{ContactConfig, ContactModel, LegIK, WholeBodyCommand, WholeBodyController};

fn main() {
    let hips = vec![
        Vec3::new(0.2, 0.15, 0.0), Vec3::new(0.2, -0.15, 0.0),
        Vec3::new(-0.2, 0.15, 0.0), Vec3::new(-0.2, -0.15, 0.0),
    ];
    let ctl = WholeBodyController::quadruped(LegIK::new(0.4, 0.4), hips, 0.4, 400.0).unwrap();
    let target = ctl.solve(&WholeBodyCommand::stand(ctl.nominal_height)).unwrap();
    let n = ctl.leg_count();
    let q = target.joint_targets.clone();
    let qd = vec![[0.0f32; 2]; n];
    let qdd = vec![[0.0f32; 2]; n];
    let contact = ContactModel::new(ContactConfig::default());
    let tau = ctl
        .dynamic_torques(&target, &q, &qd, &qdd, ctl.nominal_height - 0.005, &contact, Vec3::new(0.0, 0.0, -9.81))
        .unwrap();
    println!("dynamic [髋, 膝] torques: {tau:?}");
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-robot`、`brain-kinematics`

> **应用案例**：`brain-node/locomotion_sim_demo.rs`（步态 + 腿部 IK + 逆动力学仿真）；示例 `examples/dynamic_contact.rs`（足-地接触 + 全身动力学力矩）。
