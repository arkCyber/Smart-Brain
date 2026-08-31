# brain-sim

> Smart-Brain simulation layer: pluggable Simulator backend contract + deterministic in-process mock (Gazebo/AirSim-agnostic SITL)

**所属层**：仿真层 —— 可插拔仿真后端契约（SITL 底座）。

## 职责

对应真机部署第一阶段"先把 90% 测试跑在仿真里"：提供一个**可插拔的仿真后端契约**（`Simulator` trait），让同一套"大脑"逻辑无需改动即可接入：

- 默认确定性进程内 `MockSimulator`（2D 栅格世界 + 速度积分 + 测距 + 目标检测），开箱即用、离线可测
- 未来真机阶段可实现同一 trait 对接 Gazebo / AirSim / Isaac / 自研物理引擎，大脑代码零改动

仿真对象是**身体**（`RobotBody` 的速度指令）而非具体链路，因此天然与 `brain-transport` / `brain-zenoh` 解耦。

## 核心 API

```rust
pub use sim::{Simulator, SimState, SimDetection};
pub use mock::MockSimulator;
```

## 用法

```rust
use brain_core::Vec3;
use brain_sim::{MockSimulator, Simulator, SimState};

fn main() {
    let mut sim = MockSimulator::new(100, 100, 1.0); // 宽/高/分辨率
    sim.set_velocity_command(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO).unwrap();
    sim.step(0.1).unwrap(); // 推进仿真（返回 Result）
    let state: SimState = sim.state();
    println!("pos = {:?}, detections = {}", state.robot_pose.position, sim.detections().len());
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-robot`

> **应用案例**：`brain-node/indoor.rs`（室内感知仿真闭环）。
