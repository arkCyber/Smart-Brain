# brain-autopilot

> Smart-Brain closed-loop autonomous navigation: sense -> map -> plan -> drive -> backtrack

**所属层**：第 4 层 · 闭环自主导航 —— 感知→建图→规划→驱动→回溯。

## 职责

把多个模块串成完整自主闭环，用于在仿真里验证"大脑"的自主性，也作为单元测试的确定性场景。

- `world`：二维地面真值世界（障碍物）
- `sensor`：模拟测距传感器（多束光线扫描）
- `autopilot`：控制器——建图(`brain-mapping`) → 前沿探索(`brain-nav`) → DWA 避障(`brain-planning`) → 运动积分 → 面包屑回溯(`brain-nav`)
- `car_autopilot`：阿克曼汽车闭环导航（`set_goal` 点对点驾驶、`set_goal_pose` + `use_reeds_shepp` 倒车跟随）
- `boat_autopilot`：水面艇差分 DWA + **时变水流/潮汐**（`Tide`）+ 逆流定泊 + 多点巡航
- `ais`：**AIS 报文解析**（AIVDM 类型 1/2/3 位置报告、18 B 类、5 静态/航次），
  目标经纬度可换算局部坐标喂给避碰引擎
- `colregs`：COLREGS 会遇避让规则引擎（对遇/交叉/追越/**能见度受限 Rule 19**/
  **机动船让帆船**）

## 核心 API

```rust
pub use world::World;
pub use sensor::{RangeSensor, Scan};
pub use ais::{AisMessage, AisError, NavStatus, PositionReport, StaticVoyageData, decode_ais, to_local_offset};
pub use autopilot::{Autopilot, AutopilotConfig, RunStats, StepOutcome};
pub use car_autopilot::{CarAutopilot, CarAutopilotConfig, CarRunStats};
pub use boat_autopilot::{BoatAutopilot, BoatConfig, BoatStats};
pub use colregs::{Colregs, ColregsAction, ColregsParams, EncounterType, Propulsion, VesselPose, Visibility};
```

## 用法

```rust
use brain_autopilot::{Autopilot, AutopilotConfig, World};

fn main() {
    let world = World::new(20, 20);
    let mut pilot = Autopilot::new(
        AutopilotConfig::default(),
        world.width(),
        world.height(),
        (1.0, 1.0, 0.0), // 起点 (x, y, heading)
    );
    let stats = pilot.run(&world, 600); // 建图+探索+避障+回溯
    println!("safe = {}, explored = {:.0}%", stats.safe, stats.explored_ratio * 100.0);
}
```

**水面艇 AIS 感知 + COLREGS 避让**（完整链路见 `examples/ais_colregs.rs`）：

```rust
use brain_autopilot::{Colregs, ColregsParams, VesselPose, Visibility, decode_ais, to_local_offset};

fn main() {
    // 一帧 A 类位置报告（负载由真实 AIS 电台提供，编码示例见 examples/ais_colregs.rs）
    let sentence = "!AIVDM,1,1,,B,<payload>,0*00";
    let msg = decode_ais(sentence).unwrap();
    if let Some((lon, lat)) = msg.position() {
        // 相对本船 (10°E, 20°N) 的东/北米偏移
        let off = to_local_offset(10.0, 20.0, lon, lat);
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let other = VesselPose::new(off.x, off.y, std::f32::consts::PI); // 相向
        let params = ColregsParams { visibility: Visibility::Restricted, ..Default::default() };
        println!("{:?}", Colregs::classify(own, other, &params));
    }
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-mapping`、`brain-planning`、`brain-nav`、`brain-kinematics`

> **应用案例**：`brain-node/autopilot_demo.rs`（2D 世界自主探索）、`car_driving_demo.rs`（汽车）、`boat_demo.rs`（水面艇）；示例 `examples/ais_colregs.rs`（AIS → 局部坐标 → COLREGS 避让）。
