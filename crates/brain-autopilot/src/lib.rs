//! `brain-autopilot` — 闭环自主导航（室内探索/避障/回溯）。
//!
//! 把多个模块串成“感知→建图→规划→驱动→回溯”的完整闭环：
//! - `world`：二维地面真值世界（障碍物）
//! - `sensor`：模拟测距传感器（多束光线扫描）
//! - `autopilot`：控制器——建图(`brain-mapping`) → 前沿探索(`brain-nav`)
//!   → DWA 局部避障(`brain-planning`) → 运动积分 → 面包屑回溯(`brain-nav`)
//!
//! 用于在仿真里验证“大脑”的自主性，也作为单元测试的确定性场景。

pub mod autopilot;
pub mod boat_autopilot;
pub mod car_autopilot;
pub mod colregs;
pub mod sensor;
pub mod world;

pub use autopilot::{Autopilot, AutopilotConfig, RunStats, StepOutcome};
pub use boat_autopilot::{BoatAutopilot, BoatConfig, BoatStats};
pub use car_autopilot::{CarAutopilot, CarAutopilotConfig, CarRunStats};
pub use colregs::{Colregs, ColregsAction, ColregsParams, EncounterType, VesselPose};
pub use sensor::{RangeSensor, Scan};
pub use world::World;
