//! `brain-autopilot` — 闭环自主导航（室内探索/避障/回溯 + 汽车/水面艇）。
//!
//! 把多个模块串成“感知→建图→规划→驱动→回溯”的完整闭环：
//! - `world`：二维地面真值世界（障碍物）
//! - `sensor`：模拟测距传感器（多束光线扫描）
//! - `autopilot`：控制器——建图(`brain-mapping`) → 前沿探索(`brain-nav`)
//!   → DWA 局部避障(`brain-planning`) → 运动积分 → 面包屑回溯(`brain-nav`)
//! - `car_autopilot`：阿克曼汽车闭环导航（Dubins/Reeds-Shepp + Ackermann DWA + 倒车）
//! - `boat_autopilot`：水面艇闭环导航（时变水流/潮汐 + 多点巡航 + 逆流定泊）
//! - `ais`：AIS 报文解析（`!AIVDM` 类型 1/2/3/18/5）
//! - `colregs`：COLREGS 会遇避让规则引擎（对遇/交叉/追越 + 能见度受限 + 机动船让帆船）
//!
//! 用于在仿真里验证“大脑”的自主性，也作为单元测试的确定性场景。

pub mod ais;
pub mod autopilot;
pub mod boat_autopilot;
pub mod car_autopilot;
pub mod colregs;
pub mod sensor;
pub mod world;

pub use ais::{
    AisError, AisMessage, NavStatus, PositionReport, StaticVoyageData, decode as decode_ais,
    to_local_offset,
};
pub use autopilot::{Autopilot, AutopilotConfig, RunStats, StepOutcome};
pub use boat_autopilot::{BoatAutopilot, BoatConfig, BoatStats};
pub use car_autopilot::{CarAutopilot, CarAutopilotConfig, CarRunStats};
pub use colregs::{
    Colregs, ColregsAction, ColregsParams, EncounterType, Propulsion, VesselPose, Visibility,
};
pub use sensor::{RangeSensor, Scan};
pub use world::World;
