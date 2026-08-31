//! `brain-locomotion` — 步态与全身运动控制层。
//!
//! 面向四足/双足（以及可扩展到任意足式机器人）的**动作生成**层：把“大脑”的高层
//! 运动意图（线速度 + 角速度 + 期望身高）转换为**逐腿足端落点**与**躯干姿态目标**，
//! 供下游 `brain-kinematics` 的逆运动学（IK）求解出关节角，最终驱动 `brain-robot`
//! 的 `RobotBody`。
//!
//! 相比 if-else/状态机，这里把“怎么动”沉淀为可复现、可测试的模块：
//! - [`gait`]：周期步态相位生成（Stand/Walk/Trot/Run），输出逐腿相位与摆动标志；
//! - [`foot_trajectory`]：足端轨迹（支撑相平移 + 摆动相抬升前摆）；
//! - [`controller`]：速度指令 → 足端落点 + 躯干俯仰/身高目标的闭环编排；
//! - [`contact`]：足-地接触动力学（弹簧-阻尼地面反力，判断支撑相）；
//! - [`dynamics`]：逆动力学（RNEA），把运动学与外力解析为 `[髋, 膝]` 关节力矩；
//! - [`wbc`]：全身控制（WBC），含静力学 `joint_torques` 与**全身动力学力矩**
//!   `dynamic_torques`（接触门控 + 逐腿逆动力学）。
//!
//! 本层只做“几何与相位”，不依赖真实动力学/物理引擎——真机/仿真（`brain-sim`）
//! 只需提供 `RobotBody` 即可接入同一套步态逻辑。

pub mod contact;
pub mod controller;
pub mod dynamics;
pub mod foot_trajectory;
pub mod gait;
pub mod leg_ik;
pub mod wbc;

pub use contact::{ContactConfig, ContactModel, FootContact};
pub use controller::{LocomotionCommand, LocomotionController, LocomotionOutput};
pub use dynamics::LegDynamics;
pub use foot_trajectory::FootTrajectory;
pub use gait::{GaitConfig, GaitGenerator, GaitPhase, GaitType};
pub use leg_ik::{IkError, LegIK};
pub use wbc::{WbcError, WholeBodyCommand, WholeBodyController, WholeBodyTarget};
