//! 全身控制（Whole-Body Control, WBC）：把**躯干位姿 + 足端力分配**解析为逐腿关节角。
//!
//! 这是 `brain-locomotion` 的“最高层”动作编排：下游 [`crate::controller`] 从速度指令
//! 生成足端轨迹，本模块再往前一步——给定期望的躯干高度/姿态（以及重心横向偏移）和
//! 期望的足底力分配（如四足站立时的体重分布），把它映射为
//! 每条腿的关节角目标（经 [`crate::leg_ik::LegIK`]）与足底法向力向量。
//!
//! 模型（机体系：x 前向、y 侧向、z 向上，躯干原点位于髋部所在平面；地面在下方）：
//! - 各足在世界系中**固定于地面**（站立/支撑）；
//! - 期望躯干位姿 `T`（高度 + roll/pitch/yaw + COM 平移）确定后，某足相对躯干的期望
//!   位置 = `T⁻¹ · F_world`（世界→机体逆变换，用 [`Pose::inverse_transform_point`]）；
//! - 用该机体坐标减去对应髋部位置喂给平面双连杆 IK，得 `[髋俯仰, 膝]`；
//! - 体重按 `force_weights` 分配到各足（无权重时均匀分配），输出足底法向力。

use brain_core::{Pose, Vec3};

use crate::contact::ContactModel;
use crate::dynamics::LegDynamics;
use crate::leg_ik::{IkError, LegIK};

/// 全身控制求解错误。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WbcError {
    /// 输入非法（含 NaN/Inf、负权重、权重总和非正等）。
    Invalid(&'static str),
    /// 腿数量不一致（髋 / 名义足端 / 力权重三者长度不符）。
    LegCountMismatch {
        hips: usize,
        feet: usize,
        forces: usize,
    },
    /// 某条腿的期望足端超出 IK 可达范围。
    Unreachable { leg: usize, error: IkError },
}

impl core::fmt::Display for WbcError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            WbcError::Invalid(msg) => write!(f, "invalid whole-body command: {msg}"),
            WbcError::LegCountMismatch { hips, feet, forces } => write!(
                f,
                "leg count mismatch: hips={hips}, feet={feet}, forces={forces}"
            ),
            WbcError::Unreachable { leg, error } => {
                write!(f, "leg {leg} unreachable: {error:?}")
            }
        }
    }
}

impl std::error::Error for WbcError {}

/// 全身控制命令：期望的躯干位姿 + 足底力分配。
///
/// 由“大脑”的高层规划/平衡模块发出（例如稳定站姿、俯身、横向重心偏移、特定足发力）。
#[derive(Debug, Clone)]
pub struct WholeBodyCommand {
    /// 期望躯干高度（m，躯干原点距地面的高度）。
    pub height: f32,
    /// 期望躯干横滚（rad）。
    pub roll: f32,
    /// 期望躯干俯仰（rad，前倾为正）。
    pub pitch: f32,
    /// 期望躯干偏航（rad）。
    pub yaw: f32,
    /// 期望重心（COM）在水平面的偏移（机体系 x 前向 / y 侧向，z 忽略），用于压弯/前倾时
    /// 让躯干在水平方向平移而保持足底不动。
    pub com_offset: Vec3,
    /// 各足的期望载荷权重（≥0）。空向量 = 均匀分配；必须与腿数一致且总和 > 0。
    pub force_weights: Vec<f32>,
}

impl WholeBodyCommand {
    /// 期望位姿（位置 + 欧拉角），力权重留空以均匀分配。
    pub fn new(position: Vec3, roll: f32, pitch: f32, yaw: f32) -> Self {
        Self {
            height: position.z,
            roll,
            pitch,
            yaw,
            com_offset: Vec3::new(position.x, position.y, 0.0),
            force_weights: Vec::new(),
        }
    }

    /// 直立站立：高度 `height`、零姿态、重心居中、均匀受力。
    pub fn stand(height: f32) -> Self {
        Self::new(Vec3::new(0.0, 0.0, height), 0.0, 0.0, 0.0)
    }

    /// 带逐足力权重的版本。
    pub fn with_force_weights(mut self, weights: Vec<f32>) -> Self {
        self.force_weights = weights;
        self
    }
}

impl Default for WholeBodyCommand {
    fn default() -> Self {
        Self::stand(0.4)
    }
}

/// 一次全身控制求解的输出。
#[derive(Debug, Clone)]
pub struct WholeBodyTarget {
    /// 每条腿的关节角目标 `[髋俯仰, 膝]`（rad），顺序与腿序一致。
    pub joint_targets: Vec<[f32; 2]>,
    /// 每条腿的**期望足端位置**（机体系，相对躯干原点，x 前向 / y 侧向 / z 向上）。
    /// 供下游关节控制器/`RobotBody` 使用。
    pub foot_targets: Vec<Vec3>,
    /// 每条腿的**足底法向力**（机体系，指向足底向上的支持力）。
    pub foot_forces: Vec<Vec3>,
}

impl WholeBodyTarget {
    /// 躯干所受**净合力**与**净合力矩**（绕躯干原点，机体系）。
    ///
    /// 把逐足法向力聚合成一个合力（`Σ F`）与一个合力矩（`Σ r × F`，`r` 为各足
    /// 相对躯干原点的位置）。用于静态平衡校验：站姿稳定时合力应平衡体重、
    /// 合力矩应接近零；偏置（某一侧更重）时合力矩不为零，可作为防倾覆判定依据。
    pub fn trunk_wrench(&self) -> (Vec3, Vec3) {
        let mut net_force = Vec3::ZERO;
        let mut net_moment = Vec3::ZERO;
        for (p, f) in self.foot_targets.iter().zip(&self.foot_forces) {
            net_force = net_force.add(*f);
            net_moment = net_moment.add(p.cross(*f));
        }
        (net_force, net_moment)
    }
}

/// 全身控制器：把躯干位姿命令解析为逐腿关节角 + 足底力。
#[derive(Debug)]
pub struct WholeBodyController {
    /// 平面双连杆腿模型（每条腿共用）。
    pub ik: LegIK,
    /// 逐腿动力学参数（质量/惯量），用于动态力矩求解。
    pub leg_dynamics: LegDynamics,
    /// 各髋部位置（机体系）。
    pub hip_positions: Vec<Vec3>,
    /// 各足名义位置（机体系，躯干水平、位于 `nominal_height` 时的足端）。
    pub foot_positions: Vec<Vec3>,
    /// 名义躯干高度（m，躯干原点距地面）。
    pub nominal_height: f32,
    /// 机器人体重（N），用于足底法向力分配。
    pub total_weight: f32,
}

impl WholeBodyController {
    /// 创建控制器。要求 `hip_positions` 与 `foot_positions` 长度一致且非空，
    /// `total_weight >= 0`，否则返回 [`WbcError::Invalid`]。
    pub fn new(
        ik: LegIK,
        hip_positions: Vec<Vec3>,
        foot_positions: Vec<Vec3>,
        nominal_height: f32,
        total_weight: f32,
    ) -> Result<Self, WbcError> {
        if hip_positions.len() != foot_positions.len() {
            return Err(WbcError::LegCountMismatch {
                hips: hip_positions.len(),
                feet: foot_positions.len(),
                forces: 0,
            });
        }
        if hip_positions.is_empty() {
            return Err(WbcError::Invalid("no legs configured"));
        }
        if !nominal_height.is_finite() || total_weight < 0.0 || !total_weight.is_finite() {
            return Err(WbcError::Invalid("nominal_height/total_weight invalid"));
        }
        Ok(Self {
            ik,
            leg_dynamics: LegDynamics::from_links(ik.l1, ik.l2),
            hip_positions,
            foot_positions,
            nominal_height,
            total_weight,
        })
    }

    /// 便捷构造四足控制器：髋部 `hip_positions`，名义足端为髋部正下方 `leg_reach` 处
    /// （`z = hip.z - leg_reach`），名义躯干高度 = `leg_reach`（使足端触地）。
    pub fn quadruped(
        ik: LegIK,
        hip_positions: Vec<Vec3>,
        leg_reach: f32,
        total_weight: f32,
    ) -> Result<Self, WbcError> {
        let foot_positions = hip_positions
            .iter()
            .map(|h| Vec3::new(h.x, h.y, h.z - leg_reach))
            .collect();
        Self::new(ik, hip_positions, foot_positions, leg_reach, total_weight)
    }

    /// 腿的数量。
    pub fn leg_count(&self) -> usize {
        self.hip_positions.len()
    }

    /// 把体重按 `weights` 分配到各足，返回逐足法向力向量（z 向上）。
    ///
    /// - `weights` 为空：均匀分配（每足 `total_weight / n`）。
    /// - 否则长度必须等于腿数，且每项有限、非负、总和 > 0。
    pub fn distribute_forces(&self, weights: &[f32]) -> Result<Vec<Vec3>, WbcError> {
        let n = self.hip_positions.len();
        let per = |fz: f32| Vec3::new(0.0, 0.0, fz);

        if weights.is_empty() {
            let each = self.total_weight / n as f32;
            return Ok((0..n).map(|_| per(each)).collect());
        }
        if weights.len() != n {
            return Err(WbcError::LegCountMismatch {
                hips: n,
                feet: n,
                forces: weights.len(),
            });
        }
        if weights.iter().any(|&w| !w.is_finite() || w < 0.0) {
            return Err(WbcError::Invalid(
                "force weights must be finite and non-negative",
            ));
        }
        let sum: f32 = weights.iter().sum();
        if sum <= 0.0 {
            return Err(WbcError::Invalid(
                "force weights must sum to a positive value",
            ));
        }
        Ok(weights
            .iter()
            .map(|&w| per(self.total_weight * w / sum))
            .collect())
    }

    /// 求解全身控制命令：躯干位姿 → 逐腿关节角 + 足端位置 + 足底力。
    pub fn solve(&self, cmd: &WholeBodyCommand) -> Result<WholeBodyTarget, WbcError> {
        // 输入健壮性：拒绝 NaN/Inf，避免把坏数据带进运动求解。
        if !cmd.height.is_finite()
            || !cmd.roll.is_finite()
            || !cmd.pitch.is_finite()
            || !cmd.yaw.is_finite()
            || !cmd.com_offset.is_finite()
        {
            return Err(WbcError::Invalid("command contains non-finite values"));
        }

        let foot_forces = self.distribute_forces(&cmd.force_weights)?;
        let n = self.hip_positions.len();

        // 期望躯干位姿（含 COM 水平偏移）。
        let pose = Pose::new_euler(
            Vec3::new(cmd.com_offset.x, cmd.com_offset.y, cmd.height),
            cmd.roll,
            cmd.pitch,
            cmd.yaw,
        );

        let mut joint_targets = Vec::with_capacity(n);
        let mut foot_targets = Vec::with_capacity(n);
        for i in 0..n {
            // 世界系足端：躯干水平且位于 nominal_height 时足端落地点固定不动。
            let foot_world = Vec3::new(
                self.foot_positions[i].x,
                self.foot_positions[i].y,
                self.nominal_height + self.foot_positions[i].z,
            );
            // 世界→机体：期望躯干位姿下的足端机体坐标。
            let target_body = pose.inverse_transform_point(foot_world);
            if !target_body.is_finite() {
                return Err(WbcError::Invalid("non-finite foot target produced"));
            }
            let q = self
                .ik
                .solve_from_hip(self.hip_positions[i], target_body)
                .map_err(|error| WbcError::Unreachable { leg: i, error })?;
            joint_targets.push(q);
            foot_targets.push(target_body);
        }

        Ok(WholeBodyTarget {
            joint_targets,
            foot_targets,
            foot_forces,
        })
    }

    /// 把一次 WBC 求解结果的逐足力**下沉为关节力矩**（静力学，`τ = Jᵀ·f`）。
    ///
    /// 对每条腿用其关节角 `q` 与该足足底力，经平面双连杆雅可比转置求得
    /// `[髋, 膝]` 关节力矩。可用于：估计关节负载、校验关节力矩是否超限、或作为
    /// 力控/阻抗控制的力矩前馈。要求 `target` 的关节角与足底力长度一致。
    pub fn joint_torques(&self, target: &WholeBodyTarget) -> Result<Vec<[f32; 2]>, WbcError> {
        let n = self.hip_positions.len();
        if target.joint_targets.len() != n || target.foot_forces.len() != n {
            return Err(WbcError::LegCountMismatch {
                hips: n,
                feet: target.foot_targets.len(),
                forces: target.foot_forces.len(),
            });
        }
        Ok(target
            .joint_targets
            .iter()
            .zip(&target.foot_forces)
            .map(|(q, f)| self.ik.static_torques(*q, *f))
            .collect())
    }

    /// 覆盖默认的逐腿动力学参数（质量/惯量分布）。默认由杆长
    /// （[`LegDynamics::from_links`]）估算，真机可用标定值替换。
    pub fn with_dynamics(mut self, leg_dynamics: LegDynamics) -> Self {
        self.leg_dynamics = leg_dynamics;
        self
    }

    /// 全身动力学力矩（**整合接触门控 + 逐腿逆动力学**）。
    ///
    /// 相比 [`Self::joint_torques`]（纯静力学 `τ = Jᵀ·f`），本方法：
    /// 1. 用 [`ContactModel`] 依据当前躯干高度判断每条腿是否触地（足-地接触动力学）；
    /// 2. 触地腿承担 WBC 分配的足底力，摆动腿受力为零；
    /// 3. 用 [`LegDynamics::inverse_dynamics`] 计入惯性/科氏/重力，得到 `[髋, 膝]` 力矩。
    ///
    /// - `q`/`qd`/`qdd`：各腿关节角/角速度/角加速度（机体系，`[髋俯仰, 膝]`）；
    /// - `trunk_height`：当前躯干高度（世界系，m）——用于把机体系足端坐标换算到
    ///   世界高度喂给接触模型；
    /// - `contact`：足-地接触模型（`ground_z` 应为 0）；
    /// - `gravity`：重力加速度（机体系，通常 `(0,0,-9.81)`）。
    #[allow(clippy::too_many_arguments)]
    pub fn dynamic_torques(
        &self,
        target: &WholeBodyTarget,
        q: &[[f32; 2]],
        qd: &[[f32; 2]],
        qdd: &[[f32; 2]],
        trunk_height: f32,
        contact: &ContactModel,
        gravity: Vec3,
    ) -> Result<Vec<[f32; 2]>, WbcError> {
        let n = self.hip_positions.len();
        if target.foot_targets.len() != n
            || target.foot_forces.len() != n
            || q.len() != n
            || qd.len() != n
            || qdd.len() != n
        {
            return Err(WbcError::LegCountMismatch {
                hips: n,
                feet: target.foot_targets.len(),
                forces: target.foot_forces.len(),
            });
        }
        let mut torques = Vec::with_capacity(n);
        for i in 0..n {
            // 机体系足端 z + 躯干高度 ≈ 世界系足端高度（水平躯干假设）。
            let foot_world_z = trunk_height + target.foot_targets[i].z;
            let c = contact.contact(Vec3::new(0.0, 0.0, foot_world_z), Vec3::ZERO);
            let foot_force = if c.in_contact {
                target.foot_forces[i]
            } else {
                Vec3::ZERO
            };
            torques.push(
                self.leg_dynamics
                    .inverse_dynamics(q[i], qd[i], qdd[i], foot_force, gravity),
            );
        }
        Ok(torques)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use brain_core::Quat;

    /// 标准四足髋位（前左/前右/后左/后右，四角对称）。髋部位于躯干原点平面（z=0）。
    fn hips() -> Vec<Vec3> {
        vec![
            Vec3::new(0.2, 0.15, 0.0),
            Vec3::new(0.2, -0.15, 0.0),
            Vec3::new(-0.2, 0.15, 0.0),
            Vec3::new(-0.2, -0.15, 0.0),
        ]
    }

    fn ctl() -> WholeBodyController {
        WholeBodyController::quadruped(LegIK::new(0.4, 0.4), hips(), 0.4, 400.0).unwrap()
    }

    #[test]
    fn identity_command_recovers_nominal_feet() {
        let c = ctl();
        // 直立、名义高度、零姿态、均匀受力。
        let t = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        // 足端应回到名义位置（相对躯干原点）。
        for i in 0..c.leg_count() {
            let expect = c.foot_positions[i];
            let got = t.foot_targets[i];
            assert!(
                (got.x - expect.x).abs() < 1e-4
                    && (got.y - expect.y).abs() < 1e-4
                    && (got.z - expect.z).abs() < 1e-4,
                "leg {i}: got {got:?} expect {expect:?}"
            );
        }
        // IK 一致：关节角正解应回到目标足端。
        for i in 0..c.leg_count() {
            let f = c.ik.forward(t.joint_targets[i]);
            let rel = t.foot_targets[i].sub(c.hip_positions[i]);
            assert!(
                (f.x - rel.x).abs() < 1e-3 && (f.z - rel.z).abs() < 1e-3,
                "leg {i}"
            );
        }
    }

    #[test]
    fn raising_trunk_lowers_feet_in_body_frame() {
        let c = ctl();
        let higher = c
            .solve(&WholeBodyCommand::stand(c.nominal_height + 0.1))
            .unwrap();
        let base = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        // 躯干升高 => 相对躯干原点，足端应更低（z 更负）。
        for i in 0..c.leg_count() {
            assert!(
                higher.foot_targets[i].z < base.foot_targets[i].z - 0.09,
                "leg {i} should lower"
            );
        }
    }

    #[test]
    fn pitch_lean_creates_front_rear_vertical_differential() {
        let c = ctl();
        // 前仰（pitch 为正 = 抬头）：躯干绕髋部俯仰，前足相对躯干应下沉、后足应抬高。
        let lean = c
            .solve(&WholeBodyCommand {
                height: c.nominal_height,
                roll: 0.0,
                pitch: 0.2,
                yaw: 0.0,
                com_offset: Vec3::ZERO,
                force_weights: Vec::new(),
            })
            .unwrap();
        let base = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        // 腿 0、1 为前足（+x），腿 2、3 为后足（-x）。
        // 前仰（nose up）：前髋抬高，前足相对躯干抬高；后足相对躯干下沉。
        for &front in &[0usize, 1] {
            assert!(
                lean.foot_targets[front].z > base.foot_targets[front].z,
                "front leg {front} should rise"
            );
        }
        for &rear in &[2usize, 3] {
            assert!(
                lean.foot_targets[rear].z < base.foot_targets[rear].z,
                "rear leg {rear} should drop"
            );
        }
    }

    #[test]
    fn roll_lean_creates_lateral_asymmetry() {
        let c = ctl();
        // 左滚（roll 为正，绕 +x）：躯干左侧下沉，左足相对躯干应更低。
        let lean = c
            .solve(&WholeBodyCommand {
                height: c.nominal_height,
                roll: 0.2,
                pitch: 0.0,
                yaw: 0.0,
                com_offset: Vec3::ZERO,
                force_weights: Vec::new(),
            })
            .unwrap();
        // 腿 0、2 在 +y 侧，腿 1、3 在 -y 侧。
        let left = lean.foot_targets[0].z;
        let right = lean.foot_targets[1].z;
        assert!(left < right, "left({left}) should be below right({right})");
    }

    #[test]
    fn yaw_rotates_feet_around_origin() {
        let c = ctl();
        let turn = c
            .solve(&WholeBodyCommand {
                height: c.nominal_height,
                roll: 0.0,
                pitch: 0.0,
                yaw: std::f32::consts::FRAC_PI_2,
                com_offset: Vec3::ZERO,
                force_weights: Vec::new(),
            })
            .unwrap();
        // 偏航 90°：前脚（名义在 +x）应摆到侧向，x 大幅减小、|y| 显著增大。
        let f0 = turn.foot_targets[0];
        assert!(
            f0.x.abs() < 0.16 && f0.y.abs() > 0.15,
            "front foot should swing lateral, got {f0:?}"
        );
        // 后脚（名义在 -x）应与前脚反号摆动。
        let f2 = turn.foot_targets[2];
        assert!(
            f0.y.signum() != f2.y.signum() && f2.x.abs() < 0.16,
            "rear foot should swing opposite, got {f2:?}"
        );
    }

    #[test]
    fn com_offset_shifts_feet_opposite() {
        let c = ctl();
        // 躯干向前（+x）偏移：足端相对躯干应后移（x 更负）。
        let shifted = c
            .solve(&WholeBodyCommand {
                height: c.nominal_height,
                roll: 0.0,
                pitch: 0.0,
                yaw: 0.0,
                com_offset: Vec3::new(0.1, 0.0, 0.0),
                force_weights: Vec::new(),
            })
            .unwrap();
        let base = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        for i in 0..c.leg_count() {
            assert!(
                shifted.foot_targets[i].x < base.foot_targets[i].x,
                "leg {i}"
            );
        }
    }

    #[test]
    fn ik_unreachable_returns_error() {
        let c = ctl();
        // 躯干抬得过高 => 腿无法伸到足底（超出最大可达 0.8m），IK 应报错。
        let err = c.solve(&WholeBodyCommand::stand(1.0)).unwrap_err();
        assert!(matches!(err, WbcError::Unreachable { leg: 0, .. }));
    }

    #[test]
    fn non_finite_input_is_rejected() {
        let c = ctl();
        let bad = WholeBodyCommand {
            height: f32::NAN,
            ..WholeBodyCommand::stand(c.nominal_height)
        };
        assert!(matches!(
            c.solve(&bad),
            Err(WbcError::Invalid("command contains non-finite values"))
        ));
    }

    #[test]
    fn force_distribution_even_when_empty() {
        let c = ctl();
        let f = c.distribute_forces(&[]).unwrap();
        assert_eq!(f.len(), 4);
        let sum: f32 = f.iter().map(|v| v.z).sum();
        assert!((sum - 400.0).abs() < 1e-4);
        for v in &f {
            assert!((v.z - 100.0).abs() < 1e-4);
            assert!(v.x.abs() < 1e-6 && v.y.abs() < 1e-6);
        }
    }

    #[test]
    fn force_distribution_weighted() {
        let c = ctl();
        // 前三足各 1 份、末足 3 份 => 总 6 份；400N 分 6 份。
        let f = c.distribute_forces(&[1.0, 1.0, 1.0, 3.0]).unwrap();
        assert!((f[3].z - 200.0).abs() < 1e-4);
        assert!((f[0].z - 400.0 / 6.0).abs() < 1e-4);
        let sum: f32 = f.iter().map(|v| v.z).sum();
        assert!((sum - 400.0).abs() < 1e-4);
    }

    #[test]
    fn force_distribution_zero_or_negative_rejected() {
        let c = ctl();
        assert!(matches!(
            c.distribute_forces(&[0.0, 0.0, 0.0, 0.0]),
            Err(WbcError::Invalid(_))
        ));
        assert!(matches!(
            c.distribute_forces(&[1.0, -1.0, 1.0, 1.0]),
            Err(WbcError::Invalid(_))
        ));
        assert!(matches!(
            c.distribute_forces(&[f32::NAN, 1.0, 1.0, 1.0]),
            Err(WbcError::Invalid(_))
        ));
    }

    #[test]
    fn force_distribution_length_mismatch_rejected() {
        let c = ctl();
        assert!(matches!(
            c.distribute_forces(&[1.0, 1.0, 1.0]),
            Err(WbcError::LegCountMismatch { .. })
        ));
    }

    #[test]
    fn constructor_rejects_length_mismatch() {
        let e = WholeBodyController::new(
            LegIK::new(0.4, 0.4),
            hips(),
            vec![Vec3::new(0.0, 0.0, -0.4)],
            0.4,
            400.0,
        )
        .unwrap_err();
        assert!(matches!(e, WbcError::LegCountMismatch { .. }));
    }

    #[test]
    fn constructor_rejects_empty_or_negative_weight() {
        assert!(matches!(
            WholeBodyController::new(LegIK::new(0.4, 0.4), Vec::new(), Vec::new(), 0.4, 400.0),
            Err(WbcError::Invalid(_))
        ));
        let bad = WholeBodyController::new(LegIK::new(0.4, 0.4), hips(), hips(), 0.4, -1.0);
        assert!(matches!(bad, Err(WbcError::Invalid(_))));
    }

    #[test]
    fn force_direction_matches_total_weight_wrench() {
        // 均匀站姿：合力应与体重相等且方向竖直向上。
        let c = ctl();
        let t = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        let total: Vec3 = t.foot_forces.iter().fold(Vec3::ZERO, |acc, v| acc.add(*v));
        assert!((total.z - 400.0).abs() < 1e-3);
        assert!(total.x.abs() < 1e-6 && total.y.abs() < 1e-6);
        assert_eq!(t.foot_forces.len(), c.leg_count());
        assert_eq!(t.joint_targets.len(), c.leg_count());
    }

    #[test]
    fn euler_round_trip_matches_quat_rotate() {
        // 用 Quat 独立验证 roll 变换的几何一致性（世界→机体 = R⁻¹·(F - t)）。
        let c = ctl();
        let cmd = WholeBodyCommand {
            height: c.nominal_height,
            roll: 0.15,
            pitch: 0.0,
            yaw: 0.0,
            com_offset: Vec3::ZERO,
            force_weights: Vec::new(),
        };
        let t = c.solve(&cmd).unwrap();
        let q = Quat::from_euler(cmd.roll, cmd.pitch, cmd.yaw);
        for i in 0..c.leg_count() {
            let fw = Vec3::new(
                c.foot_positions[i].x,
                c.foot_positions[i].y,
                c.nominal_height + c.foot_positions[i].z,
            );
            let expected = q
                .inverse()
                .rotate_vec3(fw.sub(Vec3::new(0.0, 0.0, cmd.height)));
            let got = t.foot_targets[i];
            assert!(
                (got.x - expected.x).abs() < 1e-3
                    && (got.y - expected.y).abs() < 1e-3
                    && (got.z - expected.z).abs() < 1e-3,
                "leg {i}: got {got:?} expected {expected:?}"
            );
        }
    }
    #[test]
    fn trunk_wrench_balanced_stance() {
        let c = ctl();
        let t = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        let (f, m) = t.trunk_wrench();
        // 合力 = 体重，方向竖直向上。
        assert!((f.z - 400.0).abs() < 1e-2, "f.z={}", f.z);
        assert!(f.x.abs() < 1e-6 && f.y.abs() < 1e-6);
        // 对称均匀站姿：合力矩约 0（前后/左右力矩相互抵消）。
        assert!(m.norm() < 1e-3, "moment={m:?}");
    }

    #[test]
    fn trunk_wrench_asymmetric_force_creates_moment() {
        let c = ctl();
        // 右后足（腿 3）承担更多体重 => 净合力仍=体重，但产生非零合力矩。
        let cmd =
            WholeBodyCommand::stand(c.nominal_height).with_force_weights(vec![1.0, 1.0, 1.0, 2.0]);
        let t = c.solve(&cmd).unwrap();
        let (f, m) = t.trunk_wrench();
        assert!((f.z - 400.0).abs() < 1e-2);
        assert!(
            m.x.abs() > 1.0 && m.y.abs() > 1.0,
            "asymmetric moment={m:?}"
        );
    }

    #[test]
    fn joint_torques_matches_leg_statics() {
        let c = ctl();
        let t = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        let torques = c.joint_torques(&t).unwrap();
        assert_eq!(torques.len(), c.leg_count());
        for (i, (q, f)) in t.joint_targets.iter().zip(&t.foot_forces).enumerate() {
            let expect = c.ik.static_torques(*q, *f);
            assert!(
                (torques[i][0] - expect[0]).abs() < 1e-6
                    && (torques[i][1] - expect[1]).abs() < 1e-6,
                "leg {i}"
            );
        }
    }

    #[test]
    fn joint_torques_length_mismatch_errors() {
        let c = ctl();
        let t = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        let bad = WholeBodyTarget {
            joint_targets: t.joint_targets[..1].to_vec(),
            foot_targets: t.foot_targets.clone(),
            foot_forces: t.foot_forces.clone(),
        };
        assert!(matches!(
            c.joint_torques(&bad),
            Err(WbcError::LegCountMismatch { .. })
        ));
    }

    #[test]
    fn dynamic_torques_rest_matches_statics_in_contact() {
        // 静平衡（q=q̇=q̈=0、无重力）且足端接触力恰等于分配力时，
        // 动态力矩应退化为静力学 τ = Jᵀ·f。
        let c = ctl();
        let target = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        let n = c.leg_count();
        let q: Vec<[f32; 2]> = target.joint_targets.clone();
        let qd = vec![[0.0f32; 2]; n];
        let qdd = vec![[0.0f32; 2]; n];

        let k = 20_000.0f32;
        let contact = ContactModel::new(crate::contact::ContactConfig {
            stiffness: k,
            ..Default::default()
        });
        // 每足需穿透 x = weight/(n·k) 才承载 weight/n。
        let pen = (c.total_weight / n as f32) / k;
        let trunk_height = c.nominal_height - pen;

        let tau_dyn = c
            .dynamic_torques(&target, &q, &qd, &qdd, trunk_height, &contact, Vec3::ZERO)
            .unwrap();
        let stat = c.joint_torques(&target).unwrap();
        assert_eq!(tau_dyn.len(), n);
        for (i, (d, s)) in tau_dyn.iter().zip(&stat).enumerate() {
            assert!(
                (d[0] - s[0]).abs() < 1e-2 && (d[1] - s[1]).abs() < 1e-2,
                "leg {i}: tau_dyn={d:?} stat={s:?}"
            );
        }
    }

    #[test]
    fn dynamic_torques_swing_foot_no_load() {
        // 身体抬到足端离地 → 无接触 → 摆动腿受力为零；静止无重力下力矩应≈0。
        let c = ctl();
        let target = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        let n = c.leg_count();
        let q: Vec<[f32; 2]> = target.joint_targets.clone();
        let qd = vec![[0.0f32; 2]; n];
        let qdd = vec![[0.0f32; 2]; n];
        let contact = ContactModel::new(crate::contact::ContactConfig::default());

        let tau_dyn = c
            .dynamic_torques(
                &target,
                &q,
                &qd,
                &qdd,
                c.nominal_height + 0.05, // 抬躯干，足端离地
                &contact,
                Vec3::ZERO,
            )
            .unwrap();
        for (i, t) in tau_dyn.iter().enumerate() {
            assert!(
                t[0].abs() < 1e-3 && t[1].abs() < 1e-3,
                "swing leg {i} should carry no load: {t:?}"
            );
        }
    }

    #[test]
    fn dynamic_torques_length_mismatch_errors() {
        let c = ctl();
        let target = c.solve(&WholeBodyCommand::stand(c.nominal_height)).unwrap();
        let n = c.leg_count();
        let qd = vec![[0.0f32; 2]; n];
        let qdd = vec![[0.0f32; 2]; n];
        let contact = ContactModel::new(crate::contact::ContactConfig::default());
        // 提供错误的 q 长度 → 应报 LegCountMismatch。
        let bad_q = vec![[0.0f32; 2]; 1];
        assert!(matches!(
            c.dynamic_torques(
                &target,
                &bad_q,
                &qd,
                &qdd,
                c.nominal_height,
                &contact,
                Vec3::ZERO
            ),
            Err(WbcError::LegCountMismatch { .. })
        ));
    }
}
