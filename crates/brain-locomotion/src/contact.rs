//! 足-地接触动力学模型：弹簧-阻尼近似地面反力。
//!
//! 把足端对地面的挤压（穿透深度）与下压速度映射为**地面法向支持力**，并给出接触
//! 状态，供全身动力学（[`crate::wbc::WholeBodyController::dynamic_torques`]）判断
//! 哪些足处于支撑相、需要承担体重。
//!
//! 模型（机体系 / 世界系 z 向上）：`F = k·x − c·ẋ`（`x` 为穿透深度、`ẋ` 为足端竖直
//! 速度），并钳制到 `≥ 0`（地面只能推、不能拉）。下压（`ẋ<0`）增大支撑力，上抬
//! （`ẋ>0`）减小支撑力从而避免"粘附"。

use brain_core::Vec3;

/// 接触模型参数（弹簧-阻尼）。
#[derive(Debug, Clone, Copy)]
pub struct ContactConfig {
    /// 地面法向刚度（N/m）。
    pub stiffness: f32,
    /// 地面法向阻尼（N·s/m）。
    pub damping: f32,
    /// 地面高度（世界系 z，向上为正）。足端 `z < ground_z` 视为穿透。
    pub ground_z: f32,
}

impl Default for ContactConfig {
    fn default() -> Self {
        Self {
            stiffness: 20_000.0, // 20 kN/m，典型硬地面
            damping: 800.0,
            ground_z: 0.0,
        }
    }
}

/// 一次足-地接触的评估结果。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FootContact {
    /// 是否接触（穿透深度 > 0）。
    pub in_contact: bool,
    /// 穿透深度（m，>0 表示陷入地面）。
    pub penetration: f32,
    /// 地面法向支持力（z 向上，N）。
    pub normal_force: f32,
}

impl FootContact {
    /// 静止平衡所需穿透深度（m）：`F = k·x = weight` → `x = weight / k`。
    pub fn resting_penetration(weight: f32, stiffness: f32) -> f32 {
        weight / stiffness
    }
}

/// 足-地接触模型。
#[derive(Debug, Clone, Copy)]
pub struct ContactModel {
    pub config: ContactConfig,
}

impl ContactModel {
    pub fn new(config: ContactConfig) -> Self {
        Self { config }
    }

    pub fn config(&self) -> &ContactConfig {
        &self.config
    }

    /// 由足端世界位置与速度评估接触：`F = k·x − c·ẋ`（沿 +z），下限为 0（只推不拉）。
    pub fn contact(&self, foot_pos: Vec3, foot_vel: Vec3) -> FootContact {
        let pen = self.config.ground_z - foot_pos.z;
        if pen <= 0.0 {
            return FootContact {
                in_contact: false,
                penetration: 0.0,
                normal_force: 0.0,
            };
        }
        let f = self.config.stiffness * pen - self.config.damping * foot_vel.z;
        FootContact {
            in_contact: true,
            penetration: pen,
            normal_force: f.max(0.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> ContactModel {
        ContactModel::new(ContactConfig::default())
    }

    #[test]
    fn no_contact_above_ground() {
        let c = model().contact(Vec3::new(0.0, 0.0, 0.05), Vec3::ZERO);
        assert!(!c.in_contact);
        assert_eq!(c.normal_force, 0.0);
        assert_eq!(c.penetration, 0.0);
    }

    #[test]
    fn penetration_produces_stiffness_force() {
        let m = model();
        // 陷入 0.02m → F = k·x = 20000*0.02 = 400N。
        let c = m.contact(Vec3::new(0.0, 0.0, -0.02), Vec3::ZERO);
        assert!(c.in_contact);
        assert!((c.penetration - 0.02).abs() < 1e-6);
        assert!((c.normal_force - 400.0).abs() < 1e-3);
    }

    #[test]
    fn resting_equilibrium_balances_weight() {
        let m = model();
        let weight = 400.0; // ≈ 40kg 机器人
        let x = FootContact::resting_penetration(weight, m.config.stiffness);
        // 在该穿透深度、静止时支持力恰等于体重。
        let c = m.contact(Vec3::new(0.0, 0.0, -x), Vec3::ZERO);
        assert!(
            (c.normal_force - weight).abs() < 1e-3,
            "F={}",
            c.normal_force
        );
    }

    #[test]
    fn downward_velocity_increases_force() {
        let m = model();
        let static_f = m
            .contact(Vec3::new(0.0, 0.0, -0.01), Vec3::ZERO)
            .normal_force;
        // 下压 1 m/s 增大力（阻尼项 +800）。
        let moving = m.contact(Vec3::new(0.0, 0.0, -0.01), Vec3::new(0.0, 0.0, -1.0));
        assert!(
            moving.normal_force > static_f + 700.0,
            "moving={}",
            moving.normal_force
        );
    }

    #[test]
    fn upward_velocity_reduces_force_but_not_sticky() {
        let m = model();
        // 上抬很快时阻尼项为负，可能把合力压到 0（但不拉负）。
        let c = m.contact(Vec3::new(0.0, 0.0, -0.001), Vec3::new(0.0, 0.0, 5.0));
        assert!(c.normal_force >= 0.0, "force must not be negative");
    }
}
