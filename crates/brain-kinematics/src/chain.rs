//! 串联关节链：正运动学（FK）、几何雅可比、逆运动学（IK）。

use brain_core::error::{BrainError, Result};
use brain_core::{Pose, Quat, Vec3};

/// 一个转动关节链节。
///
/// `axis` 为关节旋转轴（在该链节坐标系下，单位向量），
/// `offset` 为从该关节到下一关节/末端的平移（在该链节坐标系下）。
#[derive(Debug, Clone, Copy)]
pub struct Link {
    pub axis: Vec3,
    pub offset: Vec3,
}

impl Link {
    pub fn new(axis: Vec3, offset: Vec3) -> Self {
        Self {
            axis: axis.normalized(),
            offset,
        }
    }
}

/// 串联关节链（用于机械臂 / 腿等）。
#[derive(Debug, Clone)]
pub struct KinematicChain {
    links: Vec<Link>,
    /// 当前关节角（弧度）。
    q: Vec<f32>,
}

impl KinematicChain {
    pub fn new(links: Vec<Link>) -> Self {
        let n = links.len();
        Self {
            links,
            q: vec![0.0; n],
        }
    }

    /// 自由度。
    pub fn dof(&self) -> usize {
        self.links.len()
    }

    /// 当前关节角。
    pub fn joint_angles(&self) -> &[f32] {
        &self.q
    }

    /// 设置关节角。
    pub fn set_joint_angles(&mut self, q: &[f32]) -> Result<()> {
        if q.len() != self.links.len() {
            return Err(BrainError::Config(format!(
                "expected {} joints, got {}",
                self.links.len(),
                q.len()
            )));
        }
        self.q.copy_from_slice(q);
        Ok(())
    }

    /// 正运动学：返回末端位姿以及每个关节坐标系（世界系）。
    ///
    /// 每个关节：先绕当前关节原点旋转，再沿连杆平移（标准串联链模型）。
    pub fn forward_kinematics(&self) -> (Pose, Vec<Pose>) {
        let mut frames = Vec::with_capacity(self.links.len());
        let mut t = Pose::IDENTITY;
        for (i, link) in self.links.iter().enumerate() {
            let rot = Pose {
                position: Vec3::ZERO,
                rotation: Quat::from_axis_angle(link.axis, self.q[i]),
            };
            let trans = Pose::from_translation(link.offset);
            t = t.compose(rot).compose(trans);
            frames.push(t);
        }
        (t, frames)
    }

    /// 几何雅可比（6×dof，行优先：[0..3] 线速度，[3..6] 角速度）。
    pub fn jacobian(&self) -> Vec<Vec<f32>> {
        let (ee, frames) = self.forward_kinematics();
        let n = self.dof();
        let mut jac = vec![vec![0.0f32; n]; 6];
        for (i, link) in self.links.iter().enumerate() {
            // 关节 i 的原点 = 处理 link i 之前的帧（i=0 时为原点）。
            let origin = frames
                .get(i.wrapping_sub(1))
                .map(|f| f.position)
                .unwrap_or(Vec3::ZERO);
            let base_rot = frames
                .get(i.wrapping_sub(1))
                .map(|f| f.rotation)
                .unwrap_or(Quat::IDENTITY);
            let axis_world = base_rot.rotate_vec3(link.axis);
            // 线速度部分：axis × (ee - joint_origin)。
            let lever = ee.position.sub(origin);
            let lin = axis_world.cross(lever);
            for r in 0..3 {
                jac[r][i] = if r == 0 {
                    lin.x
                } else if r == 1 {
                    lin.y
                } else {
                    lin.z
                };
                jac[r + 3][i] = if r == 0 {
                    axis_world.x
                } else if r == 1 {
                    axis_world.y
                } else {
                    axis_world.z
                };
            }
        }
        jac
    }

    /// 逆运动学（仅目标位置）：雅可比转置法。
    ///
    /// 以当前关节角为初值迭代，使末端移动到 `target`。返回新的关节角。
    pub fn inverse_kinematics_position(
        &self,
        target: Vec3,
        alpha: f32,
        max_iters: usize,
        tol: f32,
    ) -> Result<Vec<f32>> {
        if self.dof() == 0 {
            return Err(BrainError::Config("empty chain".into()));
        }
        let mut q = self.q.clone();
        for _ in 0..max_iters {
            let mut chain = self.clone();
            chain.q = q.clone();
            let (ee, _) = chain.forward_kinematics();
            let err = target.sub(ee.position);
            if err.norm() < tol {
                return Ok(q);
            }
            let jac = chain.jacobian();
            // 用线速度三行（前 3 行）。
            let mut dq = vec![0.0f32; q.len()];
            for i in 0..q.len() {
                dq[i] = alpha * (jac[0][i] * err.x + jac[1][i] * err.y + jac[2][i] * err.z);
            }
            // 限制单步角度，避免发散。
            let step = dq.iter().map(|v| v * v).sum::<f32>().sqrt().max(1e-8);
            let scale = (0.4 / step).min(1.0);
            for i in 0..q.len() {
                q[i] += dq[i] * scale;
            }
        }
        // 最终检查。
        let mut chain = self.clone();
        chain.q = q.clone();
        let (ee, _) = chain.forward_kinematics();
        if target.sub(ee.position).norm() < tol * 3.0 {
            Ok(q)
        } else {
            Err(BrainError::Config(format!(
                "IK failed to converge, residual={:.4}",
                target.sub(ee.position).norm()
            )))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 构造一个平面 2 连杆臂（沿 X 延伸，绕 Z 旋转）。
    fn planar_arm(l1: f32, l2: f32) -> KinematicChain {
        let z = Vec3::new(0.0, 0.0, 1.0);
        KinematicChain::new(vec![
            Link::new(z, Vec3::new(l1, 0.0, 0.0)),
            Link::new(z, Vec3::new(l2, 0.0, 0.0)),
        ])
    }

    #[test]
    fn fk_straight_arm() {
        let mut arm = planar_arm(1.0, 1.0);
        arm.set_joint_angles(&[0.0, 0.0]).unwrap();
        let (ee, _) = arm.forward_kinematics();
        assert!((ee.position.x - 2.0).abs() < 1e-4);
        assert!(ee.position.y.abs() < 1e-4);
    }

    #[test]
    fn fk_bent_arm() {
        let mut arm = planar_arm(1.0, 1.0);
        // 只弯折肘关节（joint1）90°：肩在前、肘在后，末端应到 (1,1)。
        arm.set_joint_angles(&[0.0, std::f32::consts::FRAC_PI_2])
            .unwrap();
        let (ee, _) = arm.forward_kinematics();
        assert!((ee.position.x - 1.0).abs() < 1e-3);
        assert!((ee.position.y - 1.0).abs() < 1e-3);
    }

    #[test]
    fn ik_reaches_target() {
        let arm = planar_arm(1.0, 1.0);
        // 目标：末端在 (1.5, 0.8)，应在两连杆可及范围内（距离 1.7 < 2）。
        let q = arm
            .inverse_kinematics_position(Vec3::new(1.5, 0.8, 0.0), 0.3, 500, 1e-3)
            .expect("IK should converge");
        let mut arm2 = arm;
        arm2.set_joint_angles(&q).unwrap();
        let (ee, _) = arm2.forward_kinematics();
        let err = Vec3::new(1.5, 0.8, 0.0).sub(ee.position).norm();
        assert!(err < 1e-2, "residual too large: {err}");
    }

    #[test]
    fn wrong_dof_rejected() {
        let mut arm = planar_arm(1.0, 1.0);
        assert!(arm.set_joint_angles(&[0.0]).is_err());
    }
}
