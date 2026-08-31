//! 腿部**逆动力学（Recursive Newton-Euler, RNEA）**：给定运动学（q, q̇, q̈）与外力，
//! 求解维持该运动所需的关节力矩 `[髋, 膝]`。
//!
//! 这是上一轮**静力学**（`LegIK::static_torques`，`τ = Jᵀ·f`）的自然推广：当
//! `q̇ = q̈ = 0` 且无重力时，本模块退化为静力传递；当考虑惯量/科氏/离心/重力时，
//! 给出**动态**关节力矩（如力矩前馈、关节负载估计、力矩超限校验）。
//!
//! 模型：平面双连杆腿（x-z 矢状面，y 为旋转轴）。约定与 [`crate::leg_ik`] 一致，
//! 即 `q[0]`=髋俯仰、`q[1]`=膝，足端位置 `(l1 cosθ1 + l2 cosθ2, l1 sinθ1 + l2 sinθ2)`，
//! 其中 `θ1 = q[0]`、`θ2 = q[0] + q[1]`。重力以“底座线加速度 = −g”技巧引入，
//! 与 Craig《Introduction to Robotics》的 Newton-Euler 逆动力学一致。

use brain_core::Vec3;

/// 平面双连杆腿的动力学参数（含质量/惯量分布）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LegDynamics {
    /// 大腿长度（m）。
    pub l1: f32,
    /// 小腿长度（m）。
    pub l2: f32,
    /// 大腿质量（kg）。
    pub m1: f32,
    /// 小腿质量（kg）。
    pub m2: f32,
    /// 大腿质心到髋关节的距离（m，沿杆方向）。
    pub rc1: f32,
    /// 小腿质心到膝关节的距离（m，沿杆方向）。
    pub rc2: f32,
    /// 大腿绕质心的转动惯量（kg·m²，绕 y 轴）。
    pub i1: f32,
    /// 小腿绕质心的转动惯量（kg·m²，绕 y 轴）。
    pub i2: f32,
}

impl LegDynamics {
    #[allow(clippy::too_many_arguments)] // 数据构造器：逐项命名质量/惯量参数最清晰
    pub fn new(l1: f32, l2: f32, m1: f32, m2: f32, rc1: f32, rc2: f32, i1: f32, i2: f32) -> Self {
        Self {
            l1,
            l2,
            m1,
            m2,
            rc1,
            rc2,
            i1,
            i2,
        }
    }

    /// 由杆长构造默认质量分布：质量正比于长度、质心在杆中点、按细杆近似转动惯量。
    /// 供 [`crate::wbc::WholeBodyController`] 快速搭起默认腿参数，再用
    /// `with_dynamics` 覆盖成更精确的标定值。
    pub fn from_links(l1: f32, l2: f32) -> Self {
        let m1 = 0.5 * l1;
        let m2 = 0.5 * l2;
        Self {
            l1,
            l2,
            m1,
            m2,
            rc1: l1 / 2.0,
            rc2: l2 / 2.0,
            i1: m1 * l1 * l1 / 12.0,
            i2: m2 * l2 * l2 / 12.0,
        }
    }

    /// 各杆质心的**世界位置**（髋部固定于原点；x 前向、z 向上）。
    ///
    /// 返回 `(质心1, 质心2)`，供重力势能/能量计算使用。
    pub fn cm_world(&self, q: [f32; 2]) -> (Vec3, Vec3) {
        let (th1, th2) = (q[0], q[0] + q[1]);
        let c1 = Vec3::new(self.rc1 * th1.cos(), 0.0, self.rc1 * th1.sin());
        let c2 = Vec3::new(
            self.l1 * th1.cos() + self.rc2 * th2.cos(),
            0.0,
            self.l1 * th1.sin() + self.rc2 * th2.sin(),
        );
        (c1, c2)
    }

    /// 逆动力学：给定关节角/角速度/角加速度与外足底力，求 `[髋, 膝]` 关节力矩（N·m）。
    ///
    /// - `foot_force`：足端受到的**外部作用力**（机体系，如地面支持力），无力则传
    ///   [`Vec3::ZERO`]。
    /// - `gravity`：重力加速度向量（机体系，通常 `(0,0,-9.81)`）。
    pub fn inverse_dynamics(
        &self,
        q: [f32; 2],
        qd: [f32; 2],
        qdd: [f32; 2],
        foot_force: Vec3,
        gravity: Vec3,
    ) -> [f32; 2] {
        let (th1, th2) = (q[0], q[0] + q[1]);
        let (c1, s1) = (th1.cos(), th1.sin());
        let (c2, s2) = (th2.cos(), th2.sin());

        // 2D 平面辅助（x,z）。
        // rot_cross(ω, p) = ω × p = (−ω·z, ω·x)；cross2d(a,b) = a_x·b_z − a_z·b_x。
        let rot_cross = |w: f32, x: f32, z: f32| -> (f32, f32) { (-w * z, w * x) };
        let cross2d = |ax: f32, az: f32, bx: f32, bz: f32| ax * bz - az * bx;
        let add = |a: (f32, f32), b: (f32, f32)| (a.0 + b.0, a.1 + b.1);
        // 刚体旋转的加速度项：α×p + ω×(ω×p)（切向 + 向心）。
        let ang_acc = |w: f32, al: f32, x: f32, z: f32| -> (f32, f32) {
            let t = rot_cross(al, x, z); // 切向：α×p
            let v = rot_cross(w, x, z); // ω×p
            let c = rot_cross(w, v.0, v.1); // 向心：ω×(ω×p)
            add(t, c)
        };

        // 正向传递：以底座线加速度 = −g 引入重力。
        let a0 = (-gravity.x, -gravity.z);
        // 关节1 → 关节2、关节2 → 足端、髋 → 质心1、膝 → 质心2 的位置向量。
        let p2 = (self.l1 * c1, self.l1 * s1);
        let p3 = (self.l2 * c2, self.l2 * s2);
        let rc1 = (self.rc1 * c1, self.rc1 * s1);
        let rc2 = (self.rc2 * c2, self.rc2 * s2);

        // 连杆 1。
        let (w1, al1) = (qd[0], qdd[0]);
        let (w2, al2) = (w1 + qd[1], al1 + qdd[1]);
        let a_j1 = a0;
        // a_cm1 = a_j1 + α1×rc1 + ω1×(ω1×rc1)
        let a_cm1 = add(a_j1, ang_acc(w1, al1, rc1.0, rc1.1));
        // a_j2 = a_j1 + α1×p2 + ω1×(ω1×p2)   [约定 A：关节 2 原点加速度用连杆 1 的角量]
        let a_j2 = add(a_j1, ang_acc(w1, al1, p2.0, p2.1));

        // a_cm2 = a_j2 + α2×rc2 + ω2×(ω2×rc2)
        let a_cm2 = add(a_j2, ang_acc(w2, al2, rc2.0, rc2.1));

        // 反向传递（Craig/Lynch Newton-Euler，力矩取关于各关节原点）：
        //   f_i = f_{i+1} + m_i·a_cm_i
        //   n_i = n_{i+1} + I_i·α_i + rc_i×(m_i·a_cm_i) + p_{i+1}×f_{i+1}
        // 末端外部力 f_3 作用在足端，外部力矩 n_3 = 0（点足）。
        let (f3x, f3z) = (foot_force.x, foot_force.z);
        let n3 = 0.0f32;

        // 连杆 2。
        let (f2x, f2z) = (self.m2 * a_cm2.0 + f3x, self.m2 * a_cm2.1 + f3z);
        let n2 = self.i2 * al2
            + n3
            + cross2d(rc2.0, rc2.1, self.m2 * a_cm2.0, self.m2 * a_cm2.1)
            + cross2d(p3.0, p3.1, f3x, f3z);
        let tau_knee = n2;

        // 连杆 1（末级，无父连杆，不需再向上传递力）。
        let n1 = self.i1 * al1
            + n2
            + cross2d(rc1.0, rc1.1, self.m1 * a_cm1.0, self.m1 * a_cm1.1)
            + cross2d(p2.0, p2.1, f2x, f2z);
        let tau_hip = n1;

        [tau_hip, tau_knee]
    }

    /// 正向动力学：给定关节角/角速度/关节力矩与外力、重力，求关节角加速度 `[q̈0, q̈1]`。
    ///
    /// 这是 [`Self::inverse_dynamics`] 的逆运算。算法：用逆动力学在 `q̈=0`、`q̇=0`
    /// 下提取质量矩阵 `M(q)`（各列），在 `q̈=0` 下提取偏置项 `b = C·q̇ + g + Jᵀf`，
    /// 再解析求解线性系统 `M(q)·q̈ = τ − b`。
    pub fn forward_dynamics(
        &self,
        q: [f32; 2],
        qd: [f32; 2],
        tau: [f32; 2],
        foot_force: Vec3,
        gravity: Vec3,
    ) -> [f32; 2] {
        // 质量矩阵各列（q̇=0、无外力、无重力 → τ = M·e_i）。
        let m0 = self.inverse_dynamics(q, [0.0, 0.0], [1.0, 0.0], Vec3::ZERO, Vec3::ZERO);
        let m1 = self.inverse_dynamics(q, [0.0, 0.0], [0.0, 1.0], Vec3::ZERO, Vec3::ZERO);
        let m = [[m0[0], m1[0]], [m0[1], m1[1]]];
        // 偏置项 b = C·q̇ + g + Jᵀf（q̈=0）。
        let b = self.inverse_dynamics(q, qd, [0.0, 0.0], foot_force, gravity);
        let (r0, r1) = (tau[0] - b[0], tau[1] - b[1]);
        // 解析求解 2×2：M·q̈ = rhs。
        let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
        let qdd0 = (m[1][1] * r0 - m[0][1] * r1) / det;
        let qdd1 = (m[0][0] * r1 - m[1][0] * r0) / det;
        [qdd0, qdd1]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 一个典型四足腿：股/胫各 0.4m，总质量 2kg，均质杆（质心在中点）。
    fn leg() -> LegDynamics {
        LegDynamics::new(0.4, 0.4, 1.0, 1.0, 0.2, 0.2, 1.0 / 12.0, 1.0 / 12.0)
    }

    #[test]
    fn static_matches_jacobian_transpose() {
        // 无重力、无运动：逆动力学应严格退化为静力传递 τ = Jᵀ·f。
        let ld = leg();
        let ik = crate::leg_ik::LegIK::new(ld.l1, ld.l2);
        for q in [[0.6, -0.9], [-0.3, 1.2], [1.2, -0.5]] {
            let f = Vec3::new(12.0, 0.0, -50.0);
            let tau = ld.inverse_dynamics(q, [0.0, 0.0], [0.0, 0.0], f, Vec3::ZERO);
            let expect = ik.static_torques(q, f);
            assert!(
                (tau[0] - expect[0]).abs() < 1e-3 && (tau[1] - expect[1]).abs() < 1e-3,
                "q={q:?}: got {tau:?} expect {expect:?}"
            );
        }
    }

    #[test]
    fn gravity_torque_matches_cm_jacobian() {
        // 无外力、无运动：逆动力学应给出与“质心雅可比转置”一致的重力矩。
        // τ_i = Σ_j m_j · g · (∂p_cm_j / ∂q_i)。
        let ld = leg();
        let g = Vec3::new(0.0, 0.0, -9.81);
        let q = [0.4, -0.7];
        let tau = ld.inverse_dynamics(q, [0.0, 0.0], [0.0, 0.0], Vec3::ZERO, g);
        // 用有限差分独立求 ∂p_cm/∂q，再与逆动力学结果比对。
        let eps = 1e-5f32;
        for joint in 0..2 {
            let mut qp = q;
            let mut qm = q;
            qp[joint] += eps;
            qm[joint] -= eps;
            let (p1p, p2p) = ld.cm_world(qp);
            let (p1m, p2m) = ld.cm_world(qm);
            let dp1 = p1p.sub(p1m) / (2.0 * eps);
            let dp2 = p2p.sub(p2m) / (2.0 * eps);
            // τ = ∂PE/∂q = ∂(−Σ m·g·p)/∂q = −Σ m·g·(∂p/∂q)。
            let t_grav = -(ld.m1 * g.dot(dp1) + ld.m2 * g.dot(dp2));
            assert!(
                (tau[joint] - t_grav).abs() < 1e-2,
                "joint {joint}: got {} expect {t_grav}",
                tau[joint]
            );
        }
    }

    #[test]
    fn mass_matrix_matches_analytic() {
        // qd=0、无重力、无外力时 τ = M(q)·q̈。用单位 q̈ 提取 M 并与解析式比对。
        let ld = leg();
        let (l1, _l2) = (ld.l1, ld.l2);
        let (rc1, rc2) = (ld.rc1, ld.rc2);
        let (i1, i2) = (ld.i1, ld.i2);
        let q: [f32; 2] = [0.4, -0.9];
        let q1 = q[1];
        // 解析平面 2R 质量矩阵。
        let m11 =
            i1 + ld.m1 * rc1 * rc1 + i2 + ld.m2 * (l1 * l1 + rc2 * rc2 + 2.0 * l1 * rc2 * q1.cos());
        let m22 = i2 + ld.m2 * rc2 * rc2;
        let m12 = i2 + ld.m2 * rc2 * rc2 + ld.m2 * l1 * rc2 * q1.cos();
        // 第一列。
        let tau = ld.inverse_dynamics(q, [0.0, 0.0], [1.0, 0.0], Vec3::ZERO, Vec3::ZERO);
        assert!(
            (tau[0] - m11).abs() < 1e-4,
            "M11: got {} expect {m11}",
            tau[0]
        );
        assert!(
            (tau[1] - m12).abs() < 1e-4,
            "M21: got {} expect {m12}",
            tau[1]
        );
        // 第二列。
        let tau = ld.inverse_dynamics(q, [0.0, 0.0], [0.0, 1.0], Vec3::ZERO, Vec3::ZERO);
        assert!(
            (tau[0] - m12).abs() < 1e-4,
            "M12: got {} expect {m12}",
            tau[0]
        );
        assert!(
            (tau[1] - m22).abs() < 1e-4,
            "M22: got {} expect {m22}",
            tau[1]
        );
        // 对称正定性。
        assert!((m11 * m22 - m12 * m12) > 0.0);
    }

    #[test]
    fn matches_virtual_work_projection() {
        // 用“解析质心加速度 + 虚功投影”独立求 τ：τ_i = Σ_j m_j a_cm_j·(∂p_cm_j/∂q_i) + I_j α_j (dω_j/dq̇_i)。
        // 反向传递的科氏/离心项是本测试的校验对象；质心加速度按正向运动学解析计算，
        // 质心对 q 的雅可比用位置有限差分（鲁棒，不受加速度差分精度影响）。
        let ld = leg();
        let q: [f32; 2] = [0.4, -0.9];
        let qd: [f32; 2] = [1.2, 3.0];
        let qdd: [f32; 2] = [0.5, -1.0];
        let tau = ld.inverse_dynamics(q, qd, qdd, Vec3::ZERO, Vec3::ZERO);

        let (th1, th2) = (q[0], q[0] + q[1]);
        let (c1, s1) = (th1.cos(), th1.sin());
        let (c2, s2) = (th2.cos(), th2.sin());
        let rot_cross = |w: f32, x: f32, z: f32| -> (f32, f32) { (-w * z, w * x) };
        let add = |a: (f32, f32), b: (f32, f32)| (a.0 + b.0, a.1 + b.1);
        let ang_acc = |w: f32, al: f32, x: f32, z: f32| -> (f32, f32) {
            let t = rot_cross(al, x, z);
            let v = rot_cross(w, x, z);
            let c = rot_cross(w, v.0, v.1);
            add(t, c)
        };
        let (w1, w2) = (qd[0], qd[0] + qd[1]);
        let (al1, al2) = (qdd[0], qdd[0] + qdd[1]);
        let p2 = (ld.l1 * c1, ld.l1 * s1);
        let rc1 = (ld.rc1 * c1, ld.rc1 * s1);
        let rc2 = (ld.rc2 * c2, ld.rc2 * s2);
        // 质心加速度（正向运动学，解析）。
        let a_cm1 = ang_acc(w1, al1, rc1.0, rc1.1);
        let a_j2 = ang_acc(w1, al1, p2.0, p2.1);
        let a_cm2 = add(a_j2, ang_acc(w2, al2, rc2.0, rc2.1));

        // 质心对 q_i 的雅可比（位置有限差分）。
        let eps = 1e-4f32;
        let dcm = |joint: usize| -> (Vec3, Vec3) {
            let mut qp = q;
            let mut qm = q;
            qp[joint] += eps;
            qm[joint] -= eps;
            let (p1p, p2p) = ld.cm_world(qp);
            let (p1m, p2m) = ld.cm_world(qm);
            ((p1p.sub(p1m)) / (2.0 * eps), (p2p.sub(p2m)) / (2.0 * eps))
        };
        // dω_j/dq̇_i：ω1=q̇0, ω2=q̇0+q̇1。
        let dq: [[f32; 2]; 2] = [
            [1.0, 1.0], // 对 q̇0
            [0.0, 1.0], // 对 q̇1
        ];
        let acc = |x: f32, z: f32| Vec3::new(x, 0.0, z);
        for joint in 0..2 {
            let (dp1, dp2) = dcm(joint);
            let t_proj = ld.m1 * acc(a_cm1.0, a_cm1.1).dot(dp1)
                + ld.m2 * acc(a_cm2.0, a_cm2.1).dot(dp2)
                + ld.i1 * al1 * dq[joint][0]
                + ld.i2 * al2 * dq[joint][1];
            assert!(
                (tau[joint] - t_proj).abs() < 1e-2,
                "joint {joint}: inverse_dynamics={} projection={t_proj}",
                tau[joint]
            );
        }
    }

    #[test]
    fn power_balances_energy_rate() {
        // 能量守恒：无外力下，关节功率 τ·q̇ 应等于系统机械能变化率 d(KE+PE)/dt。
        let ld = leg();
        let g = Vec3::new(0.0, 0.0, -9.81);
        let dt = 1e-4f32;
        let t_end = 0.2;
        // 平滑轨迹：q(t) = q0 + A·sin(ωt)。
        let q0 = [0.5, -1.0];
        let amp = [0.3, 0.4];
        let omega = [5.0, 7.0];
        let traj = |t: f32| -> ([f32; 2], [f32; 2], [f32; 2]) {
            let mut q = [0.0; 2];
            let mut qd = [0.0; 2];
            let mut qdd = [0.0; 2];
            for i in 0..2 {
                q[i] = q0[i] + amp[i] * (omega[i] * t).sin();
                qd[i] = amp[i] * omega[i] * (omega[i] * t).cos();
                qdd[i] = -amp[i] * omega[i] * omega[i] * (omega[i] * t).sin();
            }
            (q, qd, qdd)
        };
        // 解析质量矩阵（用于精确 dKE/dt）。
        let (l1, rc1, rc2) = (ld.l1, ld.rc1, ld.rc2);
        let mass = |q: [f32; 2]| -> [[f32; 2]; 2] {
            let c = q[1].cos();
            [
                [
                    ld.i1
                        + ld.m1 * rc1 * rc1
                        + ld.i2
                        + ld.m2 * (l1 * l1 + rc2 * rc2 + 2.0 * l1 * rc2 * c),
                    ld.i2 + ld.m2 * rc2 * rc2 + ld.m2 * l1 * rc2 * c,
                ],
                [
                    ld.i2 + ld.m2 * rc2 * rc2 + ld.m2 * l1 * rc2 * c,
                    ld.i2 + ld.m2 * rc2 * rc2,
                ],
            ]
        };
        // PE = −(m1 g·c1 + m2 g·c2)（重力势能，用于有限差分 dPE/dt）。
        let pe = |t: f32| -> f32 {
            let (c1, c2) = ld.cm_world(traj(t).0);
            -(ld.m1 * g.dot(c1) + ld.m2 * g.dot(c2))
        };
        let mut max_err = 0.0f32;
        let mut t = dt;
        while t < t_end {
            let (q, qd, qdd) = traj(t);
            let tau = ld.inverse_dynamics(q, qd, qdd, Vec3::ZERO, g);
            let power = tau[0] * qd[0] + tau[1] * qd[1];
            let m = mass(q);
            let (qp, _, _) = traj(t + dt);
            let (qm, _, _) = traj(t - dt);
            let dm00 = (mass(qp)[0][0] - mass(qm)[0][0]) / (2.0 * dt);
            let dm01 = (mass(qp)[0][1] - mass(qm)[0][1]) / (2.0 * dt);
            let dm11 = (mass(qp)[1][1] - mass(qm)[1][1]) / (2.0 * dt);
            let dke = qd[0] * (m[0][0] * qdd[0] + m[0][1] * qdd[1])
                + qd[1] * (m[1][0] * qdd[0] + m[1][1] * qdd[1])
                + 0.5 * (qd[0] * qd[0] * dm00 + 2.0 * qd[0] * qd[1] * dm01 + qd[1] * qd[1] * dm11);
            let dpe = (pe(t + dt) - pe(t - dt)) / (2.0 * dt);
            max_err = max_err.max((power - (dke + dpe)).abs());
            t += dt;
        }
        // 机械能变化率与关节功率应高度一致。
        assert!(max_err < 0.05, "max |power - dE/dt| = {max_err}");
    }

    #[test]
    fn power_balances_energy_rate_no_gravity() {
        // 无重力：用解析质量矩阵精确求 dKE/dt = q̇ᵀM q̈ + ½ q̇ᵀ(dM/dt) q̇，与关节功率 τ·q̇ 对比。
        // 完全独立于 inverse_dynamics 的实现（M 为解析平面 2R 质量矩阵）。
        let ld = leg();
        let (l1, rc1, rc2) = (ld.l1, ld.rc1, ld.rc2);
        let mass = |q: [f32; 2]| -> [[f32; 2]; 2] {
            let c = q[1].cos();
            let m11 = ld.i1
                + ld.m1 * rc1 * rc1
                + ld.i2
                + ld.m2 * (l1 * l1 + rc2 * rc2 + 2.0 * l1 * rc2 * c);
            let m22 = ld.i2 + ld.m2 * rc2 * rc2;
            let m12 = ld.i2 + ld.m2 * rc2 * rc2 + ld.m2 * l1 * rc2 * c;
            [[m11, m12], [m12, m22]]
        };
        let dt = 1e-4f32;
        let t_end = 0.2;
        let q0 = [0.6, -0.8];
        let amp = [0.2, 0.5];
        let omega = [6.0, 9.0];
        let traj = |t: f32| -> ([f32; 2], [f32; 2], [f32; 2]) {
            let mut q = [0.0; 2];
            let mut qd = [0.0; 2];
            let mut qdd = [0.0; 2];
            for i in 0..2 {
                q[i] = q0[i] + amp[i] * (omega[i] * t).sin();
                qd[i] = amp[i] * omega[i] * (omega[i] * t).cos();
                qdd[i] = -amp[i] * omega[i] * omega[i] * (omega[i] * t).sin();
            }
            (q, qd, qdd)
        };
        let mut max_err = 0.0f32;
        let mut t = dt;
        while t < t_end {
            let (q, qd, qdd) = traj(t);
            let tau = ld.inverse_dynamics(q, qd, qdd, Vec3::ZERO, Vec3::ZERO);
            let power = tau[0] * qd[0] + tau[1] * qd[1];
            let m = mass(q);
            // dM/dt（对 M(q(t)) 作时间中心差分）。
            let (qp, _, _) = traj(t + dt);
            let (qm, _, _) = traj(t - dt);
            let dm00 = (mass(qp)[0][0] - mass(qm)[0][0]) / (2.0 * dt);
            let dm01 = (mass(qp)[0][1] - mass(qm)[0][1]) / (2.0 * dt);
            let dm11 = (mass(qp)[1][1] - mass(qm)[1][1]) / (2.0 * dt);
            // dKE/dt = q̇ᵀM q̈ + ½ q̇ᵀ(dM/dt) q̇。
            let a = qd[0] * (m[0][0] * qdd[0] + m[0][1] * qdd[1])
                + qd[1] * (m[1][0] * qdd[0] + m[1][1] * qdd[1]);
            let b =
                0.5 * (qd[0] * qd[0] * dm00 + 2.0 * qd[0] * qd[1] * dm01 + qd[1] * qd[1] * dm11);
            let dke = a + b;
            max_err = max_err.max((power - dke).abs());
            t += dt;
        }
        assert!(
            max_err < 0.05,
            "no-gravity max |power - dKE/dt| = {max_err}"
        );
    }

    #[test]
    fn forward_inverse_round_trip() {
        // 正向动力学是逆动力学的逆：forward(inverse(q,qd,qdd)) 应还原 qdd。
        let ld = leg();
        let g = Vec3::new(0.0, 0.0, -9.81);
        let states = [
            ([0.4, -0.9], [1.2, 3.0], [0.5, -1.0]),
            ([-0.3, 0.7], [-0.8, 2.0], [1.0, 0.5]),
            ([1.1, -1.5], [2.0, -1.0], [-0.5, 0.8]),
        ];
        for (q, qd, qdd) in states {
            let f = Vec3::new(5.0, 0.0, -30.0);
            let tau = ld.inverse_dynamics(q, qd, qdd, f, g);
            let back = ld.forward_dynamics(q, qd, tau, f, g);
            assert!(
                (back[0] - qdd[0]).abs() < 1e-4 && (back[1] - qdd[1]).abs() < 1e-4,
                "q={q:?} qd={qd:?} qdd={qdd:?}: got {back:?}"
            );
        }
    }

    #[test]
    fn forward_zero_torque_free_spin_conserves() {
        // 无重力、无外力、零力矩下，正向动力学给出的 q̈ 应使系统处于“自由运动”：
        // 该 q̈ 回代逆动力学应得到零力矩（即 M·q̈ + C·q̇ = 0）。
        let ld = leg();
        let q = [0.5, -1.0];
        let qd = [1.5, 2.5];
        let qdd = ld.forward_dynamics(q, qd, [0.0, 0.0], Vec3::ZERO, Vec3::ZERO);
        let tau = ld.inverse_dynamics(q, qd, qdd, Vec3::ZERO, Vec3::ZERO);
        assert!(
            tau[0].abs() < 1e-3 && tau[1].abs() < 1e-3,
            "free spin should be torque-free, got {tau:?}"
        );
    }
}
