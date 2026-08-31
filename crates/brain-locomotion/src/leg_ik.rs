//! 腿部逆运动学（IK）：把"足端相对髋部的位置"解析为 `[髋俯仰, 膝]` 关节角。
//!
//! 使用标准的**平面双连杆 IK**（大腿 `l1` + 小腿 `l2`），把步态层输出的足端落点
//! 转换成关节角，供下游关节控制器驱动 `brain-robot` 的 `RobotBody`。
//!
//! 约定：足端相对髋部位置为**机体坐标系**（x 前向、y 侧向、z 向上，足端在下方所以
//! `z < 0`）；本模块只在该腿的矢状面（x-z 平面）内求解，忽略侧向 `y`。

use brain_core::Vec3;

/// IK 求解错误。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IkError {
    /// 足端超出可达范围（过近或过远）。
    Unreachable { distance: f32, min: f32, max: f32 },
}

/// 平面双连杆腿模型。
#[derive(Debug, Clone, Copy)]
pub struct LegIK {
    /// 大腿长度（m）。
    pub l1: f32,
    /// 小腿长度（m）。
    pub l2: f32,
}

impl LegIK {
    pub fn new(l1: f32, l2: f32) -> Self {
        Self { l1, l2 }
    }

    /// 求解：给定足端相对髋部位置（机体系，x 前向、z 向上、下方为负），
    /// 返回 `[髋俯仰, 膝]` 关节角（rad）。
    pub fn solve(&self, foot_rel_hip: Vec3) -> Result<[f32; 2], IkError> {
        let x = foot_rel_hip.x;
        let z = foot_rel_hip.z;
        let d2 = x * x + z * z;
        let d = d2.sqrt();
        let min = (self.l1 - self.l2).abs();
        let max = self.l1 + self.l2;
        let eps = 1e-4;
        if d < min - eps || d > max + eps {
            return Err(IkError::Unreachable {
                distance: d,
                min,
                max,
            });
        }
        // 余弦定理求膝角。
        let cos_knee = ((d2 - self.l1 * self.l1 - self.l2 * self.l2) / (2.0 * self.l1 * self.l2))
            .clamp(-1.0, 1.0);
        let knee = cos_knee.acos();
        // 髋角：足端方位角减去大腿与"髋-足"连线的夹角。
        let hip = z.atan2(x) - (self.l2 * knee.sin()).atan2(self.l1 + self.l2 * knee.cos());
        Ok([hip, knee])
    }

    /// 正运动学（用于验证）：给定 `[髋俯仰, 膝]` 求足端相对髋部位置（机体系）。
    pub fn forward(&self, q: [f32; 2]) -> Vec3 {
        let fx = self.l1 * q[0].cos() + self.l2 * (q[0] + q[1]).cos();
        let fz = self.l1 * q[0].sin() + self.l2 * (q[0] + q[1]).sin();
        Vec3::new(fx, 0.0, fz)
    }

    /// 便捷：给定髋部位置与足端绝对位置（均为机体系），求 `[髋俯仰, 膝]`。
    pub fn solve_from_hip(&self, hip: Vec3, foot: Vec3) -> Result<[f32; 2], IkError> {
        self.solve(foot.sub(hip))
    }

    /// 几何雅可比 `J`（2×2，矢状面 x-z）：把关节角速度映射为足端速度。
    ///
    /// `J[i][j] = ∂p_i / ∂q_j`（`i`：0=x、1=z；`j`：0=髋、1=膝）。
    pub fn jacobian(&self, q: [f32; 2]) -> [[f32; 2]; 2] {
        let (s0, c0) = (q[0].sin(), q[0].cos());
        let (s01, c01) = ((q[0] + q[1]).sin(), (q[0] + q[1]).cos());
        // dfx/dq0, dfx/dq1
        let j00 = -self.l1 * s0 - self.l2 * s01;
        let j01 = -self.l2 * s01;
        // dfz/dq0, dfz/dq1
        let j10 = self.l1 * c0 + self.l2 * c01;
        let j11 = self.l2 * c01;
        [[j00, j01], [j10, j11]]
    }

    /// 静力传递：给定关节角 `q` 与足端作用力（机体系，x 前向 / z 向上），
    /// 返回维持该力的关节力矩 `[髋, 膝]`（`τ = Jᵀ · f`，忽略侧向 `y` 的分量，
    /// 与平面双连杆模型一致）。
    pub fn static_torques(&self, q: [f32; 2], force: Vec3) -> [f32; 2] {
        let j = self.jacobian(q);
        let fx = force.x;
        let fz = force.z;
        // τ = Jᵀ · f
        let hip = j[0][0] * fx + j[1][0] * fz;
        let knee = j[0][1] * fx + j[1][1] * fz;
        [hip, knee]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_down_round_trip() {
        let ik = LegIK::new(0.5, 0.5);
        let target = Vec3::new(0.0, 0.0, -0.8); // 髋正下方 0.8m
        let q = ik.solve(target).unwrap();
        let f = ik.forward(q);
        assert!((f.x - target.x).abs() < 1e-4, "x={}", f.x);
        assert!((f.z - target.z).abs() < 1e-4, "z={}", f.z);
    }

    #[test]
    fn forward_reachable_point_round_trip() {
        let ik = LegIK::new(0.5, 0.5);
        // 前下方一点。
        let target = Vec3::new(0.6, 0.0, -0.6);
        let q = ik.solve(target).unwrap();
        let f = ik.forward(q);
        assert!((f.x - target.x).abs() < 1e-4, "x={} vs {}", f.x, target.x);
        assert!((f.z - target.z).abs() < 1e-4, "z={} vs {}", f.z, target.z);
    }

    #[test]
    fn fully_extended_straight_down() {
        let ik = LegIK::new(0.4, 0.4);
        // 完全伸直：足端在髋正下方 0.8m。
        let target = Vec3::new(0.0, 0.0, -0.8);
        let q = ik.solve(target).unwrap();
        // 膝角应接近 0（伸直），允许微小浮点误差。
        assert!(q[1].abs() < 1e-3, "knee={}", q[1]);
        assert!(
            (q[0].abs() - std::f32::consts::FRAC_PI_2).abs() < 1e-3,
            "hip={}",
            q[0]
        );
    }

    #[test]
    fn unreachable_too_far() {
        let ik = LegIK::new(0.5, 0.5);
        let err = ik.solve(Vec3::new(0.0, 0.0, -2.0)).unwrap_err();
        assert!(matches!(err, IkError::Unreachable { .. }));
    }

    #[test]
    fn unreachable_too_close() {
        // 不等长腿（l1=0.5, l2=0.2）：最小可达距离 |0.5-0.2|=0.3。
        let ik = LegIK::new(0.5, 0.2);
        let err = ik.solve(Vec3::new(0.0, 0.0, -0.1)).unwrap_err();
        assert!(matches!(err, IkError::Unreachable { .. }));
    }

    #[test]
    fn solve_from_hip_subtracts_hip() {
        let ik = LegIK::new(0.5, 0.5);
        let hip = Vec3::new(0.15, 0.0, 0.4);
        let foot = Vec3::new(0.15, 0.0, -0.4); // hip 下方 0.8m
        let q = ik.solve_from_hip(hip, foot).unwrap();
        let f = ik.forward(q);
        // 相对髋部的 (0,0,-0.8)。
        assert!((f.x - 0.0).abs() < 1e-4);
        assert!((f.z + 0.8).abs() < 1e-4);
    }

    #[test]
    fn jacobian_matches_finite_difference() {
        let ik = LegIK::new(0.4, 0.3);
        let q = [0.5, -0.8];
        let j = ik.jacobian(q);
        let eps = 1e-4f32;
        for col in 0..2 {
            let mut qp = q;
            let mut qm = q;
            qp[col] += eps;
            qm[col] -= eps;
            let fp = ik.forward(qp);
            let fm = ik.forward(qm);
            let dx = (fp.x - fm.x) / (2.0 * eps);
            let dz = (fp.z - fm.z) / (2.0 * eps);
            assert!((j[0][col] - dx).abs() < 1e-3, "J[0][{col}]");
            assert!((j[1][col] - dz).abs() < 1e-3, "J[1][{col}]");
        }
    }

    #[test]
    fn static_torques_horizontal_leg_downward_force() {
        // 水平伸直腿（q=[0,0]），足端在髋正前方 l1+l2=0.8m，施加向下力 F=100N。
        let ik = LegIK::new(0.5, 0.3);
        let force = Vec3::new(0.0, 0.0, -100.0);
        let [hip, knee] = ik.static_torques([0.0, 0.0], force);
        // 髋须抵抗力矩 = F * (l1+l2)；膝 = F * l2（力臂），符号为负（抵抗下坠）。
        assert!((hip - (-100.0 * 0.8)).abs() < 1e-3, "hip={hip}");
        assert!((knee - (-100.0 * 0.3)).abs() < 1e-3, "knee={knee}");
    }

    #[test]
    fn static_torques_satisfy_virtual_work() {
        // τ·δq = f·δp（虚功恒等式），任意位形与力均应成立。
        let ik = LegIK::new(0.4, 0.4);
        let q = [0.7, -0.5];
        let force = Vec3::new(30.0, 0.0, -80.0);
        let dq = [1e-5f32, -2e-5f32];
        let tau = ik.static_torques(q, force);
        // 虚功（关节侧）：τ·δq。
        let work_joint = tau[0] * dq[0] + tau[1] * dq[1];
        // 虚功（足端侧）：f·δp = f·(J δq)。
        let j = ik.jacobian(q);
        let dx = j[0][0] * dq[0] + j[0][1] * dq[1];
        let dz = j[1][0] * dq[0] + j[1][1] * dq[1];
        let work_foot = force.x * dx + force.z * dz;
        assert!(
            (work_joint - work_foot).abs() < 1e-3,
            "{work_joint} vs {work_foot}"
        );
    }
}
