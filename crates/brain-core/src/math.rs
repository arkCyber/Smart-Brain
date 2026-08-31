//! 通用数学原语：`Vec3`、四元数 `Quat`、等距位姿 `Pose`。
//!
//! 被 `brain-robot`（具身抽象）与 `brain-kinematics`（正/逆运动学）共用，
//! 避免各 crate 重复定义基础数学类型。全部为标量运算，零外部依赖。

use serde::{Deserialize, Serialize};

/// 三维向量。
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// 点积。
    pub fn dot(self, o: Self) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    /// 叉积。
    pub fn cross(self, o: Self) -> Self {
        Self {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }
    /// 模长。
    pub fn norm(self) -> f32 {
        self.dot(self).sqrt()
    }
    /// 归一化（零向量返回零向量）。
    pub fn normalized(self) -> Self {
        let n = self.norm();
        if n < 1e-8 {
            Self::ZERO
        } else {
            self * (1.0 / n)
        }
    }
    /// 向量相加的便捷方法（等价于 `+`，供链式调用）。
    #[allow(clippy::should_implement_trait)] // 操作符已由 `std::ops::Add` 提供
    pub fn add(self, o: Self) -> Self {
        self + o
    }
    /// 向量相减的便捷方法（等价于 `-`，供链式调用）。
    #[allow(clippy::should_implement_trait)] // 操作符已由 `std::ops::Sub` 提供
    pub fn sub(self, o: Self) -> Self {
        self - o
    }

    /// 线性插值：`a + (b - a) * t`（`t` 通常 ∈ [0,1]）。
    pub fn lerp(a: Self, b: Self, t: f32) -> Self {
        a + (b - a) * t
    }

    /// 与 `b` 的欧氏距离。
    pub fn distance(a: Self, b: Self) -> f32 {
        (b - a).norm()
    }

    /// 各分量是否均为有限值（用于坏数据/NaN 检测）。
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// 逐分量相乘（Hadamard 积）。
    pub fn component_mul(self, o: Self) -> Self {
        Self::new(self.x * o.x, self.y * o.y, self.z * o.z)
    }
}

impl std::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Self) -> Vec3 {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Self) -> Vec3 {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl std::ops::Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

/// 标量左乘：`s * v`。
impl std::ops::Mul<Vec3> for f32 {
    type Output = Vec3;
    fn mul(self, v: Vec3) -> Vec3 {
        v * self
    }
}

impl std::ops::Div<f32> for Vec3 {
    type Output = Vec3;
    fn div(self, s: f32) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}

impl std::ops::Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

/// 单位四元数（w + x i + y j + z k），表示旋转。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quat {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quat {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }.normalized()
    }

    /// 归一化。
    pub fn normalized(self) -> Self {
        let n = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        if n < 1e-8 {
            Self::IDENTITY
        } else {
            Self {
                w: self.w / n,
                x: self.x / n,
                y: self.y / n,
                z: self.z / n,
            }
        }
    }

    /// 绕单位轴旋转 angle（弧度）。
    pub fn from_axis_angle(axis: Vec3, angle: f32) -> Self {
        let a = axis.normalized();
        let h = angle * 0.5;
        let s = h.sin();
        Self::new(h.cos(), a.x * s, a.y * s, a.z * s)
    }

    /// 由欧拉角（XYZ，弧度）构造。
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Self {
        let qx = Self::from_axis_angle(Vec3::new(1.0, 0.0, 0.0), roll);
        let qy = Self::from_axis_angle(Vec3::new(0.0, 1.0, 0.0), pitch);
        let qz = Self::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), yaw);
        (qz * qy * qx).normalized()
    }

    /// 单位四元数的逆（= 共轭）。
    pub fn inverse(self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }

    /// 旋转一个向量。
    pub fn rotate_vec3(self, v: Vec3) -> Vec3 {
        let qv = Vec3::new(self.x, self.y, self.z);
        let t = qv.cross(v) * 2.0;
        v.add(t * self.w).add(qv.cross(t))
    }

    /// 复合旋转的便捷方法：`self.mul(o)` 先应用 `o` 再应用 `self`（等价于 `*`）。
    #[allow(clippy::should_implement_trait)] // 操作符已由 `std::ops::Mul` 提供
    pub fn mul(self, o: Self) -> Self {
        self * o
    }

    /// 与 `o` 的四元数点积。
    pub fn dot(self, o: Self) -> f32 {
        self.w * o.w + self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// 与 `o` 的夹角（弧度，0..=π，返回**实际旋转角**）。
    pub fn angle_to(self, o: Self) -> f32 {
        // 相对旋转 q = self * o⁻¹；其 w = cos(θ/2)，|vector| = sin(θ/2)。
        let rel = self * o.inverse();
        let v = Vec3::new(rel.x, rel.y, rel.z);
        2.0 * v.norm().atan2(rel.w)
    }

    /// 球面线性插值：`t=0` 返回 `a`，`t=1` 返回 `b`，中间沿最短路径。
    pub fn slerp(a: Self, b: Self, t: f32) -> Self {
        let mut b2 = b;
        let mut dot = a.dot(b);
        // 取最短路径：夹角 > 90° 时取反一个。
        if dot < 0.0 {
            b2 = Self {
                w: -b.w,
                x: -b.x,
                y: -b.y,
                z: -b.z,
            };
            dot = -dot;
        }
        let dot = dot.clamp(-1.0, 1.0);
        // 近似平行时用线性插值再归一化，避免除零。
        if dot > 0.9995 {
            return Self::new(
                a.w + (b2.w - a.w) * t,
                a.x + (b2.x - a.x) * t,
                a.y + (b2.y - a.y) * t,
                a.z + (b2.z - a.z) * t,
            );
        }
        let theta = dot.acos();
        let sin = theta.sin();
        let wa = ((1.0 - t) * theta).sin() / sin;
        let wb = (t * theta).sin() / sin;
        Self::new(
            a.w * wa + b2.w * wb,
            a.x * wa + b2.x * wb,
            a.y * wa + b2.y * wb,
            a.z * wa + b2.z * wb,
        )
    }

    /// 转回欧拉角 `(roll, pitch, yaw)`（弧度），`from_euler` 的逆操作。
    pub fn to_euler(self) -> (f32, f32, f32) {
        let q = self.normalized();
        let (w, x, y, z) = (q.w, q.x, q.y, q.z);
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));
        let sinp = 2.0 * (w * y - z * x);
        let pitch = if sinp.abs() >= 1.0 {
            sinp.signum() * std::f32::consts::FRAC_PI_2
        } else {
            sinp.asin()
        };
        let yaw = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));
        (roll, pitch, yaw)
    }
}

impl std::ops::Mul<Quat> for Quat {
    type Output = Quat;
    /// 复合旋转：`self * o` 先应用 o 再应用 self。
    fn mul(self, o: Quat) -> Quat {
        Self {
            w: self.w * o.w - self.x * o.x - self.y * o.y - self.z * o.z,
            x: self.w * o.x + self.x * o.w + self.y * o.z - self.z * o.y,
            y: self.w * o.y - self.x * o.z + self.y * o.w + self.z * o.x,
            z: self.w * o.z + self.x * o.y - self.y * o.x + self.z * o.w,
        }
        .normalized()
    }
}

/// 三维等距位姿：位置 + 旋转（四元数）。
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    pub position: Vec3,
    pub rotation: Quat,
}

impl Pose {
    pub const IDENTITY: Self = Self {
        position: Vec3::ZERO,
        rotation: Quat::IDENTITY,
    };

    pub fn from_translation(t: Vec3) -> Self {
        Self {
            position: t,
            rotation: Quat::IDENTITY,
        }
    }

    /// 由旋转（四元数）构造，位置为零。
    pub fn from_rotation(q: Quat) -> Self {
        Self {
            position: Vec3::ZERO,
            rotation: q,
        }
    }

    /// 由位置 + 欧拉角（XYZ）构造。
    pub fn new_euler(position: Vec3, roll: f32, pitch: f32, yaw: f32) -> Self {
        Self {
            position,
            rotation: Quat::from_euler(roll, pitch, yaw),
        }
    }

    /// 复合：`self * o` 先应用 o（局部）再应用 self（世界）。
    pub fn compose(self, o: Self) -> Self {
        Self {
            position: self.rotation.rotate_vec3(o.position).add(self.position),
            rotation: self.rotation.mul(o.rotation),
        }
    }

    /// 逆位姿。
    pub fn inverse(self) -> Self {
        let r_inv = self.rotation.inverse();
        Self {
            rotation: r_inv,
            position: r_inv.rotate_vec3(self.position) * -1.0,
        }
    }

    /// 把点从局部变换到世界。
    pub fn transform_point(self, p: Vec3) -> Vec3 {
        self.rotation.rotate_vec3(p).add(self.position)
    }

    /// 变换一个方向向量（只旋转、不平移；例如把机体系速度转到世界系）。
    pub fn transform_direction(self, d: Vec3) -> Vec3 {
        self.rotation.rotate_vec3(d)
    }

    /// 把点从世界变换回局部（`transform_point` 的逆）。
    pub fn inverse_transform_point(self, p: Vec3) -> Vec3 {
        self.inverse().transform_point(p)
    }
}

impl std::ops::Mul<Pose> for Pose {
    type Output = Pose;
    fn mul(self, o: Pose) -> Pose {
        self.compose(o)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quat_rotate_and_inverse_roundtrip() {
        let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
        let v = Vec3::new(1.0, 0.0, 0.0);
        let r = q.rotate_vec3(v);
        assert!((r.x - 0.0).abs() < 1e-4 && (r.y - 1.0).abs() < 1e-4);
        let back = q.inverse().rotate_vec3(r);
        assert!((back.x - 1.0).abs() < 1e-4 && back.y.abs() < 1e-4);
    }

    #[test]
    fn pose_compose_and_inverse() {
        let a = Pose::from_translation(Vec3::new(1.0, 0.0, 0.0));
        let b = Pose::from_translation(Vec3::new(0.0, 2.0, 0.0));
        let ab = a.compose(b);
        assert!((ab.position.x - 1.0).abs() < 1e-4 && (ab.position.y - 2.0).abs() < 1e-4);
        let id = a.compose(a.inverse());
        assert!(id.position.norm() < 1e-4);
    }

    #[test]
    fn vec3_ops_dot_cross_norm() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert_eq!(a.dot(b), 32.0);
        let c = a.cross(b);
        assert_eq!(c, Vec3::new(-3.0, 6.0, -3.0));
        assert!((Vec3::new(3.0, 4.0, 0.0).norm() - 5.0).abs() < 1e-6);
        let n = Vec3::new(0.0, 3.0, 4.0).normalized();
        assert!((n.norm() - 1.0).abs() < 1e-6);
        assert!((n.y - 0.6).abs() < 1e-6);
    }

    #[test]
    fn vec3_zero_normalized_returns_zero() {
        assert_eq!(Vec3::ZERO.normalized(), Vec3::ZERO);
    }

    #[test]
    fn vec3_arithmetic_operators() {
        let a = Vec3::new(1.0, 2.0, 3.0);
        let b = Vec3::new(4.0, 5.0, 6.0);
        assert_eq!(a + b, Vec3::new(5.0, 7.0, 9.0));
        assert_eq!(b - a, Vec3::new(3.0, 3.0, 3.0));
        assert_eq!(a * 2.0, Vec3::new(2.0, 4.0, 6.0));
        assert_eq!(a.add(b), a + b);
        assert_eq!(b.sub(a), b - a);
    }

    #[test]
    fn quat_from_euler_and_rotate() {
        // 绕 Z 转 90°，把 +x 转到 +y。
        let q = Quat::from_euler(0.0, 0.0, std::f32::consts::FRAC_PI_2);
        let v = q.rotate_vec3(Vec3::new(1.0, 0.0, 0.0));
        assert!(v.x.abs() < 1e-4 && (v.y - 1.0).abs() < 1e-4);
        // 新构造的四元数应为单位四元数。
        let q2 = Quat::new(1.0, 2.0, 3.0, 4.0);
        assert!((q2.w * q2.w + q2.x * q2.x + q2.y * q2.y + q2.z * q2.z - 1.0).abs() < 1e-4);
    }

    #[test]
    fn quat_mul_composes_rotations() {
        let q1 = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
        let q2 = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
        // 两个 90° 相加 = 180°：+x -> -x。
        let q = q1.mul(q2);
        let v = q.rotate_vec3(Vec3::new(1.0, 0.0, 0.0));
        assert!(v.x < -0.999 && v.y.abs() < 1e-3);
        // 复合后取逆应还原。
        let back = q.mul(q.inverse()).rotate_vec3(Vec3::new(1.0, 0.0, 0.0));
        assert!(back.x.abs() - 1.0 < 1e-4 && back.y.abs() < 1e-4);
    }

    #[test]
    fn pose_transform_point_and_operator() {
        // 平移 + 旋转组合。
        let p = Pose::new_euler(
            Vec3::new(1.0, 0.0, 0.0),
            0.0,
            0.0,
            std::f32::consts::FRAC_PI_2,
        );
        // 原点局部点变换后 = 平移位置。
        let w = p.transform_point(Vec3::ZERO);
        assert!((w.x - 1.0).abs() < 1e-4 && w.y.abs() < 1e-4);
        // 局部 +x 变换到世界 +y 再平移。
        let w2 = p.transform_point(Vec3::new(1.0, 0.0, 0.0));
        assert!((w2.x - 1.0).abs() < 1e-4 && (w2.y - 1.0).abs() < 1e-4);
        // 操作符等价于 compose。
        assert_eq!(p * Pose::IDENTITY, p);
    }

    #[test]
    fn quat_to_euler_roundtrip() {
        let (r, p, y) = (0.3f32, -0.4, 0.8);
        let q = Quat::from_euler(r, p, y);
        let (r2, p2, y2) = q.to_euler();
        assert!((r2 - r).abs() < 1e-3, "roll {r2} vs {r}");
        assert!((p2 - p).abs() < 1e-3, "pitch {p2} vs {p}");
        assert!((y2 - y).abs() < 1e-3, "yaw {y2} vs {y}");
    }

    #[test]
    fn quat_dot_and_angle_to() {
        let id = Quat::IDENTITY;
        let z90 = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
        assert!((id.angle_to(z90) - std::f32::consts::FRAC_PI_2).abs() < 1e-4);
        assert!((id.dot(id) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn quat_slerp_endpoints_and_midpoint() {
        let a = Quat::IDENTITY;
        let b = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
        // 端点。
        assert!(Quat::slerp(a, b, 0.0).dot(a) > 0.999);
        assert!(Quat::slerp(a, b, 1.0).dot(b) > 0.999);
        // 中点：+x 应转到 45°。
        let mid = Quat::slerp(a, b, 0.5);
        let v = mid.rotate_vec3(Vec3::new(1.0, 0.0, 0.0));
        let half = std::f32::consts::FRAC_PI_4;
        assert!((v.x - half.cos()).abs() < 1e-3 && (v.y - half.sin()).abs() < 1e-3);
    }

    #[test]
    fn vec3_lerp_distance_finite() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(2.0, 0.0, 0.0);
        assert_eq!(Vec3::lerp(a, b, 0.5), Vec3::new(1.0, 0.0, 0.0));
        assert!((Vec3::distance(a, b) - 2.0).abs() < 1e-6);
        assert!(Vec3::new(1.0, 2.0, 3.0).is_finite());
        assert!(!Vec3::new(f32::NAN, 0.0, 0.0).is_finite());
        assert!(!Vec3::new(1.0, f32::INFINITY, 0.0).is_finite());
    }

    #[test]
    fn pose_transform_direction_and_inverse_point() {
        let p = Pose::new_euler(
            Vec3::new(1.0, 0.0, 0.0),
            0.0,
            0.0,
            std::f32::consts::FRAC_PI_2,
        );
        // 方向只旋转：局部 +x → 世界 +y（不含平移）。
        let d = p.transform_direction(Vec3::new(1.0, 0.0, 0.0));
        assert!((d.x - 0.0).abs() < 1e-4 && (d.y - 1.0).abs() < 1e-4);
        // 世界→局部是 局部→世界 的逆。用沿局部 z 的点（yaw 旋转不影响 z）。
        let world = p.transform_point(Vec3::new(0.0, 0.0, 0.5));
        let local = p.inverse_transform_point(world);
        assert!(local.x.abs() < 1e-4 && local.y.abs() < 1e-4 && (local.z - 0.5).abs() < 1e-4);
    }

    #[test]
    fn quat_to_euler_gimbal_lock_no_panic() {
        // pitch = ±90°（万向锁）不应 panic，且 round-trip 成立。
        for p in [-std::f32::consts::FRAC_PI_2, std::f32::consts::FRAC_PI_2] {
            let q = Quat::from_euler(0.3, p, 0.7);
            let (r2, p2, y2) = q.to_euler();
            assert!(r2.is_finite() && p2.is_finite() && y2.is_finite());
            // 俯仰应回到 ±90°（符号一致）。
            assert!((p2.abs() - std::f32::consts::FRAC_PI_2).abs() < 1e-3);
        }
    }

    #[test]
    fn quat_angle_to_zero_and_pi() {
        let id = Quat::IDENTITY;
        assert!(id.angle_to(id) < 1e-4);
        // 绕 Z 转 180° → 夹角 π。
        let pi = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::PI);
        assert!((id.angle_to(pi) - std::f32::consts::PI).abs() < 1e-3);
    }

    #[test]
    fn quat_slerp_identical_and_shortest_path() {
        let a = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.7);
        // slerp(a, a, t) = a（线性分支）。
        let same = Quat::slerp(a, a, 0.5);
        assert!(same.dot(a) > 0.9999);
        // 最短路径：b 取反表示同一旋转，结果仍应走短弧（与 a 夹角 < 90°）。
        let b = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.7 + std::f32::consts::PI);
        let mid = Quat::slerp(a, b, 0.5);
        // 中点应接近 a 旋转 +90°（约 0.7 + π/2），即与 a 相差约 90°。
        assert!((a.angle_to(mid) - std::f32::consts::FRAC_PI_2).abs() < 1e-2);
    }

    #[test]
    fn vec3_extra_operators() {
        let v = Vec3::new(2.0, 4.0, 6.0);
        assert_eq!(v / 2.0, Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(-v, Vec3::new(-2.0, -4.0, -6.0));
        assert_eq!(3.0 * v, Vec3::new(6.0, 12.0, 18.0));
        assert_eq!(
            v.component_mul(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::new(2.0, 8.0, 18.0)
        );
    }

    #[test]
    fn pose_from_rotation_translates_nothing() {
        let q = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), std::f32::consts::FRAC_PI_2);
        let p = Pose::from_rotation(q);
        assert!(p.position.norm() < 1e-6);
        // 变换原点仍为零（只有旋转）。
        let w = p.transform_point(Vec3::ZERO);
        assert!(w.norm() < 1e-6);
        // 变换局部 +x → 世界 +y。
        let d = p.transform_point(Vec3::new(1.0, 0.0, 0.0));
        assert!((d.y - 1.0).abs() < 1e-4);
    }
}
