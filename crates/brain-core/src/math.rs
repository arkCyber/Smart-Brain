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
}
