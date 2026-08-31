//! IMU 积分：用陀螺与加速度计做姿态/位置航迹推算（无 GPS 时的“方位感”）。

use brain_core::{Pose, Quat, Vec3};

use crate::sensor::ImuSample;

/// IMU 航迹推算器。
///
/// 以机体系 IMU 数据积分出世界系下的姿态（四元数）、速度与位置。
/// 加速度计读数需减去重力以得到真实运动加速度。
pub struct ImuIntegrator {
    /// 世界系姿态（机体 → 世界）。
    attitude: Quat,
    position: Vec3,
    velocity: Vec3,
    /// 世界系重力加速度（默认沿 -Z）。
    gravity: Vec3,
    /// 上一次积分时间戳。
    last_ts: Option<u64>,
}

impl Default for ImuIntegrator {
    fn default() -> Self {
        Self::new()
    }
}

impl ImuIntegrator {
    pub fn new() -> Self {
        Self {
            attitude: Quat::IDENTITY,
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            gravity: Vec3::new(0.0, 0.0, -9.81),
            last_ts: None,
        }
    }

    /// 设置初始姿态。
    pub fn set_attitude(&mut self, q: Quat) {
        self.attitude = q;
    }

    /// 设置重力向量（用于不同标定/场景）。
    pub fn set_gravity(&mut self, g: Vec3) {
        self.gravity = g;
    }

    /// 世界系姿态。
    pub fn attitude(&self) -> Quat {
        self.attitude
    }

    /// 世界系位置。
    pub fn position(&self) -> Vec3 {
        self.position
    }

    /// 世界系速度。
    pub fn velocity(&self) -> Vec3 {
        self.velocity
    }

    /// 当前世界系位姿。
    pub fn pose(&self) -> Pose {
        Pose {
            position: self.position,
            rotation: self.attitude,
        }
    }

    /// 积分一个 IMU 样本。`now` 提供时间（毫秒），用于计算 dt。
    pub fn integrate(&mut self, sample: &ImuSample, now: u64) {
        let dt_s = match self.last_ts {
            Some(prev) => (now.saturating_sub(prev)) as f32 / 1000.0,
            None => {
                self.last_ts = Some(now);
                return;
            }
        };
        let dt = dt_s.clamp(0.0, 0.1);

        // 1) 陀螺 → 姿态增量。
        let w = sample.gyro;
        let ang = w.norm() * dt;
        if ang > 1e-12 {
            let axis = w.normalized();
            let dq = Quat::from_axis_angle(axis, ang);
            self.attitude = self.attitude.mul(dq);
        }

        // 2) 加速度（机体系）→ 世界系，去除重力 → 运动加速度。
        let accel_world = self.attitude.rotate_vec3(sample.accel).sub(self.gravity);
        self.velocity = self.velocity.add(accel_world * dt);
        self.position = self.position.add(self.velocity * dt);

        self.last_ts = Some(now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constant_accel_integrates_position() {
        let mut imu = ImuIntegrator::new();
        // 机体系 accel = +g 沿 +Z（补偿重力后为零），加一个小水平加速度测试。
        // 为简化：设重力为零，输入恒定加速度 1 m/s² 沿 +X。
        imu.set_gravity(Vec3::ZERO);
        let dt_ms = 10;
        let mut t = 0u64;
        for _ in 0..100 {
            let sample = ImuSample::new(t, Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO);
            imu.integrate(&sample, t);
            t += dt_ms;
        }
        // 1 s 后位置应为 0.5 * a * t^2 = 0.5 m。
        let p = imu.position();
        assert!((p.x - 0.5).abs() < 0.02, "x={}", p.x);
    }

    #[test]
    fn gyro_rotates_attitude() {
        let mut imu = ImuIntegrator::new();
        imu.set_gravity(Vec3::ZERO);
        // 绕 Z 轴 90°/s，转 1 s → 90°。
        let mut t = 0u64;
        for _ in 0..100 {
            imu.integrate(
                &ImuSample::new(
                    t,
                    Vec3::ZERO,
                    Vec3::new(0.0, 0.0, std::f32::consts::FRAC_PI_2),
                ),
                t,
            );
            t += 10;
        }
        // 把 +X 向量旋转后应接近 +Y。
        let v = imu.attitude().rotate_vec3(Vec3::new(1.0, 0.0, 0.0));
        assert!((v.y - 1.0).abs() < 0.1, "vy={}", v.y);
    }
}
