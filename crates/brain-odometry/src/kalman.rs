//! 卡尔曼滤波传感器融合。
//!
//! 用一个**恒加速模型**的一维卡尔曼滤波器，把高频 IMU 预测与低频视觉里程计
//! 测量融合成平滑的位置/速度估计。三个轴解耦独立滤波（`KalmanFusion3d`），
//! 常用于替代简单的"直接用 VO 位置"以抑制测量噪声。

use brain_core::Vec3;

/// 一维恒加速卡尔曼滤波器（状态 = `[位置, 速度]`）。
pub struct KalmanFilter {
    // 状态。
    pos: f32,
    vel: f32,
    // 2×2 协方差（对称，只存 p11,p12,p22）。
    p11: f32,
    p12: f32,
    p22: f32,
    /// 过程噪声（加速度白噪声强度）。
    q: f32,
    /// 测量噪声方差。
    r: f32,
}

impl KalmanFilter {
    /// 初始位置、速度、过程噪声、测量噪声。
    pub fn new(pos: f32, vel: f32, process_noise: f32, measurement_noise: f32) -> Self {
        Self {
            pos,
            vel,
            p11: 1.0,
            p12: 0.0,
            p22: 1.0,
            q: process_noise,
            r: measurement_noise,
        }
    }

    /// 预测：用加速度 `accel` 与步长 `dt` 前推状态与协方差。
    pub fn predict(&mut self, accel: f32, dt: f32) {
        // 恒加速运动模型。
        self.pos += self.vel * dt + 0.5 * accel * dt * dt;
        self.vel += accel * dt;
        // F = [[1, dt],[0, 1]]；P' = F P F^T + Q。
        let (p11, p12, p22) = (self.p11, self.p12, self.p22);
        self.p11 = p11 + 2.0 * dt * p12 + dt * dt * p22 + self.q * dt.powi(4) / 4.0;
        self.p12 = p12 + dt * p22 + self.q * dt.powi(3) / 2.0;
        self.p22 = p22 + self.q * dt * dt;
    }

    /// 用位置测量 `z` 校正。
    pub fn update(&mut self, z: f32) {
        let y = z - self.pos; // 新息
        let s = self.p11 + self.r;
        if s.abs() < 1e-12 {
            return;
        }
        let k1 = self.p11 / s;
        let k2 = self.p12 / s;
        // 状态更新。
        self.pos += k1 * y;
        self.vel += k2 * y;
        // 协方差更新：P' = (I - K H) P。
        let (p11, p12, p22) = (self.p11, self.p12, self.p22);
        self.p11 = (1.0 - k1) * p11;
        self.p12 = (1.0 - k1) * p12;
        self.p22 = p22 - k2 * p12;
    }

    /// 滤波后的位置。
    pub fn position(&self) -> f32 {
        self.pos
    }
    /// 滤波后的速度。
    pub fn velocity(&self) -> f32 {
        self.vel
    }
    /// 直接设置位置（用于首帧初始化）。
    pub fn set_position(&mut self, pos: f32) {
        self.pos = pos;
    }
}

/// 三轴解耦的卡尔曼融合（位置/速度），用于融合 IMU 预测 + VO 测量。
pub struct KalmanFusion3d {
    x: KalmanFilter,
    y: KalmanFilter,
    z: KalmanFilter,
}

impl KalmanFusion3d {
    /// 初始位姿、过程噪声、测量噪声。
    pub fn new(pos: Vec3, process_noise: f32, measurement_noise: f32) -> Self {
        Self {
            x: KalmanFilter::new(pos.x, 0.0, process_noise, measurement_noise),
            y: KalmanFilter::new(pos.y, 0.0, process_noise, measurement_noise),
            z: KalmanFilter::new(pos.z, 0.0, process_noise, measurement_noise),
        }
    }

    /// 预测：用 IMU 加速度与步长。
    pub fn predict(&mut self, accel: Vec3, dt: f32) {
        self.x.predict(accel.x, dt);
        self.y.predict(accel.y, dt);
        self.z.predict(accel.z, dt);
    }

    /// 校正：用测量位置。
    pub fn update(&mut self, measured: Vec3) {
        self.x.update(measured.x);
        self.y.update(measured.y);
        self.z.update(measured.z);
    }

    /// 直接设置位置（用于首帧初始化）。
    pub fn set_position(&mut self, pos: Vec3) {
        self.x.set_position(pos.x);
        self.y.set_position(pos.y);
        self.z.set_position(pos.z);
    }

    /// 滤波后位置。
    pub fn position(&self) -> Vec3 {
        Vec3::new(self.x.position(), self.y.position(), self.z.position())
    }
    /// 滤波后速度。
    pub fn velocity(&self) -> Vec3 {
        Vec3::new(self.x.velocity(), self.y.velocity(), self.z.velocity())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 恒速轨迹 + 噪声测量，滤波应跟踪真值且误差小于测量噪声。
    #[test]
    fn tracks_constant_velocity() {
        let true_v = 2.0f32;
        let true_x0 = 0.0f32;
        let dt = 0.1f32;
        let noise = 0.5f32;
        let mut kf = KalmanFilter::new(true_x0, 0.0, 0.01, noise * noise);
        let mut t = 0.0f32;
        for _ in 0..100 {
            kf.predict(0.0, dt); // 恒速：加速度 0
            t += dt;
            // 带噪声的测量（用固定相位代替随机以保持确定）。
            let z = true_x0 + true_v * t + noise * (t * 3.0).sin();
            kf.update(z);
        }
        let true_end = true_x0 + true_v * t;
        let err = (kf.position() - true_end).abs();
        assert!(
            err < noise,
            "filter error {err} should be below noise {noise}"
        );
    }

    /// 恒定真值 + 噪声，滤波应收敛到真值附近。
    #[test]
    fn smooths_noise_around_constant() {
        let true_val = 5.0f32;
        let noise = 1.0f32;
        let mut kf = KalmanFilter::new(0.0, 0.0, 1e-4, noise * noise);
        for i in 0..200 {
            kf.predict(0.0, 0.1);
            let z = true_val + noise * ((i as f32) * 0.7).cos();
            kf.update(z);
        }
        assert!(
            (kf.position() - true_val).abs() < 0.2,
            "pos={}",
            kf.position()
        );
    }

    /// 恒加速：预测后位置/速度符合运动学。
    #[test]
    fn prediction_uses_acceleration() {
        let mut kf = KalmanFilter::new(0.0, 0.0, 0.0, 0.0);
        let dt = 1.0f32;
        kf.predict(2.0, dt); // a=2, v0=0, x0=0 → x=1, v=2
        assert!((kf.position() - 1.0).abs() < 1e-4, "pos={}", kf.position());
        assert!((kf.velocity() - 2.0).abs() < 1e-4, "vel={}", kf.velocity());
    }

    #[test]
    fn fusion3d_updates_all_axes() {
        let mut f = KalmanFusion3d::new(Vec3::ZERO, 0.01, 0.1);
        f.predict(Vec3::new(1.0, 0.0, 0.0), 0.1);
        f.update(Vec3::new(1.0, 2.0, 3.0));
        let p = f.position();
        // x 受预测+测量影响，y/z 收敛到测量。
        assert!(p.x > 0.5);
        assert!((p.y - 2.0).abs() < 0.5);
        assert!((p.z - 3.0).abs() < 0.5);
    }
}
