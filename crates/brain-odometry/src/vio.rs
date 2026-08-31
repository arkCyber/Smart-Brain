//! 视觉惯性里程计（VIO）：融合高频 IMU 与低频视觉里程计。

use brain_core::time::Timestamp;
use brain_core::{Pose, Quat, Vec3};

use crate::imu::ImuIntegrator;
use crate::kalman::KalmanFusion3d;
use crate::sensor::{ImuSample, PointCloud};
use crate::vo::{IcpConfig, VisualOdometry};

/// 该估计的来源。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VioSource {
    /// 仅 IMU（航迹推算）。
    Imu,
    /// 仅视觉里程计。
    Visual,
    /// IMU + 视觉融合。
    Fused,
}

/// 一次 VIO 估计。
#[derive(Debug, Clone, Copy)]
pub struct VioEstimate {
    pub timestamp: Timestamp,
    pub pose: Pose,
    pub source: VioSource,
}

/// 视觉惯性里程计。
///
/// 室内无 GPS 时，用它建立厘米级三维局部坐标系：姿态主要由 IMU（陀螺）提供
/// 高频更新，位置由视觉里程计校正漂移；二者按各自刷新率叠加。
pub struct VisualInertialOdometry {
    imu: ImuIntegrator,
    vo: VisualOdometry,
    /// 位置/速度的卡尔曼融合（IMU 预测 + VO 测量）。
    kalman: KalmanFusion3d,
    /// 世界系重力（用于把机体系加速度转到世界系）。
    gravity: Vec3,
    /// 视觉是否已初始化（第一帧）。
    vo_ready: bool,
    last_ts: u64,
    last_imu_ts: u64,
    estimate: VioEstimate,
    /// 视觉位姿（原始 VO，未滤波，供参考）。
    visual_pose: Pose,
}

impl Default for VisualInertialOdometry {
    fn default() -> Self {
        Self::new(IcpConfig::default())
    }
}

impl VisualInertialOdometry {
    pub fn new(vo_cfg: IcpConfig) -> Self {
        Self {
            imu: ImuIntegrator::new(),
            vo: VisualOdometry::new(vo_cfg),
            kalman: KalmanFusion3d::new(Vec3::ZERO, 0.05, 0.1),
            gravity: Vec3::new(0.0, 0.0, -9.81),
            vo_ready: false,
            last_ts: 0,
            last_imu_ts: 0,
            estimate: VioEstimate {
                timestamp: 0,
                pose: Pose::IDENTITY,
                source: VioSource::Imu,
            },
            visual_pose: Pose::IDENTITY,
        }
    }

    /// 设置卡尔曼过程/测量噪声（默认 0.05 / 0.1）。
    pub fn set_kalman_noise(&mut self, process_noise: f32, measurement_noise: f32) {
        self.kalman = KalmanFusion3d::new(self.kalman.position(), process_noise, measurement_noise);
    }

    /// 设置初始姿态（如由电子罗盘/水平仪提供）。
    pub fn set_initial_attitude(&mut self, q: Quat) {
        self.imu.set_attitude(q);
        self.visual_pose = Pose {
            position: self.visual_pose.position,
            rotation: q,
        };
    }

    /// 输入一帧 IMU 样本（高频）：预测卡尔曼，并做姿态/位置航迹推算。
    pub fn update_imu(&mut self, sample: &ImuSample, now: u64) -> VioEstimate {
        // 用机体系加速度 + 姿态 → 世界系加速度（去重力），预测卡尔曼。
        if self.last_imu_ts != 0 {
            let dt = (now.saturating_sub(self.last_imu_ts)) as f32 / 1000.0;
            let att = self.imu.attitude();
            let accel_world = att.rotate_vec3(sample.accel).sub(self.gravity);
            self.kalman.predict(accel_world, dt.clamp(0.0, 0.1));
        }
        self.last_imu_ts = now;

        self.imu.integrate(sample, now);
        self.last_ts = now;
        let imu_pose = self.imu.pose();
        self.estimate = VioEstimate {
            timestamp: now,
            pose: imu_pose,
            source: VioSource::Imu,
        };
        self.estimate
    }

    /// 输入一帧点云（低频，来自 RGB-D 相机）：用卡尔曼校正位置。
    pub fn update_frame(&mut self, cloud: &PointCloud, now: u64) -> Option<VioEstimate> {
        let vo_pose = self.vo.process(cloud)?;
        let was_ready = self.vo_ready;
        self.vo_ready = true;
        self.visual_pose = vo_pose;
        self.last_ts = now;

        // 首帧：用 VO 位置初始化卡尔曼。
        if !was_ready {
            self.kalman.set_position(vo_pose.position);
        }
        // 用 VO 位置测量校正卡尔曼（抑制测量噪声）。
        self.kalman.update(vo_pose.position);

        // 融合：姿态取 IMU（陀螺短时可信），位置取卡尔曼滤波后的（低噪声、抗漂移）。
        let fused = Pose {
            position: self.kalman.position(),
            rotation: self.imu.attitude(),
        };
        self.estimate = VioEstimate {
            timestamp: now,
            pose: fused,
            source: VioSource::Fused,
        };
        Some(self.estimate)
    }

    /// 最近一次估计。
    pub fn estimate(&self) -> &VioEstimate {
        &self.estimate
    }

    /// 视觉是否已初始化。
    pub fn is_visual_ready(&self) -> bool {
        self.vo_ready
    }

    /// 视觉里程计位姿（位置来源）。
    pub fn visual_pose(&self) -> Pose {
        self.visual_pose
    }

    /// 卡尔曼滤波后的位置。
    pub fn filtered_position(&self) -> Vec3 {
        self.kalman.position()
    }

    /// 卡尔曼滤波后的速度。
    pub fn filtered_velocity(&self) -> Vec3 {
        self.kalman.velocity()
    }

    /// 重置（例如跟踪丢失后重新初始化）。
    pub fn reset(&mut self) {
        self.imu = ImuIntegrator::new();
        self.vo.reset();
        self.kalman = KalmanFusion3d::new(Vec3::ZERO, 0.05, 0.1);
        self.vo_ready = false;
        self.last_ts = 0;
        self.last_imu_ts = 0;
        self.visual_pose = Pose::IDENTITY;
        self.estimate = VioEstimate {
            timestamp: 0,
            pose: Pose::IDENTITY,
            source: VioSource::Imu,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 合成一个立方体点云，并施加"平移 + 确定性噪声"以模拟带噪声的 VO。
    fn noisy_cube(off: f32, noise: f32) -> PointCloud {
        (0..8)
            .map(|i| {
                let p = Vec3::new((i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32)
                    .add(Vec3::new(off, 0.0, 0.0));
                // 确定性噪声（仅 x）。
                Vec3::new(p.x + noise * (i as f32).sin(), p.y, p.z)
            })
            .collect()
    }

    #[test]
    fn kalman_smooths_vio_position() {
        let mut vio = VisualInertialOdometry::new(IcpConfig::default());
        let mut now = 0u64;

        // 静止 IMU（仅重力，无运动）。
        let imu = |ts: u64| ImuSample::new(ts, Vec3::new(0.0, 0.0, -9.81), Vec3::ZERO);

        // 首帧。
        vio.update_imu(&imu(now), now);
        now += 10;
        let first = vio.update_frame(&noisy_cube(0.0, 0.1), now).unwrap();
        assert_eq!(first.source, VioSource::Fused);

        // 相机缓慢向右平移（+x），带噪声。
        let mut true_x = 0.0f32;
        for step in 1..=20 {
            for _ in 0..10 {
                vio.update_imu(&imu(now), now);
                now += 10;
            }
            true_x = step as f32 * 0.1;
            // 立方体向 -x 移动 → 相机（VO）向 +x 运动。
            vio.update_frame(&noisy_cube(-true_x, 0.1), now).unwrap();
        }

        // 卡尔曼滤波后的位置应接近真值（无漂移）。
        let p = vio.filtered_position();
        assert!((p.x - true_x).abs() < 0.5, "x={} true={true_x}", p.x);
        assert!(p.y.abs() < 0.5, "y drift {}", p.y);
        // 姿态来自 IMU（初始恒等）。
        assert!(vio.estimate().pose.rotation.w.abs() > 0.99);
    }
    #[test]
    fn empty_cloud_no_panic() {
        let mut vio = VisualInertialOdometry::new(IcpConfig::default());
        let now = 0u64;
        // 空点云不应 panic（VO 返回恒等，卡尔曼随之校正）。
        let est = vio
            .update_frame(&PointCloud::new(), now)
            .expect("some estimate");
        assert!(est.source == VioSource::Fused);
        let est2 = vio
            .update_frame(&PointCloud::new(), now + 1)
            .expect("some estimate");
        assert!(est2.pose.position.x.is_finite());
    }

    #[test]
    fn zero_dt_imu_no_panic() {
        let mut vio = VisualInertialOdometry::new(IcpConfig::default());
        let s = ImuSample::new(100, Vec3::new(0.0, 0.0, -9.81), Vec3::ZERO);
        vio.update_imu(&s, 100);
        // 同一时间戳再喂一次 → dt=0，不应 panic。
        vio.update_imu(&s, 100);
        assert!(vio.filtered_position().x.is_finite());
    }

    #[test]
    fn stress_many_imu_samples() {
        let mut vio = VisualInertialOdometry::new(IcpConfig::default());
        let mut now = 0u64;
        // 10000 帧 IMU（静止，仅重力）→ 位置保持有限且接近 0。
        for _ in 0..10_000 {
            let s = ImuSample::new(now, Vec3::new(0.0, 0.0, -9.81), Vec3::ZERO);
            vio.update_imu(&s, now);
            now += 10;
        }
        let p = vio.filtered_position();
        assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
        assert!(
            p.x.abs() < 0.01 && p.y.abs() < 0.01,
            "stationary drift {:?}",
            p
        );
    }

    #[test]
    fn stress_many_frames() {
        let mut vio = VisualInertialOdometry::new(IcpConfig::default());
        let mut now = 0u64;
        let imu = |ts: u64| ImuSample::new(ts, Vec3::new(0.0, 0.0, -9.81), Vec3::ZERO);
        // 1000 帧：立方体向 -x 移动 → 相机向 +x。
        for step in 1..=1000 {
            for _ in 0..5 {
                vio.update_imu(&imu(now), now);
                now += 10;
            }
            let off = -step as f32 * 0.01;
            vio.update_frame(&noisy_cube(off, 0.05), now).unwrap();
        }
        let p = vio.filtered_position();
        assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite());
        // 相机应沿 +x 前进且量级合理（存在 VO 噪声与卡尔曼滞后，不做精确断言）。
        assert!(p.x > 2.0 && p.x < 15.0, "x={}", p.x);
    }

    #[test]
    fn reset_reinitializes() {
        let mut vio = VisualInertialOdometry::new(IcpConfig::default());
        let mut now = 0u64;
        let imu = |ts: u64| ImuSample::new(ts, Vec3::new(0.0, 0.0, -9.81), Vec3::ZERO);
        vio.update_imu(&imu(now), now);
        now += 10;
        vio.update_frame(&noisy_cube(0.0, 0.05), now).unwrap();
        assert!(vio.is_visual_ready());
        assert!(vio.filtered_position().x.abs() < 0.5);

        vio.reset();
        assert!(!vio.is_visual_ready());
        assert!(vio.estimate().source == VioSource::Imu);
        assert!(vio.filtered_position().x.abs() < 1e-3);
    }
}
