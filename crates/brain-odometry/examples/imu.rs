//! `brain-odometry` 最小示例：IMU 航迹推算（无 GPS 时的自定位）。
//!
//! 运行：`cargo run -p brain-odometry --example imu`

use brain_core::Vec3;
use brain_odometry::sensor::ImuSample;
use brain_odometry::ImuIntegrator;

fn main() {
    let mut imu = ImuIntegrator::new();
    imu.set_gravity(Vec3::ZERO); // 简化：忽略重力，单独观测水平加速度

    // 恒定 1 m/s² 沿 +X 加速度，10ms 采样，共 1 秒
    let mut t = 0u64;
    for _ in 0..100 {
        imu.integrate(&ImuSample::new(t, Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO), t);
        t += 10;
    }

    // 理论位置 = 0.5·a·t² = 0.5 m
    println!("after 1s of 1 m/s²: pos = {:?} vel = {:?}", imu.position(), imu.velocity());
}
