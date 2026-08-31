//! 卡尔曼滤波传感器融合演示。
//!
//! 用 `KalmanFusion3d` 融合"IMU 预测 + 视觉里程计位置测量"，对比原始测量
//! 与滤波后的位置误差，直观展示噪声抑制。

use brain_core::Vec3;
use brain_odometry::KalmanFusion3d;

/// 顶层入口。
pub fn run() {
    println!("\n=== 卡尔曼滤波传感器融合（IMU 预测 + VO 测量）===");

    // 真值：沿 x 匀速运动，速度 1 m/s，运行 10s。
    let v = 1.0f32;
    let dt = 0.1f32;
    let noise = 0.3f32; // 测量噪声幅度
    let mut kf = KalmanFusion3d::new(Vec3::ZERO, 0.01, noise * noise);

    let mut t = 0.0f32;
    let mut raw_err = 0.0f32;
    let mut filt_err = 0.0f32;
    let n = 100;
    for _ in 0..n {
        kf.predict(Vec3::ZERO, dt); // 匀速：加速度 0
        t += dt;
        let true_x = v * t;
        // 确定性噪声测量（用正弦模拟 VO 抖动）。
        let zx = true_x + noise * (t * 3.0).sin();
        let z = Vec3::new(zx, 0.0, 0.0);
        kf.update(z);

        raw_err += (zx - true_x).abs();
        filt_err += (kf.position().x - true_x).abs();
    }

    let raw_avg = raw_err / n as f32;
    let filt_avg = filt_err / n as f32;
    let reduction = (1.0 - filt_avg / raw_avg.max(1e-6)) * 100.0;
    println!("  原始测量平均误差: {raw_avg:.3} m");
    println!("  卡尔曼滤波平均误差: {filt_avg:.3} m");
    println!("  噪声抑制率: {reduction:.0}%");
}
