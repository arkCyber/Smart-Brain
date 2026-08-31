# brain-odometry

> Smart-Brain visual-inertial odometry: IMU, RGB-D visual odometry, VIO fusion

**所属层**：第 4 层 · 定位核心 —— 室内无 GPS 的厘米级自定位（VIO）。

## 职责

用视觉（RGB-D）+ 惯导（IMU）替代 GPS，建立厘米级三维局部坐标系。

- `sensor`：IMU / 针孔相机 / 深度帧模型，深度 → 点云
- `buffer`：多传感器时间戳对齐（IMU 高频 ↔ 相机低频，绝不丢包）
- `imu`：IMU 姿态 / 位置积分（航迹推算）
- `kabsch` / `vo`：RGB-D 视觉里程计（帧间 ICP 求刚体变换）
- `vio`：融合 IMU 与视觉的最终里程计估计
- `kalman`：`KalmanFilter` / `KalmanFusion3d`（扩展卡尔曼融合）

## 核心 API

```rust
pub use buffer::{TimestampAligned, align_to, AlignmentError};
pub use imu::ImuIntegrator;
pub use kalman::{KalmanFilter, KalmanFusion3d};
pub use sensor::{CameraModel, DepthFrame, ImuSample, PinholeCamera};
pub use stereo::{StereoMatcher, StereoResult};
pub use vio::{VioEstimate, VisualInertialOdometry};
pub use vo::{VisualOdometry, IcpConfig};
```

## 用法

```rust
use brain_core::Vec3;
use brain_odometry::sensor::ImuSample;
use brain_odometry::ImuIntegrator;

fn main() {
    let mut imu = ImuIntegrator::new();
    imu.set_gravity(Vec3::ZERO);
    let mut t = 0u64;
    for _ in 0..100 {
        imu.integrate(&ImuSample::new(t, Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO), t);
        t += 10;
    }
    println!("pos = {:?}", imu.position());
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-ipc`

> **应用案例**：`brain-node/kalman_demo.rs`（VIO/卡尔曼融合）、`stereo_demo.rs`（双目视觉）。
