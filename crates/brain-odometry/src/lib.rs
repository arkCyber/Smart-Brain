//! `brain-odometry` — 视觉惯性里程计（VIO），室内无 GPS 的厘米级自定位。
//!
//! 对应参考架构第 4 层“定位核心”：用视觉（RGB-D）+ 惯导（IMU）替代 GPS，
//! 建立厘米级的三维局部坐标系。本 crate 提供：
//! - `sensor`：IMU / 针孔相机 / 深度帧模型，深度→点云
//! - `buffer`：多传感器时间戳对齐（IMU 高频 ↔ 相机低频，绝不丢包）
//! - `imu`：IMU 姿态 / 位置积分（航迹推算）
//! - `kabsch` / `vo`：RGB-D 视觉里程计（帧间 ICP 求刚体变换）
//! - `vio`：融合 IMU 与视觉的最终里程计估计

pub mod buffer;
pub mod imu;
pub mod kabsch;
pub mod kalman;
pub mod sensor;
pub mod stereo;
pub mod vio;
pub mod vo;

pub use buffer::{align_to, AlignmentError, TimestampAligned};
pub use imu::ImuIntegrator;
pub use kalman::{KalmanFilter, KalmanFusion3d};
pub use sensor::{
    cloud_centroid, cloud_transform, depth_to_pointcloud, depth_to_pointcloud_subsampled,
    DepthFrame, ImuSample, PinholeCamera, PointCloud,
};
pub use stereo::{
    compute_disparity, disparity_to_pointcloud, process_stereo, StereoCamera, StereoResult,
};
pub use vio::{VioEstimate, VisualInertialOdometry};
pub use vo::{IcpConfig, VisualOdometry};
