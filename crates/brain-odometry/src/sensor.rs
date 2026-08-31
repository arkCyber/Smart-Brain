//! 传感器模型：IMU、针孔相机、深度帧与点云转换。

use brain_core::time::Timestamp;
use brain_core::Vec3;

/// IMU 样本。
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub timestamp: Timestamp,
    /// 加速度（机体系，m/s²）。
    pub accel: Vec3,
    /// 角速度（机体系，rad/s）。
    pub gyro: Vec3,
}

impl ImuSample {
    pub fn new(timestamp: Timestamp, accel: Vec3, gyro: Vec3) -> Self {
        Self {
            timestamp,
            accel,
            gyro,
        }
    }
}

/// 针孔相机模型。
#[derive(Debug, Clone, Copy)]
pub struct PinholeCamera {
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub width: usize,
    pub height: usize,
}

impl PinholeCamera {
    /// 把相机系点反投影到图像坐标（用于可视化/对齐）。
    pub fn project(&self, p: Vec3) -> Option<(f32, f32)> {
        if p.z <= 0.0 {
            return None;
        }
        Some((self.fx * p.x / p.z + self.cx, self.fy * p.y / p.z + self.cy))
    }

    /// 由像素坐标 + 深度反投影到相机系三维点。
    pub fn unproject(&self, u: f32, v: f32, depth: f32) -> Option<Vec3> {
        if depth <= 0.0 {
            return None;
        }
        Some(Vec3::new(
            (u - self.cx) / self.fx * depth,
            (v - self.cy) / self.fy * depth,
            depth,
        ))
    }
}

/// 深度帧：`depth` 长度为 width*height，单位米，0 表示无效。
pub struct DepthFrame {
    pub timestamp: Timestamp,
    pub width: usize,
    pub height: usize,
    pub depth: Vec<f32>,
}

/// 相机系三维点云。
pub type PointCloud = Vec<Vec3>;

/// 把深度帧转成相机系点云（跳过无效深度）。
pub fn depth_to_pointcloud(frame: &DepthFrame, cam: &PinholeCamera) -> PointCloud {
    let mut pts = PointCloud::new();
    for v in 0..frame.height {
        for u in 0..frame.width {
            let d = frame.depth[v * frame.width + u];
            if let Some(p) = cam.unproject(u as f32, v as f32, d) {
                pts.push(p);
            }
        }
    }
    pts
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pinhole_roundtrip() {
        let cam = PinholeCamera {
            fx: 500.0,
            fy: 500.0,
            cx: 320.0,
            cy: 240.0,
            width: 640,
            height: 480,
        };
        let p = Vec3::new(0.5, 0.3, 2.0);
        let (u, v) = cam.project(p).unwrap();
        let back = cam.unproject(u, v, p.z).unwrap();
        assert!((back.x - p.x).abs() < 1e-3);
        assert!((back.y - p.y).abs() < 1e-3);
    }

    #[test]
    fn depth_to_cloud_skips_invalid() {
        let cam = PinholeCamera {
            fx: 1.0,
            fy: 1.0,
            cx: 0.0,
            cy: 0.0,
            width: 2,
            height: 1,
        };
        let frame = DepthFrame {
            timestamp: 0,
            width: 2,
            height: 1,
            depth: vec![0.0, 2.0], // 第一像素无效
        };
        let cloud = depth_to_pointcloud(&frame, &cam);
        assert_eq!(cloud.len(), 1);
        // 有效像素 (u=1, v=0, d=2)：x=(1-0)/1*2=2。
        assert_eq!(cloud[0], Vec3::new(2.0, 0.0, 2.0));
    }
}
