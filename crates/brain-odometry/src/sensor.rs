//! 传感器模型：IMU、针孔相机、深度帧与点云转换。

use brain_core::time::Timestamp;
use brain_core::{Pose, Vec3};

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
    /// 由视场角（水平/垂直，弧度）与分辨率构造相机内参。
    pub fn from_fov(fov_x: f32, fov_y: f32, width: usize, height: usize) -> Self {
        let half_w = width as f32 * 0.5;
        let half_h = height as f32 * 0.5;
        let fx = if fov_x > 1e-6 {
            half_w / (fov_x * 0.5).tan()
        } else {
            half_w
        };
        let fy = if fov_y > 1e-6 {
            half_h / (fov_y * 0.5).tan()
        } else {
            half_h
        };
        Self {
            fx,
            fy,
            cx: half_w,
            cy: half_h,
            width,
            height,
        }
    }

    /// 水平视场角（弧度）。
    pub fn fov_x(&self) -> f32 {
        2.0 * (self.width as f32 / (2.0 * self.fx)).atan()
    }

    /// 垂直视场角（弧度）。
    pub fn fov_y(&self) -> f32 {
        2.0 * (self.height as f32 / (2.0 * self.fy)).atan()
    }

    /// 宽高比。
    pub fn aspect_ratio(&self) -> f32 {
        self.width as f32 / self.height as f32
    }

    /// 像素坐标是否落在图像范围内。
    pub fn contains(&self, u: f32, v: f32) -> bool {
        u >= 0.0 && v >= 0.0 && (u as usize) < self.width && (v as usize) < self.height
    }

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

impl DepthFrame {
    pub fn new(timestamp: Timestamp, width: usize, height: usize, depth: Vec<f32>) -> Self {
        Self {
            timestamp,
            width,
            height,
            depth,
        }
    }

    /// 取 (u, v) 处深度（越界返回 None）。
    pub fn depth_at(&self, u: usize, v: usize) -> Option<f32> {
        if u >= self.width || v >= self.height {
            return None;
        }
        self.depth.get(v * self.width + u).copied()
    }

    /// 行优先索引（越界返回 None，避免索引穿透）。
    pub fn depth_at_index(&self, idx: usize) -> Option<f32> {
        self.depth.get(idx).copied()
    }
}

/// 相机系三维点云。
pub type PointCloud = Vec<Vec3>;

/// 把深度帧转成相机系点云（跳过无效深度与越界索引，对短缓冲区安全）。
pub fn depth_to_pointcloud(frame: &DepthFrame, cam: &PinholeCamera) -> PointCloud {
    depth_to_pointcloud_subsampled(frame, cam, 1)
}

/// 按步长 `step` 采样深度帧转点云：降低点数（大分辨率深度帧常用）。
pub fn depth_to_pointcloud_subsampled(
    frame: &DepthFrame,
    cam: &PinholeCamera,
    step: usize,
) -> PointCloud {
    let step = step.max(1);
    let mut pts = PointCloud::new();
    let mut v = 0;
    while v < frame.height {
        let mut u = 0;
        while u < frame.width {
            if let Some(d) = frame.depth_at(u, v) {
                if let Some(p) = cam.unproject(u as f32, v as f32, d) {
                    pts.push(p);
                }
            }
            u += step;
        }
        v += step;
    }
    pts
}

/// 点云质心（空点云返回 None）。
pub fn cloud_centroid(cloud: &PointCloud) -> Option<Vec3> {
    if cloud.is_empty() {
        return None;
    }
    let mut s = Vec3::ZERO;
    for &p in cloud {
        s = s.add(p);
    }
    Some(s * (1.0 / cloud.len() as f32))
}

/// 用位姿 `pose` 变换（旋转 + 平移）整片点云。
pub fn cloud_transform(cloud: &PointCloud, pose: &Pose) -> PointCloud {
    cloud.iter().map(|&p| pose.transform_point(p)).collect()
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

    #[test]
    fn camera_from_fov_and_accessors() {
        let w = 640usize;
        let h = 480usize;
        let cam = PinholeCamera::from_fov(1.2, 0.9, w, h);
        // fx ≈ (w/2)/tan(fov_x/2)，中心近似在图像中点。
        assert!((cam.cx - 320.0).abs() < 1e-3);
        assert!((cam.cy - 240.0).abs() < 1e-3);
        // 视场角可往返。
        assert!((cam.fov_x() - 1.2).abs() < 1e-2, "fov_x={}", cam.fov_x());
        assert!((cam.fov_y() - 0.9).abs() < 1e-2, "fov_y={}", cam.fov_y());
        assert!((cam.aspect_ratio() - (w as f32 / h as f32)).abs() < 1e-6);
    }

    #[test]
    fn camera_contains_bounds() {
        let cam = PinholeCamera::from_fov(1.0, 0.8, 320, 240);
        assert!(cam.contains(0.0, 0.0));
        assert!(cam.contains(319.0, 239.0));
        assert!(!cam.contains(320.0, 0.0));
        assert!(!cam.contains(0.0, 240.0));
        assert!(!cam.contains(-1.0, 5.0));
    }

    #[test]
    fn depth_frame_accessors_and_short_buffer() {
        let frame = DepthFrame::new(5, 2, 2, vec![1.0, 2.0, 3.0, 4.0]);
        assert_eq!(frame.depth_at(0, 0), Some(1.0));
        assert_eq!(frame.depth_at(1, 1), Some(4.0));
        assert_eq!(frame.depth_at_index(3), Some(4.0));
        // 越界 → None。
        assert_eq!(frame.depth_at(2, 0), None);
        assert_eq!(frame.depth_at(0, 2), None);
        assert_eq!(frame.depth_at_index(4), None);

        // 缓冲区过短：不应 panic，只产出能访问到的有效点。
        let short = DepthFrame::new(0, 4, 1, vec![1.0, 2.0]);
        let cam = PinholeCamera {
            fx: 1.0,
            fy: 1.0,
            cx: 0.0,
            cy: 0.0,
            width: 4,
            height: 1,
        };
        let cloud = depth_to_pointcloud(&short, &cam);
        // 前两个像素有效；后两个越界索引被跳过。
        assert_eq!(cloud.len(), 2);
    }

    #[test]
    fn subsampled_depth_to_cloud() {
        let w = 8usize;
        let h = 8usize;
        let cam = PinholeCamera {
            fx: 1.0,
            fy: 1.0,
            cx: 0.0,
            cy: 0.0,
            width: w,
            height: h,
        };
        let frame = DepthFrame::new(0, w, h, vec![1.0; w * h]);
        let full = depth_to_pointcloud(&frame, &cam);
        assert_eq!(full.len(), 64);
        // 步长 2 → 4x4 = 16 点。
        let sub = depth_to_pointcloud_subsampled(&frame, &cam, 2);
        assert_eq!(sub.len(), 16);
        // step=0 应被 clamp 为 1。
        assert_eq!(depth_to_pointcloud_subsampled(&frame, &cam, 0).len(), 64);
    }

    #[test]
    fn cloud_centroid_and_transform() {
        let cloud = vec![
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(3.0, 0.0, 0.0),
            Vec3::new(2.0, 0.0, 0.0),
        ];
        let c = cloud_centroid(&cloud).unwrap();
        assert!((c.x - 2.0).abs() < 1e-6 && c.y.abs() < 1e-6);
        let empty: PointCloud = Vec::new();
        assert!(cloud_centroid(&empty).is_none());

        let pose = Pose::from_translation(Vec3::new(10.0, 0.0, 0.0));
        let moved = cloud_transform(&cloud, &pose);
        assert_eq!(moved.len(), 3);
        assert!((moved[0].x - 11.0).abs() < 1e-4);
        assert!((moved[2].x - 12.0).abs() < 1e-4);
    }
}
