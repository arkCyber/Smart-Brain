//! 双目立体视觉（Stereo Vision）：**两个水平对齐的相机**获得空间（深度）感知。
//!
//! 与 RGB-D 相机不同，双目无需主动深度传感器：用左右两幅图像沿**极线**做
//! **块匹配**得到视差图，再经**三角化**恢复每个像素的深度，进而生成相机系三维点云。
//!
//! 本模块假设两相机**已校正（rectified）**：内参相同、光轴平行、仅在 X 轴方向相距
//! 基线 `baseline`。此时同名点在左右图上只差一个水平位移——**视差 `d = u_left - u_right`**，
//! 深度满足 `z = f_x * baseline / d`。
//!
//! 提供：
//! - `StereoCamera`：双目相机模型（内参 + 基线 + 匹配参数），`triangulate` 三角化
//! - `compute_disparity`：SAD 块匹配 → 视差图
//! - `disparity_to_pointcloud`：视差图 → 相机系点云
//! - `process_stereo`：两幅图像 → `StereoResult{视差图, 点云}` 的完整流水线

use brain_core::Vec3;

use crate::sensor::{PinholeCamera, PointCloud};

/// 双目相机配置（已校正）。
#[derive(Debug, Clone, Copy)]
pub struct StereoCamera {
    /// 左（参考）相机内参；右相机假设与之相同。
    pub camera: PinholeCamera,
    /// 两相机光心沿 X 轴的距离（米）。
    pub baseline: f32,
    /// 最大视差（像素）；更大的视差对应更近的物体。
    pub max_disparity: usize,
    /// 块匹配窗口边长（建议奇数）。
    pub block_size: usize,
}

impl StereoCamera {
    /// 由左相机内参、基线（米）与匹配参数构造。
    pub fn new(
        camera: PinholeCamera,
        baseline: f32,
        max_disparity: usize,
        block_size: usize,
    ) -> Self {
        Self {
            camera,
            baseline,
            max_disparity: max_disparity.max(1),
            block_size: (block_size | 1).max(3), // 保证为 ≥3 的奇数
        }
    }

    /// 由分辨率、水平视场角与基线构造（内参居中）。
    pub fn from_fov(width: usize, height: usize, fov_x: f32, baseline: f32) -> Self {
        let camera =
            PinholeCamera::from_fov(fov_x, fov_x * height as f32 / width as f32, width, height);
        Self::new(camera, baseline, width / 5, 5)
    }

    /// 匹配窗口半径（半边长）。
    pub fn half_block(&self) -> i32 {
        (self.block_size / 2) as i32
    }

    /// 视差 → 深度（米）。视差 ≤ 0 视为无效（无穷远），返回 None。
    pub fn disparity_to_depth(&self, disparity: f32) -> Option<f32> {
        if disparity <= 0.0 {
            return None;
        }
        Some(self.camera.fx * self.baseline / disparity)
    }

    /// 三角化：左图像素 `(u, v)` + 视差 `disparity` → 相机系三维点。
    pub fn triangulate(&self, u: f32, v: f32, disparity: f32) -> Option<Vec3> {
        let depth = self.disparity_to_depth(disparity)?;
        self.camera.unproject(u, v, depth)
    }
}

/// 一次双目处理的结果。
#[derive(Debug, Clone)]
pub struct StereoResult {
    /// 视差图（行优先，`width*height`），无效像素为 0。
    pub disparity: Vec<f32>,
    /// 相机系三维点云（由有效视差三角化得到）。
    pub point_cloud: PointCloud,
}

/// 左、右图像块在候选视差 `d` 下的 SAD 误差（越界返回 `u32::MAX` 表示无效）。
///
/// 调用方保证 `vi ∈ [half, h-half)`，故行范围始终有效，只需检查列越界。
#[inline]
fn sad_at(left: &[u8], right: &[u8], w: usize, ui: i32, vi: i32, d: i32, half: i32) -> u32 {
    let mut acc = 0u32;
    for dy in -half..=half {
        let y = vi + dy;
        debug_assert!(y >= 0 && (y as usize) < left.len() / w);
        let lrow = &left[y as usize * w..];
        let rrow = &right[y as usize * w..];
        for dx in -half..=half {
            let lx = ui + dx;
            let rx = ui - d + dx;
            if lx < 0 || lx >= w as i32 || rx < 0 || rx >= w as i32 {
                return u32::MAX;
            }
            let li = lrow[lx as usize] as i32;
            let ri = rrow[rx as usize] as i32;
            acc += (li - ri).unsigned_abs();
        }
    }
    acc
}

/// 用 SAD 块匹配沿水平极线计算视差图。
///
/// 对每个像素，在 `[1, max_disparity]` 内搜索使左右块误差最小的视差；
/// 边缘/无匹配区域置 0（无效）。
pub fn compute_disparity(left: &[u8], right: &[u8], cam: &StereoCamera) -> Vec<f32> {
    let w = cam.camera.width;
    let h = cam.camera.height;
    debug_assert_eq!(left.len(), w * h);
    debug_assert_eq!(right.len(), w * h);

    let half = cam.half_block();
    let mut disp = vec![0.0f32; w * h];
    for v in 0..h {
        let vi = v as i32;
        if vi < half || vi >= h as i32 - half {
            continue;
        }
        for u in 0..w {
            let ui = u as i32;
            if ui < half || ui >= w as i32 - half {
                continue;
            }
            // 右块左边缘需 ≥ 0：d ≤ ui - half。
            let dmax = (cam.max_disparity as i32).min(ui - half).max(1);
            let mut best_d = 0usize;
            let mut best_sad = u32::MAX;
            for d in 1..=dmax {
                let s = sad_at(left, right, w, ui, vi, d, half);
                if s < best_sad {
                    best_sad = s;
                    best_d = d as usize;
                }
            }
            if best_sad != u32::MAX {
                disp[v * w + u] = best_d as f32;
            }
        }
    }
    disp
}

/// 视差图 → 相机系点云（跳过无效视差）。
pub fn disparity_to_pointcloud(disparity: &[f32], cam: &StereoCamera) -> PointCloud {
    let w = cam.camera.width;
    let h = cam.camera.height;
    let mut cloud = PointCloud::new();
    for v in 0..h {
        for u in 0..w {
            let d = disparity[v * w + u];
            if d > 0.0 {
                if let Some(p) = cam.triangulate(u as f32, v as f32, d) {
                    cloud.push(p);
                }
            }
        }
    }
    cloud
}

/// 完整双目流水线：两幅图像 → 视差图 + 三维点云（空间感知）。
pub fn process_stereo(left: &[u8], right: &[u8], cam: &StereoCamera) -> StereoResult {
    let disparity = compute_disparity(left, right, cam);
    let point_cloud = disparity_to_pointcloud(&disparity, cam);
    StereoResult {
        disparity,
        point_cloud,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_camera() -> StereoCamera {
        // fx=100, fy=100, 中心 (48,32)，96x64。
        let cam = PinholeCamera {
            fx: 100.0,
            fy: 100.0,
            cx: 48.0,
            cy: 32.0,
            width: 96,
            height: 64,
        };
        StereoCamera::new(cam, 0.1, 20, 5)
    }

    /// 确定性伪随机纹理（保证块匹配唯一）。
    fn tex(u: usize, v: usize) -> u8 {
        let mut x = (u as u64).wrapping_mul(73_856_093) ^ (v as u64).wrapping_mul(19_349_663);
        x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
        x ^= x >> 30;
        x = x.wrapping_mul(0xBF58_476D_1CE4_E5B9);
        x ^= x >> 27;
        ((x ^ (x >> 31)) & 0xFF) as u8
    }

    /// 渲染一块**前向平行**纹理平面（深度 `depth`）：右图 = 左图整体左移 `d0` 像素。
    fn render_plane(
        cam: &StereoCamera,
        width: usize,
        height: usize,
        depth: f32,
    ) -> (Vec<u8>, Vec<u8>) {
        let d0 = (cam.camera.fx * cam.baseline / depth).round() as usize;
        let mut left = vec![0u8; width * height];
        let mut right = vec![0u8; width * height];
        for v in 0..height {
            for u in 0..width {
                left[v * width + u] = tex(u, v);
            }
        }
        // 右像素 = 左像素向左移 d0：right[u] = left[u + d0]。
        for v in 0..height {
            for u in 0..width {
                if u + d0 < width {
                    right[v * width + u] = left[v * width + u + d0];
                }
            }
        }
        (left, right)
    }

    fn region_mean(disp: &[f32], w: usize) -> f32 {
        let mut sum = 0.0;
        let mut n = 0.0;
        for v in 8..56 {
            for u in 20..76 {
                sum += disp[v * w + u];
                n += 1.0;
            }
        }
        sum / n
    }

    #[test]
    fn triangulate_recovers_depth_and_position() {
        let cam = test_camera();
        let z = 1.0f32;
        let d = cam.camera.fx * cam.baseline / z; // 100*0.1/1 = 10
        let p = cam.triangulate(48.0, 32.0, d).unwrap();
        assert!((p.z - z).abs() < 1e-3, "z={}", p.z);
        // 中心像素 → 光轴上，x、y 应约为 0。
        assert!(p.x.abs() < 1e-3 && p.y.abs() < 1e-3, "p={p:?}");
        // 视差 <= 0 → 无效。
        assert!(cam.triangulate(48.0, 32.0, 0.0).is_none());
        assert!(cam.disparity_to_depth(-1.0).is_none());
    }

    #[test]
    fn disparity_to_depth_follows_formula() {
        let cam = test_camera();
        let d = 10.0f32;
        let depth = cam.disparity_to_depth(d).unwrap();
        assert!((depth - (100.0 * 0.1 / 10.0)).abs() < 1e-4);
    }

    #[test]
    fn block_matching_recovers_known_disparity() {
        let cam = test_camera();
        let depth = 1.0f32; // d0 = 100*0.1/1 = 10
        let (left, right) = render_plane(&cam, 96, 64, depth);
        let disp = compute_disparity(&left, &right, &cam);
        assert_eq!(disp.len(), 96 * 64);
        // 取中心区域（避开边缘），均值应 ≈ 10。
        let mean = region_mean(&disp, 96);
        assert!((mean - 10.0).abs() < 1.0, "mean={mean}");
    }

    #[test]
    fn full_pipeline_produces_cloud_at_depth() {
        let cam = test_camera();
        let depth = 1.0f32;
        let (left, right) = render_plane(&cam, 96, 64, depth);
        let res = process_stereo(&left, &right, &cam);
        // 点云非空且主要由深度 ≈ 1.0 的点构成。
        assert!(!res.point_cloud.is_empty(), "cloud should not be empty");
        let total = res.point_cloud.len();
        let near = res
            .point_cloud
            .iter()
            .filter(|p| (p.z - depth).abs() < 0.2)
            .count();
        assert!((near as f32 / total as f32) > 0.9, "near={near}/{total}");
        // 至少存在一些有效视差。
        assert!(res.disparity.iter().any(|&d| d > 0.0));
    }

    #[test]
    fn different_depth_gives_larger_disparity() {
        let cam = test_camera();
        // 近距离平面：深度 0.5m → d0=20；远距离 2.0m → d0=5。
        let (l_near, r_near) = render_plane(&cam, 96, 64, 0.5);
        let (l_far, r_far) = render_plane(&cam, 96, 64, 2.0);
        let d_near = compute_disparity(&l_near, &r_near, &cam);
        let d_far = compute_disparity(&l_far, &r_far, &cam);
        let mean_near = region_mean(&d_near, 96);
        let mean_far = region_mean(&d_far, 96);
        assert!(mean_near > mean_far, "near={mean_near} far={mean_far}");
    }
}
