//! 双目立体视觉（两相机 → 空间/深度感知）演示。
//!
//! 用 `StereoCamera`（基线 + 内参）+ SAD 块匹配，从左右两幅**合成纹理平面**图像
//! 计算出视差图，再三角化为相机系三维点云——无需主动深度传感器即可获得空间感知。

use brain_odometry::stereo::{process_stereo, StereoCamera};
use brain_odometry::PinholeCamera;

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
fn render_plane(cam: &StereoCamera, width: usize, height: usize, depth: f32) -> (Vec<u8>, Vec<u8>) {
    let d0 = (cam.camera.fx * cam.baseline / depth).round() as usize;
    let mut left = vec![0u8; width * height];
    let mut right = vec![0u8; width * height];
    for v in 0..height {
        for u in 0..width {
            left[v * width + u] = tex(u, v);
        }
    }
    for v in 0..height {
        for u in 0..width {
            if u + d0 < width {
                right[v * width + u] = left[v * width + u + d0];
            }
        }
    }
    (left, right)
}

/// 顶层入口。
pub fn run() {
    println!("\n=== 双目立体视觉（两个相机 → 空间感知）===");

    let width = 96usize;
    let height = 64usize;
    // 两个水平对齐相机：内参 fx=100，中心 (48,32)，基线 0.1m。
    let cam = StereoCamera::new(
        PinholeCamera {
            fx: 100.0,
            fy: 100.0,
            cx: 48.0,
            cy: 32.0,
            width,
            height,
        },
        0.1,
        20,
        5,
    );

    // 场景：两块不同深度的纹理平面（前景近、背景远），体现“空间”。
    let (l_near, r_near) = render_plane(&cam, width, height, 0.5); // 近平面 d0≈20
    let (l_far, r_far) = render_plane(&cam, width, height, 2.0); // 远平面 d0≈5

    for (name, l, r) in [
        ("近平面(0.5m)", &l_near, &r_near),
        ("远平面(2.0m)", &l_far, &r_far),
    ] {
        let res = process_stereo(l, r, &cam);
        // 中心区域（避开边缘匹配退化）的平均视差。
        let mut dsum = 0.0f32;
        let mut n = 0u32;
        for v in 8..56 {
            for u in 20..76 {
                dsum += res.disparity[v * width + u];
                n += 1;
            }
        }
        let d_mean = dsum / n as f32;
        // 用中心视差三角化得到该区域的重建深度。
        let z_mean = cam
            .disparity_to_depth(d_mean)
            .map(|z| format!("{z:.2} m"))
            .unwrap_or_else(|| "N/A".to_string());
        println!("  {name}: 中心平均视差 = {d_mean:.1} px -> 重建深度 ≈ {z_mean}",);
    }
}
