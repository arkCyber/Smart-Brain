//! 模拟测距传感器：向四周（或限定视场角内）发射多束光线，返回命中/自由端点的世界坐标。
//!
//! 支持视场角（FOV）与确定性测量噪声，并提供原始测距输出（`scan_ranges`）。
//! 光线投射按单元步进逐格探测，与既有导航闭环保持完全一致的行为。

use brain_core::Vec3;

use crate::world::World;

/// 一次扫描结果：命中的障碍点 + 未见障碍的“自由端点”（用于把开放区标为空闲）。
#[derive(Debug, Clone, Default)]
pub struct Scan {
    /// 命中的障碍点（世界坐标，z=0）。
    pub hits: Vec<Vec3>,
    /// 未命中、量程末端的自由端点。
    pub free_ends: Vec<Vec3>,
}

/// 2D 测距传感器。
pub struct RangeSensor {
    /// 光线数量。
    pub num_rays: usize,
    /// 最大量程（世界单元）。
    pub max_range: f32,
    /// 视场角（弧度）。默认 `2π`（全向 360°）。
    fov_rad: f32,
    /// 测量噪声标准差；0 表示无噪声。
    noise_std: f32,
    /// 确定性噪声种子（保证可复现）。
    seed: u64,
}

impl RangeSensor {
    /// 全向测距传感器。
    pub fn new(num_rays: usize, max_range: f32) -> Self {
        Self {
            num_rays,
            max_range,
            fov_rad: std::f32::consts::TAU,
            noise_std: 0.0,
            seed: 0x9E37_79B9_7F4A_7C15,
        }
    }

    /// 限定视场角（弧度，`< 2π` 时只在前方扇形内扫描）。
    pub fn with_fov(mut self, fov_rad: f32) -> Self {
        self.fov_rad = fov_rad.clamp(1e-3, std::f32::consts::TAU);
        self
    }

    /// 添加确定性测量噪声（标准差，世界单元）。
    pub fn with_noise(mut self, noise_std: f32) -> Self {
        self.noise_std = noise_std.max(0.0);
        self
    }

    /// 视场角（弧度）。
    pub fn fov(&self) -> f32 {
        self.fov_rad
    }

    /// 测量噪声标准差。
    pub fn noise(&self) -> f32 {
        self.noise_std
    }

    /// 沿相对机体前方 `bearing`（弧度，正值偏左）的方向测距，返回量程内首个障碍距离。
    ///
    /// 按单元步进逐格探测（与既有导航闭环一致）；无命中返回 `max_range`。
    /// 若设置了噪声，则叠加确定性噪声并夹在 `[0, max_range]`。
    pub fn ray_range(&self, world: &World, x: f32, y: f32, theta: f32, bearing: f32) -> f32 {
        let a = theta + bearing;
        let dx = a.cos();
        let dy = a.sin();
        let mut t = self.max_range;
        for d in 0..=self.max_range as usize {
            let px = x + dx * d as f32;
            let py = y + dy * d as f32;
            if world.is_obstacle_at(px, py) {
                t = d as f32;
                break;
            }
        }
        if self.noise_std > 0.0 {
            t = (t + self.noise_std * self.sample_noise(x, y, bearing)).clamp(0.0, self.max_range);
        }
        t
    }

    /// 返回每条光线的原始测距（世界单元），长度等于 `num_rays`。
    pub fn scan_ranges(&self, world: &World, x: f32, y: f32, theta: f32) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.num_rays);
        for i in 0..self.num_rays {
            out.push(self.ray_range(world, x, y, theta, self.bearing(i)));
        }
        out
    }

    /// 从 `(x, y, theta)` 扫描，返回命中点与自由端点。
    pub fn scan(&self, world: &World, x: f32, y: f32, theta: f32) -> Scan {
        let mut out = Scan::default();
        for i in 0..self.num_rays {
            let bearing = self.bearing(i);
            let r = self.ray_range(world, x, y, theta, bearing);
            let a = theta + bearing;
            if r < self.max_range {
                out.hits
                    .push(Vec3::new(x + a.cos() * r, y + a.sin() * r, 0.0));
            } else {
                out.free_ends.push(Vec3::new(
                    x + a.cos() * self.max_range,
                    y + a.sin() * self.max_range,
                    0.0,
                ));
            }
        }
        out
    }

    /// 第 `i` 条光线相对机体的方位角。全向时与旧分布一致（含正前方 0）；限定 FOV 时居中分布。
    fn bearing(&self, i: usize) -> f32 {
        let n = self.num_rays;
        if n == 0 {
            return 0.0;
        }
        if self.fov_rad >= std::f32::consts::TAU - 1e-6 {
            (i as f32 / n as f32) * std::f32::consts::TAU
        } else if n == 1 {
            0.0
        } else {
            -self.fov_rad * 0.5 + (i as f32 / (n - 1) as f32) * self.fov_rad
        }
    }

    /// 确定性噪声采样（[-1,1]），由位置 + 方位 + 种子哈希得到，保证可复现。
    fn sample_noise(&self, x: f32, y: f32, bearing: f32) -> f32 {
        let seed = (x.to_bits() as u64)
            ^ ((y.to_bits() as u64).wrapping_shl(17))
            ^ ((bearing.to_bits() as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15))
            ^ self.seed;
        let h = splitmix64(seed);
        ((h >> 11) as f64 / (1u64 << 53) as f64) as f32 * 2.0 - 1.0
    }
}

/// splitmix64 确定性哈希（把种子映射到 [0, 2^64)）。
fn splitmix64(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detects_wall_in_front() {
        let mut w = World::new(20, 20);
        w.wall(5..8, 0..20); // 一堵竖直墙
        let s = RangeSensor::new(16, 10.0);
        // 机器人在墙左侧朝 +x 看。
        let scan = s.scan(&w, 2.5, 2.5, 0.0);
        // 前方光线应命中 x≈5 附近的墙。
        assert!(scan
            .hits
            .iter()
            .any(|h| (h.x - 5.0).abs() < 1.5 && (h.y - 2.5).abs() < 1.5));
        // 至少应有一部分自由端点（墙未覆盖的方向）。
        assert!(!scan.free_ends.is_empty());
    }

    #[test]
    fn ray_range_hits_wall_at_expected_distance() {
        let mut w = World::new(10, 10);
        w.wall(5..6, 0..10); // 一列竖直墙（x=5）
        let s = RangeSensor::new(8, 20.0);
        // 机器人 (1,5) 朝 +x：墙在 x=5，距离 4。
        let r = s.ray_range(&w, 1.0, 5.0, 0.0, 0.0);
        assert!((r - 4.0).abs() < 1.0, "r={r}");
    }

    #[test]
    fn ray_range_returns_max_in_open_area() {
        // 大世界、中心无墙区域，量程内无阻碍 → 返回最大量程。
        let mut w = World::new(40, 40);
        w.set_obstacle(5, 5);
        let s = RangeSensor::new(8, 8.0);
        let r = s.ray_range(&w, 20.0, 20.0, 0.0, 0.0);
        assert_eq!(r, 8.0);
    }

    #[test]
    fn ray_range_hits_obstacle_at_distance() {
        // 单格障碍：应可靠命中且距离正确。
        let mut w = World::new(20, 20);
        w.set_obstacle(6, 5);
        let s = RangeSensor::new(64, 15.0);
        let r = s.ray_range(&w, 2.0, 5.0, 0.0, 0.0); // 正前方 +x
        assert!((r - 4.0).abs() < 1.5, "r={r}");
    }

    #[test]
    fn scan_ranges_length_and_bounds() {
        let mut w = World::new(10, 10);
        w.wall(5..6, 0..10);
        let s = RangeSensor::new(4, 8.0);
        let ranges = s.scan_ranges(&w, 1.0, 5.0, 0.0);
        assert_eq!(ranges.len(), 4);
        // 正前方 (+x) 命中墙 → 距离明显小于 max_range。
        assert!(ranges[0] < 8.0);
        assert!(ranges.iter().all(|&r| (0.0..=8.0).contains(&r)));
    }

    #[test]
    fn default_sensor_is_omnidirectional() {
        let s = RangeSensor::new(8, 10.0);
        assert!((s.fov() - std::f32::consts::TAU).abs() < 1e-6);
        assert_eq!(s.noise(), 0.0);
    }

    #[test]
    fn fov_restricts_scan_to_front_sector() {
        let mut w = World::new(20, 20);
        w.wall(10..12, 0..20); // 正前方竖直墙
        let s = RangeSensor::new(8, 20.0).with_fov(std::f32::consts::FRAC_PI_2);
        assert!((s.fov() - std::f32::consts::FRAC_PI_2).abs() < 1e-6);
        // 机器人朝 +x，墙在 x=10。
        let scan = s.scan(&w, 5.0, 10.0, 0.0);
        assert!(!scan.hits.is_empty(), "front sector should hit the wall");
        // 所有命中点都在前方墙上（x 接近 10）。
        for h in &scan.hits {
            assert!((h.x - 10.0).abs() < 2.0, "h={h:?}");
        }
    }

    #[test]
    fn noise_is_deterministic_and_bounded() {
        let mut w = World::new(20, 20);
        w.wall(5..6, 0..20);
        let s1 = RangeSensor::new(8, 10.0).with_noise(0.5);
        let s2 = RangeSensor::new(8, 10.0).with_noise(0.5);
        let r1 = s1.ray_range(&w, 1.0, 5.0, 0.0, 0.0);
        let r2 = s2.ray_range(&w, 1.0, 5.0, 0.0, 0.0);
        assert_eq!(r1, r2, "同种子噪声应可复现");
        // 真实距离约 4，噪声 ±0.5 → 落在 [3.5, 4.5]。
        assert!((r1 - 4.0).abs() <= 0.5 + 1e-4, "r1={r1}");
        assert!(s1.noise() > 0.0);
        assert_eq!(s2.noise(), 0.5);
    }
}
