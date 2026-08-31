//! 模拟测距传感器：向四周发射多束光线，返回命中/自由端点的世界坐标。

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
    /// 光线数量（环绕 360°）。
    pub num_rays: usize,
    /// 最大量程（世界单元）。
    pub max_range: f32,
}

impl RangeSensor {
    pub fn new(num_rays: usize, max_range: f32) -> Self {
        Self {
            num_rays,
            max_range,
        }
    }

    /// 从 `(x, y, theta)` 扫描，返回命中点与自由端点。
    pub fn scan(&self, world: &World, x: f32, y: f32, theta: f32) -> Scan {
        let mut out = Scan::default();
        let two_pi = std::f32::consts::TAU;
        for i in 0..self.num_rays {
            let a = theta + (i as f32 / self.num_rays as f32) * two_pi;
            let dx = a.cos();
            let dy = a.sin();
            let end = Vec3::new(x + dx * self.max_range, y + dy * self.max_range, 0.0);
            let mut hit = None;
            for d in 0..=self.max_range as usize {
                let px = x + dx * d as f32;
                let py = y + dy * d as f32;
                if world.is_obstacle_at(px, py) {
                    hit = Some(Vec3::new(px, py, 0.0));
                    break;
                }
            }
            match hit {
                Some(h) => out.hits.push(h),
                None => out.free_ends.push(end),
            }
        }
        out
    }
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
}
