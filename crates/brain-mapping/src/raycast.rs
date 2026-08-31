//! 光线投射更新：把传感器的“空闲 / 占据”观测写入占据网格。
//!
//! 使用三维 DDA（Amanatides & Woo）快速遍历光线穿过的体素，把光束路径上
//! 的体素标为空闲、命中点标为占据（均以 log-odds 累加，支持多次观测融合）。

use brain_core::Vec3;

use crate::grid::{Index3, OccupancyGrid3D};

/// 计算从 `origin` 沿 `dir`（单位向量）穿过、直到 `max_dist` 的体素序列。
pub fn ray_cells(grid: &OccupancyGrid3D, origin: Vec3, dir: Vec3, max_dist: f32) -> Vec<Index3> {
    let res = grid.config().resolution;
    // 把起点与方向换算到体素坐标系。
    let o = origin.sub(grid.origin()) * (1.0 / res);
    let d = dir.normalized() * (1.0 / res);
    let max_cells = max_dist / res;

    let cell = Index3::new(o.x.floor() as i32, o.y.floor() as i32, o.z.floor() as i32);

    let step_x = if d.x > 0.0 { 1 } else { -1 };
    let step_y = if d.y > 0.0 { 1 } else { -1 };
    let step_z = if d.z > 0.0 { 1 } else { -1 };

    let tdelta_x = if d.x.abs() < 1e-12 {
        f32::INFINITY
    } else {
        (1.0 / d.x).abs()
    };
    let tdelta_y = if d.y.abs() < 1e-12 {
        f32::INFINITY
    } else {
        (1.0 / d.y).abs()
    };
    let tdelta_z = if d.z.abs() < 1e-12 {
        f32::INFINITY
    } else {
        (1.0 / d.z).abs()
    };

    let tmax_x = if d.x.abs() < 1e-12 {
        f32::INFINITY
    } else if d.x > 0.0 {
        ((cell.x as f32 + 1.0) - o.x) / d.x
    } else {
        (o.x - cell.x as f32) / (-d.x)
    };
    let tmax_y = if d.y.abs() < 1e-12 {
        f32::INFINITY
    } else if d.y > 0.0 {
        ((cell.y as f32 + 1.0) - o.y) / d.y
    } else {
        (o.y - cell.y as f32) / (-d.y)
    };
    let tmax_z = if d.z.abs() < 1e-12 {
        f32::INFINITY
    } else if d.z > 0.0 {
        ((cell.z as f32 + 1.0) - o.z) / d.z
    } else {
        (o.z - cell.z as f32) / (-d.z)
    };

    let mut out = Vec::with_capacity((max_cells.max(1.0)) as usize + 2);
    let (mut tmax_x, mut tmax_y, mut tmax_z) = (tmax_x, tmax_y, tmax_z);
    let (mut cx, mut cy, mut cz) = (cell.x, cell.y, cell.z);
    // t 在每次推进分支中都会先赋值再被读取，因此无需初始化。
    let mut t: f32;

    for _ in 0..(max_cells.max(1.0) as usize + 4) {
        out.push(Index3::new(cx, cy, cz));
        if tmax_x < tmax_y && tmax_x < tmax_z {
            cx += step_x;
            t = tmax_x;
            tmax_x += tdelta_x;
        } else if tmax_y < tmax_z {
            cy += step_y;
            t = tmax_y;
            tmax_y += tdelta_y;
        } else {
            cz += step_z;
            t = tmax_z;
            tmax_z += tdelta_z;
        }
        if t > max_cells {
            break;
        }
    }
    out
}

/// 负责把一束光（或一片点云）写入网格的更新器。
pub struct RaycastUpdater {
    /// 命中（占据）log-odds 增量。
    pub log_hit: f32,
    /// 未命中（空闲）log-odds 增量。
    pub log_miss: f32,
}

impl Default for RaycastUpdater {
    fn default() -> Self {
        // 对应概率 0.7 命中 / 0.3 空闲 的 log-odds。
        Self {
            log_hit: (0.7f32 / 0.3f32).ln(),
            log_miss: (0.3f32 / 0.7f32).ln(),
        }
    }
}

impl RaycastUpdater {
    /// 沿一条光束更新网格。
    ///
    /// - `origin`：传感器世界位置
    /// - `dir`：光束方向（单位向量）
    /// - `max_range`：最大量程（米）
    /// - `hit`：测得命中距离；`None` 表示未测得（整束标记为空闲）
    pub fn update_ray(
        &self,
        grid: &mut OccupancyGrid3D,
        origin: Vec3,
        dir: Vec3,
        max_range: f32,
        hit: Option<f32>,
    ) {
        let dir = dir.normalized();
        let cells = ray_cells(grid, origin, dir, max_range.max(0.001));
        if cells.is_empty() {
            return;
        }
        // 命中的体素 = 命中点所在的体素。
        let hit_cell = hit.map(|h| grid.world_to_index(origin.add(dir * h)));
        let hit_dist = hit.unwrap_or(f32::INFINITY);

        for &c in cells.iter() {
            // 命中体素 → 占据。
            if let Some(hc) = hit_cell {
                if Some(c) == hc {
                    if let Some(lo) = grid.log_odds(c) {
                        grid.set_log_odds(c, lo + self.log_hit);
                    }
                    continue;
                }
            }
            // 命中点之前的体素 → 空闲；之后保持未知。
            let center = grid.index_to_world_center(c);
            let d = center.sub(origin).norm();
            if d < hit_dist {
                if let Some(lo) = grid.log_odds(c) {
                    grid.set_log_odds(c, lo + self.log_miss);
                }
            }
        }
    }

    /// 由一片世界系点云更新网格（从 `origin` 到每个点的光束）。
    pub fn update_point_cloud(
        &self,
        grid: &mut OccupancyGrid3D,
        origin: Vec3,
        points: &[Vec3],
        max_range: f32,
    ) {
        for &p in points {
            let d = p.sub(origin);
            let dist = d.norm();
            if dist > max_range || dist < 1e-4 {
                continue;
            }
            self.update_ray(grid, origin, d.normalized(), max_range, Some(dist));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{CellState, GridConfig};

    fn grid() -> OccupancyGrid3D {
        OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 10.0, 10.0, 10.0))
    }

    #[test]
    fn ray_marks_free_then_hit() {
        let mut g = grid();
        let updater = RaycastUpdater::default();
        // 从原点沿 +X，命中距离 3.3 → 命中点 x=3.8，位于体素 (3,0,0)。
        updater.update_ray(
            &mut g,
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(1.0, 0.0, 0.0),
            6.0,
            Some(3.3),
        );
        assert_eq!(g.state(Index3::new(3, 0, 0)), Some(CellState::Occupied));
        assert_eq!(g.state(Index3::new(1, 0, 0)), Some(CellState::Free));
        // 命中点之后的体素应仍是未知。
        assert_eq!(g.state(Index3::new(6, 0, 0)), Some(CellState::Unknown));
    }

    #[test]
    fn ray_no_hit_marks_all_free() {
        let mut g = grid();
        let updater = RaycastUpdater::default();
        updater.update_ray(
            &mut g,
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(1.0, 0.0, 0.0),
            5.0,
            None,
        );
        assert_eq!(g.state(Index3::new(2, 0, 0)), Some(CellState::Free));
        // 超过量程的仍是未知。
        assert_eq!(g.state(Index3::new(8, 0, 0)), Some(CellState::Unknown));
    }

    #[test]
    fn point_cloud_updates() {
        let mut g = grid();
        let updater = RaycastUpdater::default();
        // 三个轴上的点（明确落在对应体素内）。
        let points = vec![
            Vec3::new(2.7, 0.5, 0.5),
            Vec3::new(0.5, 2.7, 0.5),
            Vec3::new(0.5, 0.5, 2.7),
        ];
        updater.update_point_cloud(&mut g, Vec3::new(0.5, 0.5, 0.5), &points, 5.0);
        assert_eq!(g.state(Index3::new(2, 0, 0)), Some(CellState::Occupied));
        assert_eq!(g.state(Index3::new(0, 2, 0)), Some(CellState::Occupied));
        assert_eq!(g.state(Index3::new(0, 0, 2)), Some(CellState::Occupied));
        // 命中点前的路径应为空闲。
        assert_eq!(g.state(Index3::new(1, 0, 0)), Some(CellState::Free));
    }
}
