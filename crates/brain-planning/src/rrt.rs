//! 快速探索随机树（RRT）路径规划。
//!
//! 在连续空间中随机采样、向最近节点扩展，逐步长出无碰撞路径树，最终到达
//! 目标。适合高维/狭窄空间，是 A*（栅格）之外的另一经典规划算法。
//!
//! 采用**确定性种子 PRNG**（xorshift64*），保证相同参数下结果可复现、可测试，
//! 无需外部 `rand` 依赖。

use brain_core::Vec3;
use brain_mapping::OccupancyGrid3D;

/// RRT 配置。
#[derive(Debug, Clone, Copy)]
pub struct RrtConfig {
    /// 每次扩展的最大步长（世界单位）。
    pub step_size: f32,
    /// 最大迭代次数。
    pub max_iterations: usize,
    /// 采样点直接取目标的概率（0..1，加快收敛）。
    pub goal_bias: f32,
    /// 到达目标判定容差。
    pub goal_tolerance: f32,
    /// 随机种子（确定性）。
    pub seed: u64,
    /// 每段采样多少个点做碰撞检测。
    pub segment_resolution: usize,
}

impl Default for RrtConfig {
    fn default() -> Self {
        Self {
            step_size: 0.5,
            max_iterations: 2000,
            goal_bias: 0.2,
            goal_tolerance: 0.4,
            seed: 42,
            segment_resolution: 8,
        }
    }
}

/// RRT 规划器。
pub struct RrtPlanner {
    cfg: RrtConfig,
}

/// 一条 RRT 路径（世界坐标点列）。
pub type RrtPath = Vec<(f32, f32)>;

impl RrtPlanner {
    pub fn new(cfg: RrtConfig) -> Self {
        Self { cfg }
    }

    /// 在占据网格上规划 `start -> goal` 的无碰撞路径。
    ///
    /// 搜索空间取自网格世界范围（z=0）。未知体素视为可通行（乐观）。
    pub fn plan(
        &self,
        grid: &OccupancyGrid3D,
        start: (f32, f32),
        goal: (f32, f32),
    ) -> Option<RrtPath> {
        let cfg = grid.config();
        let (x_min, x_max) = (0.0f32, cfg.size_x as f32 * cfg.resolution);
        let (y_min, y_max) = (0.0f32, cfg.size_y as f32 * cfg.resolution);

        if !self.point_clear(grid, x_min, x_max, y_min, y_max, start.0, start.1)
            || !self.point_clear(grid, x_min, x_max, y_min, y_max, goal.0, goal.1)
        {
            return None;
        }
        if dist2(start, goal) <= self.cfg.goal_tolerance * self.cfg.goal_tolerance {
            return Some(vec![start, goal]);
        }

        // 节点：x, y, parent 下标。
        let mut nodes: Vec<(f32, f32, usize)> = vec![(start.0, start.1, usize::MAX)];
        let mut rng = self.cfg.seed;

        for _ in 0..self.cfg.max_iterations {
            // 采样（按目标偏置）。
            let (sx, sy) = if rand01(&mut rng) < self.cfg.goal_bias {
                goal
            } else {
                (
                    x_min + rand01(&mut rng) * (x_max - x_min),
                    y_min + rand01(&mut rng) * (y_max - y_min),
                )
            };

            // 最近节点。
            let nearest = (0..nodes.len())
                .min_by(|&a, &b| {
                    dist2((nodes[a].0, nodes[a].1), (sx, sy))
                        .partial_cmp(&dist2((nodes[b].0, nodes[b].1), (sx, sy)))
                        .unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap();

            let (nx, ny, _) = nodes[nearest];
            // 向采样点扩展一步。
            let d = ((sx - nx).powi(2) + (sy - ny).powi(2)).sqrt();
            let (tx, ty) = if d < 1e-6 {
                (nx, ny)
            } else {
                (
                    nx + (sx - nx) / d * self.cfg.step_size,
                    ny + (sy - ny) / d * self.cfg.step_size,
                )
            };

            // 碰撞检测整段（部分采样点）。
            if !self.segment_clear(grid, x_min, x_max, y_min, y_max, (nx, ny), (tx, ty)) {
                continue;
            }

            let idx = nodes.len();
            nodes.push((tx, ty, nearest));

            // 是否到达目标。
            if dist2((tx, ty), goal) <= self.cfg.goal_tolerance * self.cfg.goal_tolerance {
                return Some(reconstruct(&nodes, idx, goal));
            }
        }
        None
    }

    /// 检查点是否在边界内且未被占据。
    #[allow(clippy::too_many_arguments)] // 几何包围盒参数，语义清晰
    fn point_clear(
        &self,
        grid: &OccupancyGrid3D,
        x_min: f32,
        x_max: f32,
        y_min: f32,
        y_max: f32,
        x: f32,
        y: f32,
    ) -> bool {
        if x < x_min || y < y_min || x >= x_max || y >= y_max {
            return false; // 越界视为碰撞
        }
        !grid.is_occupied(Vec3::new(x, y, 0.0))
    }

    /// 检查线段上各采样点是否全部无障碍。
    #[allow(clippy::too_many_arguments)] // 几何包围盒参数，语义清晰
    fn segment_clear(
        &self,
        grid: &OccupancyGrid3D,
        x_min: f32,
        x_max: f32,
        y_min: f32,
        y_max: f32,
        a: (f32, f32),
        b: (f32, f32),
    ) -> bool {
        let n = self.cfg.segment_resolution.max(2);
        for i in 0..=n {
            let t = i as f32 / n as f32;
            let x = a.0 + (b.0 - a.0) * t;
            let y = a.1 + (b.1 - a.1) * t;
            if !self.point_clear(grid, x_min, x_max, y_min, y_max, x, y) {
                return false;
            }
        }
        true
    }
}

/// 确定性 xorshift64* PRNG。
fn next_u64(state: &mut u64) -> u64 {
    let mut x = *state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *state = x;
    x.wrapping_mul(0x2545F4914F6CDD1D)
}

fn rand01(state: &mut u64) -> f32 {
    // 取高 24 位 → [0,1)。
    (next_u64(state) >> 40) as f32 / (1u64 << 24) as f32
}

fn dist2(a: (f32, f32), b: (f32, f32)) -> f32 {
    (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)
}

/// 从节点 `idx` 沿父链回溯到根，重建路径（含目标点）。
fn reconstruct(nodes: &[(f32, f32, usize)], idx: usize, goal: (f32, f32)) -> RrtPath {
    let mut path = vec![goal];
    let mut cur = idx;
    while nodes[cur].2 != usize::MAX {
        path.push((nodes[cur].0, nodes[cur].1));
        cur = nodes[cur].2;
    }
    path.push((nodes[cur].0, nodes[cur].1)); // 起点
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_mapping::{GridConfig, Index3};

    fn grid() -> OccupancyGrid3D {
        OccupancyGrid3D::new(GridConfig::from_world_size(0.5, 20.0, 20.0, 1.0))
    }

    #[test]
    fn finds_path_in_open_space() {
        let g = grid();
        let rrt = RrtPlanner::new(RrtConfig::default());
        let path = rrt.plan(&g, (1.0, 1.0), (15.0, 15.0)).expect("path found");
        assert!(path.len() >= 2);
        assert!(dist2(*path.first().unwrap(), (1.0, 1.0)) < 0.5);
        assert!(dist2(*path.last().unwrap(), (15.0, 15.0)) < 0.5);
    }

    #[test]
    fn avoids_obstacle_wall() {
        let mut g = grid();
        // 一堵中段竖直墙：cell x=15（world x 7.5..8.0），y-cell 10..30（world y 5..15）。
        for y in 10..30 {
            g.set_log_odds(Index3::new(15, y, 0), 2.0);
        }
        let rrt = RrtPlanner::new(RrtConfig::default());
        let path = rrt
            .plan(&g, (1.0, 1.0), (18.0, 15.0))
            .expect("should route around wall");
        // 路径上任何点都不得落在被占据的体素内。
        for &(x, y) in &path {
            assert!(
                !g.is_occupied(Vec3::new(x, y, 0.0)),
                "path enters obstacle at ({x},{y})"
            );
        }
        // 且确实到达了墙的另一侧。
        assert!(
            path.iter().any(|&(x, _)| x > 8.0),
            "did not cross to far side of wall"
        );
    }

    #[test]
    fn deterministic_with_same_seed() {
        let g = grid();
        let cfg = RrtConfig {
            seed: 7,
            ..Default::default()
        };
        let a = RrtPlanner::new(cfg)
            .plan(&g, (1.0, 1.0), (15.0, 15.0))
            .unwrap();
        let b = RrtPlanner::new(cfg)
            .plan(&g, (1.0, 1.0), (15.0, 15.0))
            .unwrap();
        assert_eq!(a, b);
    }

    #[test]
    fn unreachable_goal_returns_none() {
        let mut g = grid();
        // 目标直接落在障碍格上 → 应返回 None。
        g.set_log_odds(Index3::new(30, 30, 0), 2.0);
        let rrt = RrtPlanner::new(RrtConfig {
            max_iterations: 200,
            ..Default::default()
        });
        assert!(rrt.plan(&g, (1.0, 1.0), (15.2, 15.2)).is_none());
    }
}
