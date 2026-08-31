//! 前沿探索：自动发现并驶向未知区域边界。

use brain_core::Vec3;
use brain_mapping::OccupancyGrid3D;

/// 探索目标。
#[derive(Debug, Clone, Copy)]
pub struct ExploreTarget {
    /// 目标世界位置。
    pub position: Vec3,
    /// 该目标所覆盖的前沿体素数量（越大越值得去）。
    pub coverage: usize,
}

/// 前沿探索器。
///
/// 在占据网格中找出“空闲且邻接未知”的前沿体素，选择离当前位置最近且
/// 覆盖较大的一簇作为下一个探索目标，驱动室内自主巡检。
pub struct Explorer {
    /// 期望飞行高度层（体素 z）。
    z: i32,
}

impl Explorer {
    pub fn new(z: i32) -> Self {
        Self { z }
    }

    /// 选取下一个探索目标（最近前沿）。无前沿返回 `None`（已探索完）。
    pub fn next_target(&self, grid: &OccupancyGrid3D, pos: Vec3) -> Option<ExploreTarget> {
        self.next_targets(grid, pos, 1).into_iter().next()
    }

    /// 返回前 `count` 个最近的探索目标（按“近处优先、覆盖多者加分”排序）。
    ///
    /// 会**过滤掉紧贴障碍（墙体）的前沿**——这类目标通常不可达/靠近墙，会让
    /// 规划器（如 A*）失败。供规划器逐个尝试，直到选出一个可达目标。
    pub fn next_targets(
        &self,
        grid: &OccupancyGrid3D,
        pos: Vec3,
        count: usize,
    ) -> Vec<ExploreTarget> {
        let frontiers = grid.frontiers();
        if frontiers.is_empty() {
            return Vec::new();
        }
        let mut scored: Vec<(f32, usize, Vec3)> = Vec::new();
        for f in frontiers {
            // 跳过紧贴障碍的前沿，避免把机器人引向墙边。
            if self.near_obstacle(grid, f) {
                continue;
            }
            let center = grid.index_to_world_center(f);
            let d2 = center.sub(pos).norm();
            let coverage = self.neighbor_unknown(grid, f);
            let score = d2 * 1.0 - coverage as f32 * 0.1;
            scored.push((score, coverage, center));
        }
        scored.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        scored
            .into_iter()
            .take(count)
            .map(|(_, cov, center)| ExploreTarget {
                position: center,
                coverage: cov,
            })
            .collect()
    }

    /// 该前沿的邻居（半径 1）内是否有已占据体素（贴墙）。
    fn near_obstacle(&self, grid: &OccupancyGrid3D, f: brain_mapping::Index3) -> bool {
        for dz in -1..=1i32 {
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let idx = brain_mapping::Index3::new(f.x + dx, f.y + dy, f.z + dz);
                    if grid.state(idx) == Some(brain_mapping::CellState::Occupied) {
                        return true;
                    }
                }
            }
        }
        false
    }

    fn neighbor_unknown(&self, grid: &OccupancyGrid3D, f: brain_mapping::Index3) -> usize {
        let mut n = 0;
        for dz in -1..=1i32 {
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    if dx == 0 && dy == 0 && dz == 0 {
                        continue;
                    }
                    let idx = brain_mapping::Index3::new(f.x + dx, f.y + dy, f.z + dz);
                    if grid.state(idx) == Some(brain_mapping::CellState::Unknown) {
                        n += 1;
                    }
                }
            }
        }
        n
    }

    /// 期望高度层。
    pub fn altitude_cell(&self) -> i32 {
        self.z
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_mapping::{GridConfig, Index3};

    #[test]
    fn picks_nearest_frontier() {
        let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 8.0, 8.0, 8.0));
        // 把左侧一个厚块标为空闲（x 0..3, y 全, z 0..3），
        // 使未知边界只在 x=3 与顶部 z=4 处。
        for x in 0..3 {
            for y in 0..8 {
                for z in 0..4 {
                    grid.set_log_odds(Index3::new(x, y, z), -2.0);
                }
            }
        }
        let ex = Explorer::new(0);
        // 位置靠近 x 边界（z 在底部），应选择水平前方的前沿（x≈3.5）。
        let target = ex
            .next_target(&grid, Vec3::new(1.0, 1.0, 0.5))
            .expect("frontier exists");
        assert!(
            target.position.x >= 2.5 && target.position.x <= 4.5,
            "x={}",
            target.position.x
        );
        assert!(target.coverage > 0);
    }

    #[test]
    fn no_frontier_when_all_unknown() {
        let grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 4.0, 4.0, 4.0));
        let ex = Explorer::new(0);
        assert!(ex.next_target(&grid, Vec3::new(0.5, 0.5, 0.5)).is_none());
    }

    #[test]
    fn avoids_wall_adjacent_frontiers() {
        let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 8.0, 8.0, 4.0));
        // 左半部分空闲，x=3 处是一堵墙（占据）。
        for x in 0..3 {
            for y in 0..8 {
                for z in 0..4 {
                    grid.set_log_odds(Index3::new(x, y, z), -2.0);
                }
            }
        }
        for y in 0..8 {
            for z in 0..4 {
                grid.set_log_odds(Index3::new(3, y, z), 2.0); // 整面墙
            }
        }
        let ex = Explorer::new(0);
        // 所有候选都不得紧贴 x=3 的墙（邻居含占据）。
        for t in ex.next_targets(&grid, Vec3::new(1.0, 1.0, 0.5), 4) {
            // 目标中心到墙线 x=3 的距离应 >= 1（至少隔一个空闲格）。
            assert!(
                t.position.x <= 1.5 || t.position.x >= 4.5,
                "wall-adjacent target x={}",
                t.position.x
            );
        }
    }
}
