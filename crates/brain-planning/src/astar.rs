//! 二维 A* 路径搜索（在占据网格的某一高度层进行）。

use std::collections::{BinaryHeap, HashMap};

use brain_core::Vec3;
use brain_mapping::{CellState, Index3, OccupancyGrid3D};

/// 二维网格坐标。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GridPoint {
    pub x: i32,
    pub y: i32,
}

/// 一条 2D 路径（按顺序排列的网格点）。
pub type Path = Vec<GridPoint>;

/// A* 节点（用于优先队列）。
#[derive(Debug, Clone, Copy, PartialEq)]
struct Node {
    f: f32,
    g: f32,
    point: GridPoint,
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}

impl Ord for Node {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        // 小顶堆：f 越小优先级越高。
        o.f.partial_cmp(&self.f)
            .unwrap_or(std::cmp::Ordering::Equal)
    }
}

/// 二维 A* 规划器。
pub struct AStar2D {
    /// 目标高度层（网格 z 坐标）。
    z: i32,
    /// 安全半径（米），用于把障碍外扩，避免贴墙。
    radius: f32,
    /// 是否允许经过“未知”区域（未知成本更高，但可通行）。
    allow_unknown: bool,
}

impl AStar2D {
    pub fn new(z: i32, radius: f32) -> Self {
        Self {
            z,
            radius,
            allow_unknown: true,
        }
    }

    /// 在网格上规划 `start → goal` 的无碰撞路径。
    pub fn plan(&self, grid: &OccupancyGrid3D, start: GridPoint, goal: GridPoint) -> Option<Path> {
        let cfg = grid.config();
        if !self.in_bounds(cfg.size_x as i32, cfg.size_y as i32, start.x, start.y)
            || !self.in_bounds(cfg.size_x as i32, cfg.size_y as i32, goal.x, goal.y)
        {
            return None;
        }
        if self.blocked(grid, goal) {
            return None;
        }

        let mut open = BinaryHeap::new();
        let mut came_from: HashMap<GridPoint, GridPoint> = HashMap::new();
        let mut g_score: HashMap<GridPoint, f32> = HashMap::new();
        g_score.insert(start, 0.0);
        open.push(Node {
            f: h(start, goal),
            g: 0.0,
            point: start,
        });

        const DIRS: [(i32, i32); 8] = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ];

        while let Some(Node { f: _, g, point }) = open.pop() {
            if point == goal {
                return Some(self.reconstruct(&came_from, start, goal));
            }
            for (dx, dy) in DIRS {
                let np = GridPoint {
                    x: point.x + dx,
                    y: point.y + dy,
                };
                if !self.in_bounds(cfg.size_x as i32, cfg.size_y as i32, np.x, np.y) {
                    continue;
                }
                if self.blocked(grid, np) {
                    continue;
                }
                let step = if dx != 0 && dy != 0 {
                    std::f32::consts::SQRT_2
                } else {
                    1.0
                };
                let ng = g + step;
                if ng < *g_score.get(&np).unwrap_or(&f32::INFINITY) {
                    g_score.insert(np, ng);
                    came_from.insert(np, point);
                    open.push(Node {
                        f: ng + h(np, goal),
                        g: ng,
                        point: np,
                    });
                }
            }
        }
        None
    }

    fn blocked(&self, grid: &OccupancyGrid3D, p: GridPoint) -> bool {
        let r = (self.radius / grid.config().resolution).ceil() as i32;
        for dy in -r..=r {
            for dx in -r..=r {
                if dx * dx + dy * dy > r * r {
                    continue;
                }
                let idx = Index3::new(p.x + dx, p.y + dy, self.z);
                match grid.state(idx) {
                    Some(CellState::Occupied) => return true,
                    Some(CellState::Unknown) if !self.allow_unknown => return true,
                    // 越界邻居视为“地图外”，不作为障碍（避免网格边缘被堵死）。
                    _ => {}
                }
            }
        }
        false
    }

    fn in_bounds(&self, w: i32, h: i32, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < w && y < h
    }

    fn reconstruct(
        &self,
        came: &HashMap<GridPoint, GridPoint>,
        start: GridPoint,
        goal: GridPoint,
    ) -> Path {
        let mut path = vec![goal];
        let mut cur = goal;
        while cur != start {
            if let Some(&prev) = came.get(&cur) {
                path.push(prev);
                cur = prev;
            } else {
                break;
            }
        }
        path.reverse();
        path
    }

    /// 把 2D 网格点转成世界坐标（供下发）。
    pub fn point_to_world(grid: &OccupancyGrid3D, p: GridPoint, z: i32) -> Vec3 {
        grid.index_to_world_center(Index3::new(p.x, p.y, z))
    }
}

fn h(a: GridPoint, b: GridPoint) -> f32 {
    let dx = (a.x - b.x).abs() as f32;
    let dy = (a.y - b.y).abs() as f32;
    (dx * dx + dy * dy).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_mapping::{GridConfig, Index3};

    fn grid() -> OccupancyGrid3D {
        OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 10.0, 10.0, 10.0))
    }

    #[test]
    fn finds_clear_path() {
        let g = grid();
        let a = AStar2D::new(0, 0.1);
        let path = a
            .plan(&g, GridPoint { x: 0, y: 0 }, GridPoint { x: 5, y: 5 })
            .unwrap();
        assert!(path.len() >= 2);
        assert_eq!(*path.first().unwrap(), GridPoint { x: 0, y: 0 });
        assert_eq!(*path.last().unwrap(), GridPoint { x: 5, y: 5 });
    }

    #[test]
    fn avoids_obstacle_wall() {
        let mut g = grid();
        // 在 x=3 竖一堵墙（y=2..7），路径应绕行（从 y=0 通道）。
        for y in 2..8 {
            g.set_log_odds(Index3::new(3, y, 0), 2.0);
        }
        let a = AStar2D::new(0, 0.1);
        let path = a
            .plan(&g, GridPoint { x: 0, y: 4 }, GridPoint { x: 6, y: 4 })
            .unwrap();
        for p in &path {
            assert!(
                !(p.x == 3 && (2..8).contains(&p.y)),
                "path crosses wall at {p:?}"
            );
        }
    }

    #[test]
    fn goal_blocked_returns_none() {
        let mut g = grid();
        g.set_log_odds(Index3::new(5, 5, 0), 2.0);
        let a = AStar2D::new(0, 0.1);
        assert!(a
            .plan(&g, GridPoint { x: 0, y: 0 }, GridPoint { x: 5, y: 5 })
            .is_none());
    }
}
