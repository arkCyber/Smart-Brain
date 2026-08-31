//! 三维占据网格与体素状态。

use brain_core::time::Timestamp;
use brain_core::Vec3;

/// 体素（cell）坐标。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Index3 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Index3 {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }
}

/// 体素状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellState {
    /// 空闲（可通行）。
    Free,
    /// 已占据（障碍）。
    Occupied,
    /// 未知（尚未观测）。
    Unknown,
}

/// 网格配置。
#[derive(Debug, Clone, Copy)]
pub struct GridConfig {
    /// 体素边长（米）。
    pub resolution: f32,
    /// 每个维度体素数量。
    pub size_x: usize,
    pub size_y: usize,
    pub size_z: usize,
}

impl GridConfig {
    /// 以世界坐标尺寸与分辨率构造网格（自动推导体素数量）。
    pub fn from_world_size(resolution: f32, wx: f32, wy: f32, wz: f32) -> Self {
        Self {
            resolution,
            size_x: (wx / resolution).ceil() as usize,
            size_y: (wy / resolution).ceil() as usize,
            size_z: (wz / resolution).ceil() as usize,
        }
    }
}

/// 判断 log-odds 值对应的体素状态。
fn state_from_log_odds(lo: f32, occ: f32, free: f32) -> CellState {
    if lo >= occ {
        CellState::Occupied
    } else if lo <= free {
        CellState::Free
    } else {
        CellState::Unknown
    }
}

/// 概率 3D 占据网格（log-odds 存储，初始为 0 = 未知）。
#[derive(Clone)]
pub struct OccupancyGrid3D {
    config: GridConfig,
    /// 网格最小角的世界坐标。
    origin: Vec3,
    /// log-odds 值，长度为 size_x*size_y*size_z。
    cells: Vec<f32>,
    /// 占据/空闲阈值。
    thr_occ: f32,
    thr_free: f32,
    last_update: Timestamp,
}

impl OccupancyGrid3D {
    /// 创建网格。所有体素初始为“未知”。
    pub fn new(config: GridConfig) -> Self {
        let total = config.size_x * config.size_y * config.size_z;
        Self {
            config,
            origin: Vec3::ZERO,
            cells: vec![0.0; total],
            thr_occ: 0.6,
            thr_free: -0.4,
            last_update: 0,
        }
    }

    /// 网格配置。
    pub fn config(&self) -> &GridConfig {
        &self.config
    }

    /// 网格最小角的世界坐标。
    pub fn origin(&self) -> Vec3 {
        self.origin
    }

    /// 世界坐标 → 体素索引；越界返回 `None`。
    pub fn world_to_index(&self, p: Vec3) -> Option<Index3> {
        let inv = 1.0 / self.config.resolution;
        let d = p.sub(self.origin) * inv;
        let x = d.x.floor() as i32;
        let y = d.y.floor() as i32;
        let z = d.z.floor() as i32;
        if x < 0 || y < 0 || z < 0 {
            return None;
        }
        let (ux, uy, uz) = (x as usize, y as usize, z as usize);
        if ux >= self.config.size_x || uy >= self.config.size_y || uz >= self.config.size_z {
            return None;
        }
        Some(Index3::new(x, y, z))
    }

    /// 体素中心的世界坐标。
    pub fn index_to_world_center(&self, i: Index3) -> Vec3 {
        Vec3::new(
            self.origin.x + (i.x as f32 + 0.5) * self.config.resolution,
            self.origin.y + (i.y as f32 + 0.5) * self.config.resolution,
            self.origin.z + (i.z as f32 + 0.5) * self.config.resolution,
        )
    }

    fn flat(&self, i: Index3) -> Option<usize> {
        if i.x < 0 || i.y < 0 || i.z < 0 {
            return None;
        }
        let (x, y, z) = (i.x as usize, i.y as usize, i.z as usize);
        if x >= self.config.size_x || y >= self.config.size_y || z >= self.config.size_z {
            return None;
        }
        Some((z * self.config.size_y + y) * self.config.size_x + x)
    }

    /// 直接设置某体素的 log-odds。
    pub fn set_log_odds(&mut self, i: Index3, lo: f32) -> bool {
        if let Some(f) = self.flat(i) {
            self.cells[f] = lo;
            true
        } else {
            false
        }
    }

    /// 获取某体素的 log-odds。
    pub fn log_odds(&self, i: Index3) -> Option<f32> {
        self.flat(i).map(|f| self.cells[f])
    }

    /// 获取某体素状态。
    pub fn state(&self, i: Index3) -> Option<CellState> {
        self.log_odds(i)
            .map(|lo| state_from_log_odds(lo, self.thr_occ, self.thr_free))
    }

    /// 世界坐标处是否被占据。
    pub fn is_occupied(&self, p: Vec3) -> bool {
        match self.world_to_index(p).and_then(|i| self.state(i)) {
            Some(CellState::Occupied) => true,
            _ => false,
        }
    }

    /// 世界坐标处是否可通行（空闲）。
    pub fn is_free(&self, p: Vec3) -> bool {
        match self.world_to_index(p).and_then(|i| self.state(i)) {
            Some(CellState::Free) => true,
            _ => false,
        }
    }

    /// 统计三种状态的数量。
    pub fn counts(&self) -> (usize, usize, usize) {
        let mut free = 0;
        let mut occ = 0;
        let mut unk = 0;
        for &lo in &self.cells {
            match state_from_log_odds(lo, self.thr_occ, self.thr_free) {
                CellState::Free => free += 1,
                CellState::Occupied => occ += 1,
                CellState::Unknown => unk += 1,
            }
        }
        (free, occ, unk)
    }

    /// 最近更新时间戳。
    pub fn last_update(&self) -> Timestamp {
        self.last_update
    }

    /// 标记一次网格更新完成。
    pub fn mark_update(&mut self, t: Timestamp) {
        self.last_update = t;
    }

    /// 障碍膨胀：把每个占据体素在 x/y 方向外扩 `radius_cells` 格。
    ///
    /// 返回新网格，为路径规划/避障预留安全边距（z 保持不变）。
    pub fn inflate(&self, radius_cells: i32) -> OccupancyGrid3D {
        let mut out = self.clone();
        let r = radius_cells.max(0);
        let occupied: Vec<Index3> = self
            .all_cells()
            .into_iter()
            .filter(|i| self.state(*i) == Some(CellState::Occupied))
            .collect();
        for idx in occupied {
            for dx in -r..=r {
                for dy in -r..=r {
                    if dx * dx + dy * dy > r * r {
                        continue;
                    }
                    out.set_log_odds(
                        Index3::new(idx.x + dx, idx.y + dy, idx.z),
                        self.thr_occ + 0.1,
                    );
                }
            }
        }
        out
    }

    /// 遍历网格中所有体素索引（迭代顺序无关紧要）。
    pub fn all_cells(&self) -> Vec<Index3> {
        let mut v = Vec::new();
        let (nx, ny, nz) = (
            self.config.size_x as i32,
            self.config.size_y as i32,
            self.config.size_z as i32,
        );
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    v.push(Index3::new(x, y, z));
                }
            }
        }
        v
    }

    /// 前沿（frontier）探测：空闲体素且存在未知邻居 → 探索目标候选。
    pub fn frontiers(&self) -> Vec<Index3> {
        let mut out = Vec::new();
        let (nx, ny, nz) = (
            self.config.size_x as i32,
            self.config.size_y as i32,
            self.config.size_z as i32,
        );
        for z in 0..nz {
            for y in 0..ny {
                for x in 0..nx {
                    let idx = Index3::new(x, y, z);
                    if self.state(idx) != Some(CellState::Free) {
                        continue;
                    }
                    // 检查 26 邻域是否有未知。
                    let mut frontier = false;
                    'nbr: for dz in -1..=1i32 {
                        for dy in -1..=1i32 {
                            for dx in -1..=1i32 {
                                if dx == 0 && dy == 0 && dz == 0 {
                                    continue;
                                }
                                let n = Index3::new(x + dx, y + dy, z + dz);
                                if self.state(n) == Some(CellState::Unknown) {
                                    frontier = true;
                                    break 'nbr;
                                }
                            }
                        }
                    }
                    if frontier {
                        out.push(idx);
                    }
                }
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn world_to_index_roundtrip() {
        let cfg = GridConfig::from_world_size(0.5, 10.0, 10.0, 10.0);
        let g = OccupancyGrid3D::new(cfg);
        let i = g.world_to_index(Vec3::new(1.25, 2.75, 0.25)).unwrap();
        assert_eq!(i, Index3::new(2, 5, 0));
        let c = g.index_to_world_center(i);
        assert!((c.x - 1.25).abs() < 1e-4);
    }

    #[test]
    fn states_after_set() {
        let cfg = GridConfig::from_world_size(0.5, 4.0, 4.0, 4.0);
        let mut g = OccupancyGrid3D::new(cfg);
        assert_eq!(g.state(Index3::new(0, 0, 0)), Some(CellState::Unknown));
        g.set_log_odds(Index3::new(1, 1, 1), 2.0);
        assert_eq!(g.state(Index3::new(1, 1, 1)), Some(CellState::Occupied));
        g.set_log_odds(Index3::new(2, 2, 2), -2.0);
        assert_eq!(g.state(Index3::new(2, 2, 2)), Some(CellState::Free));
        assert!(g.is_occupied(Vec3::new(0.75, 0.75, 0.75)));
        assert!(g.is_free(Vec3::new(1.25, 1.25, 1.25)));
    }

    #[test]
    fn out_of_bounds() {
        let cfg = GridConfig::from_world_size(1.0, 2.0, 2.0, 2.0);
        let g = OccupancyGrid3D::new(cfg);
        assert!(g.world_to_index(Vec3::new(99.0, 0.0, 0.0)).is_none());
        assert_eq!(g.state(Index3::new(-1, 0, 0)), None);
    }

    #[test]
    fn inflate_expands_obstacles() {
        let cfg = GridConfig::from_world_size(1.0, 6.0, 6.0, 1.0);
        let mut g = OccupancyGrid3D::new(cfg);
        g.set_log_odds(Index3::new(3, 3, 0), 2.0); // 单个障碍
                                                   // 膨胀半径 1：四邻域也应变为占据，但相隔 2 格的仍为空闲。
        let inflated = g.inflate(1);
        assert_eq!(
            inflated.state(Index3::new(3, 3, 0)),
            Some(CellState::Occupied)
        );
        assert_eq!(
            inflated.state(Index3::new(4, 3, 0)),
            Some(CellState::Occupied)
        );
        assert_eq!(
            inflated.state(Index3::new(3, 2, 0)),
            Some(CellState::Occupied)
        );
        assert_eq!(
            inflated.state(Index3::new(5, 3, 0)),
            Some(CellState::Unknown)
        );
    }
}
