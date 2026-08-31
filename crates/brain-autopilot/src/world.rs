//! 二维地面真值世界。

/// 一个二维栅格世界（单元尺寸由外部分辨率解释）。
pub struct World {
    width: usize,
    height: usize,
    /// 是否占据。
    occ: Vec<bool>,
}

impl World {
    pub fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            occ: vec![false; width * height],
        }
    }

    pub fn width(&self) -> usize {
        self.width
    }
    pub fn height(&self) -> usize {
        self.height
    }

    fn in_bounds(&self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && (x as usize) < self.width && (y as usize) < self.height
    }

    /// 放置障碍（单元坐标）。
    pub fn set_obstacle(&mut self, x: usize, y: usize) {
        if x < self.width && y < self.height {
            self.occ[y * self.width + x] = true;
        }
    }

    /// 查询单元是否被占据。
    pub fn is_obstacle(&self, x: i32, y: i32) -> bool {
        if !self.in_bounds(x, y) {
            return true; // 世界外视为障碍
        }
        self.occ[y as usize * self.width + x as usize]
    }

    /// 世界坐标处是否被占据（对坐标取整）。
    pub fn is_obstacle_at(&self, x: f32, y: f32) -> bool {
        self.is_obstacle(x.floor() as i32, y.floor() as i32)
    }

    /// 空闲（非障碍）单元总数。
    pub fn free_cells(&self) -> usize {
        self.occ.iter().filter(|&&o| !o).count()
    }

    /// 放置一堵墙（用迭代器在 x,y 上设置障碍）。
    pub fn wall(&mut self, xs: std::ops::Range<usize>, ys: std::ops::Range<usize>) {
        for x in xs {
            for y in ys.clone() {
                self.set_obstacle(x, y);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounds_and_obstacles() {
        let mut w = World::new(10, 10);
        assert_eq!(w.free_cells(), 100);
        w.set_obstacle(2, 3);
        assert!(w.is_obstacle(2, 3));
        assert!(!w.is_obstacle(1, 3));
        assert!(w.is_obstacle_at(2.7, 3.9)); // floor -> (2,3)
        assert!(w.is_obstacle(-1, 0)); // 越界视为障碍
    }
}
