//! 动态窗口法（DWA）局部避障。
//!
//! 在考虑无人机运动学极限（最大加速度 / 最大转弯半径）的动态窗口内采样
//! `(v, ω)` 速度组合，模拟短时轨迹，筛选无碰撞者并按“朝向目标 + 前进速度”
//! 打分，毫秒级输出最优速度指令。

use brain_core::Vec3;
use brain_mapping::{CellState, Index3, OccupancyGrid3D};

/// 速度指令（发给小脑）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VelocityCommand {
    /// 前向速度（m/s）。
    pub linear_x: f32,
    /// 偏航角速度（rad/s）。
    pub angular_z: f32,
}

/// DWA 配置。
#[derive(Debug, Clone, Copy)]
pub struct DwaConfig {
    /// 最大前向速度（m/s）。
    pub v_max: f32,
    /// 最小前向速度（m/s）。
    pub v_min: f32,
    /// 最大角速度（rad/s）。
    pub w_max: f32,
    /// 最大线加速度（m/s²）。
    pub a_max: f32,
    /// 最大角加速度（rad/s²）。
    pub alpha_max: f32,
    /// 模拟时间步（s）。
    pub dt: f32,
    /// 模拟时间跨度（s）。
    pub horizon: f32,
    /// 机器人半径（m），碰撞检测外扩。
    pub radius: f32,
    /// 采样数量。
    pub samples: usize,
    /// 打分权重。
    pub w_heading: f32,
    pub w_vel: f32,
}

impl Default for DwaConfig {
    fn default() -> Self {
        Self {
            v_max: 2.0,
            v_min: -0.5,
            w_max: 1.5,
            a_max: 1.0,
            alpha_max: 2.0,
            dt: 0.1,
            horizon: 1.0,
            radius: 0.25,
            samples: 10,
            w_heading: 0.6,
            w_vel: 0.4,
        }
    }
}

/// 动态窗口局部规划器。
pub struct DwaPlanner {
    cfg: DwaConfig,
}

impl DwaPlanner {
    pub fn new(cfg: DwaConfig) -> Self {
        Self { cfg }
    }

    /// 规划一个速度指令。
    pub fn plan(
        &self,
        grid: &OccupancyGrid3D,
        pos: Vec3,
        heading: f32,
        goal: Vec3,
        vel: (f32, f32),
    ) -> Option<VelocityCommand> {
        let (v_now, w_now) = vel;
        // 动态窗口（受加速度限制）。
        let v_min = (v_now - self.cfg.a_max * self.cfg.dt).max(self.cfg.v_min);
        let v_max = (v_now + self.cfg.a_max * self.cfg.dt).min(self.cfg.v_max);
        let w_min = (w_now - self.cfg.alpha_max * self.cfg.dt).max(-self.cfg.w_max);
        let w_max = (w_now + self.cfg.alpha_max * self.cfg.dt).min(self.cfg.w_max);

        let n = self.cfg.samples.max(2);
        let mut best: Option<(f32, VelocityCommand)> = None;

        for i in 0..n {
            let t = if n > 1 {
                i as f32 / (n - 1) as f32
            } else {
                0.0
            };
            let v = v_min + (v_max - v_min) * t;
            for j in 0..n {
                let t2 = if n > 1 {
                    j as f32 / (n - 1) as f32
                } else {
                    0.0
                };
                let w = w_min + (w_max - w_min) * t2;
                if !self.clear_path(grid, pos, heading, v, w) {
                    continue;
                }
                let end_heading = heading + w * self.cfg.horizon;
                let to_goal = goal.sub(pos);
                let goal_angle = to_goal.y.atan2(to_goal.x);
                let heading_cost = (end_heading - goal_angle).sin().abs();
                // 前进速度项（有符号，鼓励向前而非倒车）。
                let speed = if self.cfg.v_max > 0.0 {
                    v / self.cfg.v_max
                } else {
                    0.0
                };
                let score = self.cfg.w_heading * (1.0 - heading_cost) + self.cfg.w_vel * speed;
                if best.map(|(s, _)| score > s).unwrap_or(true) {
                    best = Some((
                        score,
                        VelocityCommand {
                            linear_x: v,
                            angular_z: w,
                        },
                    ));
                }
            }
        }
        best.map(|(_, cmd)| cmd)
    }

    fn clear_path(&self, grid: &OccupancyGrid3D, pos: Vec3, heading: f32, v: f32, w: f32) -> bool {
        let mut x = pos.x;
        let mut y = pos.y;
        let mut th = heading;
        let steps = (self.cfg.horizon / self.cfg.dt).ceil() as usize;
        for _ in 0..steps {
            th += w * self.cfg.dt;
            x += v * th.cos() * self.cfg.dt;
            y += v * th.sin() * self.cfg.dt;
            let p = Vec3::new(x, y, pos.z);
            if self.obstructed(grid, p) {
                return false;
            }
        }
        true
    }

    fn obstructed(&self, grid: &OccupancyGrid3D, p: Vec3) -> bool {
        let r = (self.cfg.radius / grid.config().resolution).ceil() as i32;
        let res = grid.config().resolution;
        let bx = (p.x / res).floor() as i32;
        let by = (p.y / res).floor() as i32;
        let bz = (p.z / res).floor() as i32;
        for dx in -r..=r {
            for dy in -r..=r {
                if dx * dx + dy * dy > r * r {
                    continue;
                }
                let c = Index3::new(bx + dx, by + dy, bz);
                if let Some(CellState::Occupied) = grid.state(c) {
                    return true;
                }
                // 越界视为“地图外”，不作为障碍（避免网格边缘被堵死）。
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_mapping::{GridConfig, Index3};

    #[test]
    fn steers_toward_goal_when_clear() {
        let cfg = DwaConfig::default();
        let grid = OccupancyGrid3D::new(GridConfig::from_world_size(0.5, 20.0, 20.0, 20.0));
        let p = DwaPlanner::new(cfg);
        // 停在原点，目标在 +X 前方。
        let cmd = p
            .plan(
                &grid,
                Vec3::new(1.0, 1.0, 1.0),
                0.0,
                Vec3::new(8.0, 1.0, 1.0),
                (0.0, 0.0),
            )
            .expect("should produce command");
        assert!(cmd.linear_x > 0.0);
        assert!(cmd.angular_z.abs() < 0.5);
    }

    #[test]
    fn refuses_when_fully_blocked() {
        let cfg = DwaConfig::default();
        let mut grid = OccupancyGrid3D::new(GridConfig::from_world_size(0.5, 20.0, 20.0, 20.0));
        // 一堵横贯整个 y 平面的墙（x 0.5..1.5, 全部 y），堵死前进方向。
        for x in 1..4 {
            for y in 0..20 {
                grid.set_log_odds(Index3::new(x, y, 2), 2.0);
            }
        }
        let p = DwaPlanner::new(cfg);
        // 动态窗口内无任何无碰撞指令 → 拒绝前进（安全兜底）。
        let cmd = p.plan(
            &grid,
            Vec3::new(0.5, 0.5, 1.0),
            0.0,
            Vec3::new(8.0, 0.5, 1.0),
            (0.3, 0.0),
        );
        assert!(cmd.is_none(), "should refuse to fly into a wall: {cmd:?}");
    }
}
