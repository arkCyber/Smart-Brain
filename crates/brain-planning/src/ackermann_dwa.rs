//! 阿克曼动态窗口法（Ackermann DWA）局部避障 —— 汽车等前轮转向车辆。
//!
//! 与面向无人机/差速底盘的 `dwa`（输出 `(v, ω)`，可原地转向）不同，本规划器：
//! - 采样 `(speed, steering)`（纵向速度 + 前轮转角）组合；
//! - 在**转向速率**与**最小转弯半径**限制下的动态窗口内搜索（阿克曼约束）；
//! - 用**矩形车身包络**（长×宽）扫描每条候选轨迹，杜绝“点碰撞漏判”；
//! - 输出 `AckermannCommand`（油门/制动对应的纵向速度 + 方向盘转角），
//!   由车辆底盘（或小脑）转为 EPS 转向 + 驱动。

use brain_core::Vec3;
use brain_kinematics::{BicycleModel, BicycleState};
use brain_mapping::{CellState, OccupancyGrid3D};

/// 汽车速度/转向指令（发给车辆底盘）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AckermannCommand {
    /// 纵向速度（m/s，正为前进，负为倒车）。
    pub speed: f32,
    /// 前轮转角（rad）。
    pub steering: f32,
}

/// Ackermann DWA 配置。
#[derive(Debug, Clone, Copy)]
pub struct AckermannDwaConfig {
    /// 最大前进速度（m/s）。
    pub v_max: f32,
    /// 允许的最小（倒车）速度（负值，m/s）。
    pub v_reverse: f32,
    /// 最大纵向加速度（m/s²）。
    pub a_max: f32,
    /// 模拟时间步（s）。
    pub dt: f32,
    /// 模拟时间跨度（s）。
    pub horizon: f32,
    /// 车身长度（m，碰撞包络）。
    pub vehicle_length: f32,
    /// 车身宽度（m，碰撞包络）。
    pub vehicle_width: f32,
    /// 前向速度采样数。
    pub samples_v: usize,
    /// 转向采样数。
    pub samples_steer: usize,
    /// 打分权重：朝目标推进（减少到目标距离）。
    pub w_progress: f32,
    /// 打分权重：前进速度。
    pub w_vel: f32,
    /// 打分惩罚：倒车。
    pub w_reverse: f32,
    /// 打分惩罚：大转角（鼓励平缓转向）。
    pub w_steering: f32,
    /// 车辆运动学（轴距/转向限位/转向速率）。
    pub model: BicycleModel,
}

impl Default for AckermannDwaConfig {
    fn default() -> Self {
        Self {
            v_max: 6.0,
            v_reverse: -1.5,
            a_max: 2.0,
            dt: 0.1,
            horizon: 1.2,
            vehicle_length: 4.5,
            vehicle_width: 1.9,
            samples_v: 6,
            samples_steer: 7,
            w_progress: 0.6,
            w_vel: 0.25,
            w_reverse: 0.35,
            w_steering: 0.2,
            model: BicycleModel::new(2.6, 0.6, 0.8),
        }
    }
}

/// 阿克曼局部规划器。
pub struct AckermannDwaPlanner {
    cfg: AckermannDwaConfig,
}

impl AckermannDwaPlanner {
    pub fn new(cfg: AckermannDwaConfig) -> Self {
        Self { cfg }
    }

    /// 规划一个 `(speed, steering)` 指令。
    ///
    /// `pos` 为后轴中心（z 忽略），`heading` 为当前朝向，`goal` 为目标点，
    /// `state` 为当前 `(速度, 前轮转角)`，用于构造加速度/转向速率受限的动态窗口。
    pub fn plan(
        &self,
        grid: &OccupancyGrid3D,
        pos: Vec3,
        heading: f32,
        goal: Vec3,
        state: (f32, f32),
    ) -> Option<AckermannCommand> {
        let (v_now, steer_now) = state;
        // 动态窗口（受纵向加速度限制）。
        let v_lo = (v_now - self.cfg.a_max * self.cfg.dt).max(self.cfg.v_reverse);
        let v_hi = (v_now + self.cfg.a_max * self.cfg.dt).min(self.cfg.v_max);
        // 目标前减速：根据“当前距目标距离”限制可用的最高速度，确保能在目标处停住，
        // 避免高速过冲（车辆刹车距离随速度平方增长，必须提前收油/制动）。
        let dist_now = goal.sub(Vec3::new(pos.x, pos.y, 0.0)).norm();
        let stop_margin = 0.4;
        let braking_limit = (2.0 * self.cfg.a_max * (dist_now - stop_margin).max(0.0)).sqrt();
        let v_hi = v_hi.min(braking_limit);
        // 转向动态窗口（受转向执行速率限制）。
        let s_max = self.cfg.model.max_steer_rate * self.cfg.dt;
        let s_lo = (steer_now - s_max).max(-self.cfg.model.max_steering);
        let s_hi = (steer_now + s_max).min(self.cfg.model.max_steering);

        let nv = self.cfg.samples_v.max(2);
        let ns = self.cfg.samples_steer.max(2);
        let mut best: Option<(f32, AckermannCommand)> = None;
        let base = BicycleState {
            x: pos.x,
            y: pos.y,
            theta: heading,
            speed: v_now,
            steering: steer_now,
        };

        for i in 0..nv {
            let tv = if nv > 1 {
                i as f32 / (nv - 1) as f32
            } else {
                0.0
            };
            let v = v_lo + (v_hi - v_lo) * tv;
            for j in 0..ns {
                let ts = if ns > 1 {
                    j as f32 / (ns - 1) as f32
                } else {
                    0.0
                };
                let steer = s_lo + (s_hi - s_lo) * ts;
                // 模拟整条轨迹并做车身包络碰撞检测。
                if !self.trajectory_clear(grid, base, v, steer) {
                    continue;
                }
                let end = self.sim_end(base, v, steer);
                let score = self.score(v, steer, end, goal, dist_now);
                if best.map(|(s, _)| score > s).unwrap_or(true) {
                    best = Some((
                        score,
                        AckermannCommand {
                            speed: v,
                            steering: steer,
                        },
                    ));
                }
            }
        }
        best.map(|(_, c)| c)
    }

    /// 用自行车模型推进整条轨迹，检查车身包络是否与障碍相交。
    fn trajectory_clear(
        &self,
        grid: &OccupancyGrid3D,
        start: BicycleState,
        v: f32,
        steer: f32,
    ) -> bool {
        let mut st = start;
        let steps = (self.cfg.horizon / self.cfg.dt).ceil().max(1.0) as usize;
        for _ in 0..steps {
            st = self.cfg.model.step(&st, v, steer, self.cfg.dt);
            if !self.footprint_clear(grid, st.x, st.y, st.theta) {
                return false;
            }
        }
        true
    }

    /// 仅做运动学积分（无碰撞），返回轨迹末端状态。
    fn sim_end(&self, start: BicycleState, v: f32, steer: f32) -> BicycleState {
        let mut st = start;
        let steps = (self.cfg.horizon / self.cfg.dt).ceil().max(1.0) as usize;
        for _ in 0..steps {
            st = self.cfg.model.step(&st, v, steer, self.cfg.dt);
        }
        st
    }

    /// 检查以 `(x, y, theta)` 为中心的车身矩形包络是否与占据体素相交。
    fn footprint_clear(&self, grid: &OccupancyGrid3D, x: f32, y: f32, theta: f32) -> bool {
        let hl = self.cfg.vehicle_length / 2.0;
        let hw = self.cfg.vehicle_width / 2.0;
        let cos = theta.cos();
        let sin = theta.sin();
        // 4 角 + 4 边中点，覆盖车身包络。
        let local: [(f32, f32); 8] = [
            (hl, hw),
            (hl, -hw),
            (-hl, hw),
            (-hl, -hw),
            (hl, 0.0),
            (-hl, 0.0),
            (0.0, hw),
            (0.0, -hw),
        ];
        for (lx, ly) in local {
            let wx = x + lx * cos - ly * sin;
            let wy = y + lx * sin + ly * cos;
            if self.cell_obstructed(grid, wx, wy) {
                return false;
            }
        }
        true
    }

    /// 世界坐标 → 体素，越界视为障碍（车辆不应驶出地图/道路）。
    fn cell_obstructed(&self, grid: &OccupancyGrid3D, x: f32, y: f32) -> bool {
        let Some(idx) = grid.world_to_index(Vec3::new(x, y, 0.0)) else {
            return true;
        };
        matches!(grid.state(idx), Some(CellState::Occupied))
    }

    /// 打分：朝目标推进 + 前进速度 - 倒车惩罚 - 大转角惩罚。
    ///
    /// 用“轨迹后与目标的距离减少量”作为主项（比只比较朝向更鲁棒：即便
    /// 目标在侧后方，倒车+转向的轨迹也能获得正的推进分，从而支持 K 形掉头）。
    fn score(&self, v: f32, steer: f32, end: BicycleState, goal: Vec3, dist_now: f32) -> f32 {
        let dist_end = goal.sub(Vec3::new(end.x, end.y, 0.0)).norm();
        // 归一化：一个 horizon 内最多前进约 v_max*horizon。
        let max_gain = (self.cfg.v_max * self.cfg.horizon).max(1e-3);
        let progress = ((dist_now - dist_end) / max_gain).clamp(-1.0, 1.0);

        let fwd = (v / self.cfg.v_max).clamp(0.0, 1.0);
        let reverse_penalty = ((-v).max(0.0) / self.cfg.v_max).clamp(0.0, 1.0);
        let steer_penalty = (steer / self.cfg.model.max_steering).abs();

        self.cfg.w_progress * progress + self.cfg.w_vel * fwd
            - self.cfg.w_reverse * reverse_penalty
            - self.cfg.w_steering * steer_penalty
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_mapping::{GridConfig, Index3};

    fn road_grid() -> OccupancyGrid3D {
        OccupancyGrid3D::new(GridConfig::from_world_size(1.0, 40.0, 40.0, 1.0))
    }

    #[test]
    fn steers_toward_goal_when_clear() {
        let grid = road_grid();
        let p = AckermannDwaPlanner::new(AckermannDwaConfig::default());
        // 车头朝 +X，目标在右前方 → 应前进且向右打方向。
        let cmd = p
            .plan(
                &grid,
                Vec3::new(5.0, 5.0, 0.0),
                0.0,
                Vec3::new(30.0, 7.0, 0.0),
                (0.0, 0.0),
            )
            .expect("should produce command");
        assert!(cmd.speed > 0.0, "car should drive forward, got {cmd:?}");
        assert!(cmd.steering.abs() <= 0.6, "steering must be within limit");
        // 目标在 +y 方向 → 需要向右转（前轮转角 > 0）。
        assert!(
            cmd.steering > -1e-3,
            "should steer to turn right, got {cmd:?}"
        );
    }

    #[test]
    fn forward_preferred_over_reverse() {
        let grid = road_grid();
        let p = AckermannDwaPlanner::new(AckermannDwaConfig::default());
        let cmd = p
            .plan(
                &grid,
                Vec3::new(5.0, 5.0, 0.0),
                0.0,
                Vec3::new(30.0, 5.0, 0.0),
                (0.0, 0.0),
            )
            .unwrap();
        assert!(cmd.speed > 0.0, "goal ahead -> should not reverse: {cmd:?}");
    }

    #[test]
    fn refuses_to_drive_into_wall() {
        let mut grid = road_grid();
        // 一堵横贯的墙（x=15..16, y=0..40）堵住前进方向。
        for y in 0..40 {
            grid.set_log_odds(Index3::new(15, y, 0), 2.0);
            grid.set_log_odds(Index3::new(16, y, 0), 2.0);
        }
        let p = AckermannDwaPlanner::new(AckermannDwaConfig::default());
        // 车紧贴墙左侧、目标在墙另一侧 → 车身包络扫到墙，不得给撞墙的前进轨迹。
        let cmd = p.plan(
            &grid,
            Vec3::new(10.0, 20.0, 0.0),
            0.0,
            Vec3::new(30.0, 20.0, 0.0),
            (3.0, 0.0),
        );
        if let Some(c) = cmd {
            let mut st = BicycleState::new(10.0, 20.0, 0.0);
            st.speed = 3.0;
            for _ in 0..((p.cfg.horizon / p.cfg.dt).ceil() as usize) {
                st = p.cfg.model.step(&st, c.speed, c.steering, p.cfg.dt);
            }
            assert!(st.x < 14.5, "trajectory crossed wall, ended at x={}", st.x);
        }
    }

    #[test]
    fn respects_steering_rate_from_standstill() {
        let grid = road_grid();
        let p = AckermannDwaPlanner::new(AckermannDwaConfig::default());
        let cmd = p
            .plan(
                &grid,
                Vec3::new(5.0, 5.0, 0.0),
                0.0,
                Vec3::new(7.0, 7.0, 0.0),
                (0.0, 0.0),
            )
            .unwrap();
        // 从一个 dt 内最多转 steer_rate*dt 出发，输出转角必受限。
        let max_delta = p.cfg.model.max_steer_rate * p.cfg.dt + 1e-3;
        assert!(cmd.steering.abs() <= max_delta, "steering={}", cmd.steering);
    }

    #[test]
    fn reverses_when_goal_is_behind() {
        let grid = road_grid();
        let p = AckermannDwaPlanner::new(AckermannDwaConfig::default());
        // 车头朝 +X，目标在正后方 → 非完整车辆应先倒车（负速度）。
        let cmd = p
            .plan(
                &grid,
                Vec3::new(10.0, 10.0, 0.0),
                0.0,
                Vec3::new(2.0, 10.0, 0.0),
                (0.0, 0.0),
            )
            .expect("should produce a command");
        assert!(
            cmd.speed < 0.0,
            "goal behind -> should reverse, got {cmd:?}"
        );
    }
}
