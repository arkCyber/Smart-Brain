//! 水面艇（ASV/USV）闭环自主导航 —— 双差速推进 + 水流漂移 + 定泊保持。
//!
//! 与汽车（阿克曼）不同，水面艇通常用**左右双推进器差速**（`v, ω`）转向，
//! 无法“刹车”，且受**水流漂移**影响。本控制器：
//! - 全局引导 A*/RRT + 差分 **DWA 局部避障**（输出 `(v, ω)`）；
//! - 每步叠加**水流漂移**（恒定水流向量）；
//! - 抵达目标后支持**定泊保持 / 动力定位**：让艇逆流顶流、维持目标位姿；
//! - 复用 `World`/`RangeSensor`/占据网格做水面障碍避让。

use brain_core::Vec3;
use brain_kinematics::norm_angle;
use brain_mapping::{GridConfig, OccupancyGrid3D, RaycastUpdater};
use brain_nav::{Backtracker, Explorer};
use brain_planning::{AStar2D, DwaConfig, DwaPlanner, GridPoint, RrtConfig, RrtPlanner};

use crate::sensor::RangeSensor;
use crate::world::World;
use crate::StepOutcome;

/// 一次水面艇导航运行的统计。
#[derive(Debug, Clone)]
pub struct BoatStats {
    pub steps: usize,
    pub distance: f32,
    /// 结束是否安全（不落在障碍内）。
    pub safe: bool,
    pub end_pose: (f32, f32, f32),
    /// 是否处于定泊保持状态。
    pub station_keeping: bool,
}

/// 潮汐模型：叠加在基础水流上的时变分量（正弦）。
#[derive(Debug, Clone, Copy)]
pub struct Tide {
    /// 流速幅值（m/s）。
    pub amplitude: f32,
    /// 周期（秒）。
    pub period_s: f32,
    /// 流向（世界系，rad）。
    pub direction: f32,
}

impl Default for Tide {
    fn default() -> Self {
        Self {
            amplitude: 0.0,
            period_s: 600.0,
            direction: 0.0,
        }
    }
}

/// 水面艇导航配置。
#[derive(Debug, Clone)]
pub struct BoatConfig {
    /// 占据网格分辨率（米/格）。
    pub grid_res: f32,
    /// 传感器光线数。
    pub sensor_rays: usize,
    /// 传感器量程（米）。
    pub sensor_range: f32,
    /// 差分局部避障配置。
    pub dwa: DwaConfig,
    /// 水流速度（世界系，m/s）。
    pub current: (f32, f32),
    /// 潮汐（时变水流，幅值 0 表示无潮汐）。
    pub tide: Tide,
    /// 运动积分步长（秒）。
    pub dt: f32,
    /// 到达目标容差（米）。
    pub goal_tol: f32,
    /// 定泊保持的触发半径（米）：距目标该距离内开始逆流顶住并转向。
    pub station_radius: f32,
    /// 抵达后是否定泊保持（动力定位，逆流顶住）。
    pub station_keep: bool,
}

impl Default for BoatConfig {
    fn default() -> Self {
        Self {
            grid_res: 1.0,
            sensor_rays: 32,
            sensor_range: 10.0,
            dwa: DwaConfig {
                v_max: 4.0,
                w_max: 0.8,
                v_min: 0.0, // 水面艇一般不倒退
                ..DwaConfig::default()
            },
            current: (0.0, 0.0),
            tide: Tide::default(),
            dt: 0.1,
            goal_tol: 0.8,
            station_radius: 4.0,
            station_keep: true,
        }
    }
}

/// 水面艇自主导航控制器。
pub struct BoatAutopilot {
    cfg: BoatConfig,
    pose: (f32, f32, f32),
    vel: (f32, f32),
    local: OccupancyGrid3D,
    sensor: RangeSensor,
    dwa: DwaPlanner,
    astar: AStar2D,
    rrt: RrtPlanner,
    explorer: Explorer,
    backtrack: Backtracker,
    goal: Option<Vec3>,
    /// 点对点固定目标（到达即停/定泊，而非转去探索）。
    fixed_goal: Option<Vec3>,
    /// 多点巡航航迹（依次到达各航点）。
    track: Vec<Vec3>,
    /// 当前巡航到的航点索引。
    track_idx: usize,
    path: Vec<Vec3>,
    path_idx: usize,
    stuck: usize,
    rewind: Vec<Vec3>,
    distance: f32,
    ts: u64,
    /// 当前是否处于定泊保持。
    station_keeping: bool,
}

impl BoatAutopilot {
    pub fn new(
        cfg: BoatConfig,
        width_cells: usize,
        height_cells: usize,
        start: (f32, f32, f32),
    ) -> Self {
        let zslice = 0;
        let grid = OccupancyGrid3D::new(GridConfig::from_world_size(
            cfg.grid_res,
            width_cells as f32 * cfg.grid_res,
            height_cells as f32 * cfg.grid_res,
            cfg.grid_res,
        ));
        Self {
            sensor: RangeSensor::new(cfg.sensor_rays, cfg.sensor_range),
            dwa: DwaPlanner::new(cfg.dwa),
            astar: AStar2D::new(zslice, 0.4),
            rrt: RrtPlanner::new(RrtConfig::default()),
            explorer: Explorer::new(zslice),
            backtrack: Backtracker::new(4096, 0.8),
            cfg,
            pose: start,
            vel: (0.0, 0.0),
            local: grid,
            goal: None,
            fixed_goal: None,
            track: Vec::new(),
            track_idx: 0,
            path: Vec::new(),
            path_idx: 0,
            stuck: 0,
            rewind: Vec::new(),
            distance: 0.0,
            ts: 0,
            station_keeping: false,
        }
    }

    pub fn pose(&self) -> (f32, f32, f32) {
        self.pose
    }

    /// 是否处于定泊保持。
    pub fn is_station_keeping(&self) -> bool {
        self.station_keeping
    }

    /// 累计航程（米）。
    pub fn distance(&self) -> f32 {
        self.distance
    }

    /// 点对点导航：设定目的地。
    pub fn set_goal(&mut self, goal: Vec3) {
        self.fixed_goal = Some(goal);
        self.goal = Some(goal);
        self.replan();
    }

    /// 多点巡航：依次驶向一串航点（巡航模式，到达每个航点后自动切下一段）。
    pub fn set_track(&mut self, waypoints: Vec<Vec3>) {
        self.track = waypoints;
        self.track_idx = 0;
        self.cfg.station_keep = false; // 巡航：逐个到达，不停泊
        if let Some(first) = self.track.first().copied() {
            self.set_goal(first);
            self.track_idx = 1;
        }
    }

    /// 到达当前航点后切换到下一个；返回是否还有下一段。
    fn advance_track(&mut self) -> bool {
        if self.track_idx < self.track.len() {
            let g = self.track[self.track_idx];
            self.track_idx += 1;
            self.set_goal(g);
            true
        } else {
            false
        }
    }

    /// 当前时刻的水流速度 = 基础水流 + 潮汐（时变正弦分量）。
    fn current_at(&self) -> (f32, f32) {
        let (cx, cy) = self.cfg.current;
        let t = self.ts as f32 * self.cfg.dt;
        let tide = self.cfg.tide;
        let mag = tide.amplitude * (std::f32::consts::TAU * t / tide.period_s.max(1e-3)).sin();
        (
            cx + mag * tide.direction.cos(),
            cy + mag * tide.direction.sin(),
        )
    }
    fn sense(&mut self, world: &World) {
        let (x, y, th) = self.pose;
        let scan = self.sensor.scan(world, x, y, th);
        let updater = RaycastUpdater::default();
        let origin = Vec3::new(x, y, 0.0);
        let range = self.cfg.sensor_range;
        updater.update_point_cloud(&mut self.local, origin, &scan.hits, range);
        for p in &scan.free_ends {
            let dir = p.sub(origin).normalized();
            updater.update_ray(&mut self.local, origin, dir, range, None);
        }
    }

    fn replan(&mut self) {
        self.path.clear();
        self.path_idx = 0;
        let Some(g) = self.goal else { return };
        let start = (self.pose.0, self.pose.1);
        let goal = (g.x, g.y);
        let wp: Vec<Vec3> =
            if let Some(sc) = self.local.world_to_index(Vec3::new(start.0, start.1, 0.0)) {
                let Some(gc) = self.local.world_to_index(Vec3::new(goal.0, goal.1, 0.0)) else {
                    return;
                };
                let mut out = Vec::new();
                if let Some(path) = self.astar.plan(
                    &self.local,
                    GridPoint { x: sc.x, y: sc.y },
                    GridPoint { x: gc.x, y: gc.y },
                ) {
                    for p in path {
                        let c = AStar2D::point_to_world(&self.local, p, 0);
                        out.push(Vec3::new(c.x, c.y, 0.0));
                    }
                }
                out
            } else {
                self.rrt
                    .plan(&self.local, start, goal)
                    .map(|p| p.into_iter().map(|(x, y)| Vec3::new(x, y, 0.0)).collect())
                    .unwrap_or_default()
            };
        if !wp.is_empty() {
            self.path = wp;
            self.path_idx = 0;
        }
    }

    fn refresh_plan(&mut self) {
        if !self.rewind.is_empty() {
            self.path.clear();
            self.goal = self.rewind.pop();
            return;
        }
        let reached = match self.goal {
            Some(g) => {
                Vec3::new(g.x, g.y, 0.0)
                    .sub(Vec3::new(self.pose.0, self.pose.1, 0.0))
                    .norm()
                    < self.cfg.goal_tol
            }
            None => true,
        };
        if reached || self.goal.is_none() {
            if self.fixed_goal.is_some() {
                // 点对点模式：到达固定目标 → 若开启定泊则保持（由 step_once 处理），
                // 否则清空目标结束导航。
                if !self.cfg.station_keep {
                    self.goal = None;
                    self.path.clear();
                    self.path_idx = 0;
                }
                return;
            }
            self.goal = self
                .explorer
                .next_target(&self.local, Vec3::new(self.pose.0, self.pose.1, 0.0))
                .map(|t| t.position);
            self.replan();
            return;
        }
        let (x, y, th) = self.pose;
        let (hx, hy) = (th.cos(), th.sin());
        while self.path_idx < self.path.len() {
            let wp = self.path[self.path_idx];
            let dx = wp.x - x;
            let dy = wp.y - y;
            let dist = (dx * dx + dy * dy).sqrt();
            let ahead = dx * hx + dy * hy;
            if dist < self.cfg.goal_tol || ahead < 0.0 {
                self.path_idx += 1;
            } else {
                break;
            }
        }
        if self.path_idx >= self.path.len() {
            self.path.clear();
            self.path_idx = 0;
        }
    }

    fn current_target(&self) -> Option<Vec3> {
        let mut target = self.goal;
        let mut found = false;
        for wp in self.path.iter().skip(self.path_idx) {
            let d = Vec3::new(wp.x, wp.y, 0.0)
                .sub(Vec3::new(self.pose.0, self.pose.1, 0.0))
                .norm();
            if d >= 3.0 {
                target = Some(*wp);
                found = true;
                break;
            }
        }
        if !found && self.path_idx < self.path.len() {
            target = Some(self.path[self.path.len() - 1]);
        }
        target
    }

    /// 判断命令轨迹（差分）在 dt 内是否与真值障碍相交（含水流漂移）。
    fn path_clear(&self, world: &World, v: f32, omega: f32) -> bool {
        let (x, y, th) = self.pose;
        let dt = self.cfg.dt;
        let steps = 4;
        let (cx, cy) = self.current_at();
        let mut px = x;
        let mut py = y;
        let mut pth = th;
        for _ in 0..steps {
            let s = dt / steps as f32;
            pth += omega * s;
            px += v * pth.cos() * s + cx * s;
            py += v * pth.sin() * s + cy * s;
            if world.is_obstacle_at(px, py) {
                return false;
            }
        }
        true
    }

    /// 执行一步。
    pub fn step_once(&mut self, world: &World) -> StepOutcome {
        self.ts += 1;
        self.sense(world);
        if self.goal.is_some() {
            self.replan();
        }
        self.refresh_plan();

        let (x, y, th) = self.pose;
        let pos = Vec3::new(x, y, 0.0);

        // 定泊保持：进入触发半径后逆流顶住并收敛到目标位姿（动力定位）。
        // 一旦进入保持状态则“粘住”，避免与 DWA 在边界来回切换。
        let at_goal = self
            .goal
            .map(|g| Vec3::new(g.x, g.y, 0.0).sub(pos).norm() < self.cfg.station_radius)
            .unwrap_or(false);
        if (self.station_keeping || at_goal) && self.cfg.station_keep {
            self.station_keeping = true;
            let g = self.goal.unwrap_or(pos);
            let (cx, cy) = self.current_at();
            // 位置 P 控制 + 水流前馈：所需推进速度 = k·位置误差 − 水流（逆流顶住）。
            let k = 0.5;
            let reqx = k * (g.x - x) - cx;
            let reqy = k * (g.y - y) - cy;
            let v_des = (reqx * reqx + reqy * reqy).sqrt();
            let des = reqy.atan2(reqx);
            let err = norm_angle(des - th);
            let omega = (1.5 * err).clamp(-self.cfg.dwa.w_max, self.cfg.dwa.w_max);
            let align = 1.0 - (err.abs() / std::f32::consts::PI).min(0.9);
            let v = v_des.min(self.cfg.dwa.v_max) * align;
            self.vel = (v, omega);
            self.integrate(v, omega);
            return StepOutcome::Moving;
        }
        self.station_keeping = false;

        let target = self.current_target();
        let cmd = match target {
            Some(g) => self.dwa.plan(&self.local, pos, th, g, self.vel),
            None => None,
        };

        let moved = match cmd {
            None => None,
            Some(c) => {
                // 近目标减速（水面艇无刹车，需提前收油，避免过冲）。
                let dgoal = self
                    .goal
                    .map(|g| Vec3::new(g.x, g.y, 0.0).sub(pos).norm())
                    .unwrap_or(f32::MAX);
                let v_lim = (dgoal * 0.6).min(self.cfg.dwa.v_max);
                let v = c.linear_x.min(v_lim);
                if self.path_clear(world, v, c.angular_z) {
                    Some((v, c.angular_z))
                } else {
                    self.stuck += 1;
                    None
                }
            }
        };

        match moved {
            Some((v, w)) => {
                self.stuck = 0;
                self.vel = (v, w);
                self.integrate(v, w);
                self.backtrack
                    .record(self.ts, Vec3::new(self.pose.0, self.pose.1, 0.0));
                StepOutcome::Moving
            }
            None => {
                self.stuck += 1;
                self.replan();
                if self.stuck >= 10 && self.rewind.is_empty() {
                    self.rewind = self.backtrack.start_rewind();
                    if !self.rewind.is_empty() {
                        return StepOutcome::Backtracking;
                    }
                }
                if self.goal.is_none() {
                    // 多点巡航：还有下一航点则继续，否则完成。
                    if self.advance_track() {
                        StepOutcome::Moving
                    } else {
                        StepOutcome::Done
                    }
                } else if cmd.is_some() {
                    StepOutcome::Blocked
                } else {
                    StepOutcome::Stuck
                }
            }
        }
    }

    /// 差分积分 + 水流漂移（含时变潮汐）。
    fn integrate(&mut self, v: f32, omega: f32) {
        let (x, y, th) = self.pose;
        let dt = self.cfg.dt;
        let (cx, cy) = self.current_at();
        let nth = th + omega * dt;
        let nx = x + v * nth.cos() * dt + cx * dt;
        let ny = y + v * nth.sin() * dt + cy * dt;
        self.distance += ((nx - x).powi(2) + (ny - y).powi(2)).sqrt();
        self.pose = (nx, ny, nth);
    }

    /// 跑完一次任务并返回统计。
    pub fn run(&mut self, world: &World, max_steps: usize) -> BoatStats {
        for _ in 0..max_steps {
            let o = self.step_once(world);
            if o == StepOutcome::Done {
                break;
            }
        }
        BoatStats {
            steps: self.ts as usize,
            distance: self.distance,
            safe: !world.is_obstacle_at(self.pose.0, self.pose.1),
            end_pose: self.pose,
            station_keeping: self.station_keeping,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn boat_reaches_goal_in_open_water() {
        // 无水流直航：艇应沿 +X 到达目标。
        let w = World::new(60, 30);
        let mut cfg = BoatConfig::default();
        cfg.current = (0.0, 0.0);
        cfg.station_keep = false;
        let mut boat = BoatAutopilot::new(cfg, w.width(), w.height(), (3.0, 15.0, 0.0));
        boat.set_goal(Vec3::new(50.0, 15.0, 0.0));
        let mut done = false;
        for _ in 0..1500 {
            if boat.step_once(&w) == StepOutcome::Done {
                done = true;
                break;
            }
        }
        assert!(done, "boat should reach goal, ended at {:?}", boat.pose());
        let (x, y, _) = boat.pose();
        assert!(((x - 50.0).powi(2) + (y - 15.0).powi(2)).sqrt() < 0.8);
    }

    #[test]
    fn boat_counteracts_current_when_holding() {
        // 有水流且开启定泊：艇抵达目标后应逆流顶住，保持在目标附近。
        let w = World::new(80, 40);
        let mut cfg = BoatConfig::default();
        cfg.current = (0.5, 0.0); // 正 x 方向水流（会把艇向东冲）
        cfg.station_keep = true;
        let mut boat = BoatAutopilot::new(cfg, w.width(), w.height(), (5.0, 20.0, 0.0));
        boat.set_goal(Vec3::new(40.0, 20.0, 0.0));
        // 航到目标附近。
        for i in 0..1500 {
            let o = boat.step_once(&w);
            if i < 25 {
                eprintln!(
                    "t{i}: {o:?} pose=({:.1},{:.1},{:.1})",
                    boat.pose().0,
                    boat.pose().1,
                    boat.pose().2
                );
            }
            if o == StepOutcome::Done {
                eprintln!("DONE at t{i}");
                break;
            }
            if boat.is_station_keeping() {
                eprintln!("SK at t{i}");
                break;
            }
        }
        assert!(
            boat.is_station_keeping(),
            "should enter station keeping near goal"
        );
        // 让定泊控制器把艇收敛到目标附近。
        for _ in 0..600 {
            boat.step_once(&w);
        }
        // 检查维持：收敛后应保持在目标 2m 内（逆流顶住，未被冲走）。
        let (gx, gy, _) = (40.0f32, 20.0f32, 0.0f32);
        for _ in 0..300 {
            boat.step_once(&w);
            let (x, y, _) = boat.pose();
            let d = ((x - gx).powi(2) + (y - gy).powi(2)).sqrt();
            assert!(d < 2.0, "station keeping failed, drifted to {d:.2}m");
        }
    }

    #[test]
    fn boat_avoids_obstacles_and_stays_safe() {
        // 水中障碍：艇应绕行且不落入障碍。
        let mut w = World::new(60, 40);
        w.wall(28..30, 10..30); // 一道水中障碍
        let mut cfg = BoatConfig::default();
        cfg.current = (0.0, 0.0);
        cfg.station_keep = false;
        let mut boat = BoatAutopilot::new(cfg, w.width(), w.height(), (5.0, 20.0, 0.0));
        boat.set_goal(Vec3::new(55.0, 20.0, 0.0));
        let stats = boat.run(&w, 1200);
        assert!(stats.safe, "boat ended in obstacle: {:?}", stats.end_pose);
        assert!(stats.distance > 1.0, "boat should have moved");
    }

    #[test]
    fn boat_cruises_multi_waypoint_track() {
        // 多点巡航：依次到达一串航点。
        let w = World::new(80, 30);
        let mut cfg = BoatConfig::default();
        cfg.current = (0.0, 0.0);
        cfg.station_keep = false;
        let mut boat = BoatAutopilot::new(cfg, w.width(), w.height(), (3.0, 15.0, 0.0));
        boat.set_track(vec![
            Vec3::new(20.0, 15.0, 0.0),
            Vec3::new(40.0, 20.0, 0.0),
            Vec3::new(60.0, 15.0, 0.0),
        ]);
        let mut done = false;
        for _ in 0..3000 {
            if boat.step_once(&w) == StepOutcome::Done {
                done = true;
                break;
            }
        }
        assert!(
            done,
            "boat should finish the track, ended at {:?}",
            boat.pose()
        );
        let (x, y, _) = boat.pose();
        // 应到达最后一个航点 (60,15)。
        assert!(
            ((x - 60.0).powi(2) + (y - 15.0).powi(2)).sqrt() < 1.0,
            "not at last wp"
        );
        assert!(boat.distance() > 40.0, "should have cruised a long way");
    }

    #[test]
    fn boat_tide_current_varies_over_time() {
        // 潮汐：同一时刻的水流随周期正弦变化。
        let mut cfg = BoatConfig::default();
        cfg.tide = Tide {
            amplitude: 2.0,
            period_s: 10.0,
            direction: 0.0, // +x 方向潮汐
        };
        let w = World::new(20, 20);
        let mut boat = BoatAutopilot::new(cfg, w.width(), w.height(), (2.0, 10.0, 0.0));
        let c0 = boat.current_at();
        // 推进到 t ≈ 5s（半周期，幅值反号）。
        for _ in 0..50 {
            boat.step_once(&w);
        }
        let c1 = boat.current_at();
        // t=0 时 sin(0)=0 → 仅基础水流；t=5s 时 sin(π)=0 → 也仅基础水流。
        // 而 t=2.5s 时 sin(π/2)=1 → +x 分量最大。这里验证随时间变化。
        assert!(
            (c0.0 - c1.0).abs() < 1e-4,
            "both at sin=0 should match, c0={} c1={}",
            c0.0,
            c1.0
        );
        // 推进到 t=2.5s（1/4 周期，潮汐峰值）。
        let mut boat2 = BoatAutopilot::new(
            BoatConfig {
                tide: Tide {
                    amplitude: 2.0,
                    period_s: 10.0,
                    direction: 0.0,
                },
                ..BoatConfig::default()
            },
            20,
            20,
            (2.0, 10.0, 0.0),
        );
        for _ in 0..25 {
            boat2.step_once(&w);
        }
        let c2 = boat2.current_at();
        assert!(
            c2.0 > 1.9,
            "at quarter period tide should push +x strongly, c2={}",
            c2.0
        );
    }
}
