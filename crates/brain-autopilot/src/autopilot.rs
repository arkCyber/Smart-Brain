//! 闭环自主导航控制器：感知→建图→规划→驱动→回溯。

use brain_core::Vec3;
use brain_mapping::{GridConfig, OccupancyGrid3D, RaycastUpdater};
use brain_nav::{Backtracker, Explorer};
use brain_planning::{AStar2D, DwaConfig, DwaPlanner, GridPoint, RrtConfig, RrtPlanner};

use crate::sensor::RangeSensor;
use crate::world::World;

/// 单步结果。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepOutcome {
    /// 正常前进。
    Moving,
    /// 无可行速度，原地停顿。
    Stuck,
    /// 正在回溯（沿面包屑返回）。
    Backtracking,
    /// 前方为障碍，命令被阻断（未碰撞）。
    Blocked,
    /// 探索完成 / 无目标。
    Done,
}

/// 一次运行的统计。
#[derive(Debug, Clone)]
pub struct RunStats {
    pub steps: usize,
    /// 被障碍阻断的次数（0 = 从未尝试撞墙）。
    pub blocked: usize,
    /// 回溯触发次数。
    pub backtrack_events: usize,
    /// A* 全局路径成功规划次数。
    pub planned: usize,
    /// 已探索空闲面积比例 0..1。
    pub explored_ratio: f32,
    /// 移动总距离。
    pub distance: f32,
    /// 结束是否安全（不落在障碍内）。
    pub safe: bool,
    pub end_pose: (f32, f32, f32),
}

/// 控制器配置。
#[derive(Debug, Clone)]
pub struct AutopilotConfig {
    /// 占据网格分辨率（米/格）。世界单元与网格对齐，默认 1.0。
    pub grid_res: f32,
    /// 传感器光线数。
    pub sensor_rays: usize,
    /// 传感器量程（格）。
    pub sensor_range: f32,
    pub dwa: DwaConfig,
    /// 运动积分步长（秒）。
    pub dt: f32,
    /// 连续停顿多少步触发回溯。
    pub stuck_threshold: usize,
    /// 面包屑间距（米）。
    pub crumb_spacing: f32,
    /// 到达目标容差（米）。
    pub goal_tol: f32,
    /// 全局规划器：`true` 用 RRT（连续空间），`false` 用 A*（栅格）。
    pub use_rrt: bool,
}

impl Default for AutopilotConfig {
    fn default() -> Self {
        let dwa = DwaConfig {
            v_max: 1.0,
            horizon: 0.8,
            radius: 0.3,
            ..DwaConfig::default()
        };
        Self {
            grid_res: 1.0,
            sensor_rays: 32,
            sensor_range: 6.0,
            dwa,
            dt: 0.1,
            stuck_threshold: 8,
            crumb_spacing: 0.6,
            goal_tol: 0.6,
            use_rrt: false,
        }
    }
}

/// 自主导航控制器。
pub struct Autopilot {
    cfg: AutopilotConfig,
    pose: (f32, f32, f32),
    local: OccupancyGrid3D,
    sensor: RangeSensor,
    explorer: Explorer,
    dwa: DwaPlanner,
    astar: AStar2D,
    rrt: RrtPlanner,
    backtrack: Backtracker,
    vel: (f32, f32),
    goal: Option<Vec3>,
    path: Vec<Vec3>,
    path_idx: usize,
    stuck: usize,
    rewind: Vec<Vec3>,
    blocked: usize,
    backtrack_events: usize,
    planned: usize,
    distance: f32,
    ts: u64,
}

impl Autopilot {
    /// 以世界尺寸（格）创建控制器，网格覆盖整个世界。
    pub fn new(
        cfg: AutopilotConfig,
        width_cells: usize,
        height_cells: usize,
        start: (f32, f32, f32),
    ) -> Self {
        let zslice = 0; // 2D 平面，z 索引固定为 0
        let grid = OccupancyGrid3D::new(GridConfig::from_world_size(
            cfg.grid_res,
            width_cells as f32 * cfg.grid_res,
            height_cells as f32 * cfg.grid_res,
            cfg.grid_res,
        ));
        Self {
            sensor: RangeSensor::new(cfg.sensor_rays, cfg.sensor_range),
            explorer: Explorer::new(zslice),
            dwa: DwaPlanner::new(cfg.dwa),
            astar: AStar2D::new(zslice, 0.4),
            rrt: RrtPlanner::new(RrtConfig::default()),
            backtrack: Backtracker::new(4096, cfg.crumb_spacing),
            cfg,
            pose: start,
            local: grid,
            vel: (0.0, 0.0),
            goal: None,
            path: Vec::new(),
            path_idx: 0,
            stuck: 0,
            rewind: Vec::new(),
            blocked: 0,
            backtrack_events: 0,
            planned: 0,
            distance: 0.0,
            ts: 0,
        }
    }

    pub fn pose(&self) -> (f32, f32, f32) {
        self.pose
    }

    /// 只读访问局部占据网格（调试/可视化）。
    pub fn local_grid(&self) -> &OccupancyGrid3D {
        &self.local
    }

    /// 感知：测距扫描并更新局部占据网格。
    fn sense(&mut self, world: &World) {
        let (x, y, th) = self.pose;
        let scan = self.sensor.scan(world, x, y, th);
        let updater = RaycastUpdater::default();
        let origin = Vec3::new(x, y, 0.0);
        let range = self.cfg.sensor_range;
        // 命中点：占据该点，且之前的体素标为空闲。
        updater.update_point_cloud(&mut self.local, origin, &scan.hits, range);
        // 未见障碍的开放方向：把整束标为空闲（否则开放区会一直保持“未知”）。
        for p in &scan.free_ends {
            let dir = p.sub(origin).normalized();
            updater.update_ray(&mut self.local, origin, dir, range, None);
        }
    }

    /// 规划一条到目标的全局路径（绕开已知障碍）。
    /// 按 `cfg.use_rrt` 选择 RRT（连续空间）或 A*（栅格），输出世界系航点。
    fn replan(&mut self) {
        self.path.clear();
        self.path_idx = 0;
        let Some(g) = self.goal else { return };
        let start = (self.pose.0, self.pose.1);
        let goal = (g.x, g.y);

        let wp: Vec<Vec3> = if self.cfg.use_rrt {
            self.rrt
                .plan(&self.local, start, goal)
                .map(|p| p.into_iter().map(|(x, y)| Vec3::new(x, y, 0.0)).collect())
                .unwrap_or_default()
        } else {
            let Some(sc) = self.local.world_to_index(Vec3::new(start.0, start.1, 0.0)) else {
                return;
            };
            let Some(gc) = self.local.world_to_index(Vec3::new(goal.0, goal.1, 0.0)) else {
                return;
            };
            let sg = GridPoint { x: sc.x, y: sc.y };
            let gg = GridPoint { x: gc.x, y: gc.y };
            let mut out = Vec::new();
            if let Some(path) = self.astar.plan(&self.local, sg, gg) {
                for p in path {
                    let c = AStar2D::point_to_world(&self.local, p, 0);
                    out.push(Vec3::new(c.x, c.y, 0.0));
                }
            }
            out
        };

        if !wp.is_empty() {
            self.planned += 1;
            self.path = wp;
            self.path_idx = 0;
        }
    }

    /// 刷新规划：选新目标 / 推进航点 / 回溯。
    fn refresh_plan(&mut self) {
        if !self.rewind.is_empty() {
            self.path.clear();
            self.goal = self.rewind.pop();
            return;
        }
        let reached = match self.goal {
            Some(g) => {
                let (x, y, _) = self.pose;
                Vec3::new(g.x, g.y, 0.0).sub(Vec3::new(x, y, 0.0)).norm() < self.cfg.goal_tol
            }
            None => true,
        };
        if reached || self.goal.is_none() {
            self.goal = self
                .explorer
                .next_target(&self.local, Vec3::new(self.pose.0, self.pose.1, 0.0))
                .map(|t| t.position);
            self.replan();
            return;
        }
        // 推进已到达的航点（世界坐标）。
        while self.path_idx < self.path.len() {
            let (x, y, _) = self.pose;
            let wp = self.path[self.path_idx];
            if Vec3::new(wp.x, wp.y, 0.0).sub(Vec3::new(x, y, 0.0)).norm() < self.cfg.goal_tol {
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

    /// 当前应驶向的目标（全局路径下一航点，否则最终目标）。
    fn current_target(&self) -> Option<Vec3> {
        if self.path_idx < self.path.len() {
            Some(self.path[self.path_idx])
        } else {
            self.goal
        }
    }

    /// 执行一步。
    pub fn step_once(&mut self, world: &World) -> StepOutcome {
        self.ts += 1;
        self.sense(world);
        self.refresh_plan();

        let (x, y, th) = self.pose;
        let pos = Vec3::new(x, y, 0.0);

        let target = self.current_target();
        let cmd = match target {
            Some(g) => self.dwa.plan(&self.local, pos, th, g, self.vel),
            None => None, // 探索完成
        };

        let moved = match cmd {
            None => None,
            Some(v) => {
                if self.path_clear(world, x, y, th, v.linear_x, v.angular_z, self.cfg.dt) {
                    Some((v.linear_x, v.angular_z))
                } else {
                    self.blocked += 1;
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
                self.vel = (0.0, 0.0);
                self.stuck += 1;
                self.replan(); // 地图更新后重算绕行路径
                               // 长时间无法前进（卡死/被墙反复阻挡）→ 触发回溯。
                if self.stuck >= self.cfg.stuck_threshold && self.rewind.is_empty() {
                    self.rewind = self.backtrack.start_rewind();
                    if !self.rewind.is_empty() {
                        self.backtrack_events += 1;
                        return StepOutcome::Backtracking;
                    }
                }
                if self.goal.is_none() {
                    return StepOutcome::Done;
                }
                if cmd.is_some() {
                    StepOutcome::Blocked
                } else {
                    StepOutcome::Stuck
                }
            }
        }
    }

    fn integrate(&mut self, v: f32, w: f32) {
        let (x, y, th) = self.pose;
        let dt = self.cfg.dt;
        let nx = x + v * th.cos() * dt;
        let ny = y + v * th.sin() * dt;
        let nth = th + w * dt;
        self.distance += ((nx - x) * (nx - x) + (ny - y) * (ny - y)).sqrt();
        self.pose = (nx, ny, nth);
    }

    /// 判断命令轨迹是否与真值障碍相交。
    #[allow(clippy::too_many_arguments)] // 状态量 (x,y,th,v,w,dt)，语义清晰
    fn path_clear(&self, world: &World, x: f32, y: f32, th: f32, v: f32, w: f32, dt: f32) -> bool {
        let steps = 4;
        let mut cx = x;
        let mut cy = y;
        let mut cth = th;
        for _ in 0..steps {
            cth += w * dt / steps as f32;
            cx += v * cth.cos() * dt / steps as f32;
            cy += v * cth.sin() * dt / steps as f32;
            if world.is_obstacle_at(cx, cy) {
                return false;
            }
        }
        true
    }

    /// 跑完一次任务并返回统计。
    pub fn run(&mut self, world: &World, max_steps: usize) -> RunStats {
        for _ in 0..max_steps {
            let o = self.step_once(world);
            if o == StepOutcome::Done {
                break;
            }
        }
        let (free_total, explored) = self.exploration(world);
        let safe = !world.is_obstacle_at(self.pose.0, self.pose.1);
        RunStats {
            steps: self.ts as usize,
            blocked: self.blocked,
            backtrack_events: self.backtrack_events,
            planned: self.planned,
            explored_ratio: if free_total > 0 {
                explored as f32 / free_total as f32
            } else {
                1.0
            },
            distance: self.distance,
            safe,
            end_pose: self.pose,
        }
    }

    fn exploration(&self, world: &World) -> (usize, usize) {
        let mut free_total = 0;
        let mut explored = 0;
        for y in 0..world.height() as i32 {
            for x in 0..world.width() as i32 {
                if world.is_obstacle(x, y) {
                    continue;
                }
                free_total += 1;
                let wx = x as f32 * self.cfg.grid_res;
                let wy = y as f32 * self.cfg.grid_res;
                if self.local.is_free(Vec3::new(wx, wy, 0.0)) {
                    explored += 1;
                }
            }
        }
        (free_total, explored)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open_world() -> World {
        let mut w = World::new(20, 20);
        w.wall(5..6, 0..10);
        w.wall(0..20, 12..13);
        w
    }

    #[test]
    fn explores_open_world_safely() {
        let w = open_world();
        let mut ap = Autopilot::new(
            AutopilotConfig::default(),
            w.width(),
            w.height(),
            (1.0, 1.0, 0.0),
        );
        let stats = ap.run(&w, 600);
        assert!(
            stats.safe,
            "robot must end outside obstacles: {:?}",
            stats.end_pose
        );
        assert!(
            stats.explored_ratio > 0.08,
            "explored_ratio={}",
            stats.explored_ratio
        );
        assert!(
            stats.distance > 2.0,
            "should have moved, distance={}",
            stats.distance
        );
    }

    #[test]
    fn stays_safe_in_corridor_with_wall() {
        let mut w = World::new(20, 20);
        w.wall(10..20, 4..5);
        w.wall(10..20, 15..16);
        let mut ap = Autopilot::new(
            AutopilotConfig::default(),
            w.width(),
            w.height(),
            (1.0, 8.0, 0.0),
        );
        let stats = ap.run(&w, 400);
        assert!(stats.safe, "unsafe end pose {:?}", stats.end_pose);
        assert!(stats.distance > 1.0, "should have moved");
    }

    #[test]
    fn navigates_dead_end_safely() {
        let mut w = World::new(28, 28);
        w.wall(9..10, 0..28);
        w.wall(18..19, 0..28);
        w.wall(0..28, 24..25);
        let mut ap = Autopilot::new(
            AutopilotConfig::default(),
            w.width(),
            w.height(),
            (13.0, 2.0, 0.0),
        );
        let stats = ap.run(&w, 600);
        assert!(stats.safe, "unsafe end pose {:?}", stats.end_pose);
        assert!(
            stats.distance > 1.0,
            "should have moved, dist={}",
            stats.distance
        );
        assert!(
            stats.end_pose.1 < 21.0,
            "trapped deep in dead-end, y={}",
            stats.end_pose.1
        );
        assert!(
            stats.explored_ratio > 0.05,
            "ratio={}",
            stats.explored_ratio
        );
    }
    #[test]
    fn explores_with_rrt_guidance() {
        let w = open_world();
        let cfg = AutopilotConfig {
            use_rrt: true, // 用 RRT 做全局引导
            ..AutopilotConfig::default()
        };
        let mut ap = Autopilot::new(cfg, w.width(), w.height(), (1.0, 1.0, 0.0));
        let stats = ap.run(&w, 600);
        assert!(stats.safe, "unsafe end pose {:?}", stats.end_pose);
        assert!(
            stats.distance > 1.0,
            "should have moved, dist={}",
            stats.distance
        );
        assert!(stats.planned > 0, "RRT should have planned a global path");
    }
}
