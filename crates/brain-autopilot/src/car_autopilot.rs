//! 汽车闭环自主导航控制器 —— 阿克曼前轮转向车辆的“感知→建图→规划→驱动→回溯”。
//!
//! 与 `autopilot`（面向无人机/差速底盘）不同，本控制器：
//! - 局部避障用 **Ackermann DWA**（采样 `(速度, 前轮转角)`，尊重最小转弯半径
//!   与转向速率，用车身矩形包络做碰撞检测）；
//! - 运动积分用 **自行车模型**（`brain-kinematics::BicycleModel`）；
//! - 全局引导用 A*/RRT 在占据网格上规划，供阿克曼约束的车辆跟踪。
//!
//! 单元测试验证：能安全移动、能绕障、能建图、死胡同可退出。

use brain_core::Vec3;
use brain_kinematics::{norm_angle, BicycleModel, BicycleState};
use brain_mapping::{GridConfig, OccupancyGrid3D, RaycastUpdater};
use brain_nav::{Backtracker, Explorer};
use brain_planning::{
    AStar2D, AckermannCommand, AckermannDwaConfig, AckermannDwaPlanner, DubinsConfig,
    DubinsPlanner, GridPoint, ReedsSheppConfig, ReedsSheppPath, ReedsSheppPlanner, RrtConfig,
    RrtPlanner,
};

use crate::sensor::RangeSensor;
use crate::world::World;
use crate::StepOutcome;

/// 一次汽车导航运行的统计。
#[derive(Debug, Clone)]
pub struct CarRunStats {
    pub steps: usize,
    /// 被障碍阻断的次数。
    pub blocked: usize,
    /// 回溯触发次数。
    pub backtrack_events: usize,
    /// 全局路径成功规划次数。
    pub planned: usize,
    /// 已探索空闲面积比例 0..1。
    pub explored_ratio: f32,
    /// 移动总距离。
    pub distance: f32,
    /// 结束是否安全（车身不落入障碍内）。
    pub safe: bool,
    pub end_pose: (f32, f32, f32),
}

/// 汽车导航控制器配置。
#[derive(Debug, Clone)]
pub struct CarAutopilotConfig {
    /// 占据网格分辨率（米/格）。
    pub grid_res: f32,
    /// 传感器光线数。
    pub sensor_rays: usize,
    /// 传感器量程（格）。
    pub sensor_range: f32,
    /// 阿克曼局部避障配置（含车辆运动学）。
    pub ackermann: AckermannDwaConfig,
    /// 运动积分步长（秒）。
    pub dt: f32,
    /// 连续停顿多少步触发回溯。
    pub stuck_threshold: usize,
    /// 面包屑间距（米）。
    pub crumb_spacing: f32,
    /// 到达目标容差（米）。
    pub goal_tol: f32,
    /// 纯追踪前瞻距离（米）：沿全局路径选取前方约该距离的航点作为局部目标，
    /// 避免追逐已越过的航点导致非完整约束车辆“过冲后卡住”。
    pub lookahead: f32,
    /// 全局规划器：`true` 用 RRT，`false` 用 A*。
    pub use_rrt: bool,
    /// 是否用 Dubins 曲线平滑全局路径（把折线航点变成受最小转弯半径约束的圆弧轨迹）。
    pub use_dubins: bool,
    /// 是否在“最终接近”阶段用 Reeds-Shepp 规划可倒车的掉头/泊车轨迹并切换倒车跟随。
    pub use_reeds_shepp: bool,
    /// Reeds-Shepp 最终接近的触发半径（米）：距目标该距离内开始规划掉头/泊车轨迹。
    pub final_radius: f32,
}

impl Default for CarAutopilotConfig {
    fn default() -> Self {
        Self {
            grid_res: 1.0,
            sensor_rays: 32,
            sensor_range: 10.0,
            ackermann: AckermannDwaConfig::default(),
            dt: 0.1,
            stuck_threshold: 10,
            crumb_spacing: 0.8,
            goal_tol: 0.8,
            lookahead: 6.0,
            use_rrt: false,
            use_dubins: false,
            use_reeds_shepp: false,
            final_radius: 8.0,
        }
    }
}

/// 汽车自主导航控制器。
pub struct CarAutopilot {
    cfg: CarAutopilotConfig,
    model: BicycleModel,
    st: BicycleState,
    local: OccupancyGrid3D,
    sensor: RangeSensor,
    explorer: Explorer,
    dwa: AckermannDwaPlanner,
    astar: AStar2D,
    rrt: RrtPlanner,
    backtrack: Backtracker,
    goal: Option<Vec3>,
    /// 固定目标（点对点导航时设置；为 None 则用探索器自动找目标）。
    fixed_goal: Option<Vec3>,
    /// 期望的最终位姿（含朝向），用于 Reeds-Shepp 掉头/泊车最终接近。
    final_pose: Option<(f32, f32, f32)>,
    /// Reeds-Shepp 最终接近轨迹。
    rs_path: Option<ReedsSheppPath>,
    /// 当前所处的 Reeds-Shepp 段索引。
    rs_seg: usize,
    /// 当前段内已行进距离（米）。
    rs_progress: f32,
    path: Vec<Vec3>,
    path_idx: usize,
    stuck: usize,
    rewind: Vec<Vec3>,
    blocked: usize,
    backtrack_events: usize,
    planned: usize,
    distance: f32,
    ts: u64,
    /// 最近一步实际执行的 `(速度, 前轮转角)` 指令（供驱动真实身体）。
    last_cmd: Option<AckermannCommand>,
}

impl CarAutopilot {
    /// 以世界尺寸（格）创建控制器，起点为后轴中心 `(x, y, theta)`。
    pub fn new(
        cfg: CarAutopilotConfig,
        width_cells: usize,
        height_cells: usize,
        start: (f32, f32, f32),
    ) -> Self {
        let zslice = 0; // 2D 平面。
        let grid = OccupancyGrid3D::new(GridConfig::from_world_size(
            cfg.grid_res,
            width_cells as f32 * cfg.grid_res,
            height_cells as f32 * cfg.grid_res,
            cfg.grid_res,
        ));
        Self {
            model: cfg.ackermann.model,
            sensor: RangeSensor::new(cfg.sensor_rays, cfg.sensor_range),
            explorer: Explorer::new(zslice),
            dwa: AckermannDwaPlanner::new(cfg.ackermann),
            astar: AStar2D::new(zslice, 0.5),
            rrt: RrtPlanner::new(RrtConfig::default()),
            backtrack: Backtracker::new(4096, cfg.crumb_spacing),
            st: BicycleState::new(start.0, start.1, start.2),
            cfg,
            local: grid,
            goal: None,
            fixed_goal: None,
            final_pose: None,
            rs_path: None,
            rs_seg: 0,
            rs_progress: 0.0,
            path: Vec::new(),
            path_idx: 0,
            stuck: 0,
            rewind: Vec::new(),
            blocked: 0,
            backtrack_events: 0,
            planned: 0,
            distance: 0.0,
            ts: 0,
            last_cmd: None,
        }
    }

    /// 当前位姿 `(x, y, theta)`。
    pub fn pose(&self) -> (f32, f32, f32) {
        (self.st.x, self.st.y, self.st.theta)
    }

    /// 最近一步实际执行的底层指令 `(速度, 前轮转角)`，可喂给 `CarBody::drive` 驱动身体。
    pub fn current_command(&self) -> Option<AckermannCommand> {
        self.last_cmd
    }

    /// 只读访问局部占据网格。
    pub fn local_grid(&self) -> &OccupancyGrid3D {
        &self.local
    }

    /// 累计行驶里程（米）。
    pub fn distance(&self) -> f32 {
        self.distance
    }

    /// 点对点导航：设定一个固定目的地，并沿 A*/RRT 全局路径驶向它。
    /// 到达后 `step_once` 会返回 `Done`。
    pub fn set_goal(&mut self, goal: Vec3) {
        self.fixed_goal = Some(goal);
        self.goal = Some(goal);
        self.final_pose = None;
        self.rs_path = None;
        self.replan();
    }

    /// 点对点导航（带期望朝向）：设定目的地与最终朝向。
    /// 启用 `use_reeds_shepp` 时，会在距目标 `final_radius` 内规划 Reeds-Shepp
    /// 掉头/泊车轨迹并**切换倒车跟随**，以任意朝向抵达。
    pub fn set_goal_pose(&mut self, goal: (f32, f32, f32)) {
        let pos = Vec3::new(goal.0, goal.1, 0.0);
        self.fixed_goal = Some(pos);
        self.goal = Some(pos);
        self.final_pose = Some(goal);
        self.rs_path = None;
        self.rs_seg = 0;
        self.rs_progress = 0.0;
        self.replan();
    }

    /// 感知：测距扫描并更新局部占据网格。
    fn sense(&mut self, world: &World) {
        let (x, y, th) = self.pose();
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

    /// 规划一条到目标的全局路径（绕开已知障碍）。
    fn replan(&mut self) {
        self.path.clear();
        self.path_idx = 0;
        let Some(g) = self.goal else { return };
        let start = (self.st.x, self.st.y);
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
            let path = if self.cfg.use_dubins {
                self.smooth_path(wp)
            } else {
                wp
            };
            self.path = path;
            self.path_idx = 0;
        }
    }

    /// 用 Dubins 曲线把折线航点平滑成受最小转弯半径约束的连续圆弧轨迹。
    /// 相邻航点间的朝向取沿路径的切线方向；Dubins 失败时回退为直线段。
    fn smooth_path(&self, waypoints: Vec<Vec3>) -> Vec<Vec3> {
        if waypoints.len() < 2 {
            return waypoints;
        }
        let rho = (self.cfg.ackermann.model.min_turning_radius() * 1.3).max(2.0);
        let planner = DubinsPlanner::new(DubinsConfig {
            turning_radius: rho,
            ..DubinsConfig::default()
        });
        let mut out: Vec<Vec3> = Vec::new();
        let mut prev_heading = self.st.theta;
        for i in 0..waypoints.len() - 1 {
            let a = waypoints[i];
            let b = waypoints[i + 1];
            let seg_angle = (b.y - a.y).atan2(b.x - a.x);
            match planner.plan((a.x, a.y, prev_heading), (b.x, b.y, seg_angle)) {
                Some(dp) => {
                    for (k, pt) in dp.points.iter().enumerate() {
                        // 第一个点通常是段起点 a，跳过以免重复。
                        if i == 0 || k > 0 {
                            out.push(Vec3::new(pt.0, pt.1, 0.0));
                        }
                    }
                    if let Some(last) = dp.points.last() {
                        prev_heading = last.2;
                    }
                }
                None => {
                    if i == 0 {
                        out.push(a);
                    }
                    prev_heading = seg_angle;
                }
            }
        }
        // 确保终点在路径末尾。
        let end = waypoints[waypoints.len() - 1];
        if out
            .last()
            .map(|p| (p.x - end.x).abs() > 1e-3 || (p.y - end.y).abs() > 1e-3)
            .unwrap_or(true)
        {
            out.push(end);
        }
        out
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
                Vec3::new(g.x, g.y, 0.0)
                    .sub(Vec3::new(self.st.x, self.st.y, 0.0))
                    .norm()
                    < self.cfg.goal_tol
            }
            None => true,
        };
        if reached || self.goal.is_none() {
            if self.fixed_goal.is_some() {
                // 点对点模式：已到达固定目标 → 清空路径并结束导航。
                self.goal = None;
                self.path.clear();
                self.path_idx = 0;
                return;
            }
            self.goal = self
                .explorer
                .next_target(&self.local, Vec3::new(self.st.x, self.st.y, 0.0))
                .map(|t| t.position);
            self.replan();
            return;
        }
        // 推进航点：已到达或已落在车后方的航点直接跳过（纯追踪式）。
        let (x, y, th) = self.pose();
        let (hx, hy) = (th.cos(), th.sin());
        while self.path_idx < self.path.len() {
            let wp = self.path[self.path_idx];
            let dx = wp.x - x;
            let dy = wp.y - y;
            let dist = (dx * dx + dy * dy).sqrt();
            let ahead = dx * hx + dy * hy; // 沿车头方向的投影
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

    /// 当前应驶向的目标：沿全局路径取前方约 `lookahead` 米的航点（纯追踪），
    /// 若无则取路径末点，最后回退到最终目标。
    fn current_target(&self) -> Option<Vec3> {
        let mut target = self.goal;
        let mut found = false;
        for wp in self.path.iter().skip(self.path_idx) {
            let d = Vec3::new(wp.x, wp.y, 0.0)
                .sub(Vec3::new(self.st.x, self.st.y, 0.0))
                .norm();
            if d >= self.cfg.lookahead {
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

    /// 是否处于“最终接近”阶段（距目标足够近，且设定了期望朝向）。
    fn in_final_approach(&self) -> bool {
        if !self.cfg.use_reeds_shepp {
            return false;
        }
        let Some(fp) = self.final_pose else {
            return false;
        };
        let d = ((self.st.x - fp.0).powi(2) + (self.st.y - fp.1).powi(2)).sqrt();
        d < self.cfg.final_radius
    }

    /// 从当前位姿规划 Reeds-Shepp 最终接近轨迹（可倒车），并重置进度索引。
    fn plan_final_rs(&mut self) {
        let Some(fp) = self.final_pose else { return };
        let rho = self.cfg.ackermann.model.min_turning_radius() * 1.1;
        let planner = ReedsSheppPlanner::new(ReedsSheppConfig {
            turning_radius: rho,
            step: 0.4,
            ..ReedsSheppConfig::default()
        });
        if let Some(path) = planner.plan(self.pose(), fp) {
            self.rs_path = Some(path);
            self.rs_seg = 0;
            self.rs_progress = 0.0;
        }
    }

    /// 车到 Reeds-Shepp 路径的最近距离（用于检测漂移后重规划）。
    fn rs_deviation(&self) -> f32 {
        let Some(rs) = &self.rs_path else { return 0.0 };
        let mut best = f32::MAX;
        for p in &rs.points {
            let d = ((p.0 - self.st.x).powi(2) + (p.1 - self.st.y).powi(2)).sqrt();
            if d < best {
                best = d;
            }
        }
        best
    }

    /// Reeds-Shepp 段跟随：按当前段发出 `(速度, 前轮转角)`。
    /// 段长符号决定前进/倒车；转向按段类型（L 左满舵 / R 右满舵 / S 直行）。
    fn step_final_rs(&mut self) -> Option<AckermannCommand> {
        let rs = self.rs_path.as_ref()?;
        if self.rs_seg >= rs.segments.len() {
            return Some(AckermannCommand {
                speed: 0.0,
                steering: 0.0,
            });
        }
        let (mode, len) = rs.segments[self.rs_seg];
        let dir = len.signum();
        let abs_len = len.abs();
        // 近段末减速，避免过冲。
        let remaining = (abs_len - self.rs_progress).max(0.0);
        let base = 0.9f32;
        let speed = dir * base * (remaining / 1.2).clamp(0.12, 1.0);
        let max_steer = self.cfg.ackermann.model.max_steering;
        let steering = match mode {
            'L' => max_steer,
            'R' => -max_steer,
            _ => 0.0,
        };
        Some(AckermannCommand { speed, steering })
    }

    /// 按本步实际位移推进 Reeds-Shepp 段内进度（越过段长则进入下一段）。
    fn advance_rs(&mut self, displacement: f32) {
        let Some(rs) = &self.rs_path else { return };
        let segs: Vec<(char, f32)> = rs.segments.clone();
        let mut d = displacement;
        while self.rs_seg < segs.len() {
            let len = segs[self.rs_seg].1.abs();
            if self.rs_progress + d >= len {
                d -= len - self.rs_progress;
                self.rs_progress = 0.0;
                self.rs_seg += 1;
            } else {
                self.rs_progress += d;
                break;
            }
        }
    }

    /// Reeds-Shepp 最终接近是否完成（抵达目标位姿）。
    fn rs_final_reached(&self) -> bool {
        let Some(fp) = self.final_pose else {
            return false;
        };
        let d = ((self.st.x - fp.0).powi(2) + (self.st.y - fp.1).powi(2)).sqrt();
        let th_err = norm_angle(self.st.theta - fp.2).abs();
        d < self.cfg.goal_tol && th_err < 0.25
    }

    /// 判断命令轨迹（速度+转角，dt 内）是否与真值障碍相交（车身包络）。
    fn path_clear(&self, world: &World, v: f32, steer: f32, dt: f32) -> bool {
        let mut st = self.st;
        let steps = 4;
        let sub_dt = dt / steps as f32;
        for _ in 0..steps {
            st = self.model.step(&st, v, steer, sub_dt);
            if !self.footprint_clear_world(world, st.x, st.y, st.theta) {
                return false;
            }
        }
        true
    }

    /// 车身矩形包络是否与真值障碍相交。
    fn footprint_clear_world(&self, world: &World, x: f32, y: f32, theta: f32) -> bool {
        let hl = self.cfg.ackermann.vehicle_length / 2.0;
        let hw = self.cfg.ackermann.vehicle_width / 2.0;
        let cos = theta.cos();
        let sin = theta.sin();
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
            if world.is_obstacle_at(wx, wy) {
                return false;
            }
        }
        true
    }

    /// 执行一步。
    pub fn step_once(&mut self, world: &World) -> StepOutcome {
        self.ts += 1;
        self.sense(world);
        // 非完整约束车辆必须“预判”：地图一旦更新就重算全局路径，
        // 以便在障碍尚远时就开始转向，避免到跟前才发觉无法转过去。
        if self.goal.is_some() {
            self.replan();
        }
        self.refresh_plan();

        let pos = Vec3::new(self.st.x, self.st.y, 0.0);
        // Reeds-Shepp 最终接近：进入掉头/泊车阶段则切换到可倒车的局部目标。
        let final_mode = self.in_final_approach();
        let path_done = self
            .rs_path
            .as_ref()
            .map(|rs| self.rs_seg >= rs.segments.len())
            .unwrap_or(true);
        if final_mode && (self.rs_path.is_none() || self.rs_deviation() > 1.5 || path_done) {
            self.plan_final_rs();
        }
        // Reeds-Shepp 最终接近完成：抵达目标位姿 → 结束导航。
        if final_mode && self.rs_final_reached() {
            self.goal = None;
            self.rs_path = None;
            return StepOutcome::Done;
        }
        let cmd = if final_mode {
            if self.rs_path.is_some() {
                self.step_final_rs()
            } else {
                None
            }
        } else {
            let target = self.current_target();
            match target {
                Some(g) => self.dwa.plan(
                    &self.local,
                    pos,
                    self.st.theta,
                    g,
                    (self.st.speed, self.st.steering),
                ),
                None => None,
            }
        };

        let moved = match cmd {
            None => None,
            Some(c) => {
                if self.path_clear(world, c.speed, c.steering, self.cfg.dt) {
                    Some((c.speed, c.steering))
                } else {
                    self.blocked += 1;
                    None
                }
            }
        };
        self.last_cmd = moved.map(|(v, steer)| AckermannCommand {
            speed: v,
            steering: steer,
        });

        match moved {
            Some((v, steer)) => {
                self.stuck = 0;
                let (bx, by) = (self.st.x, self.st.y);
                self.drive(v, steer);
                // Reeds-Shepp 段跟随：按实际位移推进段进度。
                if final_mode {
                    let disp = ((self.st.x - bx).powi(2) + (self.st.y - by).powi(2)).sqrt();
                    self.advance_rs(disp);
                }
                self.backtrack
                    .record(self.ts, Vec3::new(self.st.x, self.st.y, 0.0));
                StepOutcome::Moving
            }
            None => {
                self.stuck += 1;
                self.replan();
                if self.stuck >= self.cfg.stuck_threshold && self.rewind.is_empty() {
                    self.rewind = self.backtrack.start_rewind();
                    if !self.rewind.is_empty() {
                        self.backtrack_events += 1;
                        return StepOutcome::Backtracking;
                    }
                }
                if self.goal.is_none() {
                    StepOutcome::Done
                } else if cmd.is_some() {
                    StepOutcome::Blocked
                } else {
                    StepOutcome::Stuck
                }
            }
        }
    }

    /// 自行车模型积分一步并累计里程。
    fn drive(&mut self, v: f32, steer: f32) {
        let (x0, y0) = (self.st.x, self.st.y);
        self.st = self.model.step(&self.st, v, steer, self.cfg.dt);
        self.distance += ((self.st.x - x0).powi(2) + (self.st.y - y0).powi(2)).sqrt();
    }

    /// 跑完一次任务并返回统计。
    pub fn run(&mut self, world: &World, max_steps: usize) -> CarRunStats {
        for _ in 0..max_steps {
            let o = self.step_once(world);
            if o == StepOutcome::Done {
                break;
            }
        }
        let (free_total, explored) = self.exploration(world);
        let safe = self.footprint_clear_world(world, self.st.x, self.st.y, self.st.theta);
        CarRunStats {
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
            end_pose: self.pose(),
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
        let mut w = World::new(40, 40);
        w.wall(10..11, 0..40); // 一条竖直墙，验证能绕开。
        w
    }

    #[test]
    fn car_explores_open_world_safely() {
        let w = open_world();
        let mut car = CarAutopilot::new(
            CarAutopilotConfig::default(),
            w.width(),
            w.height(),
            (4.0, 20.0, 0.0),
        );
        let stats = car.run(&w, 700);
        assert!(
            stats.safe,
            "car must end outside obstacles: {:?}",
            stats.end_pose
        );
        assert!(
            stats.distance > 2.0,
            "car should move, distance={}",
            stats.distance
        );
        assert!(
            stats.explored_ratio > 0.05,
            "ratio={}",
            stats.explored_ratio
        );
    }

    #[test]
    fn car_avoids_wall_and_stays_safe() {
        let mut w = World::new(50, 40);
        // 一堵横贯上下的墙，留出上下通道。
        w.wall(25..27, 8..32);
        let mut car = CarAutopilot::new(
            CarAutopilotConfig::default(),
            w.width(),
            w.height(),
            (5.0, 20.0, 0.0),
        );
        let stats = car.run(&w, 900);
        assert!(stats.safe, "unsafe end pose {:?}", stats.end_pose);
        assert!(
            stats.distance > 1.0,
            "should have moved, dist={}",
            stats.distance
        );
    }

    #[test]
    fn car_exits_dead_end_safely() {
        let mut w = World::new(50, 50);
        w.wall(20..21, 0..50);
        w.wall(38..39, 0..50);
        w.wall(0..50, 46..47);
        let mut car = CarAutopilot::new(
            CarAutopilotConfig::default(),
            w.width(),
            w.height(),
            (30.0, 4.0, 0.0),
        );
        let stats = car.run(&w, 1000);
        assert!(stats.safe, "unsafe end pose {:?}", stats.end_pose);
        assert!(
            stats.distance > 1.0,
            "should have moved, dist={}",
            stats.distance
        );
        assert!(
            stats.end_pose.1 < 40.0,
            "trapped deep in dead-end, y={}",
            stats.end_pose.1
        );
    }

    #[test]
    fn car_navigates_point_to_point_around_obstacles() {
        // 车道 y=20，两侧错开立柱，车需变道绕行后回到车道并抵达目标。
        let mut w = World::new(60, 40);
        w.set_obstacle(25, 23);
        w.set_obstacle(26, 23);
        w.set_obstacle(40, 17);
        w.set_obstacle(41, 17);
        let mut cfg = CarAutopilotConfig::default();
        cfg.use_rrt = true;
        let mut car = CarAutopilot::new(cfg, w.width(), w.height(), (3.0, 20.0, 0.0));
        car.set_goal(Vec3::new(55.0, 20.0, 0.0));
        let mut done = false;
        for _ in 0..1200 {
            if car.step_once(&w) == StepOutcome::Done {
                done = true;
                break;
            }
        }
        assert!(done, "car should reach the goal, ended at {:?}", car.pose());
        // 抵达目标（goal_tol 内），且全程安全。
        let gd = (Vec3::new(car.pose().0 - 55.0, car.pose().1 - 20.0, 0.0)).norm();
        assert!(gd <= 0.8, "car did not reach goal, gd={gd}");
        assert!(
            car.distance > 40.0,
            "car should have driven a long path, dist={}",
            car.distance
        );
    }

    #[test]
    fn car_reaches_goal_in_open_lane() {
        // 无障碍直道：车应沿 +X 加速，目标前减速并抵达前方目标。
        let w = World::new(80, 20);
        let mut car = CarAutopilot::new(
            CarAutopilotConfig::default(),
            w.width(),
            w.height(),
            (3.0, 10.0, 0.0),
        );
        car.set_goal(Vec3::new(45.0, 10.0, 0.0));
        let mut done = false;
        for _ in 0..3000 {
            if car.step_once(&w) == StepOutcome::Done {
                done = true;
                break;
            }
        }
        assert!(
            done,
            "car should reach goal in open lane, ended at {:?}",
            car.pose()
        );
    }

    #[test]
    fn car_navigates_with_dubins_smoothing() {
        // 启用 Dubins 平滑后，全局路径由圆弧/直线组成，车仍应绕障抵达目标。
        let mut w = World::new(60, 40);
        w.set_obstacle(25, 23);
        w.set_obstacle(26, 23);
        w.set_obstacle(40, 17);
        w.set_obstacle(41, 17);
        let mut cfg = CarAutopilotConfig::default();
        cfg.use_rrt = true;
        cfg.use_dubins = true;
        let mut car = CarAutopilot::new(cfg, w.width(), w.height(), (3.0, 20.0, 0.0));
        car.set_goal(Vec3::new(55.0, 20.0, 0.0));
        let mut done = false;
        for _ in 0..1500 {
            if car.step_once(&w) == StepOutcome::Done {
                done = true;
                break;
            }
        }
        assert!(
            done,
            "dubins-smoothed car should reach goal, ended at {:?}",
            car.pose()
        );
        assert!(
            car.distance() > 40.0,
            "should have driven a long path, dist={}",
            car.distance()
        );
    }

    #[test]
    fn car_commands_respect_physical_limits() {
        // 全程不变量：每一步下发的速度/前轮转角都在车辆物理极限内。
        let mut w = World::new(60, 40);
        w.set_obstacle(25, 23);
        w.set_obstacle(26, 23);
        w.set_obstacle(40, 17);
        w.set_obstacle(41, 17);
        let mut cfg = CarAutopilotConfig::default();
        cfg.use_rrt = true;
        let mut car = CarAutopilot::new(cfg, w.width(), w.height(), (3.0, 20.0, 0.0));
        car.set_goal(Vec3::new(55.0, 20.0, 0.0));
        let v_max = car.cfg.ackermann.v_max;
        let v_rev = car.cfg.ackermann.v_reverse;
        let s_max = car.cfg.ackermann.model.max_steering;
        let mut n = 0;
        for _ in 0..900 {
            if car.step_once(&w) == StepOutcome::Done {
                break;
            }
            if let Some(cmd) = car.current_command() {
                assert!(
                    cmd.speed >= v_rev - 0.01 && cmd.speed <= v_max + 0.01,
                    "speed {:.3} out of [{v_rev},{v_max}]",
                    cmd.speed
                );
                assert!(
                    cmd.steering.abs() <= s_max + 0.01,
                    "steering {:.3} exceeds {s_max}",
                    cmd.steering
                );
                n += 1;
            }
        }
        assert!(n > 100, "should have produced many commands, got {n}");
    }

    #[test]
    fn car_reverse_parks_with_reeds_shepp() {
        // 开放式场景：车需沿 Reeds-Shepp 轨迹驶入一个要求倒车/调向的泊车位，
        // 验证“最终接近阶段切换倒车跟随”确实能用倒车抵达目标位姿。
        let w = World::new(60, 40);
        let mut cfg = CarAutopilotConfig::default();
        cfg.use_reeds_shepp = true;
        cfg.use_rrt = true;
        let mut car = CarAutopilot::new(cfg, w.width(), w.height(), (6.0, 20.0, 0.0));
        // 目标位姿在右上方、朝向 -90°（需要前进再倒车入位）。
        let goal = (10.0f32, 24.0f32, -std::f32::consts::FRAC_PI_2);
        car.set_goal_pose(goal);
        let mut reversed = false;
        let mut done = false;
        for _ in 0..2500 {
            if car.step_once(&w) == StepOutcome::Done {
                done = true;
                break;
            }
            if let Some(c) = car.current_command() {
                if c.speed < 0.0 {
                    reversed = true;
                }
            }
        }
        assert!(
            done,
            "car should reverse-park to {goal:?}, ended at {:?}",
            car.pose()
        );
        assert!(reversed, "parking should have used reverse at some point");
        let (x, y, th) = car.pose();
        assert!(
            ((x - goal.0).powi(2) + (y - goal.1).powi(2)).sqrt() < 0.8,
            "pos err"
        );
        assert!(
            norm_angle(th - goal.2).abs() < 0.25,
            "heading err={}",
            norm_angle(th - goal.2)
        );
    }
}
