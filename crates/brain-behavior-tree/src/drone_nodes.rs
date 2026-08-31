//! 飞行专用行为节点：起飞/巡航/发现目标/跟踪/返航/降落/fail-safe 等。

use brain_message::telemetry::FixType;
use brain_message::{CommandTarget, Mode};
use brain_middleware::bus::topics;

use super::core::{BehaviorContext, Node, Status};

/// 记录一条日志并返回 Success。用于观察树执行路径。
pub struct LogNode {
    message: String,
}

impl LogNode {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl Node for LogNode {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        log::info!("[tree] {}", self.message);
        ctx.out.note = self.message.clone();
        Status::Success
    }
}

/// 条件节点：GPS 是否具备 3D 定位。
pub struct GpsFixCheck {
    min_satellites: u8,
}

impl GpsFixCheck {
    pub fn new(min_satellites: u8) -> Self {
        Self { min_satellites }
    }
}

impl Node for GpsFixCheck {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        let telemetry = ctx
            .bus
            .topic::<brain_message::Telemetry>(topics::TELEMETRY)
            .and_then(|t| t.peek());
        match telemetry {
            Some(telem)
                if telem.gps.fix_type == FixType::Fix3D
                    && telem.gps.satellites >= self.min_satellites =>
            {
                Status::Success
            }
            Some(_) => {
                log::warn!("[condition] GPS fix insufficient");
                Status::Failure
            }
            None => Status::Running,
        }
    }
}

/// 行为节点：起飞。进入 Takeoff 模式并等待到达目标高度。
pub struct Takeoff {
    target_alt: f32,
}

impl Takeoff {
    pub fn new(target_alt: f32) -> Self {
        Self { target_alt }
    }
}

impl Node for Takeoff {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        ctx.out.mode = Mode::Takeoff;
        ctx.out.target = CommandTarget::Position {
            north: 0.0,
            east: 0.0,
            down: -self.target_alt,
        };
        ctx.out.note = format!("takeoff to {}m", self.target_alt);

        // 从遥测读取高度，到达目标则成功。
        let telemetry = ctx
            .bus
            .topic::<brain_message::Telemetry>(topics::TELEMETRY)
            .and_then(|t| t.peek());
        match telemetry {
            Some(t) if t.gps.alt >= self.target_alt - 0.5 => Status::Success,
            _ => Status::Running,
        }
    }
}

/// 行为节点：巡航。进入 Cruise 模式并沿航点推进。
pub struct Cruise {
    waypoints: Vec<CommandTarget>,
    idx: usize,
}

impl Cruise {
    pub fn new(waypoints: Vec<CommandTarget>) -> Self {
        Self { waypoints, idx: 0 }
    }
}

impl Node for Cruise {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        if self.idx >= self.waypoints.len() {
            return Status::Success; // 巡航完成
        }
        ctx.out.mode = Mode::Cruise;
        ctx.out.target = self.waypoints[self.idx].clone();
        ctx.out.note = format!("cruise waypoint {}", self.idx);

        // 简单推进：假定每个航点一拍完成（SITL 中由仿真推进）。
        self.idx += 1;
        if self.idx >= self.waypoints.len() {
            Status::Success
        } else {
            Status::Running
        }
    }

    fn reset(&mut self) {
        self.idx = 0;
    }
}

/// 条件节点：是否在感知总线上发现高置信目标。
pub struct DetectTarget {
    min_confidence: f32,
}

impl DetectTarget {
    pub fn new(min_confidence: f32) -> Self {
        Self { min_confidence }
    }
}

impl Node for DetectTarget {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        let det = ctx
            .bus
            .topic::<brain_message::Detection>(topics::DETECTIONS)
            .and_then(|t| t.peek());
        match det {
            Some(d) if d.confidence >= self.min_confidence => {
                log::info!("[condition] target detected conf={:.2}", d.confidence);
                Status::Success
            }
            Some(_) | None => Status::Failure,
        }
    }
}

/// 行为节点：跟踪目标。进入 Track 模式并把目标方位转为速度指令。
pub struct TrackTarget {
    gain: f32,
}

impl TrackTarget {
    pub fn new(gain: f32) -> Self {
        Self { gain }
    }
}

impl Node for TrackTarget {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        let det = ctx
            .bus
            .topic::<brain_message::Detection>(topics::DETECTIONS)
            .and_then(|t| t.peek());
        match det {
            Some(d) => {
                // 简单比例控制器：把方位误差映射到机体速度指令。
                let vx = self.gain * d.bearing_yaw;
                let vy = self.gain * d.bearing_pitch;
                ctx.out.mode = Mode::Track;
                ctx.out.target = CommandTarget::Velocity(brain_message::telemetry::Vec3 {
                    x: vx,
                    y: vy,
                    z: 0.0,
                });
                ctx.out.note = format!("tracking target range={:.1}m", d.range_m);
                Status::Success
            }
            None => {
                // 目标丢失：返回失败，让上层决定是否返航。
                ctx.out.mode = Mode::Loiter;
                ctx.out.target = CommandTarget::None;
                ctx.out.note = "target lost, loiter".into();
                Status::Failure
            }
        }
    }
}

/// 行为节点：返航。
pub struct ReturnHome {
    done: bool,
}

impl ReturnHome {
    pub fn new() -> Self {
        Self { done: false }
    }
}

impl Default for ReturnHome {
    fn default() -> Self {
        Self::new()
    }
}

impl Node for ReturnHome {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        ctx.out.mode = Mode::ReturnHome;
        ctx.out.target = CommandTarget::Position {
            north: 0.0,
            east: 0.0,
            down: 0.0,
        };
        ctx.out.note = "returning home".into();
        if self.done {
            Status::Success
        } else {
            self.done = true;
            Status::Running
        }
    }

    fn reset(&mut self) {
        self.done = false;
    }
}

/// 行为节点：降落。进入 Land 模式并等待高度归零。
pub struct Land {
    reached: bool,
}

impl Land {
    pub fn new() -> Self {
        Self { reached: false }
    }
}

impl Default for Land {
    fn default() -> Self {
        Self::new()
    }
}

impl Node for Land {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        ctx.out.mode = Mode::Land;
        ctx.out.target = CommandTarget::None;
        ctx.out.note = "landing".into();
        let telemetry = ctx
            .bus
            .topic::<brain_message::Telemetry>(topics::TELEMETRY)
            .and_then(|t| t.peek());
        match telemetry {
            Some(t) if t.gps.alt <= 0.5 => {
                self.reached = true;
                Status::Success
            }
            _ if self.reached => Status::Success,
            _ => Status::Running,
        }
    }

    fn reset(&mut self) {
        self.reached = false;
    }
}

/// 行为节点：电池低电量检查。
pub struct BatteryCheck {
    min_pct: f32,
}

impl BatteryCheck {
    pub fn new(min_pct: f32) -> Self {
        Self { min_pct }
    }
}

impl Node for BatteryCheck {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        let telemetry = ctx
            .bus
            .topic::<brain_message::Telemetry>(topics::TELEMETRY)
            .and_then(|t| t.peek());
        match telemetry {
            Some(t) if t.battery.remaining_pct >= self.min_pct => Status::Success,
            Some(t) => {
                log::warn!(
                    "[condition] battery low {:.1}% < {}",
                    t.battery.remaining_pct,
                    self.min_pct
                );
                Status::Failure
            }
            None => Status::Running,
        }
    }
}

/// 行为节点：Fail-safe。强制进入 Loiter（自动悬停）。
pub struct Failsafe {
    message: String,
}

impl Failsafe {
    pub fn new(message: impl Into<String>) -> Self {
        Self {
            message: message.into(),
        }
    }
}

impl Node for Failsafe {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        ctx.out.mode = Mode::Loiter;
        ctx.out.target = CommandTarget::None;
        ctx.out.note = format!("FAILSAFE: {}", self.message);
        Status::Success
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::BrainOutput;
    use brain_message::telemetry::FixType;
    use brain_message::{Detection, Telemetry};
    use brain_middleware::DataBus;

    fn telem(alt: f32, fix: FixType, sats: u8, batt: f32) -> Telemetry {
        let mut t = Telemetry::default_at(0);
        t.gps.alt = alt;
        t.gps.fix_type = fix;
        t.gps.satellites = sats;
        t.battery.remaining_pct = batt;
        t
    }

    fn det(conf: f32, yaw: f32, pitch: f32) -> Detection {
        Detection {
            class_id: 0,
            confidence: conf,
            bearing_yaw: yaw,
            bearing_pitch: pitch,
            range_m: 5.0,
        }
    }

    fn ctx<'a>(bus: &'a DataBus, out: &'a mut BrainOutput) -> BehaviorContext<'a> {
        BehaviorContext { bus, now: 0, out }
    }

    #[test]
    fn log_node_writes_note() {
        let mut node = LogNode::new("hello");
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
        assert_eq!(out.note, "hello");
    }

    #[test]
    fn gps_fix_check_success_failure_running() {
        let mut node = GpsFixCheck::new(6);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        // 无遥测 -> Running。
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
        // 3D fix 但卫星不足 -> Failure。
        bus.publish(topics::TELEMETRY, telem(30.0, FixType::Fix3D, 4, 100.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Failure);
        // 卫星足够 -> Success。
        bus.publish(topics::TELEMETRY, telem(30.0, FixType::Fix3D, 8, 100.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
    }

    #[test]
    fn takeoff_waits_for_target_altitude() {
        let mut node = Takeoff::new(30.0);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        // 无遥测 -> Running。
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
        assert_eq!(out.mode, Mode::Takeoff);
        // 高度未达 -> Running。
        bus.publish(topics::TELEMETRY, telem(10.0, FixType::Fix3D, 8, 100.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
        // 高度达到 -> Success。
        bus.publish(topics::TELEMETRY, telem(30.0, FixType::Fix3D, 8, 100.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
    }

    #[test]
    fn cruise_advances_through_waypoints_then_succeeds() {
        let wps = vec![
            CommandTarget::Position {
                north: 0.0,
                east: 0.0,
                down: -30.0,
            },
            CommandTarget::Position {
                north: 80.0,
                east: 80.0,
                down: -30.0,
            },
        ];
        let mut node = Cruise::new(wps.clone());
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        // 第一拍：航点 0，Running。
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
        assert_eq!(out.mode, Mode::Cruise);
        assert_eq!(out.target, wps[0]);
        // 第二拍：最后一个航点，Success。
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
        assert_eq!(out.target, wps[1]);
        // 复位后重新从头开始。
        node.reset();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
    }

    #[test]
    fn land_succeeds_at_ground() {
        let mut node = Land::new();
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        // 无遥测 -> Running。
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
        // 高空 -> Running。
        bus.publish(topics::TELEMETRY, telem(20.0, FixType::Fix3D, 8, 50.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
        // 落地 -> Success。
        bus.publish(topics::TELEMETRY, telem(0.2, FixType::Fix3D, 8, 50.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
        assert_eq!(out.mode, Mode::Land);
    }

    #[test]
    fn failsafe_forces_loiter() {
        let mut node = Failsafe::new("watchdog");
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
        assert_eq!(out.mode, Mode::Loiter);
        assert_eq!(out.target, CommandTarget::None);
        assert!(out.note.contains("watchdog"));
    }

    #[test]
    fn return_home_runs_then_succeeds() {
        let mut node = ReturnHome::new();
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
        assert_eq!(out.mode, Mode::ReturnHome);
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
    }

    #[test]
    fn detect_target_confidence_gate() {
        let mut node = DetectTarget::new(0.5);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        // 无检测 -> Failure。
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Failure);
        // 低置信 -> Failure。
        bus.publish(topics::DETECTIONS, det(0.3, 0.0, 0.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Failure);
        // 高置信 -> Success。
        bus.publish(topics::DETECTIONS, det(0.9, 0.2, 0.1), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
    }

    #[test]
    fn track_target_writes_velocity_or_loiters() {
        let mut node = TrackTarget::new(2.0);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        // 有目标 -> Track + 速度指令。
        bus.publish(topics::DETECTIONS, det(0.9, 0.2, -0.1), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
        assert_eq!(out.mode, Mode::Track);
        match out.target {
            CommandTarget::Velocity(v) => {
                assert!((v.x - 0.4).abs() < 1e-5);
                assert!((v.y + 0.2).abs() < 1e-5);
                assert_eq!(v.z, 0.0);
            }
            other => panic!("expected velocity target, got {other:?}"),
        }
        // 无目标 -> Failure + Loiter。
        let bus2 = DataBus::new();
        let mut out2 = BrainOutput::idle();
        assert_eq!(node.tick(&mut ctx(&bus2, &mut out2)), Status::Failure);
        assert_eq!(out2.mode, Mode::Loiter);
        assert_eq!(out2.target, CommandTarget::None);
    }

    #[test]
    fn battery_check_success_failure_running() {
        let mut node = BatteryCheck::new(30.0);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Running);
        bus.publish(topics::TELEMETRY, telem(30.0, FixType::Fix3D, 8, 25.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Failure);
        bus.publish(topics::TELEMETRY, telem(30.0, FixType::Fix3D, 8, 80.0), 0)
            .unwrap();
        assert_eq!(node.tick(&mut ctx(&bus, &mut out)), Status::Success);
    }
}
