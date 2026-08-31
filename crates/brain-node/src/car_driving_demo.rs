//! 汽车驾驶导航演示：阿克曼前轮转向车辆的闭环自主导航 + 具身抽象。
//!
//! 展示两件事：
//! 1. `CarAutopilot`：感知→建图→A*/RRT 全局引导→Ackermann DWA 局部避障
//!    →自行车模型积分→面包屑回溯，跑在一个 2D 道路/障碍世界里；
//! 2. `CarBody`：把同一辆汽车包装成 `RobotBody`，用统一接口驱动，
//!    与无人机/四足完全一致（验证“汽车只是众多身体之一”）。

use brain_autopilot::{CarAutopilot, CarAutopilotConfig, World};
use brain_core::Vec3;
use brain_kinematics::BicycleModel;
use brain_robot::car::CarBody;
use brain_robot::command::{EffectorCommand, LocomotionMode, Task, TaskTarget};
use brain_robot::RobotBody;

/// 汽车自主驾驶演示。
pub fn run() {
    println!("\n=== 汽车驾驶导航（阿克曼前轮转向 / 闭环自主驾驶）===");

    // ---- 场景：车道 y=20，两侧错开立柱，车需变道绕行后回到车道抵达终点 ----
    let mut world = World::new(60, 40);
    world.set_obstacle(25, 23);
    world.set_obstacle(26, 23);
    world.set_obstacle(40, 17);
    world.set_obstacle(41, 17);

    let mut cfg = CarAutopilotConfig::default();
    cfg.use_rrt = true; // 用 RRT 连续空间全局引导绕障
    cfg.use_dubins = true; // 用 Dubins 曲线把全局路径平滑成受最小转弯半径约束的圆弧轨迹
                           // 起点在车道 y=20，车头朝 +X；目标在 x=55 的同一车道。
    let start = (3.0f32, 20.0f32, 0.0f32);
    let mut autopilot = CarAutopilot::new(cfg, world.width(), world.height(), start);
    autopilot.set_goal(Vec3::new(55.0, 20.0, 0.0));

    // 同一辆车的“身体”（小脑侧），由大脑（autopilot）下发的 AckermannCommand 驱动。
    let model = BicycleModel::new(2.6, 0.6, 0.8);
    let mut body = CarBody::new(model, start.0, start.1, start.2);

    let mut done = false;
    let mut max_sync_err = 0.0f32;
    for _ in 0..1500 {
        if autopilot.step_once(&world) == brain_autopilot::StepOutcome::Done {
            done = true;
        }
        // 大脑 → 身体：把规划器命令喂给车身小脑。
        if let Some(cmd) = autopilot.current_command() {
            body.drive(cmd.speed, cmd.steering);
        }
        // 身体应紧跟大脑（同模型同 dt 积分）。
        let (bx, by, bth) = body.pose();
        let (ax, ay, ath) = autopilot.pose();
        max_sync_err =
            max_sync_err.max(((bx - ax).powi(2) + (by - ay).powi(2)).sqrt() + (bth - ath).abs());
        if done {
            break;
        }
    }
    let (ex, ey, _) = autopilot.pose();
    let (bx, by, _) = body.pose();

    println!(
        "  目标 (55,20)，{}，终点 ({:.1}, {:.1})，里程 {:.1}m",
        if done { "已到达" } else { "未到达" },
        ex,
        ey,
        autopilot.distance(),
    );
    println!("  大脑/身体同步：身体终点 ({bx:.1}, {by:.1})，最大偏差 {max_sync_err:.3}m");
    println!("  （全局引导：RRT 绕障 → Dubins 平滑成圆弧；局部避障：Ackermann DWA 采样速度+前轮转角，尊重最小转弯半径；纯追踪前瞻沿全局路径行驶）");

    // ---- 具身抽象：同一辆汽车作为 RobotBody 被统一接口驱动 ----
    demonstrate_car_body();

    // ---- Reeds-Shepp：可倒车的掉头 / 泊车轨迹 ----
    demonstrate_reeds_shepp();
}

/// Reeds-Shepp 演示：可前进/倒车的掉头与泊车轨迹（Dubins 做不到的短路径）。
fn demonstrate_reeds_shepp() {
    use brain_planning::{DubinsConfig, DubinsPlanner, ReedsSheppConfig, ReedsSheppPlanner};

    println!("  --- Reeds-Shepp 掉头/泊车规划（可倒车） ---");
    let rho = 3.8f32;
    let rs = ReedsSheppPlanner::new(ReedsSheppConfig {
        turning_radius: rho,
        ..ReedsSheppConfig::default()
    });
    let db = DubinsPlanner::new(DubinsConfig {
        turning_radius: rho,
        ..DubinsConfig::default()
    });

    // 原地 180° 掉头。
    let s = (0.0f32, 0.0f32, 0.0f32);
    let uturn = (0.0f32, 0.0f32, std::f32::consts::PI);
    let rs_p = rs.plan(s, uturn).expect("rs uturn");
    let db_len = db.plan(s, uturn).map(|p| p.length).unwrap_or(f32::MAX);
    println!(
        "  原地180°掉头：Reeds-Shepp 长度 {:.1}m（段 {:?}），Dubins {:.1}m",
        rs_p.length, rs_p.segments, db_len
    );

    // 侧方位泊车（终点在侧后方、朝向翻转）。
    let park = (4.0f32, -3.0f32, -std::f32::consts::FRAC_PI_2);
    let p_p = rs.plan(s, park).expect("rs park");
    let has_reverse = p_p.segments.iter().any(|(_, len)| *len < 0.0);
    let last = p_p.points.last().unwrap();
    println!(
        "  侧方泊车：长度 {:.1}m，含倒车段={}，终点 ({:.1},{:.1}) 朝向 {:.1}°",
        p_p.length,
        has_reverse,
        last.0,
        last.1,
        last.2.to_degrees()
    );

    // 闭环倒车跟随：CarAutopilot 在“最终接近”阶段规划 Reeds-Shepp 并实际倒车入位。
    demonstrate_reverse_parking_closed_loop();
}

/// 闭环演示：`CarAutopilot` 用 Reeds-Shepp 掉头/泊车轨迹并切换倒车跟随，实车执行到目标位姿。
fn demonstrate_reverse_parking_closed_loop() {
    use brain_autopilot::{CarAutopilot, CarAutopilotConfig, StepOutcome, World};

    println!("  --- 闭环倒车跟随（最终接近切 Reeds-Shepp） ---");
    let world = World::new(60, 40);
    let mut cfg = CarAutopilotConfig::default();
    cfg.use_reeds_shepp = true;
    let mut autopilot = CarAutopilot::new(cfg, world.width(), world.height(), (6.0, 20.0, 0.0));
    let goal = (10.0f32, 24.0f32, -std::f32::consts::FRAC_PI_2);
    autopilot.set_goal_pose(goal);

    let mut reversed = false;
    let mut done = false;
    for _ in 0..2500 {
        if autopilot.step_once(&world) == StepOutcome::Done {
            done = true;
            break;
        }
        if let Some(c) = autopilot.current_command() {
            if c.speed < 0.0 {
                reversed = true;
            }
        }
    }
    let (x, y, th) = autopilot.pose();
    println!(
        "  目标位姿 ({:.0},{:.0},-90°)，{}，终点 ({:.1},{:.1}) 朝向 {:.0}°，用倒车={}",
        goal.0,
        goal.1,
        if done { "已到达" } else { "未到达" },
        x,
        y,
        th.to_degrees(),
        reversed
    );
}

/// 用统一 `RobotBody` 接口驱动一辆汽车（与驱动无人机方式一致）。
fn demonstrate_car_body() {
    let model = BicycleModel::new(2.6, 0.6, 0.8);
    let mut car = CarBody::new(model, 0.0, 0.0, 0.0);

    println!("  --- RobotBody 具身抽象演示（CarBody） ---");
    println!("  kind = {:?}", car.kind());

    let nav = EffectorCommand {
        timestamp: 0,
        locomotion: LocomotionMode::Navigate,
        task: Task::NavigateTo(TaskTarget::Point(Vec3::new(30.0, 0.0, 0.0))),
    };
    car.send_command(&nav).expect("navigate");

    // 模拟车身推进若干步（小脑按速度/转向积分），再读取身体状态。
    for _ in 0..40 {
        car.step();
    }
    if let Ok(s) = car.read_state() {
        let (sx, sy) = (s.base.pose.position.x, s.base.pose.position.y);
        let steer = s
            .joints
            .iter()
            .find(|j| j.name == "steer_fl")
            .map(|j| j.position)
            .unwrap_or(0.0);
        println!(
            "  after NavigateTo -> base=({sx:.1}, {sy:.1}) 前轮转角={steer:.2}rad 轮接触={}",
            s.contacts.len()
        );
    }
    println!("  （四足/机械臂/人形/汽车只需各自实现 RobotBody，上层逻辑无需改动）");
}
