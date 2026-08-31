//! 水面自动驾驶艇（ASV/USV）演示：双差速推进闭环导航 + 水流漂移 + 定泊保持。
//!
//! 展示：
//! 1. `BoatAutopilot`：差分 DWA 导航 + 水流漂移，逆流定泊（动力定位）；
//! 2. `BoatBody`：把同一艘艇包装成 `RobotBody`，用统一接口驱动（与无人机/汽车一致）。

use brain_autopilot::{BoatAutopilot, BoatConfig, StepOutcome, World};
use brain_core::Vec3;
use brain_robot::boat::BoatBody;
use brain_robot::command::{EffectorCommand, LocomotionMode, Task, TaskTarget};
use brain_robot::RobotBody;

/// 水面艇演示。
pub fn run() {
    println!("\n=== 水面自动驾驶艇（ASV/USV / 双差速推进）===");

    // ---- 场景：有水流的水域，艇需到点并逆流定泊 ----
    let mut world = World::new(80, 40);
    world.wall(55..57, 5..20); // 一处水中障碍（浅滩/浮标区），需绕行
    world.wall(20..22, 26..38);

    let cfg = BoatConfig {
        current: (0.5, 0.0), // 正 x 方向水流（会顺流/逆流，考验定泊）
        ..BoatConfig::default()
    };
    let mut boat = BoatAutopilot::new(cfg, world.width(), world.height(), (4.0, 10.0, 0.0));
    boat.set_goal(Vec3::new(40.0, 10.0, 0.0));

    // 顺流航行 + 绕过障碍 + 抵达目标后进入定泊。
    let mut arrived = false;
    let mut holding = false;
    for _ in 0..2000 {
        let o = boat.step_once(&world);
        if o == StepOutcome::Done {
            arrived = true;
            break;
        }
        if boat.is_station_keeping() {
            holding = true;
            break;
        }
    }
    let (x, y, _) = boat.pose();
    println!(
        "  目标 (40,10)：{}，终点 ({:.1},{:.1})，航程 {:.1}m，定泊保持={}",
        if arrived || holding {
            "已到达"
        } else {
            "未到达"
        },
        x,
        y,
        boat.distance(),
        holding
    );

    // 定泊稳定后打印维持误差（逆流顶住，未漂走）。
    let gx = 40.0f32;
    let gy = 10.0f32;
    // 先让定泊控制器收敛到目标附近，再测量漂移。
    for _ in 0..400 {
        boat.step_once(&world);
    }
    let mut drift = 0.0f32;
    for _ in 0..300 {
        boat.step_once(&world);
        let (px, py, _) = boat.pose();
        drift = drift.max(((px - gx).powi(2) + (py - gy).powi(2)).sqrt());
    }
    let (px, py, _) = boat.pose();
    println!("  逆流定泊收敛后终点 ({px:.1},{py:.1})，300 步最大漂移 {drift:.2}m");
    println!("  （导航：差分 DWA 局部避障 + 全局引导；水流：恒定向量 + 潮汐；定泊：位置 P 控制 + 水流前馈）");

    // ---- 多点巡航：沿一串航点依次行驶 ----
    demonstrate_track();

    // ---- COLREGS 会遇避让 ----
    demonstrate_colregs();

    // ---- 具身抽象：同一艘艇作为 RobotBody 被统一接口驱动 ----
    demonstrate_boat_body();
}

/// 多点巡航演示：依次驶向一串航点。
fn demonstrate_track() {
    use brain_autopilot::{BoatAutopilot, BoatConfig, StepOutcome, World};
    println!("  --- 多点巡航（航迹） ---");
    let world = World::new(80, 30);
    let cfg = BoatConfig::default();
    let mut boat = BoatAutopilot::new(cfg, world.width(), world.height(), (3.0, 15.0, 0.0));
    let track = vec![
        Vec3::new(20.0, 15.0, 0.0),
        Vec3::new(40.0, 22.0, 0.0),
        Vec3::new(60.0, 15.0, 0.0),
    ];
    boat.set_track(track.clone());
    let mut done = false;
    for _ in 0..2500 {
        if boat.step_once(&world) == StepOutcome::Done {
            done = true;
            break;
        }
    }
    let (x, y, _) = boat.pose();
    println!(
        "  航迹 3 点：{}，终点 ({:.1},{:.1})，总航程 {:.1}m",
        if done { "巡航完成" } else { "未完成" },
        x,
        y,
        boat.distance()
    );
}

/// COLREGS 会遇避让演示：分类 + 两船会遇模拟避让。
fn demonstrate_colregs() {
    use brain_autopilot::{Colregs, ColregsAction, ColregsParams, VesselPose};
    println!("  --- COLREGS 会遇避让（简化规则） ---");
    let p = ColregsParams::default();

    let (t1, a1) = Colregs::classify(
        VesselPose::new(0.0, 0.0, 0.0),
        VesselPose::new(10.0, 0.0, std::f32::consts::PI),
        &p,
    );
    println!("  对遇：{t1:?} → 本船 {a1:?}");
    let (t2, a2) = Colregs::classify(
        VesselPose::new(0.0, 0.0, 0.0),
        VesselPose::new(8.0, 5.0, -1.1),
        &p,
    );
    println!("  交叉(右舷有船)：{t2:?} → 本船 {a2:?}");
    let (t3, a3) = Colregs::classify(
        VesselPose::new(0.0, 0.0, 0.0),
        VesselPose::new(5.0, 0.0, 0.0),
        &p,
    );
    println!("  追越(同向前船)：{t3:?} → 本船 {a3:?}");

    let min_sep = simulate_colregs_avoidance();
    println!("  两船交叉会遇模拟最小间距：{min_sep:.1}m（>0 即避让成功）");
    let _ = ColregsAction::SlowDown;
}

/// 两船交叉会遇：各自按 COLREGS 让路/保向，测量最小间距。
fn simulate_colregs_avoidance() -> f32 {
    use brain_autopilot::{Colregs, ColregsAction, ColregsParams, VesselPose};
    // A 从 (0,0) 朝 +x，B 从 (40,-40) 朝 +y，二者会在 (40,0) 附近交叉。
    let (mut ax, mut ay, mut ah) = (0.0f32, 0.0f32, 0.0f32);
    let (mut bx, mut by, mut bh) = (40.0f32, -40.0f32, std::f32::consts::FRAC_PI_2);
    let p = ColregsParams::default();
    let mut min_sep = f32::MAX;
    for _ in 0..250 {
        min_sep = min_sep.min(((ax - bx).powi(2) + (ay - by).powi(2)).sqrt());
        let (_, aa) =
            Colregs::classify(VesselPose::new(ax, ay, ah), VesselPose::new(bx, by, bh), &p);
        let (_, ba) =
            Colregs::classify(VesselPose::new(bx, by, bh), VesselPose::new(ax, ay, ah), &p);
        // 让路 → 向右转（starboard）；保向 → 直行。
        ah -= if aa == ColregsAction::GiveWayStarboard {
            0.05
        } else {
            0.0
        };
        bh -= if ba == ColregsAction::GiveWayStarboard {
            0.05
        } else {
            0.0
        };
        let sp = 2.0;
        let dt = 0.1;
        ax += sp * ah.cos() * dt;
        ay += sp * ah.sin() * dt;
        bx += sp * bh.cos() * dt;
        by += sp * bh.sin() * dt;
    }
    min_sep
}

/// 用统一 `RobotBody` 接口驱动一艘水面艇（与驱动无人机/汽车方式一致）。
fn demonstrate_boat_body() {
    let mut boat = BoatBody::new(0.0, 0.0, 0.0);
    println!("  --- RobotBody 具身抽象演示（BoatBody） ---");
    println!("  kind = {:?}", boat.kind());

    let nav = EffectorCommand {
        timestamp: 0,
        locomotion: LocomotionMode::Navigate,
        task: Task::NavigateTo(TaskTarget::Point(Vec3::new(30.0, 0.0, 0.0))),
    };
    boat.send_command(&nav).expect("navigate");
    for _ in 0..40 {
        boat.step();
    }
    if let Ok(s) = boat.read_state() {
        let (sx, sy) = (s.base.pose.position.x, s.base.pose.position.y);
        let thruster = s
            .joints
            .iter()
            .find(|j| j.name == "thruster_stbd")
            .map(|j| j.velocity)
            .unwrap_or(0.0);
        println!("  after NavigateTo -> base=({sx:.1}, {sy:.1}) 推进器速度={thruster:.2}m/s 吃水线接触={}", s.contacts.len());
    }
    println!("  （四足/机械臂/人形/汽车/水面艇只需各自实现 RobotBody，上层逻辑无需改动）");
}
