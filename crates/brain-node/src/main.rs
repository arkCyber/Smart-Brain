//! Smart-Brain 主程序（brain-node）。
//!
//! 把各层模块装配成一个可运行的任务计算机原型，在 SITL（mock 飞控）中
//! 演示完整闭环：
//!   起飞 -> 巡航 -> 感知发现目标 -> 跟踪 -> 返航 -> 降落
//! 并演示 Fail-safe 看门狗在“大脑卡死”时强制接管进入自动悬停。

mod agent_demo;
mod autopilot_demo;
mod boat_demo;
mod car_driving_demo;
mod comprehensive_demo;
mod embodiment;
mod indoor;
mod kalman_demo;
mod tree_builder;
mod zenoh_demo;
mod zenoh_fcu_demo;

use brain_behavior_tree::core::{BrainOutput, Tree};
use brain_behavior_tree::Status;
use brain_core::config::BrainConfig;
use brain_core::time::instant_now;
use brain_message::{Command, CommandTarget, Mode, Telemetry};
use brain_middleware::bus::topics;
use brain_middleware::DataBus;
use brain_mission::{Mission, SwarmLink, SwarmRole, SwarmShare, Waypoint};
use brain_perception::backend::MockModelBackend;
use brain_perception::pipeline::{VisionConfig, VisionPipeline};
use brain_state::{FailsafeWatchdog, FlightState, StateMachine};
use brain_transport::{FcuTransport, MockTransport};

/// 主流程：装配并运行一次完整任务演示。
fn run_mission_demo(iterations: usize) {
    let cfg = BrainConfig::default();
    cfg.validate().expect("invalid config");

    let bus = DataBus::new();

    // ---- 硬件接口：与“小脑”的传输（默认 mock = SITL） ----
    let mut transport = MockTransport::new();

    // ---- 感知层：仿真推理后端 ----
    let mut perception =
        VisionPipeline::new(Box::new(MockModelBackend::new()), VisionConfig::default());
    perception
        .load_model("models/yolov8n.onnx")
        .expect("load model");

    // ---- 状态机与看门狗 ----
    let mut state_machine = StateMachine::new();
    let mut watchdog = FailsafeWatchdog::new(cfg.failsafe_timeout_ms);

    // ---- 任务层：航线规划与蜂群链路 ----
    let mission = Mission::new(
        "survey-01",
        vec![
            Waypoint {
                sequence: 0,
                north: 0.0,
                east: 0.0,
                alt: 30.0,
                accept_radius: 2.0,
            },
            Waypoint {
                sequence: 1,
                north: 80.0,
                east: 0.0,
                alt: 30.0,
                accept_radius: 2.0,
            },
            Waypoint {
                sequence: 2,
                north: 80.0,
                east: 80.0,
                alt: 30.0,
                accept_radius: 2.0,
            },
        ],
    );
    let _ = &mission;
    let swarm = SwarmLink::new(&cfg.node_id, SwarmRole::Leader);

    // ---- 决策层：行为树 ----
    let mut tree = Tree::new(tree_builder::build_mission_tree());
    let mut output = BrainOutput::idle();

    println!("\n=== Smart-Brain demo: {} ticks ===\n", iterations);

    let mut cmd_counter = 0usize;
    let mut landing_requested = false;
    let mut mission_complete = false;
    let mut completion_printed = false;
    for i in 0..iterations {
        let now = instant_now();

        // 1. 感知：推理并发布检测/跟踪。
        perception.tick(&bus, now);
        perception.publish_tracking(&bus, now);

        // 2. 决策：行为树输出控制意图。
        let _status: Status = tree.tick(&bus, now, &mut output);

        // 任务完成（已落地）后锁定为 Idle，避免行为树循环重跑起飞。
        if mission_complete {
            output.mode = Mode::Idle;
            output.target = CommandTarget::None;
            output.note = "mission complete, holding at ground".into();
        }
        if output.mode == Mode::Land {
            landing_requested = true;
        }

        // 3. 执行：把意图转成飞控指令下发。
        let cmd = Command {
            timestamp: now,
            mode: output.mode,
            target: output.target.clone(),
        };
        transport.send_command(&cmd).expect("send command");
        let _ = bus.publish(topics::COMMAND, cmd.clone(), now);
        cmd_counter += 1;

        // 4. 读取遥测并发布到总线（供感知/决策节点读取）。
        if let Ok(Some(telem)) = transport.try_recv_telemetry() {
            // 已请求降落且高度归零 → 判定任务完成。
            if landing_requested && telem.gps.alt <= 0.5 {
                mission_complete = true;
            }
            let _ = bus.publish(topics::TELEMETRY, telem, now);
        }

        // 5. 状态机同步。
        let target_state = match output.mode {
            Mode::Idle => FlightState::Ground,
            Mode::Takeoff => FlightState::TakingOff,
            Mode::Cruise => FlightState::Cruising,
            Mode::Track => FlightState::Tracking,
            Mode::ReturnHome => FlightState::ReturningHome,
            Mode::Land => FlightState::Landing,
            Mode::Loiter => FlightState::Loitering,
        };
        let _ = state_machine.transition(target_state);

        // 任务完成时打印一次，并展示完整状态流转。
        if mission_complete && !completion_printed {
            println!("\n>>> mission complete: landed & returned to Ground\n");
            completion_printed = true;
        }

        // 6. Fail-safe 看门狗：正常喂狗。
        watchdog.feed(now);
        if let Some(ev) = watchdog.check(now) {
            log::info!("watchdog event: {ev:?}");
        }

        // 7. 心跳。
        let _ = bus.publish(topics::HEARTBEAT, now, now);

        // 8. 蜂群广播态势。
        let telem = bus
            .topic::<Telemetry>(topics::TELEMETRY)
            .and_then(|t| t.peek())
            .unwrap_or_else(|| Telemetry::default_at(now));
        let share = SwarmShare::new(
            &cfg.node_id,
            now,
            telem.velocity,
            telem.attitude.yaw,
            matches!(
                perception.tracking(),
                brain_message::TrackingStatus::Locked { .. }
            ),
            telem.battery.remaining_pct,
        );
        swarm.broadcast(&share, &bus, now);

        // 打印任务执行期间的每个 tick（任务完成后进入静默保持）。
        if !mission_complete {
            println!(
                "[tick {:>3}] state={:<11} mode={:<12} note={}",
                i,
                format!("{:?}", state_machine.current()),
                format!("{:?}", output.mode),
                output.note
            );
        }
        output.reset();
    }

    println!("\n=== done after {iterations} ticks, {cmd_counter} commands sent ===");
    println!("final flight state: {:?}", state_machine.current());
    println!("assembled topics: {}", bus.len());
}

/// 演示 Fail-safe：模拟大脑“卡死”（停止喂狗），验证看门狗强制进入 Loiter。
fn demonstrate_failsafe() {
    let mut watchdog = FailsafeWatchdog::new(50);
    println!(
        "\n=== Fail-safe demo (watchdog timeout = {}ms) ===",
        watchdog.timeout()
    );

    // 前 2 次正常喂狗。
    for _ in 0..2 {
        watchdog.feed(instant_now());
        assert!(watchdog.check(instant_now()).is_none());
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    println!("heartbeat ok, watchdog armed.");

    // 模拟大脑卡死：不喂狗，等待超时。
    std::thread::sleep(std::time::Duration::from_millis(80));
    let ev = watchdog.check(instant_now());
    match ev {
        Some(brain_state::FailsafeEvent::Trip { missed_ms }) => {
            println!(">>> WATCHDOG TRIPPED after {missed_ms}ms -> forcing LOITER (auto-hover)");
        }
        other => println!(">>> unexpected: {other:?}"),
    }
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    // 默认运行一次完整任务演示（SITL）。
    run_mission_demo(30);

    // 演示独立看门狗的安全兜底机制。
    demonstrate_failsafe();

    // 演示“无人机只是众多身体之一”：通过统一 RobotBody 接口驱动。
    embodiment::demonstrate(0);

    // 室内无 GPS 自主避障（视觉定位）流水线演示。
    indoor::run();

    // Zenoh 统一通信（Pub/Sub + Store/Query + Compute）演示。
    zenoh_demo::run();

    // 闭环自主导航（感知→建图→规划→驱动→回溯）演示。
    autopilot_demo::run();

    // 汽车驾驶导航（阿克曼前轮转向 / Ackermann DWA / 自行车模型）演示。
    car_driving_demo::run();

    // 水面自动驾驶艇（双差速推进 / 水流漂移 / 定泊保持）演示。
    boat_demo::run();

    // 小脑（飞控）经 Zenoh 桥接（zenoh-pico 思路）演示。
    zenoh_fcu_demo::run();

    // Agent / LLM 思考层（工具调用驱动底层能力）演示。
    agent_demo::run();

    // 卡尔曼滤波传感器融合（IMU 预测 + VO 测量）演示。
    kalman_demo::run();

    // 综合任务演示（任务文件 → 执行+进度 → 蜂群 → MAVLink）。
    comprehensive_demo::run();

    println!("\nSmart-Brain prototype finished. Next: wire real serial/CAN + ONNX model.");
}
