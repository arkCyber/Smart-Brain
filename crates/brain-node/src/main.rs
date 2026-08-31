//! Smart-Brain 主程序（brain-node）。
//!
//! 把各层模块装配成一个可运行的任务计算机原型，在 SITL（mock 飞控）中
//! 演示完整闭环：
//!   起飞 -> 巡航 -> 感知发现目标 -> 跟踪 -> 返航 -> 降落
//! 并演示 Fail-safe 看门狗在“大脑卡死”时强制接管进入自动悬停。

mod agent_demo;
#[cfg(feature = "async")]
mod async_runtime;
mod autopilot_demo;
mod boat_demo;
mod car_driving_demo;
mod comprehensive_demo;
mod embodiment;
mod generic_demo;
mod indoor;
mod kalman_demo;
mod locomotion_sim_demo;
mod parallel;
mod safety_guard;
mod stereo_demo;
mod swarm_coord_demo;
mod time_sync_demo;
mod tree_builder;
mod zenoh_demo;
mod zenoh_fcu_demo;

use brain_behavior_tree::core::{BrainOutput, Tree};
use brain_behavior_tree::Status;
use brain_core::config::BrainConfig;
use brain_core::time::instant_now;
use brain_core::Vec3;
use brain_message::{Command, CommandTarget, Mode, Telemetry};
use brain_middleware::bus::topics;
use brain_middleware::DataBus;
use brain_mission::{Mission, SwarmLink, SwarmRole, SwarmShare, Waypoint};
use brain_perception::backend::MockModelBackend;
use brain_perception::pipeline::{VisionConfig, VisionPipeline};
use brain_state::safety::{BatteryMonitor, Geofence, PreArmCheck, PreArmConfig};
use brain_state::{FailsafeWatchdog, FlightState, StateMachine, WatchdogStatus};
use brain_transport::MockTransport;

/// 主流程：装配并运行一次完整任务演示。
fn run_mission_demo(iterations: usize, cfg: &BrainConfig) {
    cfg.validate().expect("invalid config");

    let bus = DataBus::new();

    // ---- 硬件接口：与“小脑”的传输（按配置选择，默认 mock = SITL）----
    // 真机在 config.json 里把 transport 设为 serial/can/udp；本机无硬件时回退 mock。
    let mut transport = match brain_transport::open_transport(&cfg.fcu.transport) {
        Ok(t) => {
            log::info!("transport backend: {}", cfg.fcu.transport);
            t
        }
        Err(e) => {
            log::warn!(
                "transport {} unavailable ({e}); falling back to mock",
                cfg.fcu.transport
            );
            Box::new(MockTransport::new())
        }
    };

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

    // ---- 安全监督：围栏 / 电量 / pre-arm（兜底覆盖指令），参数来自配置 ----
    let s = cfg.safety;
    let safety = safety_guard::SafetySupervisor::new(
        Geofence::new(
            Vec3::ZERO,
            s.geofence_radius_m,
            s.geofence_max_altitude_m,
            0.0,
        ),
        BatteryMonitor::new(s.battery_rth_pct, s.battery_critical_pct, s.battery_low_pct),
        PreArmCheck::new(PreArmConfig {
            min_gps_satellites: s.prearm_min_gps_satellites,
            require_fix3d: s.prearm_require_fix3d,
            min_battery_pct: s.prearm_min_battery_pct,
            require_home: s.prearm_require_home,
        }),
    );

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

        // 3. 执行：把意图转成飞控指令下发。先经安全监督覆盖（越界→返航，低电→降落）。
        let telem_now = bus
            .topic::<Telemetry>(topics::TELEMETRY)
            .and_then(|t| t.peek());
        let pos = telem_now
            .as_ref()
            .map(|t| Vec3::new(0.0, 0.0, -t.gps.alt))
            .unwrap_or(Vec3::ZERO);
        let battery_pct = telem_now
            .as_ref()
            .map(|t| t.battery.remaining_pct)
            .unwrap_or(100.0);
        let watchdog_armed = watchdog.status() == WatchdogStatus::Armed;
        let mut cmd = Command {
            timestamp: now,
            mode: output.mode,
            target: output.target.clone(),
        };
        if safety.apply(&mut cmd, pos, battery_pct, watchdog_armed) {
            log::warn!("safety override -> {:?}", cmd.mode);
        }
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
fn demonstrate_failsafe(timeout_ms: u64) {
    let mut watchdog = FailsafeWatchdog::new(timeout_ms);
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
    // 用单调 Stopwatch 测量整套演示的总耗时（真正单调，不受系统时间调整影响）。
    let wall = brain_core::time::Stopwatch::start();

    // 加载配置：优先环境变量 SMART_BRAIN_CONFIG 指向的路径，其次 config.json，
    // 最后回退到仓库自带的 config.example.json；全部缺失时使用内置默认值。
    let env_path = std::env::var("SMART_BRAIN_CONFIG").unwrap_or_else(|_| String::new());
    let mut candidates: Vec<&str> = Vec::new();
    if !env_path.is_empty() {
        candidates.push(&env_path);
    }
    candidates.push("config.json");
    candidates.push("config.example.json");
    let cfg = BrainConfig::load_candidates(&candidates);

    // 默认运行一次完整任务演示（SITL）。
    run_mission_demo(30, &cfg);

    // 演示独立看门狗的安全兜底机制。
    demonstrate_failsafe(cfg.failsafe_timeout_ms);

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

    // 步态 + 仿真集成（四足行走 / 确定性仿真后端）演示。
    locomotion_sim_demo::run();

    // 时间同步（NTP 风格四时间戳握手）演示。
    time_sync_demo::run();

    // 双目立体视觉（两个相机 → 视差 → 三维点云）演示。
    stereo_demo::run();

    // 通用状态机（RobotState）+ 通用传感器话题（sensor/imu 等）演示。
    generic_demo::run();

    // 并行（多线程）流水线：感知线程 + 决策线程共享总线。
    parallel::run_parallel_demo();

    // 安全监督器：geofence / 电量 / pre-arm 集成演示。
    safety_guard::run_safety_demo();

    // tokio 异步流水线（需 `--features async` 编译）。
    #[cfg(feature = "async")]
    async_runtime::run();

    // 蜂群协同：Leader 选举 + 任务分配演示。
    swarm_coord_demo::run();

    // 综合任务演示（任务文件 → 执行+进度 → 蜂群 → MAVLink）。
    comprehensive_demo::run();

    println!(
        "\nSmart-Brain prototype finished in {:.2}s. Real backends available: serial / CAN (Linux) / ONNX + NMS.",
        wall.elapsed_secs()
    );
}
