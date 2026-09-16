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
#[cfg(feature = "hermes")]
mod hermes_demo;
mod indoor;
mod kalman_demo;
mod locomotion_sim_demo;
mod model_factory;
#[cfg(feature = "ollama")]
mod ollama_demo;
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
use brain_mission::{SwarmLink, SwarmRole, SwarmShare};
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

    // ---- 任务层：蜂群链路（航点任务由行为树 / `comprehensive_demo` 承担）----
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

/// 可用演示注册表（名称 → 描述），供 `--demo` / `--list` 使用。
const DEMOS: &[(&str, &str)] = &[
    (
        "mission",
        "完整任务闭环（SITL）：起飞→巡航→发现目标→跟踪→降落",
    ),
    ("failsafe", "Fail-safe 看门狗：心跳中断强制自动悬停"),
    ("embodiment", "具身抽象：把无人机包装成 RobotBody 驱动"),
    ("indoor", "室内无 GPS 自主避障（视觉定位）"),
    ("zenoh", "Zenoh 统一通信（Pub/Sub + Store/Query + Compute）"),
    ("autopilot", "闭环自主导航（感知→建图→规划→驱动→回溯）"),
    ("car", "汽车驾驶导航（阿克曼 / Ackermann DWA / 自行车模型）"),
    ("boat", "水面自动驾驶艇（双差速 / 水流漂移 / 定泊）"),
    ("zenoh_fcu", "小脑（飞控）经 Zenoh 桥接"),
    ("agent", "Agent / LLM 思考层（工具调用）"),
    ("kalman", "卡尔曼传感器融合（IMU + VO）"),
    ("locomotion", "步态 + 仿真集成（四足行走）"),
    ("time_sync", "NTP 风格时间同步"),
    ("stereo", "双目立体视觉（视差 → 点云）"),
    ("generic", "通用状态机 + 通用传感器话题"),
    ("parallel", "多线程并行流水线（感知 + 决策）"),
    ("model", "模型后端工厂（按配置选 mock/ollama/hermes）"),
    ("safety", "安全监督器（geofence / 电量 / pre-arm）"),
    ("swarm", "蜂群协同（Leader 选举 + 任务分配）"),
    ("comprehensive", "综合任务（任务文件→执行→蜂群→MAVLink）"),
];

/// 仅在 `--features async` 下可用的演示。
#[cfg(feature = "async")]
const DEMOS_ASYNC: &[(&str, &str)] = &[("async", "tokio 异步任务并发")];

/// 仅在 `--features ollama` 下可用的演示。
#[cfg(feature = "ollama")]
const DEMOS_OLLAMA: &[(&str, &str)] = &[("ollama", "Ollama 推理（端口 11434 / 工具调用）")];

/// 仅在 `--features hermes` 下可用的演示。
#[cfg(feature = "hermes")]
const DEMOS_HERMES: &[(&str, &str)] =
    &[("hermes", "Hermes 智能体 daemon（端口 11438 / OpenAI 兼容）")];

/// 命令行参数。
struct Args {
    demo: Option<String>,
    config: Option<String>,
    iterations: usize,
    list: bool,
    help: bool,
    version: bool,
}

/// 解析命令行参数（无第三方依赖）。
fn parse_args() -> Result<Args, String> {
    parse_args_from(std::env::args().skip(1))
}

/// 从迭代器解析参数（与 `std::env::args` 解耦，便于单元测试）。
fn parse_args_from<I: IntoIterator<Item = String>>(raw: I) -> Result<Args, String> {
    let mut out = Args {
        demo: None,
        config: None,
        iterations: 30,
        list: false,
        help: false,
        version: false,
    };
    let mut it = raw.into_iter();
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "-h" | "--help" => out.help = true,
            "-V" | "--version" => out.version = true,
            "--list" => out.list = true,
            "--demo" => {
                let v = it
                    .next()
                    .ok_or_else(|| "--demo requires a name".to_string())?;
                out.demo = Some(v);
            }
            "--config" => {
                let v = it
                    .next()
                    .ok_or_else(|| "--config requires a path".to_string())?;
                out.config = Some(v);
            }
            "--iterations" => {
                let v = it
                    .next()
                    .ok_or_else(|| "--iterations requires a number".to_string())?;
                out.iterations = v.parse().map_err(|_| format!("invalid iterations: {v}"))?;
                if out.iterations == 0 {
                    return Err("iterations must be > 0".into());
                }
            }
            other => return Err(format!("unknown argument: {other}")),
        }
    }
    Ok(out)
}

fn print_help() {
    println!(
        "Smart-Brain main executable\n\
         \n\
         USAGE:\n    brain-node [OPTIONS]\n\
         \n\
         OPTIONS:\n    --demo <name>       Run a single demo (see --list)\n    --list              List available demos\n    --config <path>     Path to config file (overrides $SMART_BRAIN_CONFIG)\n    --iterations <n>    Ticks for the mission demo (default 30)\n    -h, --help          Print help\n    -V, --version       Print version\n"
    );
}

fn list_demos() {
    println!("available demos:");
    for (name, desc) in DEMOS {
        println!("  {name:<14} {desc}");
    }
    #[cfg(feature = "async")]
    for (name, desc) in DEMOS_ASYNC {
        println!("  {name:<14} {desc} (--features async)");
    }
    #[cfg(feature = "ollama")]
    for (name, desc) in DEMOS_OLLAMA {
        println!("  {name:<14} {desc} (--features ollama)");
    }
    #[cfg(feature = "hermes")]
    for (name, desc) in DEMOS_HERMES {
        println!("  {name:<14} {desc} (--features hermes)");
    }
}

/// 按名称运行一个演示；返回是否找到。
fn run_demo_by_name(name: &str, cfg: &BrainConfig, iterations: usize) -> bool {
    match name {
        "mission" => run_mission_demo(iterations, cfg),
        "failsafe" => demonstrate_failsafe(cfg.failsafe_timeout_ms),
        "embodiment" => embodiment::demonstrate(0),
        "indoor" => indoor::run(),
        "zenoh" => zenoh_demo::run(),
        "autopilot" => autopilot_demo::run(),
        "car" => car_driving_demo::run(),
        "boat" => boat_demo::run(),
        "zenoh_fcu" => zenoh_fcu_demo::run(),
        "agent" => agent_demo::run(),
        "kalman" => kalman_demo::run(),
        "locomotion" => locomotion_sim_demo::run(),
        "time_sync" => time_sync_demo::run(),
        "stereo" => stereo_demo::run(),
        "generic" => generic_demo::run(),
        "parallel" => parallel::run_parallel_demo(),
        "safety" => safety_guard::run_safety_demo(),
        "swarm" => swarm_coord_demo::run(),
        "comprehensive" => comprehensive_demo::run(),
        "model" => model_factory::run(cfg),
        #[cfg(feature = "async")]
        "async" => async_runtime::run(),
        #[cfg(feature = "ollama")]
        "ollama" => ollama_demo::run(cfg),
        #[cfg(feature = "hermes")]
        "hermes" => hermes_demo::run(cfg),
        _ => return false,
    }
    true
}

/// 依序运行全部演示（默认行为，与旧版一致）。
fn run_all_demos(cfg: &BrainConfig, iterations: usize) {
    for (name, _) in DEMOS {
        run_demo_by_name(name, cfg, iterations);
    }
    #[cfg(feature = "async")]
    for (name, _) in DEMOS_ASYNC {
        run_demo_by_name(name, cfg, iterations);
    }
    #[cfg(feature = "ollama")]
    for (name, _) in DEMOS_OLLAMA {
        run_demo_by_name(name, cfg, iterations);
    }
    #[cfg(feature = "hermes")]
    for (name, _) in DEMOS_HERMES {
        run_demo_by_name(name, cfg, iterations);
    }
}

/// 加载配置：`--config` > `$SMART_BRAIN_CONFIG` > `config.json` > `config.example.json` > 默认。
fn load_config(explicit: Option<&str>) -> BrainConfig {
    let mut candidates: Vec<String> = Vec::new();
    if let Some(p) = explicit {
        candidates.push(p.to_string());
    }
    if let Ok(env_path) = std::env::var("SMART_BRAIN_CONFIG") {
        if !env_path.is_empty() {
            candidates.push(env_path);
        }
    }
    candidates.push("config.json".into());
    candidates.push("config.example.json".into());
    let refs: Vec<&str> = candidates.iter().map(|s| s.as_str()).collect();
    BrainConfig::load_candidates(&refs)
}

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let args = match parse_args() {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            print_help();
            std::process::exit(2);
        }
    };
    if args.help {
        print_help();
        return;
    }
    if args.version {
        println!("brain-node {}", env!("CARGO_PKG_VERSION"));
        return;
    }
    if args.list {
        list_demos();
        return;
    }

    // 加载配置并校验（无效配置以非零退出码终止，而非 panic）。
    let cfg = load_config(args.config.as_deref());
    if let Err(e) = cfg.validate() {
        eprintln!("error: invalid config: {e}");
        std::process::exit(1);
    }

    // 用单调 Stopwatch 测量整套演示的总耗时（真正单调，不受系统时间调整影响）。
    let wall = brain_core::time::Stopwatch::start();

    // 单演示模式：`--demo <name>`。
    if let Some(name) = args.demo.as_deref() {
        if run_demo_by_name(name, &cfg, args.iterations) {
            println!(
                "\nSmart-Brain demo '{name}' finished in {:.2}s.",
                wall.elapsed_secs()
            );
        } else {
            eprintln!("error: unknown demo '{name}' (see --list)");
            std::process::exit(2);
        }
        return;
    }

    // 默认：依序运行全部演示（SITL）。
    run_all_demos(&cfg, args.iterations);
    println!(
        "\nSmart-Brain prototype finished in {:.2}s. Real backends available: serial / CAN (Linux) / ONNX + NMS.",
        wall.elapsed_secs()
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn parse(args: &[&str]) -> Result<Args, String> {
        parse_args_from(args.iter().map(|s| s.to_string()))
    }

    #[test]
    fn parses_defaults() {
        let a = parse(&[]).unwrap();
        assert!(a.demo.is_none() && a.config.is_none());
        assert_eq!(a.iterations, 30);
        assert!(!a.list && !a.help && !a.version);
    }

    #[test]
    fn parses_flags_and_values() {
        let a = parse(&["--demo", "zenoh", "--iterations", "5"]).unwrap();
        assert_eq!(a.demo.as_deref(), Some("zenoh"));
        assert_eq!(a.iterations, 5);
        let a = parse(&["--config", "/tmp/cfg.json"]).unwrap();
        assert_eq!(a.config.as_deref(), Some("/tmp/cfg.json"));
        assert!(parse(&["--list"]).unwrap().list);
        assert!(parse(&["--version"]).unwrap().version);
        assert!(parse(&["-h"]).unwrap().help);
        assert!(parse(&["--help"]).unwrap().help);
    }

    #[test]
    fn rejects_invalid_input() {
        assert!(parse(&["--bogus"]).is_err());
        assert!(parse(&["--demo"]).is_err()); // 缺值
        assert!(parse(&["--iterations"]).is_err());
        assert!(parse(&["--iterations", "abc"]).is_err());
        assert!(parse(&["--iterations", "0"]).is_err()); // 必须 > 0
        assert!(parse(&["--config"]).is_err());
    }

    #[test]
    fn demo_registry_names_are_unique_and_nonempty() {
        assert!(!DEMOS.is_empty());
        let mut names: Vec<&str> = DEMOS.iter().map(|(n, _)| *n).collect();
        let before = names.len();
        names.sort_unstable();
        names.dedup();
        assert_eq!(names.len(), before, "demo names must be unique");
        for (_, desc) in DEMOS {
            assert!(!desc.is_empty());
        }
    }
}
