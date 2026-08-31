//! 综合任务演示：任务文件 → 执行+进度 → 蜂群协同 → MAVLink 命令下发。
//!
//! 串联多个模块：
//!   1. `brain-mission`：任务构建/存文件/加载、执行器、进度跟踪
//!   2. `brain-mission`：SwarmLink 蜂群协同（态势共享）
//!   3. `brain-transport`：MavLinkTransport 把航点转成 MAVLink 命令帧下发

use brain_core::Vec3;
use brain_message::{Command, CommandTarget, FrameReader, Mode};
use brain_middleware::DataBus;
use brain_mission::{Mission, MissionExecutor, SwarmLink, SwarmRole, SwarmShare, Waypoint};
use brain_transport::{decode_stream, FcuTransport, MavLinkTransport, MavMessage};

/// 构造一条巡检任务。
fn build_mission() -> Mission {
    Mission::new(
        "survey-line",
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
            Waypoint {
                sequence: 3,
                north: 0.0,
                east: 80.0,
                alt: 30.0,
                accept_radius: 2.0,
            },
        ],
    )
}

/// 模拟从 `pos` 朝目标移动一步，返回是否到达。
fn step_toward(pos: &mut Vec3, target: (f32, f32), step: f32) -> bool {
    let (tx, ty) = target;
    let dx = tx - pos.x;
    let dy = ty - pos.y;
    let d = (dx * dx + dy * dy).sqrt();
    if d < 0.5 {
        pos.x = tx;
        pos.y = ty;
        return true;
    }
    pos.x += dx / d * step;
    pos.y += dy / d * step;
    false
}

/// 顶层入口。
pub fn run() {
    println!("\n=== 综合任务演示：任务文件 → 执行+进度 → 蜂群 → MAVLink ===");

    // ---- 1. 任务文件：构建 → 存文件 → 加载 ----
    let mission = build_mission();
    let path = std::env::temp_dir().join("smart_brain_task.json");
    mission.save(&path).expect("save mission");
    let loaded = Mission::load(&path).expect("load mission");
    println!(
        "[任务] 构建并持久化任务 '{}'，{} 个航点，总距离 {:.0} m",
        loaded.id,
        loaded.waypoints.len(),
        loaded.total_distance()
    );
    std::fs::remove_file(&path).ok();

    // ---- 2. 执行 + 进度上报 ----
    let mut ex = MissionExecutor::new(loaded).expect("executor");
    ex.start();
    println!("[执行] 开始执行，阶段={:?}", ex.phase());
    let mut pos = Vec3::new(0.0, 0.0, 30.0);
    while ex.phase() == brain_mission::MissionPhase::InProgress {
        // 取当前航点目标，模拟飞行逼近。
        if let Some(CommandTarget::Position { north, east, .. }) = ex.current_target() {
            if step_toward(&mut pos, (north, east), 8.0) {
                ex.advance(); // 到达当前航点
            }
        }
        ex.update_position(pos); // 上报位置，累计里程
                                 // 周期打印进度。
        if ex.progress().waypoints_completed().is_multiple_of(2)
            && ex.phase() == brain_mission::MissionPhase::InProgress
        {
            println!(
                "  进度: {:.0}% 航点 {}/{} 已飞 {:.0}m 剩余 {:.0}m",
                ex.progress().percent_complete(),
                ex.progress().waypoints_completed(),
                ex.progress().total_waypoints(),
                ex.progress().distance_traveled(),
                ex.progress().distance_remaining()
            );
        }
    }
    println!(
        "[执行] 完成，阶段={:?}，进度 100%，总里程 {:.0} m",
        ex.phase(),
        ex.progress().distance_traveled()
    );

    // ---- 3. 蜂群协同：领导机广播态势，并接收同伴共享 ----
    let bus = DataBus::new();
    let mut leader = SwarmLink::new("brain-01", SwarmRole::Leader);
    let share = SwarmShare::new("brain-01", 100, pos, 0.5, true /*发现目标*/, 82.0);
    leader.broadcast(&share, &bus, 100);
    // 同伴共享态势。
    let follower = SwarmShare::new(
        "brain-02",
        100,
        Vec3::new(10.0, 0.0, 30.0),
        0.0,
        false,
        90.0,
    );
    leader.ingest(follower);
    println!(
        "[蜂群] 领导机已广播态势，收到 {} 架同伴共享，同伴发现目标={}",
        leader.peers().len(),
        leader.any_peer_target_seen()
    );

    // ---- 4. MAVLink 命令下发：把首个航点转成命令帧 ----
    let mut tx = MavLinkTransport::new();
    let cmd = Command {
        timestamp: 200,
        mode: Mode::Cruise,
        target: CommandTarget::Position {
            north: 80.0,
            east: 0.0,
            down: -30.0,
        },
    };
    tx.send_command(&cmd).expect("mav send");
    let bytes = tx.tx_bytes().to_vec();
    println!("[MAVLink] 已下发 Cruise 命令帧，共 {} 字节", bytes.len());

    // 用 FrameReader 解码命令帧，确认接收端能还原。
    let mut reader = FrameReader::new();
    let mut msgs = Vec::new();
    for m in decode_stream(&mut reader, &bytes).expect("decode stream") {
        msgs.push(m);
    }
    if let Some(MavMessage::Command(back)) = msgs.first() {
        match back.target {
            CommandTarget::Position { north, east, down } => {
                println!(
                    "[MAVLink] 对端解码成功：mode={:?} target=({:.0},{:.0},{:.0})",
                    back.mode, north, east, down
                );
            }
            _ => println!("[MAVLink] 对端解码成功：mode={:?} target=(none)", back.mode),
        }
    }
}
