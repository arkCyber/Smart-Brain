//! `brain-mission` 最小示例：定义航点任务 → 执行器逐点生成巡航指令。
//!
//! 运行：`cargo run -p brain-mission --example mission`

use brain_mission::{Mission, MissionExecutor, Waypoint};

fn main() {
    let mission = Mission::new(
        "survey-01",
        vec![
            Waypoint { sequence: 0, north: 0.0, east: 0.0, alt: 30.0, accept_radius: 2.0 },
            Waypoint { sequence: 1, north: 100.0, east: 0.0, alt: 30.0, accept_radius: 2.0 },
            Waypoint { sequence: 2, north: 100.0, east: 100.0, alt: 30.0, accept_radius: 2.0 },
        ],
    );

    let mut exec = MissionExecutor::new(mission).expect("valid mission");
    exec.start();

    // 逐个航点下发
    while let Some(target) = exec.current_target() {
        println!("waypoint[{}] target = {target:?}", exec.index());
        exec.advance();
    }
    println!("mission phase = {:?}", exec.phase());
}
