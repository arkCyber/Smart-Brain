//! `brain-sim` 最小示例：用确定性 MockSimulator 推进世界并查询感知。
//!
//! 运行：`cargo run -p brain-sim --example sim`

use brain_core::Vec3;
use brain_sim::{MockSimulator, Simulator};

fn main() {
    // 100×100 单元、1m/单元的世界
    let mut sim = MockSimulator::new(100, 100, 1.0);

    // 下发速度指令并推进 1 秒
    sim.set_velocity_command(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO)
        .unwrap();
    sim.step(1.0).unwrap();

    let st = sim.state();
    println!(
        "pos = {:?} | collisions = {} | steps = {}",
        st.robot_pose.position, st.collisions, st.steps
    );
    println!(
        "front range = {:.1} m | detections = {}",
        sim.range(0.0),
        sim.detections().len()
    );
}
