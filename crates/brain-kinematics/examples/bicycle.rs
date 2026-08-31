//! `brain-kinematics` 最小示例：自行车（Ackermann）车辆运动学积分。
//!
//! 运行：`cargo run -p brain-kinematics --example bicycle`

use brain_kinematics::{BicycleModel, BicycleState};

fn main() {
    let model = BicycleModel::new(2.6, 0.6, 0.8); // 轴距 / 最大转向 / 转向速率
    println!("min turning radius = {:.2} m", model.min_turning_radius());

    // 以 4 m/s、持续 0.3 rad 转向前进 5 秒
    let mut state = BicycleState::new(0.0, 0.0, 0.0);
    for _ in 0..50 {
        state = model.step(&state, 4.0, 0.3, 0.1);
    }
    println!(
        "after 5s: x={:.2} y={:.2} heading={:.2} speed={:.2}",
        state.x, state.y, state.theta, state.speed
    );
}
