//! `brain-locomotion` 最小示例：四足小跑（Trot）步态相位生成。
//!
//! 运行：`cargo run -p brain-locomotion --example gait`

use brain_locomotion::{GaitConfig, GaitGenerator, GaitType};

fn main() {
    // 2Hz 的小跑步态，对角腿同相
    let mut gen = GaitGenerator::new(GaitConfig::quadruped(GaitType::Trot, 2.0));

    // 以 16ms 控制周期推进约 1 秒
    for _ in 0..60 {
        gen.advance(0.016);
    }

    let ph = gen.snapshot();
    println!(
        "cycle = {:.2} | legs = {} | swinging = {} | all_stance = {}",
        ph.cycle,
        ph.legs.len(),
        ph.swing_count(),
        ph.all_stance()
    );
}
