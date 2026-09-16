//! `brain-locomotion` 示例：足-地接触 + 全身动力学力矩。
//!
//! 运行：`cargo run -p brain-locomotion --example dynamic_contact`
//!
//! 展示从"躯干位姿命令"到"逐腿关节力矩"的全身动力学整合：
//! 1. `WholeBodyController::solve` 把站姿命令解析为逐腿关节角与足底力分配；
//! 2. `ContactModel` 依据躯干高度判断哪些足触地（弹簧-阻尼地面反力）；
//! 3. `dynamic_torques` 用逐腿逆动力学（RNEA）算出计入重力的 `[髋, 膝]` 力矩。

use brain_core::Vec3;
use brain_locomotion::{ContactConfig, ContactModel, LegIK, WholeBodyCommand, WholeBodyController};

fn main() {
    // 标准四足髋位（前左/前右/后左/后右），杆长 0.4+0.4m，体重 400N。
    let hips = vec![
        Vec3::new(0.2, 0.15, 0.0),
        Vec3::new(0.2, -0.15, 0.0),
        Vec3::new(-0.2, 0.15, 0.0),
        Vec3::new(-0.2, -0.15, 0.0),
    ];
    let ctl = WholeBodyController::quadruped(LegIK::new(0.4, 0.4), hips, 0.4, 400.0).unwrap();

    // 1) 直立站姿
    let target = ctl
        .solve(&WholeBodyCommand::stand(ctl.nominal_height))
        .unwrap();
    let n = ctl.leg_count();
    let q: Vec<[f32; 2]> = target.joint_targets.clone();
    let qd = vec![[0.0f32; 2]; n];
    let qdd = vec![[0.0f32; 2]; n];
    let gravity = Vec3::new(0.0, 0.0, -9.81);
    let contact = ContactModel::new(ContactConfig::default());

    // 2) 静力学力矩（纯 τ = Jᵀ·f，不计腿质量/重力）
    let stat = ctl.joint_torques(&target).unwrap();

    // 3) 全身动力学力矩（接触门控 + RNEA，计入重力）。
    //    躯干略降 0.005m → 每足穿透 0.005m，接触力 ≈ 体重/n，处于支撑相。
    let trunk_height = ctl.nominal_height - 0.005;
    let tau = ctl
        .dynamic_torques(&target, &q, &qd, &qdd, trunk_height, &contact, gravity)
        .unwrap();

    for i in 0..n {
        println!(
            "leg {i}: static=[{:7.2}, {:7.2}] N·m  dynamic=[{:7.2}, {:7.2}] N·m",
            stat[i][0], stat[i][1], tau[i][0], tau[i][1]
        );
    }
    println!("(动态比静态多了腿自重与重力的贡献)");
}
