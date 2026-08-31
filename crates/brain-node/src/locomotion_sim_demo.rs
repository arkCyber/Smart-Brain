//! 步态 + 仿真集成演示：在确定性仿真世界里驱动一只四足机器人行走。
//!
//! 展示 `brain-locomotion`（步态相位 → 足端落点）与 `brain-sim`（仿真后端）
//! 如何协同：大脑下发速度指令 → 步态控制器产出逐腿足端目标 → 仿真推进世界。

use brain_core::Vec3;
use brain_locomotion::gait::GaitType;
use brain_locomotion::leg_ik::LegIK;
use brain_locomotion::{
    GaitConfig, LegDynamics, LocomotionCommand, LocomotionController, WholeBodyCommand,
    WholeBodyController,
};
use brain_sim::sim::Simulator;
use brain_sim::MockSimulator;

/// 四条腿的名义髋部位置（局部系：x 前向、y 侧向、z 向下）。
fn hips() -> Vec<Vec3> {
    vec![
        Vec3::new(0.15, 0.12, -0.4),   // FL
        Vec3::new(0.15, -0.12, -0.4),  // FR
        Vec3::new(-0.15, 0.12, -0.4),  // HL
        Vec3::new(-0.15, -0.12, -0.4), // HR
    ]
}

/// 运行步态 + 仿真闭环演示。
pub fn run() {
    println!("\n=== Locomotion + Sim 集成演示 ===");

    // 1) 确定性 2D 仿真世界（1m 分辨率，放一堵前方 4m 的墙）。
    let mut sim = MockSimulator::new(100, 100, 1.0);
    sim.add_wall(4.0, -3.0, 4.0, 3.0);
    sim.set_target(8.0, 1.5, 0); // 远处一个目标

    // 2) 四足 trot 步态控制器 + 腿部 IK（大腿/小腿各 0.4m，总伸长 0.8m）。
    let cfg = GaitConfig::quadruped(GaitType::Trot, 2.0);
    let mut ctl = LocomotionController::new(cfg, hips());
    let ik = LegIK::new(0.4, 0.4);

    // 3) 向前行走 3 秒。
    let cmd = LocomotionCommand {
        linear_x: 0.5,
        linear_y: 0.0,
        angular_z: 0.0,
        body_height: 0.4,
    };
    let dt = 0.02;
    for i in 0..150 {
        let out = ctl.step(&cmd, dt);
        sim.set_velocity_command(Vec3::new(0.5, 0.0, 0.0), Vec3::ZERO)
            .unwrap();
        sim.step(dt).unwrap();
        if i % 50 == 0 {
            let st = sim.state();
            let front_range = sim.range(0.0);
            // 步态 -> IK -> 关节角（每条腿 [髋, 膝]）。
            let joint_repr = ctl
                .hip_positions
                .iter()
                .enumerate()
                .map(|(idx, &hip)| {
                    let q = ik.solve_from_hip(hip, out.foot_targets[idx]);
                    match q {
                        Ok([h, k]) => format!("leg{idx}=[{h:.2},{k:.2}]"),
                        Err(_) => format!("leg{idx}=[unreachable]"),
                    }
                })
                .collect::<Vec<_>>()
                .join(" ");
            println!(
                "[t {:>3}] pose=({:.2},{:.2}) range={:.2} collisions={} swing={}/4 | {}",
                i,
                st.robot_pose.position.x,
                st.robot_pose.position.y,
                front_range,
                st.collisions,
                out.swing_count(),
                joint_repr
            );
        }
    }

    // 4) 打印感知结果。
    let dets = sim.detections();
    println!(
        "detections: {} (class {} @ {:.1}m, bearing {:.2} rad)",
        dets.len(),
        dets.first().map(|d| d.class_id).unwrap_or(0),
        dets.first().map(|d| d.range_m).unwrap_or(0.0),
        dets.first().map(|d| d.bearing_rad).unwrap_or(0.0)
    );

    // 5) 全身控制（WBC）演示：静态站姿命令（躯干位姿 + 逐足体重分配）-> 关节角 + 足底力。
    let wbc = WholeBodyController::quadruped(ik, hips(), 0.4, 400.0).unwrap();
    let pose_cmd = WholeBodyCommand::stand(0.4).with_force_weights(vec![1.0, 1.0, 1.0, 2.0]); // 右后足承担更多体重
    match wbc.solve(&pose_cmd) {
        Ok(target) => {
            println!("\n[WBC] 站姿命令 -> 逐腿关节角 + 足底法向力：");
            for (i, (q, f)) in target
                .joint_targets
                .iter()
                .zip(&target.foot_forces)
                .enumerate()
            {
                println!(
                    "  leg{i}: joint=[{:.2},{:.2}]  foot={:.2}m  force={:.1}N",
                    q[0],
                    q[1],
                    target.foot_targets[i].norm(),
                    f.z
                );
            }
            let total: f32 = target.foot_forces.iter().map(|v| v.z).sum();
            println!("  Σ 足底力 = {total:.1}N（应≈体重 400N）");
            // 静力学下沉：足底力 -> 关节力矩（τ = Jᵀ·f）。
            match wbc.joint_torques(&target) {
                Ok(torques) => {
                    let t = torques
                        .iter()
                        .enumerate()
                        .map(|(i, q)| format!("leg{i}=[{:.1},{:.1}]Nm", q[0], q[1]))
                        .collect::<Vec<_>>()
                        .join(" ");
                    println!("  [WBC] 关节力矩（静力传递）: {t}");
                }
                Err(e) => println!("  [WBC] 关节力矩求解失败: {e}"),
            }
            // 躯干合力矩（静态平衡/防倾覆校验）。
            let (_f, m) = target.trunk_wrench();
            println!(
                "  [WBC] 躯干合力矩 = ({:.1},{:.1},{:.1}) N·m（|m|={:.1}，非对称载荷产生倾覆矩）",
                m.x,
                m.y,
                m.z,
                m.norm()
            );
        }
        Err(e) => println!("[WBC] 站姿命令求解失败: {e}"),
    }

    // 6) 逆动力学（RNEA）演示：给定运动（q, q̇, q̈）与重力，求维持该运动所需的关节力矩。
    let ld = LegDynamics::new(0.4, 0.4, 1.0, 1.0, 0.2, 0.2, 1.0 / 12.0, 1.0 / 12.0);
    let q = [0.6, -0.8];
    let qd = [1.0, 2.0];
    let qdd = [0.5, -1.0];
    let tau = ld.inverse_dynamics(q, qd, qdd, Vec3::ZERO, Vec3::new(0.0, 0.0, -9.81));
    println!("\n[RNEA] 逆动力学 q={q:?} q̇={qd:?} q̈={qdd:?}（含惯量+科氏/离心+重力）：");
    println!("  τ_hip={:.2} N·m   τ_knee={:.2} N·m", tau[0], tau[1]);

    // 7) 正向动力学（仿真）演示：给定力矩求关节角加速度，并做一小段欧拉积分。
    let qdd_f = ld.forward_dynamics(q, qd, tau, Vec3::ZERO, Vec3::new(0.0, 0.0, -9.81));
    println!(
        "\n[FD] 正向动力学（逆动力学的逆）：τ → q̈=[{:.3},{:.3}]（回环应还原输入 q̈={qdd:?}）",
        qdd_f[0], qdd_f[1]
    );
    // 欧拉积分一步后，再逆动力学应得到与输入力矩相近的 τ（回环自洽）。
    let dt = 1e-3f32;
    let q1 = [q[0] + qd[0] * dt, q[1] + qd[1] * dt];
    let qd1 = [qd[0] + qdd_f[0] * dt, qd[1] + qdd_f[1] * dt];
    let tau1 = ld.inverse_dynamics(q1, qd1, qdd_f, Vec3::ZERO, Vec3::new(0.0, 0.0, -9.81));
    println!(
        "  欧拉一步后 q≈[{:.3},{:.3}] q̇≈[{:.3},{:.3}]，再逆动力学 τ1=[{:.2},{:.2}]",
        q1[0], q1[1], qd1[0], qd1[1], tau1[0], tau1[1]
    );

    println!("步态+仿真闭环演示完成（向前行走，前方有墙则限位）。");
}
