//! 步态控制器：把“大脑”的高层运动指令编排成逐腿足端落点与躯干姿态目标。

use brain_core::Vec3;

use crate::foot_trajectory::FootTrajectory;
use crate::gait::{GaitConfig, GaitGenerator};

/// 高层运动指令（大脑 → 步态层）。
#[derive(Debug, Clone, Copy)]
pub struct LocomotionCommand {
    /// 期望前向速度（m/s）。
    pub linear_x: f32,
    /// 期望侧向速度（m/s）。
    pub linear_y: f32,
    /// 期望转向角速度（rad/s，绕竖轴）。
    pub angular_z: f32,
    /// 期望躯干高度（m，地面到躯干）。
    pub body_height: f32,
}

impl LocomotionCommand {
    /// 原地站立。
    pub fn stand(height: f32) -> Self {
        Self {
            linear_x: 0.0,
            linear_y: 0.0,
            angular_z: 0.0,
            body_height: height,
        }
    }
}

/// 一次步态控制输出的逐腿足端与躯干目标。
#[derive(Debug, Clone)]
pub struct LocomotionOutput {
    /// 每条腿的足端目标（局部系，相对该腿髋部的 x/z，y 侧向由名义位姿给出）。
    pub foot_offsets: Vec<Vec3>,
    /// 每条腿的**绝对足端位置**（机体系，x 前向、y 侧向、z 向上）。
    /// 直接喂给 [`crate::leg_ik::LegIK::solve_from_hip`] 求关节角。
    pub foot_targets: Vec<Vec3>,
    /// 每条腿是否处于摆动相。
    pub swing: Vec<bool>,
    /// 躯干前倾（sagittal lean，rad）。加速时前倾、减速时后仰。
    pub body_pitch: f32,
    /// 躯干高度（m）。
    pub body_height: f32,
    /// 转向时躯干侧倾的符号（-1/0/+1，可作横滚方向提示）。
    pub turn_direction: i32,
}

impl LocomotionOutput {
    /// 摆动腿数量。
    pub fn swing_count(&self) -> usize {
        self.swing.iter().filter(|&&s| s).count()
    }

    /// 是否所有腿都在支撑相（站立）。
    pub fn all_stance(&self) -> bool {
        self.swing.iter().all(|&s| !s)
    }
}

/// 步态控制器：内部持有一个 [`GaitGenerator`]，按指令逐拍推进并输出足端/躯干目标。
pub struct LocomotionController {
    pub gait: GaitGenerator,
    pub foot: FootTrajectory,
    /// 每条腿的名义髋部位置（局部系），用于叠加侧向偏置。
    pub hip_positions: Vec<Vec3>,
    /// 每条腿的名义足端位置（机体系，静止时足端在髋下方）。
    pub foot_positions: Vec<Vec3>,
    /// 躯干最大前倾（rad）。
    pub max_lean: f32,
}

impl LocomotionController {
    /// 创建控制器。`leg_positions` 为每条腿名义髋部位置（局部系）。
    /// 名义足端位置默认为髋下方 `DEFAULT_LEG_REACH` 处，可用
    /// [`with_foot_positions`](Self::with_foot_positions) 覆盖。
    pub fn new(cfg: GaitConfig, leg_positions: Vec<Vec3>) -> Self {
        let foot = FootTrajectory::new(cfg.step_length, cfg.step_height);
        let foot_positions = leg_positions
            .iter()
            .map(|h| Vec3::new(h.x, h.y, h.z - Self::DEFAULT_LEG_REACH))
            .collect();
        Self {
            gait: GaitGenerator::new(cfg),
            foot,
            hip_positions: leg_positions,
            foot_positions,
            max_lean: 0.3,
        }
    }

    /// 默认腿部伸长（名义足端相对髋的下沉距离，m）。
    pub const DEFAULT_LEG_REACH: f32 = 0.4;

    /// 覆盖名义足端位置（机体系）。
    pub fn with_foot_positions(mut self, foot_positions: Vec<Vec3>) -> Self {
        self.foot_positions = foot_positions;
        self
    }

    /// 推进一步：先推进步态相位，再按指令计算足端与躯干目标。
    pub fn step(&mut self, cmd: &LocomotionCommand, dt: f32) -> LocomotionOutput {
        self.gait.advance(dt);
        let ph = self.gait.snapshot();
        let cfg = self.gait.config();
        let duty = cfg.duty_factor;
        let cadence = cfg.cadence_hz;

        let n = ph.legs.len();
        let mut foot_offsets = Vec::with_capacity(n);
        let mut foot_targets = Vec::with_capacity(n);
        for i in 0..n {
            // 基础足端相位偏移（支撑后扫 / 摆动抬升前摆）。
            let off = self
                .foot
                .foot_offset(ph.legs[i], duty, cmd.linear_x, cadence);
            // 叠加该腿的名义侧向位置，并附上侧向指令的一阶影响。
            let hip = self.hip_positions.get(i).copied().unwrap_or(Vec3::ZERO);
            foot_offsets.push(Vec3::new(hip.x + off.x, hip.y + off.y, hip.z + off.z));
            // 绝对足端位置（机体系）：名义足端 + 步态偏移。
            let nominal = self.foot_positions.get(i).copied().unwrap_or(Vec3::ZERO);
            foot_targets.push(Vec3::new(
                nominal.x + off.x,
                nominal.y + off.y,
                nominal.z + off.z,
            ));
        }

        // 躯干前倾：与纵向加速度方向相反（加速前倾、减速后仰），带饱和。
        let lean_signal = cmd.linear_x.clamp(-self.max_lean, self.max_lean);
        let body_pitch = -lean_signal * 0.15;

        let turn_direction = if cmd.angular_z > 1e-3 {
            1
        } else if cmd.angular_z < -1e-3 {
            -1
        } else {
            0
        };

        LocomotionOutput {
            foot_offsets,
            foot_targets,
            swing: ph.swing,
            body_pitch,
            body_height: cmd.body_height,
            turn_direction,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gait::GaitType;

    fn hips() -> Vec<Vec3> {
        vec![
            Vec3::new(0.15, 0.12, -0.4),   // FL
            Vec3::new(0.15, -0.12, -0.4),  // FR
            Vec3::new(-0.15, 0.12, -0.4),  // HL
            Vec3::new(-0.15, -0.12, -0.4), // HR
        ]
    }

    #[test]
    fn stand_output_all_stance() {
        let cfg = GaitConfig::quadruped(GaitType::Stand, 1.0);
        let mut ctl = LocomotionController::new(cfg, hips());
        let out = ctl.step(&LocomotionCommand::stand(0.4), 0.0);
        assert!(out.all_stance());
        assert_eq!(out.swing_count(), 0);
        assert_eq!(out.body_height, 0.4);
        assert_eq!(out.turn_direction, 0);
    }

    #[test]
    fn trot_produces_two_swinging_feet() {
        let cfg = GaitConfig::quadruped(GaitType::Trot, 2.0);
        let mut ctl = LocomotionController::new(cfg, hips());
        let cmd = LocomotionCommand {
            linear_x: 0.5,
            linear_y: 0.0,
            angular_z: 0.0,
            body_height: 0.4,
        };
        // 推进足够时间覆盖整相。
        for _ in 0..100 {
            let out = ctl.step(&cmd, 0.01);
            // 任意时刻摆动腿数应为 2（trot 对角步态）。
            assert_eq!(out.swing_count(), 2, "trot should keep 2 legs swinging");
            assert_eq!(out.foot_offsets.len(), 4);
        }
    }

    #[test]
    fn forward_velocity_leans_pitch() {
        let cfg = GaitConfig::quadruped(GaitType::Trot, 2.0);
        let mut ctl = LocomotionController::new(cfg, hips());
        let fwd = ctl.step(
            &LocomotionCommand {
                linear_x: 1.0,
                linear_y: 0.0,
                angular_z: 0.0,
                body_height: 0.4,
            },
            0.0,
        );
        // 加速前倾 => body_pitch 为负（前低）。
        assert!(fwd.body_pitch < 0.0);
        let stand = ctl.step(&LocomotionCommand::stand(0.4), 0.0);
        // 站立应接近水平。
        assert!(stand.body_pitch.abs() < 1e-6);
    }

    #[test]
    fn turning_reports_direction() {
        let cfg = GaitConfig::quadruped(GaitType::Walk, 1.5);
        let mut ctl = LocomotionController::new(cfg, hips());
        let left = ctl.step(
            &LocomotionCommand {
                linear_x: 0.2,
                linear_y: 0.0,
                angular_z: 0.5,
                body_height: 0.4,
            },
            0.0,
        );
        assert_eq!(left.turn_direction, 1);
        let right = ctl.step(
            &LocomotionCommand {
                linear_x: 0.2,
                linear_y: 0.0,
                angular_z: -0.7,
                body_height: 0.4,
            },
            0.0,
        );
        assert_eq!(right.turn_direction, -1);
    }

    #[test]
    fn lateral_hips_are_preserved() {
        let cfg = GaitConfig::quadruped(GaitType::Trot, 2.0);
        let mut ctl = LocomotionController::new(cfg, hips());
        let out = ctl.step(
            &LocomotionCommand {
                linear_x: 0.0,
                linear_y: 0.0,
                angular_z: 0.0,
                body_height: 0.4,
            },
            0.0,
        );
        // 支撑相时足端 y 应保留髋部侧向符号（左右分开）。
        assert!(out.foot_offsets[0].y > 0.0 && out.foot_offsets[1].y < 0.0);
        assert!(out.foot_offsets[2].y > 0.0 && out.foot_offsets[3].y < 0.0);
    }

    #[test]
    fn foot_targets_are_ik_solvable() {
        use crate::leg_ik::LegIK;
        // 步态输出 -> 2 连杆 IK 求解，全部腿都可达。
        let cfg = GaitConfig::quadruped(GaitType::Trot, 2.0);
        let mut ctl = LocomotionController::new(cfg, hips());
        let ik = LegIK::new(0.4, 0.4); // 总伸长 0.8 > 名义 0.4 可达
        let cmd = LocomotionCommand {
            linear_x: 0.5,
            linear_y: 0.0,
            angular_z: 0.0,
            body_height: 0.4,
        };
        for _ in 0..100 {
            let out = ctl.step(&cmd, 0.01);
            assert_eq!(out.foot_targets.len(), 4);
            for i in 0..4 {
                let q = ik.solve_from_hip(ctl.hip_positions[i], out.foot_targets[i]);
                // 足端目标应在可达范围内，IK 必须成功。
                assert!(q.is_ok(), "leg {i} IK failed: {q:?}");
            }
        }
    }
}
