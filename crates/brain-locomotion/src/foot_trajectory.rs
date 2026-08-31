//! 足端轨迹生成：把“腿相位 + 身体速度”映射为相对髋部的足端位置。
//!
//! 模型（局部坐标系，x 前向、y 侧向、z 向上）：
//! - **支撑相**：足端相对地面固定，身体前进 ⇒ 相对髋部向后扫过 `body_vel / cadence`；
//! - **摆动相**：足端从最后方抬升、前摆到最前方，使用正弦抬升避免突变。

use brain_core::Vec3;

/// 足端轨迹生成器。
#[derive(Debug, Clone)]
pub struct FootTrajectory {
    /// 单步水平步长（m）。
    pub step_length: f32,
    /// 摆动相抬升高度（m）。
    pub step_height: f32,
}

impl FootTrajectory {
    pub fn new(step_length: f32, step_height: f32) -> Self {
        Self {
            step_length,
            step_height,
        }
    }

    /// 计算某腿在相位 `phase`（0..1）、支撑占比 `duty`（0..1）下的足端相对
    /// 偏移（相对该腿的名义站立点，局部系）。`body_vel_x` 为前向速度（m/s），
    /// `cadence_hz` 为步态频率（Hz，用于把支撑相的后扫折算成位移）。
    ///
    /// 返回的 z 为抬升高度（摆动相 > 0，支撑相 ≈ 0）。
    pub fn foot_offset(&self, phase: f32, duty: f32, body_vel_x: f32, cadence_hz: f32) -> Vec3 {
        let duty = duty.clamp(0.0, 1.0);
        let p = phase.fract();
        if p < duty {
            // 支撑相：足端固定，身体前移 → 相对身体后移。
            // 一个周期内身体前移 = body_vel / cadence；支撑相只占 duty，故全幅后扫。
            let t = p / duty.max(1e-6);
            let x = -body_vel_x / cadence_hz.max(1e-6) * t;
            Vec3::new(x, 0.0, 0.0)
        } else {
            // 摆动相：从最末位抬升并前摆到最前位。
            let span = (1.0 - duty).max(1e-6);
            let t = (p - duty) / span;
            let x = -self.step_length / 2.0 + self.step_length * t;
            let z = self.step_height * (std::f32::consts::PI * t).sin();
            Vec3::new(x, 0.0, z)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stance_moves_back_with_velocity() {
        let ft = FootTrajectory::new(0.3, 0.12);
        // 支撑相起点：足端刚着地，相对身体处于最前。
        let start = ft.foot_offset(0.0, 0.5, 1.0, 2.0);
        assert!(start.z.abs() < 1e-6, "stance should not lift");
        assert!(start.x < 1e-6, "start slightly forward/zero");
        // 支撑相末尾：足端后扫到最远。
        let end = ft.foot_offset(0.49, 0.5, 1.0, 2.0);
        assert!(end.x < start.x, "foot should move backward during stance");
    }

    #[test]
    fn swing_lifts_and_advances() {
        let ft = FootTrajectory::new(0.3, 0.12);
        let mid = ft.foot_offset(0.75, 0.5, 1.0, 2.0);
        // 摆动相中点：足端抬至最高，前向接近 0（从中点到前）。
        assert!(mid.z > 0.10, "swing should lift, got z={}", mid.z);
        // 摆动相末尾（接近触地）：足端已前移到最前方，抬升回到接近 0。
        let land = ft.foot_offset(0.999, 0.5, 1.0, 2.0);
        assert!(land.z < 0.01, "landing should be near ground, z={}", land.z);
        assert!(land.x > 0.0, "foot should be forward at touchdown");
        // 触地进入支撑相起点（p≈0 环绕回相位 0）：抬升严格为 0。
        let stance_start = ft.foot_offset(0.0, 0.5, 1.0, 2.0);
        assert!(
            (stance_start.z).abs() < 1e-6,
            "stance start z={}",
            stance_start.z
        );
    }

    #[test]
    fn no_swing_when_standing() {
        let ft = FootTrajectory::new(0.3, 0.12);
        // duty=1.0：全部支撑，永不抬升。
        let off = ft.foot_offset(0.5, 1.0, 0.0, 1.0);
        assert!(off.z.abs() < 1e-6);
        assert!(off.x.abs() < 1e-6);
    }

    #[test]
    fn swing_peak_height_is_step_height() {
        let ft = FootTrajectory::new(0.3, 0.12);
        // 摆动相中点 t=0.5 -> sin(pi/2)=1 -> z = step_height。
        let mid = ft.foot_offset(0.75, 0.5, 0.0, 1.0);
        assert!((mid.z - 0.12).abs() < 1e-5);
    }
}
