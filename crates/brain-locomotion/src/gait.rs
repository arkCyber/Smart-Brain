//! 周期步态相位生成：把机器人双腿/四足的周期性运动描述为逐腿相位。

/// 步态类型。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GaitType {
    /// 站立：所有腿处于支撑相。
    Stand,
    /// 行走：低负载因子，慢速。
    Walk,
    /// 小跑：对角步态，中等速度。
    Trot,
    /// 奔跑：低负载因子，高速。
    Run,
}

/// 步态参数。
#[derive(Debug, Clone)]
pub struct GaitConfig {
    /// 步态类型。
    pub gait: GaitType,
    /// 完整步态周期频率（Hz）。相位以此推进。
    pub cadence_hz: f32,
    /// 支撑相占整个周期的比例（0..1，其余为摆动相）。
    pub duty_factor: f32,
    /// 单步水平步长（m，用于足端轨迹）。
    pub step_length: f32,
    /// 摆动相足端抬升高度（m）。
    pub step_height: f32,
    /// 每条腿的相位偏移（0..1，相对整体周期）。
    pub phase_offset: Vec<f32>,
}

impl GaitConfig {
    /// 构造标准四足步态（腿序：FL/FR/HL/HR，即前左/前右/后左/后右）。
    ///
    /// - Walk：`duty=0.75`，相位 `[0, 0.5, 0.75, 0.25]`（典型四拍）。
    /// - Trot：`duty=0.5`，相位 `[0, 0.5, 0.5, 0]`（对角同相）。
    /// - Run：`duty=0.4`，同 Trot 对角。
    /// - Stand：`duty=1.0`，全部 0 相位（无摆动）。
    pub fn quadruped(gait: GaitType, cadence_hz: f32) -> Self {
        let (duty_factor, phase_offset) = match gait {
            GaitType::Stand => (1.0, vec![0.0, 0.0, 0.0, 0.0]),
            GaitType::Walk => (0.75, vec![0.0, 0.5, 0.75, 0.25]),
            GaitType::Trot => (0.5, vec![0.0, 0.5, 0.5, 0.0]),
            GaitType::Run => (0.4, vec![0.0, 0.5, 0.5, 0.0]),
        };
        Self {
            gait,
            cadence_hz,
            duty_factor,
            step_length: 0.3,
            step_height: 0.12,
            phase_offset,
        }
    }

    /// 构造双足步态（腿序：L/R）。`duty≈0.5`、两腿反相。
    pub fn biped(gait: GaitType, cadence_hz: f32) -> Self {
        Self {
            gait,
            cadence_hz,
            duty_factor: match gait {
                GaitType::Stand => 1.0,
                GaitType::Walk => 0.6,
                _ => 0.5,
            },
            step_length: 0.5,
            step_height: 0.2,
            phase_offset: vec![0.0, 0.5],
        }
    }
}

/// 某一时刻的逐腿步态相位。
#[derive(Debug, Clone)]
pub struct GaitPhase {
    /// 整体周期相位（0..1）。
    pub cycle: f32,
    /// 每条腿的相位（0..1）。
    pub legs: Vec<f32>,
    /// 每条腿是否处于摆动相（支撑相之外）。
    pub swing: Vec<bool>,
}

impl GaitPhase {
    /// 摆动相腿的数量。
    pub fn swing_count(&self) -> usize {
        self.swing.iter().filter(|&&s| s).count()
    }

    /// 是否所有腿都处于支撑相（站立）。
    pub fn all_stance(&self) -> bool {
        self.swing.iter().all(|&s| !s)
    }
}

/// 步态生成器：按节拍推进整体相位，并派生出逐腿相位与摆动标志。
pub struct GaitGenerator {
    cfg: GaitConfig,
    /// 整体周期相位（0..1）。
    phase: f32,
}

impl GaitGenerator {
    /// 以给定配置创建生成器，初始相位为 0。
    pub fn new(cfg: GaitConfig) -> Self {
        Self { cfg, phase: 0.0 }
    }

    /// 推进 `dt` 秒。相位按 `cadence_hz` 累加并在 [0,1) 内取余。
    pub fn advance(&mut self, dt: f32) {
        let d = (self.cfg.cadence_hz * dt).max(0.0);
        self.phase = (self.phase + d).fract();
    }

    /// 把整体相位跳到指定值（测试/同步用）。
    pub fn set_phase(&mut self, phase: f32) {
        self.phase = phase.fract();
    }

    /// 当前整体相位。
    pub fn phase(&self) -> f32 {
        self.phase
    }

    /// 计算当前逐腿相位快照。
    pub fn snapshot(&self) -> GaitPhase {
        let legs = self
            .cfg
            .phase_offset
            .iter()
            .map(|o| (self.phase + o).fract())
            .collect::<Vec<f32>>();
        let swing = legs
            .iter()
            .map(|&p| p >= self.cfg.duty_factor)
            .collect::<Vec<bool>>();
        GaitPhase {
            cycle: self.phase,
            legs,
            swing,
        }
    }

    /// 步态配置。
    pub fn config(&self) -> &GaitConfig {
        &self.cfg
    }

    /// 腿的数量。
    pub fn leg_count(&self) -> usize {
        self.cfg.phase_offset.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trot_diagonal_pairs_in_phase() {
        let g = GaitConfig::quadruped(GaitType::Trot, 2.0);
        let mut gen = GaitGenerator::new(g);
        // 对角对：FL(0) 与 HR(0) 同相；FR(0.5) 与 HL(0.5) 同相。
        // 在 cycle=0.6 时，FL/HR 相位=0.6（摆动相 duty=0.5），FR/HL 相位=0.1（支撑）。
        gen.set_phase(0.6);
        let ph = gen.snapshot();
        assert_eq!(ph.legs.len(), 4);
        assert!((ph.legs[0] - ph.legs[3]).abs() < 1e-6);
        assert!((ph.legs[1] - ph.legs[2]).abs() < 1e-6);
        // 对角对同相摆动：FL 与 HR 在摆，FR 与 HL 在支撑。
        assert!(ph.swing[0] && ph.swing[3]);
        assert!(!ph.swing[1] && !ph.swing[2]);
        assert_eq!(ph.swing_count(), 2);
    }

    #[test]
    fn stand_has_no_swing() {
        let g = GaitConfig::quadruped(GaitType::Stand, 1.0);
        let mut gen = GaitGenerator::new(g);
        gen.advance(1.0); // 完整周期
        let ph = gen.snapshot();
        assert!(ph.all_stance());
        assert_eq!(ph.swing_count(), 0);
    }

    #[test]
    fn phase_wraps_after_full_cycle() {
        let g = GaitConfig::quadruped(GaitType::Walk, 1.0);
        let mut gen = GaitGenerator::new(g);
        gen.advance(0.5);
        assert!((gen.phase() - 0.5).abs() < 1e-6);
        gen.advance(1.0); // 累加 1.0 -> 取余回到 0.5
        assert!((gen.phase() - 0.5).abs() < 1e-6);
    }

    #[test]
    fn biped_legs_opposite_phase() {
        let g = GaitConfig::biped(GaitType::Walk, 1.5);
        let mut gen = GaitGenerator::new(g);
        gen.set_phase(0.1);
        let ph = gen.snapshot();
        assert_eq!(ph.legs.len(), 2);
        let diff = (ph.legs[0] - ph.legs[1]).abs();
        // 两腿反相：相位差 0.5（含 0 环绕）。
        assert!(diff > 0.49 && diff < 0.51);
    }
}
