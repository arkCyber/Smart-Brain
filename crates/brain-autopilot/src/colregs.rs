//! COLREGS（国际海上避碰规则）简化避让引擎 —— 水面艇会遇避让。
//!
//! 面向水面自动驾驶艇，把两艇会遇态势分类并给出避让动作建议：
//! - **对遇（Head-on）**：双方各向右转（Give Way Starboard）；
//! - **交叉（Crossing）**：本船右舷有来船时本船让路（右转），左舷有来船时保向；
//! - **追越（Overtaking）**：追越船让路（通常从右舷超越）；
//! - 距离足够远则“无风险 / 不动作”。
//!
//! 这是规则层（高层决策），具体由 `BoatAutopilot`/避障层执行。

use brain_kinematics::norm_angle;

/// 一艇的位姿（位置 + 航向）。
#[derive(Debug, Clone, Copy)]
pub struct VesselPose {
    pub x: f32,
    pub y: f32,
    pub heading: f32,
}

impl VesselPose {
    pub fn new(x: f32, y: f32, heading: f32) -> Self {
        Self { x, y, heading }
    }
}

/// 相遇态势分类。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncounterType {
    /// 对遇（相向而行）。
    HeadOn,
    /// 交叉相遇。
    Crossing,
    /// 追越。
    Overtaking,
    /// 无风险。
    NoRisk,
}

/// 避让动作建议。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColregsAction {
    /// 向右转向让路。
    GiveWayStarboard,
    /// 保向（直行，对方让路）。
    StandOn,
    /// 减速（必要时配合转向）。
    SlowDown,
    /// 不动作。
    None,
}

/// COLREGS 简化避让引擎。
pub struct Colregs;

/// 会遇判定参数。
pub struct ColregsParams {
    /// 超过该距离视为无风险（米）。
    pub risk_radius: f32,
    /// 追越判定：相对航向角阈值（rad）。
    pub overtake_heading_tol: f32,
    /// 对遇判定：相对航向角阈值（rad，接近 π 为对遇）。
    pub headon_heading_tol: f32,
}

impl Default for ColregsParams {
    fn default() -> Self {
        Self {
            risk_radius: 50.0,
            overtake_heading_tol: std::f32::consts::FRAC_PI_4, // 45°
            headon_heading_tol: std::f32::consts::FRAC_PI_4,   // 45°（与 π 的偏差）
        }
    }
}

impl Colregs {
    /// 分类两艇会遇态势，并给出本船（`own`）的动作建议。
    pub fn classify(
        own: VesselPose,
        other: VesselPose,
        p: &ColregsParams,
    ) -> (EncounterType, ColregsAction) {
        let dx = other.x - own.x;
        let dy = other.y - own.y;
        let dist = (dx * dx + dy * dy).sqrt();
        if dist > p.risk_radius {
            return (EncounterType::NoRisk, ColregsAction::None);
        }
        if dist < 1e-3 {
            // 几乎重合：保守向右转避让。
            return (EncounterType::Crossing, ColregsAction::GiveWayStarboard);
        }

        // 对方相对本船的方位（本船坐标系，-π..π；>0 表示在右舷）。
        let bearing = dy.atan2(dx);
        let rel_bearing = norm_angle(bearing - own.heading);
        // 对方相对航向（与 0 的差衡量是否同向/相向）。
        let rel_course = norm_angle(other.heading - own.heading);

        // 追越：对方大致同向且在其前。
        if rel_course.abs() < p.overtake_heading_tol {
            let other_ahead = rel_bearing.abs() < p.overtake_heading_tol;
            if other_ahead {
                // 本船在追越对方（同向前方）→ 本船让路，通常向右超越。
                return (EncounterType::Overtaking, ColregsAction::GiveWayStarboard);
            }
            return (EncounterType::NoRisk, ColregsAction::None);
        }

        // 对遇：相对航向接近 π（相向）。
        if (rel_course.abs() - std::f32::consts::PI).abs() < p.headon_heading_tol {
            // 双方各向右转（本船向右 = 右舷转）。
            return (EncounterType::HeadOn, ColregsAction::GiveWayStarboard);
        }

        // 交叉：对方在本船右舷（rel_bearing>0）→ 本船让路右转；在左舷 → 保向。
        if rel_bearing > 0.0 {
            (EncounterType::Crossing, ColregsAction::GiveWayStarboard)
        } else {
            (EncounterType::Crossing, ColregsAction::StandOn)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p() -> ColregsParams {
        ColregsParams::default()
    }

    #[test]
    fn head_on_both_give_way_starboard() {
        // 本船朝 +x，对方在正前方朝 -x 驶来 → 对遇，向右转。
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let other = VesselPose::new(10.0, 0.0, std::f32::consts::PI);
        let (t, a) = Colregs::classify(own, other, &p());
        assert_eq!(t, EncounterType::HeadOn);
        assert_eq!(a, ColregsAction::GiveWayStarboard);
    }

    #[test]
    fn crossing_give_way_when_other_on_starboard() {
        // 本船朝 +x，对方从右舷（+y 方向偏右前）驶来 → 本船让路右转。
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let other = VesselPose::new(8.0, 5.0, -1.1);
        let (t, a) = Colregs::classify(own, other, &p());
        assert_eq!(t, EncounterType::Crossing);
        assert_eq!(a, ColregsAction::GiveWayStarboard);
    }

    #[test]
    fn crossing_stand_on_when_other_on_port() {
        // 对方在左舷 → 本船保向（对方让路）。
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let other = VesselPose::new(8.0, -5.0, 1.1);
        let (t, a) = Colregs::classify(own, other, &p());
        assert_eq!(t, EncounterType::Crossing);
        assert_eq!(a, ColregsAction::StandOn);
    }

    #[test]
    fn overtaking_vessel_gives_way() {
        // 本船朝 +x，对方同向在前 → 本船追越，让路。
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let other = VesselPose::new(5.0, 0.0, 0.0);
        let (t, a) = Colregs::classify(own, other, &p());
        assert_eq!(t, EncounterType::Overtaking);
        assert_eq!(a, ColregsAction::GiveWayStarboard);
    }

    #[test]
    fn no_risk_when_far_away() {
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let other = VesselPose::new(100.0, 100.0, 0.5);
        let (t, a) = Colregs::classify(own, other, &p());
        assert_eq!(t, EncounterType::NoRisk);
        assert_eq!(a, ColregsAction::None);
    }
}
