//! COLREGS（国际海上避碰规则）简化避让引擎 —— 水面艇会遇避让。
//!
//! 面向水面自动驾驶艇，把两艇会遇态势分类并给出避让动作建议：
//! - **对遇（Head-on）**：双方各向右转（Give Way Starboard）；
//! - **交叉（Crossing）**：本船右舷有来船时本船让路（右转），左舷有来船时保向；
//!   **若对方是帆船而本船是机动船，机动船仍须让路（帆船优先通行权）**；
//! - **追越（Overtaking）**：追越船让路（通常从右舷超越）；
//! - **能见度受限（Restricted Visibility）**：双方均须安全航速并主动让路
//!   （Rule 19），不再区分让路/保向船；
//! - 距离足够远则“无风险 / 不动作”。
//!
//! 这是规则层（高层决策），具体由 `BoatAutopilot`/避障层执行。AIS 解码见
//! [`crate::ais`]，可把多艇目标喂给本引擎做会遇分类。

use brain_kinematics::norm_angle;

/// 船舶动力类型（影响交叉相遇的让路/保向判定）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Propulsion {
    /// 机动船（动力推进）：交叉相遇时须让帆船。
    PowerDriven,
    /// 帆船（在航，非机动船）：交叉相遇时具有优先通行权。
    Sailing,
}

/// 能见度状况。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Visibility {
    /// 能见度良好（互见）。
    Clear,
    /// 能见度受限（雾/霾/夜航等，Rule 19：双方均须主动让路、安全航速）。
    Restricted,
}

/// 一艇的位姿（位置 + 航向 + 动力类型）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VesselPose {
    pub x: f32,
    pub y: f32,
    pub heading: f32,
    pub propulsion: Propulsion,
}

impl VesselPose {
    /// 机动船位姿（默认动力推进）。
    pub fn new(x: f32, y: f32, heading: f32) -> Self {
        Self {
            x,
            y,
            heading,
            propulsion: Propulsion::PowerDriven,
        }
    }

    /// 帆船位姿。
    pub fn sailing(x: f32, y: f32, heading: f32) -> Self {
        Self {
            x,
            y,
            heading,
            propulsion: Propulsion::Sailing,
        }
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
    /// 能见度受限下的会遇（双方均须主动让路）。
    RestrictedVisibility,
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
    /// 能见度受限：以安全航速行驶（Rule 19）。
    ProceedSafeSpeed,
    /// 不动作。
    None,
}

/// 一次多目标会遇评估中，单个目标船的分类结果。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Encounter {
    /// 目标在输入数组中的下标。
    pub target_index: usize,
    /// 相遇态势。
    pub encounter: EncounterType,
    /// 建议动作。
    pub action: ColregsAction,
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
    /// 能见度状况（默认良好）。
    pub visibility: Visibility,
}

impl Default for ColregsParams {
    fn default() -> Self {
        Self {
            risk_radius: 50.0,
            overtake_heading_tol: std::f32::consts::FRAC_PI_4, // 45°
            headon_heading_tol: std::f32::consts::FRAC_PI_4,   // 45°（与 π 的偏差）
            visibility: Visibility::Clear,
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

        // 能见度受限（Rule 19）：双方均须安全航速并主动让路，不再判定让路/保向船。
        if p.visibility == Visibility::Restricted {
            let converging = rel_bearing.abs() < p.headon_heading_tol
                || (rel_course.abs() - std::f32::consts::PI).abs() < p.headon_heading_tol;
            return (
                EncounterType::RestrictedVisibility,
                if converging {
                    ColregsAction::GiveWayStarboard
                } else {
                    ColregsAction::ProceedSafeSpeed
                },
            );
        }

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

        // 交叉：对方在右舷 → 本船让路右转；
        //      对方在左舷 → 若对方是帆船且本船是机动船，机动船仍须让路（帆船优先）；
        //                   否则本船保向（对方让路）。
        let give_way = rel_bearing > 0.0
            || (other.propulsion == Propulsion::Sailing
                && own.propulsion == Propulsion::PowerDriven);
        if give_way {
            (EncounterType::Crossing, ColregsAction::GiveWayStarboard)
        } else {
            (EncounterType::Crossing, ColregsAction::StandOn)
        }
    }

    /// 对本船与多个目标船逐一分类，返回每条的分类结果（多目标协同避让输入）。
    pub fn classify_many(
        own: VesselPose,
        targets: &[VesselPose],
        params: &ColregsParams,
    ) -> Vec<Encounter> {
        targets
            .iter()
            .enumerate()
            .map(|(i, t)| {
                let (encounter, action) = Colregs::classify(own, *t, params);
                Encounter {
                    target_index: i,
                    encounter,
                    action,
                }
            })
            .collect()
    }

    /// 聚合多目标动作（优先级从高到低）：
    /// 任一目标要求本船让路 → 让路；否则受限能见度 → 安全航速；
    /// 否则存在保向 → 保向；否则减速；再否则不动作。
    pub fn aggregate(encounters: &[Encounter]) -> ColregsAction {
        if encounters
            .iter()
            .any(|e| e.action == ColregsAction::GiveWayStarboard)
        {
            return ColregsAction::GiveWayStarboard;
        }
        if encounters
            .iter()
            .any(|e| e.action == ColregsAction::ProceedSafeSpeed)
        {
            return ColregsAction::ProceedSafeSpeed;
        }
        if encounters
            .iter()
            .any(|e| e.action == ColregsAction::StandOn)
        {
            return ColregsAction::StandOn;
        }
        if encounters
            .iter()
            .any(|e| e.action == ColregsAction::SlowDown)
        {
            return ColregsAction::SlowDown;
        }
        ColregsAction::None
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

    #[test]
    fn restricted_visibility_give_way_when_converging() {
        // 能见度受限 + 正前方有船相向 → 双方均主动让路（右转），不再有保向船。
        let params = ColregsParams {
            visibility: Visibility::Restricted,
            ..ColregsParams::default()
        };
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let other = VesselPose::new(10.0, 0.0, std::f32::consts::PI); // 相向
        let (t, a) = Colregs::classify(own, other, &params);
        assert_eq!(t, EncounterType::RestrictedVisibility);
        assert_eq!(a, ColregsAction::GiveWayStarboard);
    }

    #[test]
    fn restricted_visibility_safe_speed_when_off_bearing() {
        // 能见度受限但对方不在会遇航向上 → 保持安全航速，不贸然转向。
        let params = ColregsParams {
            visibility: Visibility::Restricted,
            ..ColregsParams::default()
        };
        let own = VesselPose::new(0.0, 0.0, 0.0);
        // 对方在侧后方且同向（非对遇/追越圆锥内）。
        let other = VesselPose::new(-10.0, 20.0, 1.0);
        let (t, a) = Colregs::classify(own, other, &params);
        assert_eq!(t, EncounterType::RestrictedVisibility);
        assert_eq!(a, ColregsAction::ProceedSafeSpeed);
    }

    #[test]
    fn power_driven_gives_way_to_sailing_on_port() {
        // 对方（帆船）在本船左舷 → 本船（机动船）仍须让路（帆船优先通行权）。
        let own = VesselPose::new(0.0, 0.0, 0.0); // 机动船
        let other = VesselPose::sailing(8.0, -5.0, 1.1); // 左舷帆船
        let (t, a) = Colregs::classify(own, other, &p());
        assert_eq!(t, EncounterType::Crossing);
        assert_eq!(a, ColregsAction::GiveWayStarboard);
    }

    #[test]
    fn power_driven_stand_on_when_other_power_driven_on_port() {
        // 对方也是机动船且在左舷 → 本船保向（对方让路）。
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let other = VesselPose::new(8.0, -5.0, 1.1); // 默认机动船
        let (t, a) = Colregs::classify(own, other, &p());
        assert_eq!(t, EncounterType::Crossing);
        assert_eq!(a, ColregsAction::StandOn);
    }

    #[test]
    fn classify_many_and_aggregate_give_way() {
        // 本船朝 +x：目标 0 在右舷（须让路），目标 1 在左舷（保向）。
        // 聚合结果应为让路（只要有一个要求让路）。
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let targets = [
            VesselPose::new(8.0, 5.0, -1.1), // 右舷 → GiveWay
            VesselPose::new(8.0, -5.0, 1.1), // 左舷 → StandOn
        ];
        let enc = Colregs::classify_many(own, &targets, &p());
        assert_eq!(enc.len(), 2);
        assert_eq!(enc[0].action, ColregsAction::GiveWayStarboard);
        assert_eq!(enc[1].action, ColregsAction::StandOn);
        assert_eq!(Colregs::aggregate(&enc), ColregsAction::GiveWayStarboard);
    }

    #[test]
    fn aggregate_none_when_all_no_risk() {
        let own = VesselPose::new(0.0, 0.0, 0.0);
        let targets = [
            VesselPose::new(100.0, 100.0, 0.5),
            VesselPose::new(200.0, -50.0, 1.0),
        ];
        let enc = Colregs::classify_many(own, &targets, &p());
        assert_eq!(enc.len(), 2);
        assert_eq!(Colregs::aggregate(&enc), ColregsAction::None);
    }
}
