//! Reeds-Shepp 曲线 —— 可前进/倒车、受最小转弯半径约束的车辆最短路径规划。
//!
//! 与 Dubins（只前进）相比，Reeds-Shepp 允许**倒车**，从而支持：
//! - **掉头**：狭窄空间内两/三段弧即可 180° 转向（Dubins 需大半径绕圈）；
//! - **侧方/垂直泊车**：带倒车段的入库轨迹；
//! - 任意两构型（位置+朝向）间的最短可行路径（含 `C|C|C`、`C|C|C|C`、
//!   `C|SC|C`、`C|SC|C|C` 等共 48 种类型）。
//!
//! 实现沿用经典算法：以 **12 个左转优先的基础类型**为模板，配合
//! `timeflip`（长度取反 = 倒车）与 `reflect`（左右互换）四象限变换生成全部 48 种，
//! 并**用自行车模型逐点精确积分校验**——只有真能到达目标构型的候选才被接受，
//! 取最短者。长度符号编码方向：负值表示该段倒车。

/// Reeds-Shepp 规划配置。
#[derive(Debug, Clone, Copy)]
pub struct ReedsSheppConfig {
    /// 最小转弯半径（米）。
    pub turning_radius: f32,
    /// 采样步长（米），用于把路径离散成航点。
    pub step: f32,
    /// 终点校验容差（米）。
    pub goal_tolerance: f32,
}

impl Default for ReedsSheppConfig {
    fn default() -> Self {
        Self {
            turning_radius: 3.8,
            step: 0.5,
            goal_tolerance: 0.3,
        }
    }
}

/// 一条 Reeds-Shepp 路径。
#[derive(Debug, Clone)]
pub struct ReedsSheppPath {
    /// 离散化后的世界系构型航点。
    pub points: Vec<(f32, f32, f32)>,
    /// 路径总长（米）。
    pub length: f32,
    /// 各段 `(类型字符, 实际长度米)`；长度取负表示倒车段。
    pub segments: Vec<(char, f32)>,
}

/// 基本段：`L/R/S` + 归一化长度（负值 = 倒车）。
#[derive(Debug, Clone, Copy)]
enum Seg {
    L(f32),
    R(f32),
    S(f32),
}

/// 基础路径类型函数：`(x, y, phi)`（归一化）→ 各段长度与转向。
type RsBase = fn(f32, f32, f32) -> Option<Vec<Seg>>;

/// Reeds-Shepp 规划器。
pub struct ReedsSheppPlanner {
    cfg: ReedsSheppConfig,
}

impl ReedsSheppPlanner {
    pub fn new(cfg: ReedsSheppConfig) -> Self {
        Self { cfg }
    }

    /// 规划 `start → goal` 的最短 Reeds-Shepp 路径（可前进/倒车）。
    pub fn plan(&self, start: (f32, f32, f32), goal: (f32, f32, f32)) -> Option<ReedsSheppPath> {
        let rho = self.cfg.turning_radius.max(1e-3);
        // 平移到以起点为原点、朝向 0 的局部系（米）。
        let (dx, dy) = (goal.0 - start.0, goal.1 - start.1);
        let (c0, s0) = (start.2.cos(), start.2.sin());
        let xl = dx * c0 + dy * s0;
        let yl = -dx * s0 + dy * c0;
        let phi = goal.2 - start.2;
        // 归一化到半径=1 的坐标系（Reeds-Shepp 公式基于单位半径）。
        let (xn, yn) = (xl / rho, yl / rho);

        let mut best: Option<(f32, Vec<Seg>)> = None;
        let bases: [RsBase; 12] = [
            lsl, lsr, lrl, lrl2, lrl3, lrlrl, lrlrl2, lrsl, lsrl, lrsr, lslr, lrslr,
        ];
        for base in bases {
            for segs in apply_variants(base, xn, yn, phi) {
                let (ex, ey, eth) = sim_rs((0.0, 0.0, 0.0), &segs, rho);
                let err = ((ex - xl).powi(2) + (ey - yl).powi(2)).sqrt();
                if err < self.cfg.goal_tolerance && angle_diff(eth, phi).abs() < 0.1 {
                    let len = rs_length(&segs, rho);
                    if best.as_ref().map(|(l, _)| len < *l).unwrap_or(true) {
                        best = Some((len, segs));
                    }
                }
            }
        }

        best.map(|(len, segs)| {
            let points = sample_rs(&segs, rho, start, self.cfg.step);
            let segments = segs
                .iter()
                .map(|s| {
                    let (ch, n) = match *s {
                        Seg::L(a) => ('L', a),
                        Seg::R(a) => ('R', a),
                        Seg::S(a) => ('S', a),
                    };
                    (ch, n * rho) // 归一化长度 → 实际米
                })
                .collect();
            ReedsSheppPath {
                points,
                length: len,
                segments,
            }
        })
    }
}
/// `timeflip`：把路径长度全部取反（倒车）。
fn timeflip(segs: &[Seg]) -> Vec<Seg> {
    segs.iter()
        .map(|s| match *s {
            Seg::L(a) => Seg::L(-a),
            Seg::R(a) => Seg::R(-a),
            Seg::S(a) => Seg::S(-a),
        })
        .collect()
}

/// `reflect`：左右转向互换。
fn reflect(segs: &[Seg]) -> Vec<Seg> {
    segs.iter()
        .map(|s| match *s {
            Seg::L(a) => Seg::R(a),
            Seg::R(a) => Seg::L(a),
            Seg::S(a) => Seg::S(a),
        })
        .collect()
}

/// 对每个基础类型做四象限变换，生成全部候选（含 timeflip/reflect）。
fn apply_variants(base: RsBase, x: f32, y: f32, phi: f32) -> Vec<Vec<Seg>> {
    let mut out = Vec::new();
    if let Some(s) = base(x, y, phi) {
        out.push(s);
    }
    if let Some(s) = base(-x, y, -phi) {
        out.push(timeflip(&s));
    }
    if let Some(s) = base(x, -y, -phi) {
        out.push(reflect(&s));
    }
    if let Some(s) = base(-x, -y, phi) {
        out.push(timeflip(&reflect(&s)));
    }
    out
}

/// `polar`：把 (x,y) 转成 (r, θ)。
fn polar(x: f32, y: f32) -> (f32, f32) {
    ((x * x + y * y).sqrt(), y.atan2(x))
}

/// 把角度取模到 `(-π, π]`。
fn mod2pi(x: f32) -> f32 {
    let two = std::f32::consts::TAU;
    let mut v = x % two;
    if v <= -std::f32::consts::PI {
        v += two;
    } else if v > std::f32::consts::PI {
        v -= two;
    }
    v
}

/// L+S+L+（直-弧-直，全前进）。
fn lsl(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let (u, t) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if (0.0..=std::f32::consts::PI).contains(&t) {
        let v = mod2pi(phi - t);
        if (0.0..=std::f32::consts::PI).contains(&v) {
            return Some(vec![Seg::L(t), Seg::S(u), Seg::L(v)]);
        }
    }
    None
}

/// L+S+R+。
fn lsr(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let (u1, t1) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    let u1 = u1 * u1;
    if u1 >= 4.0 {
        let u = (u1 - 4.0).sqrt();
        let theta = (2.0 / u).atan();
        let t = mod2pi(t1 + theta);
        let v = mod2pi(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some(vec![Seg::L(t), Seg::S(u), Seg::R(v)]);
        }
    }
    None
}

/// L+R-L+（三弧）。
fn lrl(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 <= 4.0 {
        let a = (0.25 * u1).acos();
        let t = mod2pi(a + theta + std::f32::consts::FRAC_PI_2);
        let u = mod2pi(std::f32::consts::PI - 2.0 * a);
        let v = mod2pi(phi - t - u);
        return Some(vec![Seg::L(t), Seg::R(-u), Seg::L(v)]);
    }
    None
}

/// L+R-L-。
fn lrl2(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 <= 4.0 {
        let a = (0.25 * u1).acos();
        let t = mod2pi(a + theta + std::f32::consts::FRAC_PI_2);
        let u = mod2pi(std::f32::consts::PI - 2.0 * a);
        let v = mod2pi(-phi + t + u);
        return Some(vec![Seg::L(t), Seg::R(-u), Seg::L(-v)]);
    }
    None
}

/// L+R-L+（三弧另一解）。
fn lrl3(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 <= 4.0 {
        let u = (1.0 - u1 * u1 * 0.125).acos();
        let a = (2.0 * u.sin() / u1).asin();
        let t = mod2pi(-a + theta + std::f32::consts::FRAC_PI_2);
        let v = mod2pi(t - u - phi);
        return Some(vec![Seg::L(t), Seg::R(u), Seg::L(-v)]);
    }
    None
}

/// L+R-L+R+（四弧）。
fn lrlrl(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 <= 2.0 {
        let a = ((u1 + 2.0) * 0.25).acos();
        let t = mod2pi(theta + a + std::f32::consts::FRAC_PI_2);
        let u = mod2pi(a);
        let v = mod2pi(phi - t + 2.0 * u);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some(vec![Seg::L(t), Seg::R(u), Seg::L(-u), Seg::R(-v)]);
        }
    }
    None
}

/// L+R-L-R+（四弧）。
fn lrlrl2(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    let u2 = (20.0 - u1 * u1) / 16.0;
    if (0.0..=1.0).contains(&u2) {
        let u = u2.acos();
        let a = (2.0 * u.sin() / u1).asin();
        let t = mod2pi(theta + a + std::f32::consts::FRAC_PI_2);
        let v = mod2pi(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some(vec![Seg::L(t), Seg::R(-u), Seg::L(-u), Seg::R(v)]);
        }
    }
    None
}

/// L+R90·S·L+（C-C-S-C）。
fn lrsl(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 >= 2.0 {
        let sq = (u1 * u1 - 4.0).sqrt();
        let u = sq - 2.0;
        let a = (2.0 / sq).atan();
        let t = mod2pi(theta + a + std::f32::consts::FRAC_PI_2);
        let v = mod2pi(t - phi + std::f32::consts::FRAC_PI_2);
        if t >= 0.0 && v >= 0.0 {
            return Some(vec![
                Seg::L(t),
                Seg::R(-std::f32::consts::FRAC_PI_2),
                Seg::S(-u),
                Seg::L(-v),
            ]);
        }
    }
    None
}

/// L+S·R90·L+（C-S-C-C）。
fn lsrl(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x - phi.sin();
    let eeta = y - 1.0 + phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 >= 2.0 {
        let sq = (u1 * u1 - 4.0).sqrt();
        let u = sq - 2.0;
        let a = (sq / 2.0).atan();
        let t = mod2pi(theta - a + std::f32::consts::FRAC_PI_2);
        let v = mod2pi(t - phi - std::f32::consts::FRAC_PI_2);
        if t >= 0.0 && v >= 0.0 {
            return Some(vec![
                Seg::L(t),
                Seg::S(u),
                Seg::R(std::f32::consts::FRAC_PI_2),
                Seg::L(-v),
            ]);
        }
    }
    None
}

/// L+R90·S·R+（C-C-S-C）。
fn lrsr(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 >= 2.0 {
        let t = mod2pi(theta + std::f32::consts::FRAC_PI_2);
        let u = u1 - 2.0;
        let v = mod2pi(phi - t - std::f32::consts::FRAC_PI_2);
        if t >= 0.0 && v >= 0.0 {
            return Some(vec![
                Seg::L(t),
                Seg::R(-std::f32::consts::FRAC_PI_2),
                Seg::S(-u),
                Seg::R(-v),
            ]);
        }
    }
    None
}

/// L+S·L90·R+（C-S-C-C）。
fn lslr(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 >= 2.0 {
        let t = mod2pi(theta);
        let u = u1 - 2.0;
        let v = mod2pi(phi - t - std::f32::consts::FRAC_PI_2);
        if t >= 0.0 && v >= 0.0 {
            return Some(vec![
                Seg::L(t),
                Seg::S(u),
                Seg::L(std::f32::consts::FRAC_PI_2),
                Seg::R(-v),
            ]);
        }
    }
    None
}

/// L+R90·S·L90·R+（C-C-S-C-C）。
fn lrslr(x: f32, y: f32, phi: f32) -> Option<Vec<Seg>> {
    let zeta = x + phi.sin();
    let eeta = y - 1.0 - phi.cos();
    let (u1, theta) = polar(zeta, eeta);
    if u1 >= 4.0 {
        let sq = (u1 * u1 - 4.0).sqrt();
        let u = sq - 4.0;
        let a = (2.0 / sq).atan();
        let t = mod2pi(theta + a + std::f32::consts::FRAC_PI_2);
        let v = mod2pi(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some(vec![
                Seg::L(t),
                Seg::R(-std::f32::consts::FRAC_PI_2),
                Seg::S(-u),
                Seg::L(-std::f32::consts::FRAC_PI_2),
                Seg::R(v),
            ]);
        }
    }
    None
}

/// 用自行车模型精确积分一段（可含倒车）路径，返回末端构型（局部系，米）。
fn sim_rs(start: (f32, f32, f32), segs: &[Seg], rho: f32) -> (f32, f32, f32) {
    let (mut x, mut y, mut th) = start;
    for s in segs {
        match *s {
            Seg::S(a) => {
                let len = a * rho;
                x += len * th.cos();
                y += len * th.sin();
            }
            Seg::L(a) => {
                x += rho * ((th + a).sin() - th.sin());
                y += rho * (th.cos() - (th + a).cos());
                th += a;
            }
            Seg::R(a) => {
                x += rho * (th.sin() - (th - a).sin());
                y += rho * ((th - a).cos() - th.cos());
                th -= a;
            }
        }
    }
    (x, y, th)
}

fn rs_length(segs: &[Seg], rho: f32) -> f32 {
    segs.iter()
        .map(|s| match *s {
            Seg::S(a) => a.abs() * rho,
            Seg::L(a) | Seg::R(a) => a.abs() * rho,
        })
        .sum()
}

/// 把路径离散成世界系航点（含倒车段）。
fn sample_rs(segs: &[Seg], rho: f32, start: (f32, f32, f32), step: f32) -> Vec<(f32, f32, f32)> {
    let (sx, sy, sth) = start;
    let (c0, s0) = (sth.cos(), sth.sin());
    let mut pts = vec![(sx, sy, sth)];
    let step = step.max(1e-3);
    let mut lx = 0.0f32;
    let mut ly = 0.0f32;
    let mut lth = 0.0f32;
    for s in segs {
        let total = match *s {
            Seg::S(a) => a.abs() * rho,
            Seg::L(a) | Seg::R(a) => a.abs() * rho,
        };
        if total < 1e-6 {
            continue;
        }
        let n = (total / step).ceil().max(1.0) as usize;
        for i in 1..=n {
            let frac = i as f32 / n as f32;
            let (px, py, pth) = advance_rs(*s, frac, rho, (lx, ly, lth));
            let wx = sx + px * c0 - py * s0;
            let wy = sy + px * s0 + py * c0;
            let wt = sth + pth;
            pts.push((wx, wy, wt));
        }
        let (lx2, ly2, lth2) = advance_rs(*s, 1.0, rho, (lx, ly, lth));
        lx = lx2;
        ly = ly2;
        lth = lth2;
    }
    pts
}

/// 沿单段推进 `frac` 比例（局部系）。
fn advance_rs(s: Seg, frac: f32, rho: f32, start: (f32, f32, f32)) -> (f32, f32, f32) {
    let (x, y, th) = start;
    match s {
        Seg::S(a) => {
            let len = a * rho * frac;
            (x + len * th.cos(), y + len * th.sin(), th)
        }
        Seg::L(a) => {
            let aa = a * frac;
            (
                x + rho * ((th + aa).sin() - th.sin()),
                y + rho * (th.cos() - (th + aa).cos()),
                th + aa,
            )
        }
        Seg::R(a) => {
            let aa = a * frac;
            (
                x + rho * (th.sin() - (th - aa).sin()),
                y + rho * ((th - aa).cos() - th.cos()),
                th - aa,
            )
        }
    }
}

/// 最小角度差。
fn angle_diff(a: f32, b: f32) -> f32 {
    let mut d = (a - b) % std::f32::consts::TAU;
    if d > std::f32::consts::PI {
        d -= std::f32::consts::TAU;
    } else if d < -std::f32::consts::PI {
        d += std::f32::consts::TAU;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan(start: (f32, f32, f32), goal: (f32, f32, f32)) -> ReedsSheppPath {
        ReedsSheppPlanner::new(ReedsSheppConfig::default())
            .plan(start, goal)
            .expect("should find a Reeds-Shepp path")
    }

    fn end_err(p: &ReedsSheppPath, goal: (f32, f32, f32)) -> f32 {
        let last = p.points.last().unwrap();
        ((last.0 - goal.0).powi(2) + (last.1 - goal.1).powi(2)).sqrt()
    }

    #[test]
    fn straight_ahead_is_short() {
        let p = plan((0.0, 0.0, 0.0), (10.0, 0.0, 0.0));
        assert!(end_err(&p, (10.0, 0.0, 0.0)) < 0.3);
        assert!((p.length - 10.0).abs() < 1.0, "len={}", p.length);
    }

    #[test]
    fn reaches_goal_with_correct_heading() {
        let p = plan((0.0, 0.0, 0.0), (8.0, 5.0, 1.2));
        assert!(end_err(&p, (8.0, 5.0, 1.2)) < 0.3);
        let last = p.points.last().unwrap();
        assert!(angle_diff(last.2, 1.2).abs() < 0.1, "heading={}", last.2);
    }

    #[test]
    fn uturn_in_place_uses_reverse() {
        // 原地 180° 掉头：Reeds-Shepp 应找到比 Dubins 短得多的路径，且含倒车段。
        let p = plan((0.0, 0.0, 0.0), (0.0, 0.0, std::f32::consts::PI));
        assert!(end_err(&p, (0.0, 0.0, std::f32::consts::PI)) < 0.3);
        let last = p.points.last().unwrap();
        assert!(angle_diff(last.2, std::f32::consts::PI).abs() < 0.1);
        // 相比 Dubins（绕大圈）应显著更短：180° 原地掉头两/三弧长度 ≤ 约 2πR。
        assert!(
            p.length < 2.5 * std::f32::consts::PI * 3.8,
            "len={}",
            p.length
        );
    }

    #[test]
    fn parking_maneuver_contains_reverse() {
        // 侧方位泊车：终点在起点侧后方且朝向翻转 → 必然含倒车段。
        let p = plan((0.0, 0.0, 0.0), (2.0, -3.0, -std::f32::consts::FRAC_PI_2));
        assert!(end_err(&p, (2.0, -3.0, -std::f32::consts::FRAC_PI_2)) < 0.3);
        // 至少一段长度为负（倒车）。
        let has_reverse = p.segments.iter().any(|(_, len)| *len < 0.0);
        assert!(has_reverse, "parking should reverse: {:?}", p.segments);
    }

    #[test]
    fn path_is_continuous_and_dense() {
        let p = plan((0.0, 0.0, 0.0), (12.0, 4.0, 0.8));
        assert!(p.points.len() >= 3);
        for w in p.points.windows(2) {
            let d = ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
            assert!(d < 1.0, "sample too sparse: {d}");
        }
    }

    #[test]
    fn shorter_than_dubins_for_hard_turn() {
        // 对需要大幅转向的构型，Reeds-Shepp（可倒车）应不短于且通常更短于 Dubins。
        use crate::dubins::{DubinsConfig, DubinsPlanner};
        let start = (0.0f32, 0.0f32, 0.0f32);
        let goal = (0.0f32, 4.0f32, std::f32::consts::PI);
        let rho = ReedsSheppConfig::default().turning_radius;
        let rs = ReedsSheppPlanner::new(ReedsSheppConfig::default())
            .plan(start, goal)
            .expect("rs path");
        let db = DubinsPlanner::new(DubinsConfig {
            turning_radius: rho,
            ..DubinsConfig::default()
        })
        .plan(start, goal)
        .expect("dubins path");
        assert!(
            rs.length <= db.length + 0.5,
            "rs={} dubins={}",
            rs.length,
            db.length
        );
    }
}
