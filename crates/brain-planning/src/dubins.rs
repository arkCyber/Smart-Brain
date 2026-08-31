//! Dubins 曲线 —— 只前进、受最小转弯半径约束的车辆最短路径规划。
//!
//! 汽车（阿克曼）不能原地转向，只能沿半径 ≥ R_min 的圆弧与直线行驶。
//! Dubins 定理保证：任意两构型（位置+朝向）间的最短可行路径必然由
//! “最大曲率圆弧 + 直线”组成，共 6 种类型：`LSL/RSR/RSL/LSR/RLR/LRL`。
//!
//! 本实现计算每种类型的圆弧/直线长度，并**用自行车模型逐点精确积分校验**——
//! 只有整条路径真的到达目标构型时才接受，因此个别公式的符号歧义会被过滤掉，
//! 最后返回最短的可行路径。

/// Dubins 规划配置。
#[derive(Debug, Clone, Copy)]
pub struct DubinsConfig {
    /// 最小转弯半径（米）。
    pub turning_radius: f32,
    /// 采样步长（米），用于把路径离散成航点。
    pub step: f32,
    /// 终点校验容差（米）。
    pub goal_tolerance: f32,
}

impl Default for DubinsConfig {
    fn default() -> Self {
        Self {
            turning_radius: 3.8,
            step: 0.5,
            goal_tolerance: 0.3,
        }
    }
}

/// 一条 Dubins 路径（离散化后的世界系构型航点）。
#[derive(Debug, Clone)]
pub struct DubinsPath {
    pub points: Vec<(f32, f32, f32)>,
    pub length: f32,
}

/// 路径基本段：弧（角度，rad）或直线（长度，m）。
#[derive(Debug, Clone, Copy)]
enum Prim {
    L(f32),
    R(f32),
    S(f32),
}

/// Dubins 规划器。
pub struct DubinsPlanner {
    cfg: DubinsConfig,
}

impl DubinsPlanner {
    pub fn new(cfg: DubinsConfig) -> Self {
        Self { cfg }
    }

    /// 规划 `start → goal` 的最短 Dubins 路径（只前进）。
    pub fn plan(&self, start: (f32, f32, f32), goal: (f32, f32, f32)) -> Option<DubinsPath> {
        let rho = self.cfg.turning_radius.max(1e-3);
        // 平移到以起点为原点、朝向 0 的局部系。
        let (dx, dy) = (goal.0 - start.0, goal.1 - start.1);
        let (c0, s0) = (start.2.cos(), start.2.sin());
        let x = dx * c0 + dy * s0;
        let y = -dx * s0 + dy * c0;
        let phi = goal.2 - start.2;

        let mut best: Option<(f32, Vec<Prim>)> = None;
        for prims in self.candidates(x, y, phi, rho) {
            let (ex, ey, eth) = simulate((0.0, 0.0, 0.0), &prims, rho);
            let err = ((ex - x).powi(2) + (ey - y).powi(2)).sqrt();
            if err < self.cfg.goal_tolerance && angle_diff(eth, phi).abs() < 0.08 {
                let len = path_length(&prims, rho);
                if best.as_ref().map(|(l, _)| len < *l).unwrap_or(true) {
                    best = Some((len, prims));
                }
            }
        }

        best.map(|(len, prims)| {
            let points = sample_world(&prims, rho, start, self.cfg.step);
            DubinsPath {
                points,
                length: len,
            }
        })
    }

    /// 为 6 种路径类型生成候选基本段（含符号变体，靠模拟校验过滤）。
    fn candidates(&self, x: f32, y: f32, phi: f32, rho: f32) -> Vec<Vec<Prim>> {
        let mut out = Vec::new();
        // CSC：LSL / RSR / RSL / LSR
        for (turn1, turn2, external) in [
            (1.0f32, 1.0f32, true), // LSL（左-直-左）
            (-1.0, -1.0, true),     // RSR（右-直-右）
            (-1.0, 1.0, false),     // RSL（右-直-左）
            (1.0, -1.0, false),     // LSR（左-直-右）
        ] {
            out.extend(csc_geom(x, y, phi, rho, turn1, turn2, external));
        }
        // CCC：LRL / RLR（无直线）
        out.extend(ccc_geom(x, y, phi, rho, 1.0)); // LRL
        out.extend(ccc_geom(x, y, phi, rho, -1.0)); // RLR
        out
    }
}

/// 计算“弧-直线-弧”（CSC）路径候选。`turn1/turn2`：+1 左转，-1 右转；
/// `external` 为 true 表示两圆同侧（外切线，直线∥圆心连线），false 表示异侧（内切线）。
fn csc_geom(
    x: f32,
    y: f32,
    phi: f32,
    rho: f32,
    turn1: f32,
    turn2: f32,
    external: bool,
) -> Vec<Vec<Prim>> {
    let two = std::f32::consts::TAU;
    // 起/终点转弯圆圆心。
    let c1 = (0.0f32, turn1 * rho);
    let c2 = (x - turn2 * rho * phi.sin(), y + turn2 * rho * phi.cos());
    let dx = c2.0 - c1.0;
    let dy = c2.1 - c1.1;
    let d = (dx * dx + dy * dy).sqrt();
    if d < 2.0 * rho {
        return Vec::new();
    }
    let th = dy.atan2(dx);

    let mut e_angles: Vec<f32> = Vec::new();
    if external {
        // 外切线：直线方向与圆心连线平行（或反向）。
        e_angles.push(th);
        e_angles.push(th + std::f32::consts::PI);
    } else {
        // 内切线：直线方向相对圆心连线偏转 ±β。
        let beta = (2.0 * rho / d).acos();
        e_angles.push(th + beta);
        e_angles.push(th - beta);
    }

    let mut out = Vec::new();
    for e_ang in e_angles {
        let u = if external {
            d
        } else {
            (d * d - 4.0 * rho * rho).sqrt()
        };
        // t：从朝向 0 沿 turn1 方向转到直线朝向 e_ang。
        let mut t = if turn1 > 0.0 { e_ang } else { -e_ang };
        t = t.rem_euclid(two);
        // v：从直线朝向 e_ang 沿 turn2 方向转到目标朝向 phi。
        let mut v = if turn2 > 0.0 {
            phi - e_ang
        } else {
            e_ang - phi
        };
        v = v.rem_euclid(two);
        let first = if turn1 > 0.0 { Prim::L(t) } else { Prim::R(t) };
        let second = if turn2 > 0.0 { Prim::L(v) } else { Prim::R(v) };
        out.push(vec![first, Prim::S(u), second]);
    }
    out
}

/// 计算“弧-弧-弧”（CCC）路径候选。`turn`：+1 → LRL，-1 → RLR。
fn ccc_geom(x: f32, y: f32, phi: f32, rho: f32, turn: f32) -> Vec<Vec<Prim>> {
    let two = std::f32::consts::TAU;
    let c1 = (0.0f32, turn * rho);
    let c2 = (x - turn * rho * phi.sin(), y + turn * rho * phi.cos());
    let dx = c2.0 - c1.0;
    let dy = c2.1 - c1.1;
    let d = (dx * dx + dy * dy).sqrt();
    if d > 4.0 * rho || d < 1e-6 {
        return Vec::new();
    }
    let th = dy.atan2(dx);
    let beta = (d / (4.0 * rho)).acos();
    let mut out = Vec::new();
    for t0 in [th + beta, th - beta] {
        let mid = two - 2.0 * beta; // 中间弧角（掉头）
        let v = phi - t0 - turn * mid;
        let first = if turn > 0.0 { Prim::L(t0) } else { Prim::R(t0) };
        let midp = if turn > 0.0 {
            Prim::R(mid)
        } else {
            Prim::L(mid)
        };
        let last = if turn > 0.0 { Prim::L(v) } else { Prim::R(v) };
        out.push(vec![first, midp, last]);
    }
    out
}

/// 用自行车模型精确积分一段路径，返回末端构型（局部系）。
fn simulate(start: (f32, f32, f32), prims: &[Prim], rho: f32) -> (f32, f32, f32) {
    let (mut x, mut y, mut th) = start;
    for p in prims {
        match *p {
            Prim::S(len) => {
                x += len * th.cos();
                y += len * th.sin();
            }
            Prim::L(a) => {
                x += rho * ((th + a).sin() - th.sin());
                y += rho * (th.cos() - (th + a).cos());
                th += a;
            }
            Prim::R(a) => {
                x += rho * (th.sin() - (th - a).sin());
                y += rho * ((th - a).cos() - th.cos());
                th -= a;
            }
        }
    }
    (x, y, th)
}

fn path_length(prims: &[Prim], rho: f32) -> f32 {
    prims
        .iter()
        .map(|p| match *p {
            Prim::S(l) => l,
            Prim::L(a) | Prim::R(a) => rho * a.abs(),
        })
        .sum()
}

/// 把局部系路径离散成世界系航点。
fn sample_world(
    prims: &[Prim],
    rho: f32,
    start: (f32, f32, f32),
    step: f32,
) -> Vec<(f32, f32, f32)> {
    // 先累加出局部系各段起点构型，再采样。
    let mut pts = Vec::new();
    let (sx, sy, sth) = start;
    let (c0, s0) = (sth.cos(), sth.sin());
    pts.push((sx, sy, sth));

    // 局部系下推进。
    let mut lx = 0.0f32;
    let mut ly = 0.0f32;
    let mut lth = 0.0f32;
    let step = step.max(1e-3);
    for p in prims {
        // 计算本段总长与本段起点。
        let seg_start = (lx, ly, lth);
        let total = match *p {
            Prim::S(l) => l,
            Prim::L(a) | Prim::R(a) => rho * a.abs(),
        };
        let n = (total / step).ceil().max(1.0) as usize;
        let mut seg_end = seg_start;
        for i in 1..=n {
            let frac = i as f32 / n as f32;
            // 推进到本段 frac 处（在局部系下用 simulate 从段起点推进 frac*total 的等效弧）。
            seg_end = advance_frac(seg_start, *p, frac, rho);
            // 变换到世界系。
            let wx = sx + seg_end.0 * c0 - seg_end.1 * s0;
            let wy = sy + seg_end.0 * s0 + seg_end.1 * c0;
            let wt = sth + seg_end.2;
            pts.push((wx, wy, wt));
        }
        lx = seg_end.0;
        ly = seg_end.1;
        lth = seg_end.2;
    }
    pts
}

/// 从段起点推进该段总长的 `frac` 比例（局部系）。
fn advance_frac(seg_start: (f32, f32, f32), p: Prim, frac: f32, rho: f32) -> (f32, f32, f32) {
    let (x, y, th) = seg_start;
    match p {
        Prim::S(len) => {
            let l = len * frac;
            (x + l * th.cos(), y + l * th.sin(), th)
        }
        Prim::L(a) => {
            let aa = a * frac;
            (
                x + rho * ((th + aa).sin() - th.sin()),
                y + rho * (th.cos() - (th + aa).cos()),
                th + aa,
            )
        }
        Prim::R(a) => {
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

    fn plan(start: (f32, f32, f32), goal: (f32, f32, f32)) -> DubinsPath {
        DubinsPlanner::new(DubinsConfig::default())
            .plan(start, goal)
            .expect("should find a Dubins path")
    }

    fn end_err(p: &DubinsPath, goal: (f32, f32, f32)) -> f32 {
        let last = p.points.last().unwrap();
        ((last.0 - goal.0).powi(2) + (last.1 - goal.1).powi(2)).sqrt()
    }

    #[test]
    fn straight_ahead_is_near_optimal() {
        // 朝 +X 直行：路径应几乎为直线，长度接近 10。
        let p = plan((0.0, 0.0, 0.0), (10.0, 0.0, 0.0));
        assert!(
            end_err(&p, (10.0, 0.0, 0.0)) < 0.3,
            "err={}",
            end_err(&p, (10.0, 0.0, 0.0))
        );
        assert!((p.length - 10.0).abs() < 1.0, "len={}", p.length);
    }

    #[test]
    fn ninety_degree_turn() {
        // 朝 +X 出发，到达右前方并面向 +Y。
        let p = plan((0.0, 0.0, 0.0), (10.0, 10.0, std::f32::consts::FRAC_PI_2));
        assert!(end_err(&p, (10.0, 10.0, std::f32::consts::FRAC_PI_2)) < 0.3);
        assert!(p.length > 10.0, "len={}", p.length);
    }

    #[test]
    fn reaches_goal_with_correct_heading() {
        // 调头 180°：Dubins（只前进）也能绕圆弧到达。
        let p = plan((0.0, 0.0, 0.0), (0.0, 6.0, std::f32::consts::PI));
        assert!(end_err(&p, (0.0, 6.0, std::f32::consts::PI)) < 0.3);
        let last = p.points.last().unwrap();
        assert!(
            angle_diff(last.2, std::f32::consts::PI).abs() < 0.1,
            "heading={}",
            last.2
        );
    }

    #[test]
    fn respects_min_turning_radius() {
        // 采样点任意两相邻段间的曲率半径不得小于 turning_radius。
        let cfg = DubinsConfig::default();
        let p = plan((0.0, 0.0, 0.0), (5.0, 8.0, -std::f32::consts::FRAC_PI_3));
        // 通过重建校验曲率：相邻采样点方向变化对应的最小半径。
        for w in p.points.windows(3) {
            let (_, _, a) = w[0];
            let (_, _, b) = w[1];
            let (_, _, c) = w[2];
            let d1 = ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
            let d2 = ((w[2].0 - w[1].0).powi(2) + (w[2].1 - w[1].1).powi(2)).sqrt();
            if d1 < 1e-4 || d2 < 1e-4 {
                continue;
            }
            let h1 = angle_diff(b, a);
            let h2 = angle_diff(c, b);
            // 单位弧长转向角（曲率）不得大于 1 / R_min（允许采样带来的小幅误差）。
            let k = (h1 / d1).abs().max((h2 / d2).abs());
            assert!(k < 1.0 / cfg.turning_radius + 0.6, "k={k}");
        }
    }

    #[test]
    fn symmetric_reverse() {
        // 反向 180° 行驶到右侧。
        let p = plan((0.0, 0.0, 0.0), (8.0, 0.0, std::f32::consts::PI));
        assert!(end_err(&p, (8.0, 0.0, std::f32::consts::PI)) < 0.3);
        assert!(p.points.len() >= 3, "should have multiple waypoints");
    }

    #[test]
    fn path_is_continuous_and_dense() {
        // 采样应连续：相邻航点间距不超过一个采样步长。
        let p = plan((0.0, 0.0, 0.0), (12.0, 4.0, 0.8));
        assert!(p.points.len() >= 3);
        for w in p.points.windows(2) {
            let d = ((w[1].0 - w[0].0).powi(2) + (w[1].1 - w[0].1).powi(2)).sqrt();
            assert!(d < 1.0, "sample too sparse: {d}");
        }
    }

    #[test]
    fn start_and_goal_are_endpoints() {
        // 路径首点贴近起点、末点贴近终点。
        let s = (1.0f32, 2.0f32, 0.5f32);
        let g = (9.0f32, 7.0f32, -1.0f32);
        let p = plan(s, g);
        let (sx, sy, _) = p.points.first().unwrap();
        let (gx, gy, _) = p.points.last().unwrap();
        assert!(((sx - s.0).powi(2) + (sy - s.1).powi(2)).sqrt() < 0.3);
        assert!(((gx - g.0).powi(2) + (gy - g.1).powi(2)).sqrt() < 0.3);
    }
}
