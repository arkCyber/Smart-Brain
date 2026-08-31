//! 自行车（Ackermann）运动学模型 —— 汽车等前轮转向车辆的质心运动。
//!
//! 与无人机/差速底盘（可原地转向、`v,ω`）不同，汽车受**阿克曼转向几何**约束：
//! - 只能靠前轮转向改变航向，存在**最小转弯半径** `R_min = L / tan(δ_max)`
//! - 前轮转向角与转向执行机构速率受限（不能瞬间打满方向盘）
//! - 纵向速度为主，倒车仅用于泊车/脱困
//!
//! 该模型是“汽车导航”的运动学契约，供局部避障（Ackermann DWA）与
//! 汽车闭环自主导航（`brain-autopilot::car_autopilot`）共用。

/// 自行车/阿克曼模型参数。
#[derive(Debug, Clone, Copy)]
pub struct BicycleModel {
    /// 轴距（前后轴中心距，米）。
    pub wheelbase: f32,
    /// 前轮最大转向角（rad，对称，|δ|≤该值）。
    pub max_steering: f32,
    /// 前轮最大转向角速率（rad/s，方向盘执行机构限制）。
    pub max_steer_rate: f32,
}

impl BicycleModel {
    pub fn new(wheelbase: f32, max_steering: f32, max_steer_rate: f32) -> Self {
        Self {
            wheelbase,
            max_steering,
            max_steer_rate,
        }
    }

    /// 最小转弯半径（米）：轴距越大 / 最大转向越小，转弯半径越大。
    pub fn min_turning_radius(&self) -> f32 {
        self.wheelbase / self.max_steering.tan().max(1e-6)
    }

    /// 由纵向速度与前轮转角得到偏航角速度 `ω = v·tan(δ)/L`。
    pub fn angular_velocity(&self, speed: f32, steering: f32) -> f32 {
        speed * steering.tan() / self.wheelbase
    }

    /// 由纵向速度与期望偏航角速度反解前轮转角（并夹到转向限位）。
    /// 速度接近 0 时返回 0（无法原地转向）。
    pub fn steering_for_angular(&self, speed: f32, angular: f32) -> f32 {
        if speed.abs() < 1e-3 {
            return 0.0;
        }
        (angular * self.wheelbase / speed)
            .atan()
            .clamp(-self.max_steering, self.max_steering)
    }

    /// 按自行车模型积分一步（含转向速率限制）。
    pub fn step(
        &self,
        st: &BicycleState,
        v_target: f32,
        delta_target: f32,
        dt: f32,
    ) -> BicycleState {
        // 转向执行机构速率受限：一个 dt 内最大转向变化 = steer_rate*dt。
        let dmax = self.max_steer_rate * dt;
        let steering = (delta_target - st.steering).clamp(-dmax, dmax) + st.steering;
        let steering = steering.clamp(-self.max_steering, self.max_steering);
        // 纵向速度（加速度限制由上层 DWA 动态窗口处理）。
        let speed = v_target;
        let theta = st.theta + speed * steering.tan() / self.wheelbase * dt;
        let x = st.x + speed * theta.cos() * dt;
        let y = st.y + speed * theta.sin() * dt;
        BicycleState {
            x,
            y,
            theta: norm_angle(theta),
            speed,
            steering,
        }
    }
}

/// 车辆当前运动状态。
#[derive(Debug, Clone, Copy)]
pub struct BicycleState {
    /// 世界系后轴中心 x（米）。
    pub x: f32,
    /// 世界系后轴中心 y（米）。
    pub y: f32,
    /// 朝向（rad，世界系）。
    pub theta: f32,
    /// 纵向速度（m/s，沿车头方向）。
    pub speed: f32,
    /// 前轮转角（rad）。
    pub steering: f32,
}

impl BicycleState {
    pub fn new(x: f32, y: f32, theta: f32) -> Self {
        Self {
            x,
            y,
            theta,
            speed: 0.0,
            steering: 0.0,
        }
    }
}

/// 把角度归一化到 `(-π, π]`。
pub fn norm_angle(a: f32) -> f32 {
    let mut a = a % std::f32::consts::TAU;
    if a > std::f32::consts::PI {
        a -= std::f32::consts::TAU;
    } else if a <= -std::f32::consts::PI {
        a += std::f32::consts::TAU;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    fn car() -> BicycleModel {
        BicycleModel::new(2.6, 0.6, 0.8)
    }

    #[test]
    fn min_radius_and_turn_math() {
        let m = car();
        // R_min = L / tan(δ_max) ≈ 2.6 / tan(0.6) ≈ 3.7m
        let r = m.min_turning_radius();
        assert!(r > 2.0 && r < 8.0, "min turning radius = {r}");
        // 直行：转向角 0 → 角速度为 0。
        assert!(m.angular_velocity(5.0, 0.0).abs() < 1e-6);
        // 满舵低速 → 绕圈角速度 = v·tan(δ)/L。
        let w = m.angular_velocity(3.0, 0.6);
        assert!((w - 3.0 * 0.6f32.tan() / 2.6).abs() < 1e-5);
    }

    #[test]
    fn straight_drive_advances_along_heading() {
        let m = car();
        let s = BicycleState::new(0.0, 0.0, 0.0);
        let s2 = m.step(&s, 4.0, 0.0, 0.1);
        // 直行 0.4m。
        assert!((s2.x - 0.4).abs() < 1e-3);
        assert!(s2.y.abs() < 1e-3);
        assert!(s2.theta.abs() < 1e-3);
    }

    #[test]
    fn steering_rate_limited() {
        let m = car();
        let s = BicycleState::new(0.0, 0.0, 0.0);
        // 目标满舵，但一个 dt 内最多转 steer_rate*dt = 0.08 rad。
        let s2 = m.step(&s, 1.0, 0.6, 0.1);
        assert!(
            (s2.steering - 0.08).abs() < 1e-3,
            "steering={}",
            s2.steering
        );
    }

    #[test]
    fn full_lock_arcs_approx_min_radius() {
        let m = car();
        // 满舵低速绕圈一圈，终点位移应近似 0（回到原点附近）。
        let mut s = BicycleState::new(0.0, 0.0, 0.0);
        let r_min = m.min_turning_radius();
        let dt = 0.01;
        let mut cum = 0.0f32; // 累计转角（不受 norm_angle 回绕影响）。
        let mut steps = 0;
        while cum < std::f32::consts::TAU && steps < 5000 {
            s = m.step(&s, 2.0, m.max_steering, dt);
            cum += m.angular_velocity(s.speed, s.steering) * dt;
            steps += 1;
        }
        assert!(steps < 5000, "did not complete a circle in time");
        let dist = ((s.x - 0.0).powi(2) + (s.y - 0.0).powi(2)).sqrt();
        // 绕满一圈后应接近原点，位移远小于一圈弧长（即半径≈R_min）。
        assert!(
            dist / r_min < 0.5,
            "dist={dist} r_min={r_min} steps={steps}"
        );
    }
}
