//! 确定性进程内仿真后端：2D 栅格世界 + 速度积分 + 测距 + 目标检测。
//!
//! 作为 [`Simulator`](crate::sim::Simulator) 的默认实现，供 SITL 与单元测试
//! 直接使用；无任何系统级依赖、可复现。

use brain_core::time::instant_now;
use brain_core::{Pose, Quat, Vec3};

use crate::sim::{SimDetection, SimState, Simulator};

/// 从四元数提取偏航角（rad，绕竖轴）。
fn yaw_of(q: Quat) -> f32 {
    let siny = 2.0 * (q.w * q.z + q.x * q.y);
    let cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    siny.atan2(cosy)
}

/// 2D 栅格仿真世界。
pub struct MockSimulator {
    width: usize,
    height: usize,
    /// 单元尺寸（m）。
    resolution: f32,
    /// 是否占据。
    occ: Vec<bool>,
    // 机器人状态
    pose: Pose,
    yaw: f32,
    linear: Vec3,
    angular: Vec3,
    // 参数
    max_speed: f32,
    max_range: f32,
    // 感知目标（世界系）
    target_pos: Vec3,
    target_class: u32,
    // 统计
    collisions: u64,
    steps: u64,
}

impl MockSimulator {
    /// 创建 `width x height` 单元、单元尺寸 `resolution` 米的世界。
    pub fn new(width: usize, height: usize, resolution: f32) -> Self {
        Self {
            width,
            height,
            resolution,
            occ: vec![false; width * height],
            pose: Pose::IDENTITY,
            yaw: 0.0,
            linear: Vec3::ZERO,
            angular: Vec3::ZERO,
            max_speed: 5.0,
            max_range: 20.0,
            target_pos: Vec3::new(5.0, 0.0, 0.0),
            target_class: 0,
            collisions: 0,
            steps: 0,
        }
    }

    fn in_bounds(&self, cx: i32, cy: i32) -> bool {
        cx >= 0 && cy >= 0 && (cx as usize) < self.width && (cy as usize) < self.height
    }

    /// 世界坐标 -> 网格单元。
    fn cell(&self, x: f32, y: f32) -> (i32, i32) {
        (
            (x / self.resolution).floor() as i32,
            (y / self.resolution).floor() as i32,
        )
    }

    /// 单元是否被占据（越界视为占据）。
    pub fn is_obstacle_cell(&self, cx: i32, cy: i32) -> bool {
        if !self.in_bounds(cx, cy) {
            return true;
        }
        self.occ[cy as usize * self.width + cx as usize]
    }

    /// 世界坐标处是否被占据。
    pub fn is_obstacle_at(&self, x: f32, y: f32) -> bool {
        let (cx, cy) = self.cell(x, y);
        self.is_obstacle_cell(cx, cy)
    }

    /// 放置障碍（世界坐标）。
    pub fn set_obstacle(&mut self, x: f32, y: f32) {
        let (cx, cy) = self.cell(x, y);
        if self.in_bounds(cx, cy) {
            self.occ[cy as usize * self.width + cx as usize] = true;
        }
    }

    /// 放置一面矩形墙（世界坐标范围，含端点）。
    pub fn add_wall(&mut self, x0: f32, y0: f32, x1: f32, y1: f32) {
        let (sx, sy) = (x0.min(x1), y0.min(y1));
        let (ex, ey) = (x0.max(x1), y0.max(y1));
        let mut x = sx;
        while x <= ex {
            let mut y = sy;
            while y <= ey {
                self.set_obstacle(x, y);
                y += self.resolution;
            }
            x += self.resolution;
        }
    }

    /// 设置感知目标（世界坐标 + 类别）。
    pub fn set_target(&mut self, x: f32, y: f32, class_id: u32) {
        self.target_pos = Vec3::new(x, y, 0.0);
        self.target_class = class_id;
    }

    /// 最大测距。
    pub fn set_max_range(&mut self, m: f32) {
        self.max_range = m;
    }

    fn robot_x(&self) -> f32 {
        self.pose.position.x
    }
    fn robot_y(&self) -> f32 {
        self.pose.position.y
    }

    /// 沿世界系方向角 `theta` 从机器人位置测距（DDA 采样）。
    fn raycast(&self, theta: f32) -> f32 {
        let mut t = 0.0f32;
        let step = self.resolution * 0.5;
        let dx = theta.cos();
        let dy = theta.sin();
        while t < self.max_range {
            t += step;
            let x = self.robot_x() + dx * t;
            let y = self.robot_y() + dy * t;
            if self.is_obstacle_at(x, y) {
                return t;
            }
        }
        self.max_range
    }
}

impl Simulator for MockSimulator {
    fn step(&mut self, dt: f32) -> brain_core::Result<()> {
        if dt <= 0.0 {
            return Ok(()); // 无推进
        }
        // 限速。
        let speed = self.linear.norm();
        let scale = if speed > self.max_speed {
            self.max_speed / speed
        } else {
            1.0
        };
        let lv = self.linear * scale;

        // 机体系 -> 世界系（仅 yaw）。
        let vx_w = lv.x * self.yaw.cos() - lv.y * self.yaw.sin();
        let vy_w = lv.x * self.yaw.sin() + lv.y * self.yaw.cos();

        let nx = self.robot_x() + vx_w * dt;
        let ny = self.robot_y() + vy_w * dt;
        let nyaw = self.yaw + self.angular.z * dt;

        // 碰撞检测：目标单元被占据则不移动（但累计碰撞）。
        if self.is_obstacle_at(nx, ny) {
            self.collisions += 1;
        } else {
            self.pose.position = Vec3::new(nx, ny, 0.0);
            self.yaw = nyaw;
            self.pose.rotation = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), self.yaw);
        }
        self.steps += 1;
        Ok(())
    }

    fn state(&self) -> SimState {
        SimState {
            timestamp: instant_now(),
            robot_pose: self.pose,
            robot_linear_vel: self.linear,
            robot_angular_vel: self.angular,
            collisions: self.collisions,
            steps: self.steps,
        }
    }

    fn set_velocity_command(&mut self, linear: Vec3, angular: Vec3) -> brain_core::Result<()> {
        self.linear = linear;
        self.angular = angular;
        Ok(())
    }

    fn range(&self, bearing_rad: f32) -> f32 {
        let theta = self.yaw + bearing_rad;
        self.raycast(theta)
    }

    fn detections(&self) -> Vec<SimDetection> {
        let dx = self.target_pos.x - self.robot_x();
        let dy = self.target_pos.y - self.robot_y();
        let range = (dx * dx + dy * dy).sqrt();
        if range > self.max_range {
            return Vec::new();
        }
        // 世界系目标方位角，再转为相对机体（bearing 正值偏左）。
        let world_angle = dy.atan2(dx);
        let mut bearing = world_angle - self.yaw;
        // 归一化到 (-pi, pi]。
        while bearing > std::f32::consts::PI {
            bearing -= 2.0 * std::f32::consts::PI;
        }
        while bearing <= -std::f32::consts::PI {
            bearing += 2.0 * std::f32::consts::PI;
        }
        vec![SimDetection {
            class_id: self.target_class,
            confidence: 0.9,
            range_m: range,
            bearing_rad: bearing,
        }]
    }

    fn reset(&mut self, pose: Pose) {
        self.pose = pose;
        self.yaw = yaw_of(pose.rotation);
        self.linear = Vec3::ZERO;
        self.angular = Vec3::ZERO;
        self.collisions = 0;
        self.steps = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::Simulator;

    #[test]
    fn integrates_forward_velocity() {
        let mut s = MockSimulator::new(100, 100, 1.0);
        s.set_velocity_command(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO)
            .unwrap();
        s.step(1.0).unwrap();
        let st = s.state();
        assert!((st.robot_pose.position.x - 1.0).abs() < 1e-4);
        assert!(st.robot_pose.position.y.abs() < 1e-4);
    }

    #[test]
    fn obstacle_stops_and_counts_collision() {
        let mut s = MockSimulator::new(100, 100, 1.0);
        s.set_obstacle(1.0, 0.0);
        s.set_velocity_command(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO)
            .unwrap();
        s.step(1.0).unwrap();
        let st = s.state();
        assert_eq!(st.collisions, 1);
        assert!((st.robot_pose.position.x - 0.0).abs() < 1e-6);
    }

    #[test]
    fn range_sensor_hits_wall() {
        let mut s = MockSimulator::new(100, 100, 1.0);
        s.add_wall(5.0, -1.0, 5.0, 1.0);
        let r = s.range(0.0); // 正前方
        assert!(r > 4.5 && r < 5.5, "range={r}");
    }

    #[test]
    fn range_sensor_max_when_free() {
        let mut s = MockSimulator::new(100, 100, 1.0);
        s.set_max_range(8.0);
        assert!((s.range(0.0) - 8.0).abs() < 1e-3);
    }

    #[test]
    fn detection_returns_bearing_and_range() {
        let mut s = MockSimulator::new(100, 100, 1.0);
        s.set_target(3.0, 0.0, 7);
        let dets = s.detections();
        assert_eq!(dets.len(), 1);
        assert_eq!(dets[0].class_id, 7);
        assert!((dets[0].range_m - 3.0).abs() < 1e-3);
        assert!(dets[0].bearing_rad.abs() < 1e-3);
    }

    #[test]
    fn detection_off_forward_not_seen() {
        let mut s = MockSimulator::new(100, 100, 1.0);
        // 目标在身后很远 -> 超出 max_range。
        s.set_target(-50.0, 0.0, 0);
        assert!(s.detections().is_empty());
    }

    #[test]
    fn reset_restores_pose_and_state() {
        let mut s = MockSimulator::new(100, 100, 1.0);
        s.set_velocity_command(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO)
            .unwrap();
        s.step(1.0).unwrap();
        s.reset(Pose::IDENTITY);
        let st = s.state();
        assert_eq!(st.collisions, 0);
        assert_eq!(st.steps, 0);
        assert!(st.robot_pose.position.norm() < 1e-6);
    }

    #[test]
    fn velocity_is_clamped_to_max_speed() {
        let mut s = MockSimulator::new(100, 100, 1.0);
        s.set_velocity_command(Vec3::new(100.0, 0.0, 0.0), Vec3::ZERO)
            .unwrap();
        s.step(0.1).unwrap();
        let st = s.state();
        // 0.1s * max_speed(5) = 0.5m，而不是 10m。
        assert!(
            (st.robot_pose.position.x - 0.5).abs() < 1e-3,
            "x={}",
            st.robot_pose.position.x
        );
    }
}
