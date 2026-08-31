//! 视觉里程计：帧间 RGB-D 点云对齐（ICP）估计相机运动。

use brain_core::Pose;
use brain_core::{Quat, Vec3};

use crate::kabsch::{apply_transform, rigid_transform};
use crate::sensor::PointCloud;

/// ICP 参数。
#[derive(Debug, Clone, Copy)]
pub struct IcpConfig {
    /// 最大迭代次数。
    pub max_iters: usize,
    /// 收敛容差（平均对齐误差，米）。
    pub tol: f32,
    /// 对应点距离阈值（米），超阈值视为离群点。
    pub inlier_dist: f32,
}

impl Default for IcpConfig {
    fn default() -> Self {
        Self {
            max_iters: 20,
            tol: 1e-3,
            inlier_dist: 0.5,
        }
    }
}

/// 在 `target` 中找到离 `p` 最近的点（O(n) 暴力，适合小点云）。
fn nearest(target: &[Vec3], p: Vec3) -> Vec3 {
    target
        .iter()
        .min_by(|a, b| {
            a.sub(p)
                .norm()
                .partial_cmp(&b.sub(p).norm())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .copied()
        .unwrap_or(p)
}

/// 点到点 ICP：把 `source` 对齐到 `target`，返回刚体变换 `(q, t)`。
pub fn icp(source: &[Vec3], target: &[Vec3], cfg: &IcpConfig) -> (Quat, Vec3) {
    if source.len() < 3 || target.len() < 3 {
        return (Quat::IDENTITY, Vec3::ZERO);
    }
    let mut cur = source.to_vec();
    let mut q_total = Quat::IDENTITY;
    let mut t_total = Vec3::ZERO;
    for _ in 0..cfg.max_iters {
        let src: Vec<Vec3> = cur.clone();
        let tgt: Vec<Vec3> = src.iter().map(|&p| nearest(target, p)).collect();
        // 仅用距离阈值内的对应点。
        let mut fs = Vec::new();
        let mut ft = Vec::new();
        for i in 0..src.len() {
            if src[i].sub(tgt[i]).norm() < cfg.inlier_dist {
                fs.push(src[i]);
                ft.push(tgt[i]);
            }
        }
        if fs.len() < 3 {
            break;
        }
        let (dq, dt) = rigid_transform(&fs, &ft).unwrap_or((Quat::IDENTITY, Vec3::ZERO));
        // 应用更新并累计总变换。
        let mut err2 = 0.0;
        for p in cur.iter_mut() {
            *p = apply_transform(*p, dq, dt);
            err2 += p.sub(nearest(target, *p)).norm().min(1.0);
        }
        t_total = dq.rotate_vec3(t_total).add(dt);
        q_total = dq.mul(q_total);
        if (err2 / cur.len() as f32) < cfg.tol {
            break;
        }
    }
    (q_total, t_total)
}

/// RGB-D 视觉里程计。
pub struct VisualOdometry {
    prev_cloud: Option<PointCloud>,
    /// 相机在世界系下的位姿。
    pose: Pose,
    cfg: IcpConfig,
}

impl VisualOdometry {
    pub fn new(cfg: IcpConfig) -> Self {
        Self {
            prev_cloud: None,
            pose: Pose::IDENTITY,
            cfg,
        }
    }

    /// 处理一帧新的点云，返回更新后的相机位姿。
    pub fn process(&mut self, cloud: &PointCloud) -> Option<Pose> {
        match &self.prev_cloud {
            None => {
                self.prev_cloud = Some(cloud.clone());
                Some(self.pose)
            }
            Some(prev) => {
                // 相对变换 T：当前帧 → 上一帧。
                let (q, t) = icp(cloud, prev, &self.cfg);
                // P_cur = P_prev * T。
                let t_cur = self.pose.rotation.rotate_vec3(t).add(self.pose.position);
                let q_cur = self.pose.rotation.mul(q);
                self.pose = Pose {
                    position: t_cur,
                    rotation: q_cur,
                };
                self.prev_cloud = Some(cloud.clone());
                Some(self.pose)
            }
        }
    }

    /// 当前相机位姿。
    pub fn pose(&self) -> Pose {
        self.pose
    }

    /// 重置（例如丢失跟踪后）。
    pub fn reset(&mut self) {
        self.prev_cloud = None;
        self.pose = Pose::IDENTITY;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn icp_recovers_motion() {
        let cfg = IcpConfig::default();
        // 上一帧：一个简单立方体角点云。
        let prev: Vec<Vec3> = (0..8)
            .map(|i| Vec3::new((i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32))
            .collect();
        // 当前帧 = 上一帧平移 (0.1, 0.0, 0.0)。
        let cur: Vec<Vec3> = prev
            .iter()
            .map(|&p| p.add(Vec3::new(0.1, 0.0, 0.0)))
            .collect();
        let (q, t) = icp(&cur, &prev, &cfg);
        // 把当前帧用 (q,t) 变换应回到上一帧。
        let aligned: Vec<Vec3> = cur.iter().map(|&p| apply_transform(p, q, t)).collect();
        for i in 0..aligned.len() {
            let e = aligned[i].sub(prev[i]).norm();
            assert!(e < 0.02, "point {i} err {e}");
        }
    }

    #[test]
    fn vo_tracks_translation() {
        let mut vo = VisualOdometry::new(IcpConfig::default());
        let cube = |off: Vec3| -> Vec<Vec3> {
            (0..8)
                .map(|i| {
                    Vec3::new((i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32).add(off)
                })
                .collect()
        };
        vo.process(&cube(Vec3::ZERO));
        let p1 = vo.process(&cube(Vec3::new(0.1, 0.0, 0.0))).unwrap();
        // 相机应沿 -x 运动（点云向 +x 移动）。
        assert!((p1.position.x - (-0.1)).abs() < 0.02, "x={}", p1.position.x);
    }
}
