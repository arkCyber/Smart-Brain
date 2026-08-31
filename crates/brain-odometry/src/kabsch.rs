//! Kabsch 算法：由两组对应点求最佳刚体变换（旋转 + 平移）。
//!
//! 视觉里程计用它把“上一帧点云”与“当前帧点云”对齐，估计相对位姿。
//! 旋转通过 SVD 求解（标准 Kabsch），内部用 Jacobi 方法做 3×3 特征分解。

use brain_core::error::{BrainError, Result};
use brain_core::{Quat, Vec3};

type M3 = [[f64; 3]; 3];

fn mat_mul(a: M3, b: M3) -> M3 {
    let mut c = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            c[i][j] = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    c
}

fn mat_transpose(a: M3) -> M3 {
    let mut t = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            t[i][j] = a[j][i];
        }
    }
    t
}

/// 对称矩阵的 Jacobi 特征分解。返回 (特征值降序, 特征向量为列)。
///
/// `p/q/k` 需索引二维矩阵的行列与三角范围，属数值算法固有写法。
#[allow(clippy::needless_range_loop)]
fn jacobi_eigen(m: M3) -> ([f64; 3], M3) {
    let mut a = m;
    let mut v: M3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..100 {
        let mut off = 0.0;
        for p in 0..3 {
            for q in (p + 1)..3 {
                off += a[p][q] * a[p][q];
            }
        }
        if off < 1e-20 {
            break;
        }
        for p in 0..2 {
            for q in (p + 1)..3 {
                if a[p][q].abs() < 1e-16 {
                    continue;
                }
                let app = a[p][p];
                let aqq = a[q][q];
                let apq = a[p][q];
                let theta = 0.5 * (2.0 * apq).atan2(aqq - app);
                let c = theta.cos();
                let s = theta.sin();
                // 更新 a 的行/列（用旧值）。
                for k in 0..3 {
                    let akp = a[k][p];
                    let akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                // 精确设置对角与消除 off-diagonal。
                a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
                a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
                a[p][q] = 0.0;
                a[q][p] = 0.0;
                // 累积 v = v J。
                for k in 0..3 {
                    let vkp = v[k][p];
                    let vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    let mut idx = [0usize; 3];
    let mut eig = [a[0][0], a[1][1], a[2][2]];
    for (i, slot) in idx.iter_mut().enumerate() {
        *slot = i;
    }
    for i in 0..3 {
        for j in i + 1..3 {
            if eig[j] > eig[i] {
                eig.swap(i, j);
                idx.swap(i, j);
            }
        }
    }
    let mut out = [[0.0; 3]; 3];
    for (col, &src) in idx.iter().enumerate() {
        for row in 0..3 {
            out[row][col] = v[row][src];
        }
    }
    (eig, out)
}

/// 3×3 SVD：`m = u * diag(s) * vt`。
fn svd3(m: M3) -> (M3, [f64; 3], M3) {
    let mt = mat_transpose(m);
    let ata = mat_mul(mt, m);
    let (eig, v) = jacobi_eigen(ata);
    let mut s = [0.0; 3];
    for i in 0..3 {
        s[i] = eig[i].max(0.0).sqrt();
    }
    let mut u = [[0.0; 3]; 3];
    for i in 0..3 {
        if s[i] > 1e-10 {
            for r in 0..3 {
                let mut acc = 0.0;
                for k in 0..3 {
                    acc += m[r][k] * v[k][i];
                }
                u[r][i] = acc / s[i];
            }
        } else {
            u[i][i] = 1.0;
        }
    }
    (u, s, mat_transpose(v))
}

/// 由旋转矩阵（3×3）转四元数（Shepperd 法）。
fn quat_from_matrix(m: M3) -> Quat {
    let (m00, m01, m02) = (m[0][0], m[0][1], m[0][2]);
    let (m10, m11, m12) = (m[1][0], m[1][1], m[1][2]);
    let (m20, m21, m22) = (m[2][0], m[2][1], m[2][2]);
    let tr = m00 + m11 + m22;
    if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        Quat::new(
            (0.25 * s) as f32,
            ((m21 - m12) / s) as f32,
            ((m02 - m20) / s) as f32,
            ((m10 - m01) / s) as f32,
        )
    } else if m00 > m11 && m00 > m22 {
        let s = (1.0 + m00 - m11 - m22).sqrt() * 2.0;
        Quat::new(
            ((m21 - m12) / s) as f32,
            (0.25 * s) as f32,
            ((m01 + m10) / s) as f32,
            ((m02 + m20) / s) as f32,
        )
    } else if m11 > m22 {
        let s = (1.0 + m11 - m00 - m22).sqrt() * 2.0;
        Quat::new(
            ((m02 - m20) / s) as f32,
            ((m01 + m10) / s) as f32,
            (0.25 * s) as f32,
            ((m12 + m21) / s) as f32,
        )
    } else {
        let s = (1.0 + m22 - m00 - m11).sqrt() * 2.0;
        Quat::new(
            ((m10 - m01) / s) as f32,
            ((m02 + m20) / s) as f32,
            ((m12 + m21) / s) as f32,
            (0.25 * s) as f32,
        )
    }
}

/// 计算把点集 `from` 对齐到点集 `to` 的最优刚体变换 `(R, t)`。
pub fn rigid_transform(from: &[Vec3], to: &[Vec3]) -> Result<(Quat, Vec3)> {
    if from.len() != to.len() || from.len() < 3 {
        return Err(BrainError::Config(format!(
            "rigid_transform needs >=3 corresponding points, got {} vs {}",
            from.len(),
            to.len()
        )));
    }
    let mut cf = Vec3::ZERO;
    let mut ct = Vec3::ZERO;
    for i in 0..from.len() {
        cf = cf.add(from[i]);
        ct = ct.add(to[i]);
    }
    let n = from.len() as f32;
    cf = cf * (1.0 / n);
    ct = ct * (1.0 / n);

    let mut h = [[0.0f64; 3]; 3];
    for i in 0..from.len() {
        let a = from[i].sub(cf);
        let b = to[i].sub(ct);
        for r in 0..3 {
            for c in 0..3 {
                let av = [a.x, a.y, a.z][r] as f64;
                let bv = [b.x, b.y, b.z][c] as f64;
                h[r][c] += av * bv;
            }
        }
    }

    let (u, _s, vt) = svd3(h);
    let v = mat_transpose(vt);
    let vu = mat_mul(vt, mat_transpose(u)); // V^T * U^T
    let sign = {
        let m = vu;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let mut d = [[0.0; 3]; 3];
    d[0][0] = 1.0;
    d[1][1] = 1.0;
    d[2][2] = if sign < 0.0 { -1.0 } else { 1.0 };

    let du = mat_mul(d, mat_transpose(u));
    let r = mat_mul(v, du);

    let q = quat_from_matrix(r);
    let t = ct.sub(q.rotate_vec3(cf));
    Ok((q, t))
}

/// 应用刚体变换 `(q, t)` 到点 `p`：`q·p + t`。
pub fn apply_transform(p: Vec3, q: Quat, t: Vec3) -> Vec3 {
    q.rotate_vec3(p).add(t)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn recovers_known_transform() {
        let q_gt = Quat::from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.6);
        let t_gt = Vec3::new(1.0, 2.0, 3.0);
        // 用立方体角点（张成 3D，避免奇异）。
        let pts: Vec<Vec3> = (0..8)
            .map(|i| Vec3::new((i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32))
            .collect();
        let moved: Vec<Vec3> = pts
            .iter()
            .map(|&p| apply_transform(p, q_gt, t_gt))
            .collect();
        let (q_est, t_est) = rigid_transform(&pts, &moved).unwrap();
        // 旋转误差：q_est^-1 * q_gt 的 w 应接近 1。
        let err_rot = (q_est.inverse().mul(q_gt)).w;
        assert!((err_rot - 1.0).abs() < 1e-2, "rotation error w={err_rot}");
        let terr = t_est.sub(t_gt).norm();
        assert!(terr < 1e-2, "translation error {terr}");
    }

    #[test]
    fn rejects_fewer_than_three() {
        let a = [Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)];
        let b = [Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0)];
        assert!(rigid_transform(&a, &b).is_err());
    }
}
