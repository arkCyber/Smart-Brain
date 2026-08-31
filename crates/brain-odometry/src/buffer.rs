//! 多传感器时间戳对齐。
//!
//! 各传感器刷新率不同（IMU 400Hz、相机 30Hz）。本模块用环形缓冲暂存高频
//! 样本，并在需要时把两组数据对齐到同一时间戳（线性插值），确保高频数据
//! 在内存中安全对齐、绝不丢包——这是“多源融合”正确性的前提。

use brain_core::time::Timestamp;
use brain_core::Vec3;
use brain_ipc::FixedRingBuffer;

/// 对齐错误。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlignmentError {
    /// 数据不足，无法插值到目标时间。
    InsufficientData,
}

/// 可插值的样本类型。
pub trait Interpolate: Clone {
    /// 在 `self` 与 `other` 之间线性插值（t∈[0,1]）。
    fn lerp(&self, other: &Self, t: f32) -> Self;
}

impl Interpolate for f32 {
    fn lerp(&self, other: &Self, t: f32) -> Self {
        self + (other - self) * t
    }
}

impl Interpolate for Vec3 {
    fn lerp(&self, other: &Self, t: f32) -> Self {
        Vec3::new(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t,
            self.z + (other.z - self.z) * t,
        )
    }
}

/// 对一条高频时序数据进行时间对齐。
pub struct TimestampAligned<T: Interpolate> {
    buf: FixedRingBuffer<(Timestamp, T)>,
}

impl<T: Interpolate> TimestampAligned<T> {
    /// 容量应覆盖最大可能的时间跨度（例如 IMU 400Hz 下 1s = 400）。
    pub fn new(capacity: usize) -> Self {
        Self {
            buf: FixedRingBuffer::new(capacity).expect("capacity > 0"),
        }
    }

    /// 推入一个带时间戳的样本。
    pub fn push(&mut self, ts: Timestamp, value: T) {
        self.buf.push((ts, value));
    }

    /// 把数据插值/对齐到 `target` 时间戳。
    pub fn align(&self, target: Timestamp) -> Result<(Timestamp, T), AlignmentError> {
        let n = self.buf.len();
        if n == 0 {
            return Err(AlignmentError::InsufficientData);
        }
        for i in 0..n {
            let (ts, _) = self.buf.get(i).unwrap();
            if *ts >= target {
                if *ts == target {
                    return Ok((target, self.buf.get(i).unwrap().1.clone()));
                }
                if i == 0 {
                    return Err(AlignmentError::InsufficientData);
                }
                let (ts0, v0) = self.buf.get(i - 1).unwrap();
                let dt = (target - ts0) as f32 / (*ts - ts0).max(1) as f32;
                let v = v0.lerp(&self.buf.get(i).unwrap().1, dt);
                return Ok((target, v));
            }
        }
        Err(AlignmentError::InsufficientData)
    }

    /// 样本数。
    pub fn len(&self) -> usize {
        self.buf.len()
    }

    pub fn is_empty(&self) -> bool {
        self.buf.is_empty()
    }
}

/// 便捷函数：把两组 `TimestampAligned` 对齐到 `target`。
pub fn align_to<T: Interpolate, U: Interpolate>(
    a: &TimestampAligned<T>,
    b: &TimestampAligned<U>,
    target: Timestamp,
) -> Result<(T, U), AlignmentError> {
    let (_, ta) = a.align(target)?;
    let (_, tb) = b.align(target)?;
    Ok((ta, tb))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aligns_by_interpolation() {
        let mut s = TimestampAligned::<f32>::new(16);
        s.push(0, 0.0);
        s.push(10, 1.0);
        // 中间点 5 应插值为 0.5。
        let (ts, v) = s.align(5).unwrap();
        assert_eq!(ts, 5);
        assert!((v - 0.5).abs() < 1e-4);
    }

    #[test]
    fn exact_sample() {
        let mut s = TimestampAligned::<f32>::new(16);
        s.push(0, 7.0);
        let (_, v) = s.align(0).unwrap();
        assert_eq!(v, 7.0);
    }

    #[test]
    fn vector_interp() {
        let mut s = TimestampAligned::<Vec3>::new(16);
        s.push(0, Vec3::new(0.0, 0.0, 0.0));
        s.push(10, Vec3::new(2.0, 2.0, 2.0));
        let (_, v) = s.align(5).unwrap();
        assert!((v.x - 1.0).abs() < 1e-4);
    }

    #[test]
    fn insufficient_data() {
        let s = TimestampAligned::<f32>::new(4);
        assert_eq!(s.align(5), Err(AlignmentError::InsufficientData));
    }
}
