//! 面包屑回溯：基于历史安全轨迹的原路返回，用于 SLAM 失败/全盲时脱困。

use brain_core::time::Timestamp;
use brain_core::Vec3;

/// 回溯模式。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BacktrackMode {
    /// 正常记录前进轨迹。
    Recording,
    /// 正在回溯（返回历史轨迹）。
    Rewinding,
    /// 已回到出发点。
    Home,
}

/// 面包屑回溯器。
///
/// 前进时每隔一段距离在内存中记录一个“面包屑”（安全位置）。一旦感知丢失
/// 或前方无法通行，切换到回溯模式，沿面包屑逐个返回，退出危险区域。
pub struct Backtracker {
    /// 面包屑队列（按时间顺序，最后一个是最近位置）。
    crumbs: Vec<Vec3>,
    /// 最大容量。
    max: usize,
    /// 记录最小间距（米）。
    min_spacing: f32,
    /// 上一个记录点。
    last_crumb: Option<Vec3>,
    mode: BacktrackMode,
    /// 回溯进度（下一个要去的面包屑下标）。
    rewind_idx: usize,
}

impl Backtracker {
    pub fn new(max: usize, min_spacing: f32) -> Self {
        Self {
            crumbs: Vec::with_capacity(max),
            max,
            min_spacing,
            last_crumb: None,
            mode: BacktrackMode::Recording,
            rewind_idx: 0,
        }
    }

    /// 记录当前安全位置（仅在间距足够时新增）。
    pub fn record(&mut self, now: Timestamp, pos: Vec3) {
        let _ = now;
        if let Some(last) = self.last_crumb {
            if last.sub(pos).norm() < self.min_spacing {
                // 更新最近面包屑位置即可，避免过密。若 `max` 为 0（不保留历史），
                // `crumbs` 可能为空，需安全处理而非 panic。
                if let Some(slot) = self.crumbs.last_mut() {
                    *slot = pos;
                }
                self.last_crumb = Some(pos);
                return;
            }
        }
        self.crumbs.push(pos);
        if self.crumbs.len() > self.max {
            self.crumbs.remove(0);
        }
        self.last_crumb = Some(pos);
    }

    /// 开始回溯：把轨迹反转，返回要依次经过的位置序列（不含当前最近点）。
    pub fn start_rewind(&mut self) -> Vec<Vec3> {
        self.mode = BacktrackMode::Rewinding;
        let mut path: Vec<Vec3> = self.crumbs.clone();
        // 移除最后一段（当前所在处）以避免原地打转。
        path.pop();
        path.reverse();
        self.rewind_idx = 0;
        path
    }

    /// 下一步要前往的位置（回溯模式）。返回 `None` 表示已回到出发点。
    pub fn next_rewind_target(&mut self) -> Option<Vec3> {
        if self.rewind_idx >= self.crumbs.len() {
            self.mode = BacktrackMode::Home;
            return None;
        }
        let idx = self.crumbs.len() - 1 - self.rewind_idx;
        self.rewind_idx += 1;
        Some(self.crumbs[idx])
    }

    /// 当前模式。
    pub fn mode(&self) -> BacktrackMode {
        self.mode
    }

    /// 已记录面包屑数量。
    pub fn len(&self) -> usize {
        self.crumbs.len()
    }

    /// 是否尚未记录任何面包屑。
    pub fn is_empty(&self) -> bool {
        self.crumbs.is_empty()
    }

    /// 全部面包屑（按时间顺序）。
    pub fn trail(&self) -> &[Vec3] {
        &self.crumbs
    }

    /// 重置。
    pub fn reset(&mut self) {
        self.crumbs.clear();
        self.last_crumb = None;
        self.mode = BacktrackMode::Recording;
        self.rewind_idx = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records_spaced_crumbs() {
        let mut b = Backtracker::new(100, 1.0);
        // 每隔 1m 记录一次。
        for i in 0..5 {
            b.record(0, Vec3::new(i as f32, 0.0, 0.0));
        }
        assert_eq!(b.len(), 5);
    }

    #[test]
    fn skips_too_close() {
        let mut b = Backtracker::new(100, 1.0);
        b.record(0, Vec3::new(0.0, 0.0, 0.0));
        b.record(0, Vec3::new(0.1, 0.0, 0.0)); // 太近，不新增
        b.record(0, Vec3::new(2.0, 0.0, 0.0)); // 足够远
        assert_eq!(b.len(), 2);
    }

    #[test]
    fn rewind_returns_reverse_path() {
        let mut b = Backtracker::new(100, 1.0);
        for i in 0..5 {
            b.record(0, Vec3::new(i as f32, 0.0, 0.0));
        }
        let path = b.start_rewind();
        // 回溯应回到 3,2,1,0。
        assert_eq!(path.len(), 4);
        assert_eq!(path[0], Vec3::new(3.0, 0.0, 0.0));
        assert_eq!(path[3], Vec3::new(0.0, 0.0, 0.0));
        // 最后一步后应 Home。
        assert_eq!(b.mode(), BacktrackMode::Rewinding);
    }

    #[test]
    fn caps_memory() {
        let mut b = Backtracker::new(3, 0.0);
        for i in 0..10 {
            b.record(0, Vec3::new(i as f32, 0.0, 0.0));
        }
        assert!(b.len() <= 3);
    }

    #[test]
    fn zero_max_does_not_panic() {
        // 回归：`max == 0` 时 `last_crumb` 仍为 Some 但 `crumbs` 被清空，
        // 更新最近面包屑不应 panic。
        let mut b = Backtracker::new(0, 1.0);
        b.record(0, Vec3::new(0.0, 0.0, 0.0));
        b.record(0, Vec3::new(0.1, 0.0, 0.0)); // 太近 → 走“更新”分支
        b.record(0, Vec3::new(0.2, 0.0, 0.0));
        assert_eq!(b.len(), 0); // 不保留历史
    }
}
