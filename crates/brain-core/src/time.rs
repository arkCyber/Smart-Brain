//! 时间原语。
//!
//! 提供：
//! - [`instant_now`]：墙钟毫秒时间戳（心跳/时间对齐用）。
//! - [`Clock`] trait + [`SystemClock`] / [`ManualClock`]：可注入的时间源，
//!   生产用真实时钟、测试用可控时钟。
//! - [`Stopwatch`]：基于 `std::time::Instant` 的**真正单调**计时器，
//!   用于测量真实耗时（不受系统时间调整影响）。

use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

/// 单调相对时间戳（毫秒）。所有模块用它作为“心跳时间戳”。
pub type Timestamp = u64;

/// 获取当前相对时间戳（毫秒，基于墙钟）。
pub fn instant_now() -> Timestamp {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0)
}

/// 时间戳差值（毫秒），用于计算两次事件间隔。
pub fn elapsed_since(last: Timestamp) -> u64 {
    instant_now().saturating_sub(last)
}

/// 可注入的时间源抽象。生产用 [`SystemClock`]，测试/仿真用 [`ManualClock`]。
pub trait Clock: Send + Sync {
    /// 当前时间戳（毫秒）。
    fn now_ms(&self) -> Timestamp;
}

/// 基于系统墙钟的时钟。
#[derive(Debug, Clone, Copy, Default)]
pub struct SystemClock;

impl Clock for SystemClock {
    fn now_ms(&self) -> Timestamp {
        instant_now()
    }
}

/// 可手动推进的时钟（测试/仿真用），初始值可设定。
#[derive(Debug, Default)]
pub struct ManualClock {
    now: AtomicU64,
}

impl ManualClock {
    /// 以 `start` 毫秒初始化。
    pub fn new(start: Timestamp) -> Self {
        Self {
            now: AtomicU64::new(start),
        }
    }

    /// 推进 `ms` 毫秒。
    pub fn advance(&self, ms: u64) {
        self.now.fetch_add(ms, Ordering::Relaxed);
    }

    /// 设置到指定时刻（可回溯）。
    pub fn set(&self, t: Timestamp) {
        self.now.store(t, Ordering::Relaxed);
    }
}

impl Clock for ManualClock {
    fn now_ms(&self) -> Timestamp {
        self.now.load(Ordering::Relaxed)
    }
}

/// 真正单调的计时器，用于测量实际耗时（不受墙钟调整影响）。
#[derive(Debug, Clone)]
pub struct Stopwatch {
    start: Instant,
}

impl Default for Stopwatch {
    fn default() -> Self {
        Self::start()
    }
}

impl Stopwatch {
    /// 启动计时。
    pub fn start() -> Self {
        Self {
            start: Instant::now(),
        }
    }

    /// 已耗时（毫秒）。
    pub fn elapsed_ms(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }

    /// 已耗时（秒，浮点）。
    pub fn elapsed_secs(&self) -> f32 {
        self.start.elapsed().as_secs_f32()
    }

    /// 重置计时起点。
    pub fn restart(&mut self) {
        self.start = Instant::now();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn timestamps_monotonic() {
        let a = instant_now();
        let b = instant_now();
        assert!(b >= a);
    }

    #[test]
    fn system_clock_is_increasing() {
        let c = SystemClock;
        let a = c.now_ms();
        let b = c.now_ms();
        assert!(b >= a);
    }

    #[test]
    fn manual_clock_is_controllable() {
        let c = ManualClock::new(1000);
        assert_eq!(c.now_ms(), 1000);
        c.advance(250);
        assert_eq!(c.now_ms(), 1250);
        c.set(5);
        assert_eq!(c.now_ms(), 5);
    }

    #[test]
    fn clock_trait_object_works() {
        let boxes: Vec<Box<dyn Clock>> =
            vec![Box::new(SystemClock), Box::new(ManualClock::new(42))];
        // 手动时钟可预测。
        assert_eq!(boxes[1].now_ms(), 42);
    }

    #[test]
    fn stopwatch_measures_and_restarts() {
        let mut sw = Stopwatch::start();
        let t0 = sw.elapsed_ms();
        assert!(t0 < 1000);
        sw.restart();
        let t1 = sw.elapsed_ms();
        assert!(t1 < 1000);
    }
}
