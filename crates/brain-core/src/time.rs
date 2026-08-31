//! 时间原语。
//!
//! 提供：
//! - [`instant_now`]：墙钟毫秒时间戳（心跳/时间对齐用）。
//! - [`Clock`] trait + [`SystemClock`] / [`ManualClock`]：可注入的时间源，
//!   生产用真实时钟、测试用可控时钟。
//! - [`Stopwatch`]：基于 `std::time::Instant` 的**真正单调**计时器，
//!   用于测量真实耗时（不受系统时间调整影响）。
//! - [`SyncSample`] / [`TimeSync`] / [`SyncedClock`] / [`SyncDriver`]：**NTP 风格时间同步**，
//!   用四时间戳握手估计并滤波出稳定的时钟偏移，可把远端参考时间同步到本地；
//!   [`SyncDriver`] 通过 [`SyncExchange`] 自动完成多轮握手（传输无关）。

use crate::BrainError;
use std::collections::VecDeque;
use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::sync::Mutex;
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

// =============================================================================
// 时间同步（NTP 风格四时间戳偏移估计）
// =============================================================================

/// 一次时间同步采样：握手两侧的四个时间戳。
///
/// - `t1`：客户端发送请求时的**本地**时刻。
/// - `t2`：服务端（参考时钟）收到请求时的**参考**时刻。
/// - `t3`：服务端发送响应时的**参考**时刻。
/// - `t4`：客户端收到响应时的**本地**时刻。
///
/// 在对称单程延迟的假设下满足经典 NTP 偏移公式：
/// `offset = ((t2 - t1) + (t3 - t4)) / 2`，`rtt = (t4 - t1) - (t3 - t2)`。
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct SyncSample {
    pub t1: Timestamp,
    pub t2: Timestamp,
    pub t3: Timestamp,
    pub t4: Timestamp,
}

impl SyncSample {
    pub fn new(t1: Timestamp, t2: Timestamp, t3: Timestamp, t4: Timestamp) -> Self {
        Self { t1, t2, t3, t4 }
    }

    /// 采样是否满足时间单调性（`t1 <= t4` 且 `t2 <= t3`）。不满足视为无效采样。
    pub fn is_valid(&self) -> bool {
        self.t1 <= self.t4 && self.t2 <= self.t3
    }

    /// 估计的时钟偏移（毫秒）：`参考时间 - 本地时间`。正数表示参考时钟更快。
    /// 用 `i128` 中间量避免大时间戳相减溢出。
    pub fn offset_ms(&self) -> i64 {
        let a = (self.t2 as i128).wrapping_sub(self.t1 as i128);
        let b = (self.t3 as i128).wrapping_sub(self.t4 as i128);
        ((a + b) / 2) as i64
    }

    /// 往返时延 RTT（毫秒），最小钳制为 0（时钟抖动可能产生轻微负值）。
    pub fn rtt_ms(&self) -> u64 {
        let rtt = (self.t4 as i128).wrapping_sub(self.t1 as i128)
            - (self.t3 as i128).wrapping_sub(self.t2 as i128);
        rtt.max(0) as u64
    }
}

/// 时间同步估计器：维护一个滑动窗口的 [`SyncSample`]，用**中位数**剔除抖动与
/// 异常值，并拒绝 RTT 超过阈值的采样，最终给出稳定可靠的时钟偏移。
///
/// - 线程安全（内部原子量 + `Mutex`）。
/// - 偏移量 `offset = 参考时间 - 本地时间`。
/// - 生产用真实时钟、测试用 [`ManualClock`] 注入。
#[derive(Debug)]
pub struct TimeSync {
    offset: AtomicI64,
    last_rtt: AtomicU64,
    samples: Mutex<VecDeque<SyncSample>>,
    max_samples: usize,
    max_rtt_ms: u64,
    /// 达到该样本数即视为“已同步”（用于健康判定）。
    min_samples: usize,
    /// 最近一次被接受采样的 `t4`（用于同步时效/看门狗判定）。
    last_update: AtomicU64,
}

impl Default for TimeSync {
    fn default() -> Self {
        Self {
            offset: AtomicI64::new(0),
            last_rtt: AtomicU64::new(0),
            samples: Mutex::new(VecDeque::with_capacity(Self::DEFAULT_MAX_SAMPLES)),
            max_samples: Self::DEFAULT_MAX_SAMPLES,
            max_rtt_ms: Self::DEFAULT_MAX_RTT_MS,
            min_samples: Self::DEFAULT_MIN_SAMPLES,
            last_update: AtomicU64::new(0),
        }
    }
}

impl TimeSync {
    /// 默认滑动窗口大小。
    pub const DEFAULT_MAX_SAMPLES: usize = 8;
    /// 默认 RTT 拒收阈值（毫秒）。
    pub const DEFAULT_MAX_RTT_MS: u64 = 1000;
    /// 默认“已同步”所需的最小样本数。
    pub const DEFAULT_MIN_SAMPLES: usize = 3;

    /// 以默认参数创建一个偏移为 0 的估计器。
    pub fn new() -> Self {
        Self::default()
    }

    /// 覆盖滑动窗口大小（至少为 1）。
    pub fn with_max_samples(mut self, n: usize) -> Self {
        self.max_samples = n.max(1);
        self
    }

    /// 覆盖 RTT 拒收阈值（毫秒）。RTT 超过该值的采样会被丢弃。
    pub fn with_max_rtt(mut self, ms: u64) -> Self {
        self.max_rtt_ms = ms;
        self
    }

    /// 覆盖“已同步”所需的最小样本数（至少为 1）。
    pub fn with_min_samples(mut self, n: usize) -> Self {
        self.min_samples = n.max(1);
        self
    }

    /// 滑动窗口大小。
    pub fn max_samples(&self) -> usize {
        self.max_samples
    }

    /// RTT 拒收阈值（毫秒）。
    pub fn max_rtt_ms(&self) -> u64 {
        self.max_rtt_ms
    }

    /// “已同步”所需的最小样本数。
    pub fn min_samples(&self) -> usize {
        self.min_samples
    }

    /// 当前估计的时钟偏移（毫秒）。
    pub fn offset_ms(&self) -> i64 {
        self.offset.load(Ordering::Relaxed)
    }

    /// 最近一次被接受的采样的 RTT（毫秒）。
    pub fn last_rtt_ms(&self) -> u64 {
        self.last_rtt.load(Ordering::Relaxed)
    }

    /// 加锁采样窗口。若某线程曾在持锁时 panic 导致互斥量中毒，则忽略中毒
    /// （`into_inner` 恢复内部数据）继续运行，避免后续所有调用永久 panic——
    /// 时间同步属于 fail-safe/看门狗路径，绝不能因一次异常而瘫痪。
    fn window(&self) -> std::sync::MutexGuard<'_, VecDeque<SyncSample>> {
        self.samples.lock().unwrap_or_else(|p| p.into_inner())
    }

    /// 当前窗口内已收集的有效采样数。
    pub fn samples(&self) -> usize {
        self.window().len()
    }

    /// 是否已达到“已同步”所需的最小样本数。
    pub fn is_synced(&self) -> bool {
        self.samples() >= self.min_samples
    }

    /// 若已同步则返回当前偏移估计，否则返回 `None`（尚未收敛）。
    pub fn estimate_offset(&self) -> Option<i64> {
        if self.is_synced() {
            Some(self.offset_ms())
        } else {
            None
        }
    }

    /// 最近一次被接受的采样的 `t4`（毫秒）；尚无采样时为 0。
    pub fn last_update(&self) -> Timestamp {
        self.last_update.load(Ordering::Relaxed)
    }

    /// 距最近一次被接受的采样已过去多少毫秒（`now - last_update`）。
    pub fn age(&self, now: Timestamp) -> u64 {
        now.saturating_sub(self.last_update())
    }

    /// 同步是否已失效：要么从未达到最小样本数，要么距上次同步超过 `timeout_ms`。
    pub fn is_stale(&self, now: Timestamp, timeout_ms: u64) -> bool {
        !self.is_synced() || self.age(now) > timeout_ms
    }

    /// 直接设定偏移（例如收到权威时间基准或冷启动已知偏移时）。
    pub fn set_offset(&self, ms: i64) {
        self.offset.store(ms, Ordering::Relaxed);
    }

    /// 用一次握手采样更新估计。采样无效或 RTT 超阈值时返回 `None` 且不改动状态，
    /// 否则写入新偏移并返回 `Some(新偏移)`。
    pub fn observe(&self, s: SyncSample) -> Option<i64> {
        if !s.is_valid() {
            return None;
        }
        if s.rtt_ms() > self.max_rtt_ms {
            return None;
        }

        let mut window = self.window();
        window.push_back(s);
        while window.len() > self.max_samples {
            window.pop_front();
        }
        let offsets: Vec<i64> = window.iter().map(SyncSample::offset_ms).collect();
        let median = median_i64(&offsets);
        self.offset.store(median, Ordering::Relaxed);
        self.last_rtt.store(s.rtt_ms(), Ordering::Relaxed);
        self.last_update.store(s.t4, Ordering::Relaxed);
        Some(median)
    }

    /// [`observe`](Self::observe) 的便捷封装：直接传入四时间戳。
    pub fn estimate(
        &self,
        t1: Timestamp,
        t2: Timestamp,
        t3: Timestamp,
        t4: Timestamp,
    ) -> Option<i64> {
        self.observe(SyncSample::new(t1, t2, t3, t4))
    }

    /// 清空采样窗口并把偏移归零。
    pub fn reset(&self) {
        self.offset.store(0, Ordering::Relaxed);
        self.last_rtt.store(0, Ordering::Relaxed);
        self.last_update.store(0, Ordering::Relaxed);
        self.window().clear();
    }

    /// 把本地时间戳换算成参考（远端）时间：`local + offset`，结果钳制非负。
    pub fn to_reference(&self, local: Timestamp) -> Timestamp {
        apply_offset(local, self.offset_ms())
    }

    /// 把参考时间戳换算成本地时间：`reference - offset`，结果钳制非负。
    pub fn to_local(&self, reference: Timestamp) -> Timestamp {
        apply_offset(reference, -self.offset_ms())
    }

    /// 基于任意本地时钟返回当前参考时间。
    pub fn now<C: Clock>(&self, local: &C) -> Timestamp {
        self.to_reference(local.now_ms())
    }
}

/// 把偏移（可为负）加到时间戳上，结果钳制在 `[0, u64::MAX]`。
fn apply_offset(t: Timestamp, offset: i64) -> Timestamp {
    let r = (t as i128) + (offset as i128);
    if r <= 0 {
        0
    } else {
        r as u64
    }
}

/// 求 `i64` 列表的中位数（输入非空）。
fn median_i64(v: &[i64]) -> i64 {
    debug_assert!(!v.is_empty());
    let mut sorted = v.to_vec();
    sorted.sort_unstable();
    let n = sorted.len();
    if n % 2 == 1 {
        sorted[n / 2]
    } else {
        (sorted[n / 2 - 1] + sorted[n / 2]) / 2
    }
}

/// 一次完整时间同步握手的抽象：负责发起握手并返回客户端观察到的四个时间戳。
///
/// 实现方应自行读取本地时钟的 `t1`/`t4`（本地）与服务端时钟的 `t2`/`t3`（参考），
/// 可对接 UDP/串口/Zenoh 等真实传输；握手失败返回 [`BrainError`]。
pub trait SyncExchange: Send {
    /// 执行一次完整握手，返回客户端观察到的 [`SyncSample`]。
    fn exchange(&mut self) -> Result<SyncSample, BrainError>;
}

/// 时间同步驱动：把 [`TimeSync`] 的估计逻辑与具体的握手交换组合起来，自动完成
/// 多轮同步。传输无关（通过 [`SyncExchange`] 注入），可安全地在多个线程中并发调用。
#[derive(Debug)]
pub struct SyncDriver {
    sync: TimeSync,
}

impl Default for SyncDriver {
    fn default() -> Self {
        Self {
            sync: TimeSync::new(),
        }
    }
}

impl SyncDriver {
    /// 以指定的 [`TimeSync`] 构造驱动。
    pub fn new(sync: TimeSync) -> Self {
        Self { sync }
    }

    /// 底层估计器的引用（可读取偏移/健康状态，或配置阈值）。
    pub fn sync(&self) -> &TimeSync {
        &self.sync
    }

    /// 执行一轮握手并吸收采样。握手失败时返回 `Err`；采样被拒绝（无效/RTT 超阈值）
    /// 时返回 `Ok(None)`，否则返回 `Ok(Some(新偏移))`。
    pub fn run_round<E: SyncExchange>(&self, ex: &mut E) -> Result<Option<i64>, BrainError> {
        let sample = ex.exchange()?;
        Ok(self.sync.observe(sample))
    }

    /// 自动执行至多 `rounds` 轮握手：一旦达到“已同步”所需最小样本数即提前停止。
    /// 返回最终偏移估计（未收敛时返回 `None`）。任一握手失败立即返回 `Err`。
    pub fn synchronize<E: SyncExchange>(
        &self,
        ex: &mut E,
        rounds: usize,
    ) -> Result<Option<i64>, BrainError> {
        for _ in 0..rounds {
            self.run_round(ex)?;
            if self.sync.is_synced() {
                break;
            }
        }
        Ok(self.sync.estimate_offset())
    }
}

/// 包装一个本地时钟与一个 [`TimeSync`] 偏移，对外表现为“参考时钟”。
/// 可把远端（飞控/服务器）时间当作统一时间基准广播给各模块。
#[derive(Debug)]
pub struct SyncedClock<C: Clock> {
    local: C,
    sync: TimeSync,
}

impl<C: Clock> SyncedClock<C> {
    /// 以本地时钟与同步器构造。
    pub fn new(local: C, sync: TimeSync) -> Self {
        Self { local, sync }
    }

    /// 用固定偏移直接构造（例如已知权威偏移的冷启动）。
    pub fn with_offset(local: C, offset_ms: i64) -> Self {
        let sync = TimeSync::new();
        sync.set_offset(offset_ms);
        Self { local, sync }
    }

    /// 底层的同步器引用。
    pub fn sync(&self) -> &TimeSync {
        &self.sync
    }

    /// 底层本地时钟的时间。
    pub fn local_time(&self) -> Timestamp {
        self.local.now_ms()
    }
}

impl<C: Clock> Clock for SyncedClock<C> {
    fn now_ms(&self) -> Timestamp {
        self.sync.to_reference(self.local.now_ms())
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

    // ---------------------------------------------------------------------
    // 时间同步
    // ---------------------------------------------------------------------

    /// 构造一个满足物理模型的采样：
    /// `t1`(本地) -> `t2 = t1 + offset + d1`(参考) -> `t3 = t2 + p`(参考) -> `t4 = t3 - offset + d2`(本地)。
    fn make_sample(offset: i64, d1: u64, d2: u64, p: u64, t1: Timestamp) -> SyncSample {
        let t2 = (t1 as i128 + offset as i128 + d1 as i128) as u64;
        let t3 = t2 + p;
        let t4 = (t3 as i128 - offset as i128 + d2 as i128) as u64;
        SyncSample::new(t1, t2, t3, t4)
    }

    #[test]
    fn sync_sample_offset_symmetric_delay() {
        // 对称延迟：估计应精确等于真实偏移。
        let s = make_sample(5_000, 10, 10, 5, 100_000);
        assert_eq!(s.offset_ms(), 5_000);
        assert_eq!(s.rtt_ms(), 20);
        assert!(s.is_valid());
    }

    #[test]
    fn sync_sample_offset_asymmetric_delay() {
        // 非对称延迟 d1=30, d2=10：偏移带 (d1-d2)/2 = 10 的偏差。
        let s = make_sample(5_000, 30, 10, 5, 100_000);
        assert_eq!(s.offset_ms(), 5_010);
        assert_eq!(s.rtt_ms(), 40);
    }

    #[test]
    fn sync_sample_negative_offset() {
        // 参考时钟比本地慢：偏移为负。
        let s = make_sample(-4_000, 10, 10, 5, 100_000);
        assert_eq!(s.offset_ms(), -4_000);
        assert_eq!(s.rtt_ms(), 20);
    }

    #[test]
    fn sync_sample_invalid_ordering_rejected() {
        // t1 > t4：不可能发生，判定为无效。
        let bad = SyncSample::new(200, 300, 400, 100);
        assert!(!bad.is_valid());
        // t2 > t3：服务端在收到前就发送，无效。
        let bad2 = SyncSample::new(100, 400, 300, 500);
        assert!(!bad2.is_valid());
    }

    #[test]
    fn timesync_converges_on_true_offset() {
        let sync = TimeSync::new();
        for i in 0..16u64 {
            let s = make_sample(7_777, 5, 5, 2, 1_000 + i * 100);
            sync.observe(s).unwrap();
        }
        assert_eq!(sync.offset_ms(), 7_777);
        assert_eq!(sync.last_rtt_ms(), 10);
    }

    #[test]
    fn timesync_rejects_rtt_over_threshold() {
        let sync = TimeSync::with_max_rtt(TimeSync::new(), 50);
        // RTT = 2000，超阈值 -> 拒绝，偏移不变。
        let bad = make_sample(3_000, 1000, 1000, 0, 100_000);
        assert_eq!(bad.rtt_ms(), 2000);
        assert!(sync.observe(bad).is_none());
        assert_eq!(sync.offset_ms(), 0);
        assert_eq!(sync.samples(), 0);

        // RTT = 20，可接受。
        let good = make_sample(3_000, 10, 10, 0, 100_000);
        assert_eq!(sync.observe(good), Some(3_000));
        assert_eq!(sync.samples(), 1);
    }

    #[test]
    fn timesync_rejects_invalid_sample() {
        let sync = TimeSync::new();
        assert!(sync.observe(SyncSample::new(200, 300, 400, 100)).is_none());
        assert_eq!(sync.samples(), 0);
    }

    #[test]
    fn timesync_median_rejects_outliers() {
        // 5 个正确采样 + 1 个被噪声污染的采样（RTT 仍在阈值内，但偏移巨大）。
        let sync = TimeSync::new();
        for i in 0..5u64 {
            let s = make_sample(2_000, 8, 8, 2, 50_000 + i * 100);
            sync.observe(s).unwrap();
        }
        let outlier = make_sample(2_000, 8, 8, 2, 50_600);
        let outlier = SyncSample {
            t4: outlier.t4 + 100, // 本地到达时间异常，使 offset 失真
            ..outlier
        };
        sync.observe(outlier).unwrap();
        // 中位数滤波器让估计仍接近真实偏移 2000。
        assert_eq!(sync.offset_ms(), 2_000);
    }

    #[test]
    fn timesync_window_trims_to_max_samples() {
        let sync = TimeSync::with_max_samples(TimeSync::new(), 3);
        for i in 0..10u64 {
            let s = make_sample(1_000, 5, 5, 1, i * 100);
            sync.observe(s).unwrap();
        }
        assert_eq!(sync.samples(), 3);
    }

    #[test]
    fn timesync_to_reference_and_to_local_roundtrip() {
        let sync = TimeSync::new();
        sync.observe(make_sample(5_000, 10, 10, 5, 100_000))
            .unwrap();
        let local = 123_456u64;
        let reference = sync.to_reference(local);
        assert_eq!(reference, local + 5_000);
        assert_eq!(sync.to_local(reference), local);
    }

    #[test]
    fn timesync_negative_offset_clamps_to_zero() {
        let sync = TimeSync::new();
        sync.set_offset(-9_999);
        assert_eq!(sync.to_reference(100), 0);
        assert_eq!(sync.to_reference(9_999), 0);
        assert_eq!(sync.to_reference(10_000), 1);
    }

    #[test]
    fn timesync_set_and_reset() {
        let sync = TimeSync::new();
        sync.set_offset(1_234);
        assert_eq!(sync.offset_ms(), 1_234);
        sync.observe(make_sample(9_999, 5, 5, 1, 10)).unwrap();
        assert_eq!(sync.samples(), 1);
        sync.reset();
        assert_eq!(sync.offset_ms(), 0);
        assert_eq!(sync.samples(), 0);
        assert_eq!(sync.last_rtt_ms(), 0);
    }

    #[test]
    fn timesync_now_with_manual_clock() {
        let sync = TimeSync::new();
        sync.observe(make_sample(2_000, 5, 5, 1, 100)).unwrap();
        let local = ManualClock::new(10_000);
        assert_eq!(sync.now(&local), 12_000);
    }

    #[test]
    fn synced_clock_reports_reference_time() {
        let local = ManualClock::new(1_000);
        let sync = TimeSync::new();
        sync.set_offset(5_000);
        let clock = SyncedClock::new(local, sync);
        assert_eq!(clock.local_time(), 1_000);
        assert_eq!(clock.now_ms(), 6_000);
    }

    #[test]
    fn synced_clock_with_offset_constructor() {
        let clock = SyncedClock::with_offset(ManualClock::new(500), -250);
        assert_eq!(clock.now_ms(), 250);
    }

    #[test]
    fn synced_clock_is_a_clock_trait_object() {
        let boxes: Vec<Box<dyn Clock>> = vec![
            Box::new(SyncedClock::with_offset(ManualClock::new(1_000), 5_000)),
            Box::new(SystemClock),
        ];
        assert_eq!(boxes[0].now_ms(), 6_000);
    }

    #[test]
    fn time_sync_is_send_sync_and_shareable() {
        // TimeSync 需可跨线程共享（原子 + 互斥）。
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<TimeSync>();
        let sync = std::sync::Arc::new(TimeSync::new());
        let handle = {
            let sync = sync.clone();
            std::thread::spawn(move || {
                sync.observe(make_sample(1_000, 5, 5, 1, 100)).unwrap();
                sync.offset_ms()
            })
        };
        assert_eq!(handle.join().unwrap(), 1_000);
    }

    // ---------------------------------------------------------------------
    // 时间同步：健康状态、serde、握手驱动
    // ---------------------------------------------------------------------

    #[test]
    fn sync_sample_serde_roundtrip() {
        let s = make_sample(4_000, 5, 5, 1, 100_000);
        let json = serde_json::to_string(&s).unwrap();
        let back: SyncSample = serde_json::from_str(&json).unwrap();
        assert_eq!(back, s);
    }

    #[test]
    fn timesync_is_synced_threshold() {
        // 默认 min_samples = 3。
        let sync = TimeSync::new();
        assert!(!sync.is_synced());
        assert_eq!(sync.estimate_offset(), None);

        for i in 0..3u64 {
            sync.observe(make_sample(1_000, 5, 5, 1, 10_000 + i * 100))
                .unwrap();
            assert_eq!(sync.is_synced(), i >= 2);
        }
        assert_eq!(sync.estimate_offset(), Some(1_000));
    }

    #[test]
    fn timesync_min_samples_configurable() {
        let sync = TimeSync::with_min_samples(TimeSync::new(), 1);
        assert!(!sync.is_synced());
        sync.observe(make_sample(2_000, 5, 5, 1, 50)).unwrap();
        assert!(sync.is_synced());
    }

    #[test]
    fn timesync_health_and_staleness() {
        // min_samples = 1：一次采样即视为已同步，便于单独验证“时效”逻辑。
        let sync = TimeSync::with_min_samples(TimeSync::new(), 1);
        // 无采样：无 last_update、立即失效。
        assert_eq!(sync.last_update(), 0);
        assert!(sync.is_stale(100_000, 1000));

        let s = make_sample(3_000, 5, 5, 1, 50_000);
        sync.observe(s).unwrap();
        // last_update 记录最近一次接受的 t4。
        assert_eq!(sync.last_update(), s.t4);
        assert_eq!(sync.age(s.t4 + 100), 100);
        // 已同步且未超时 -> 新鲜。
        assert!(sync.is_synced());
        assert!(!sync.is_stale(s.t4 + 100, 1000));
        // 超过超时阈值即失效。
        assert!(sync.is_stale(s.t4 + 1001, 1000));
    }

    #[test]
    fn timesync_reset_clears_health() {
        let sync = TimeSync::new();
        let s = make_sample(3_000, 5, 5, 1, 50_000);
        sync.observe(s).unwrap();
        sync.reset();
        assert_eq!(sync.last_update(), 0);
        assert!(sync.is_stale(100_000, 1000));
    }

    /// 模拟一个带往返延迟与固定偏移的服务器，实现 [`SyncExchange`]。
    struct MockExchange {
        offset: i64,
        d1: u64,
        d2: u64,
        p: u64,
        t: u64,
        fail_after: Option<u64>,
        calls: u64,
    }

    impl MockExchange {
        fn new(offset: i64) -> Self {
            Self {
                offset,
                d1: 3,
                d2: 3, // 对称延迟：偏移估计精确等于真值
                p: 2,
                t: 1_000_000,
                fail_after: None,
                calls: 0,
            }
        }
    }

    impl SyncExchange for MockExchange {
        fn exchange(&mut self) -> Result<SyncSample, BrainError> {
            self.calls += 1;
            if let Some(n) = self.fail_after {
                if self.calls > n {
                    return Err(BrainError::Transport("link down".into()));
                }
            }
            let t1 = self.t;
            let t2 = (t1 as i128 + self.offset as i128 + self.d1 as i128) as u64;
            let t3 = t2 + self.p;
            let t4 = (t3 as i128 - self.offset as i128 + self.d2 as i128) as u64;
            self.t += 100; // 每轮推进本地时间
            Ok(SyncSample::new(t1, t2, t3, t4))
        }
    }

    #[test]
    fn sync_driver_converges_end_to_end() {
        let driver = SyncDriver::new(TimeSync::new()); // min_samples = 3
        let mut ex = MockExchange::new(5_000);
        let estimate = driver.synchronize(&mut ex, 8).unwrap();
        assert_eq!(estimate, Some(5_000));
        assert!(driver.sync().is_synced());
        assert_eq!(driver.sync().samples(), 3); // 达到 3 个即提前停止
    }

    #[test]
    fn sync_driver_returns_none_when_not_synced() {
        let sync = TimeSync::with_min_samples(TimeSync::new(), 10);
        let driver = SyncDriver::new(sync);
        let mut ex = MockExchange::new(5_000);
        // 只允许 3 轮，不足 10 -> 未同步。
        let estimate = driver.synchronize(&mut ex, 3).unwrap();
        assert_eq!(estimate, None);
    }

    #[test]
    fn sync_driver_run_round_reports_rejection() {
        let driver = SyncDriver::new(TimeSync::with_max_rtt(TimeSync::new(), 5));
        let mut ex = MockExchange::new(5_000); // RTT = d1+d2 = 7 > 5
        assert_eq!(driver.run_round(&mut ex).unwrap(), None);
    }

    #[test]
    fn sync_driver_propagates_exchange_error() {
        let driver = SyncDriver::new(TimeSync::new());
        let mut ex = MockExchange::new(5_000);
        ex.fail_after = Some(1);
        // 第二轮握手失败 -> Err。
        let err = driver.synchronize(&mut ex, 4).unwrap_err();
        assert!(matches!(err, BrainError::Transport(_)));
    }
}
