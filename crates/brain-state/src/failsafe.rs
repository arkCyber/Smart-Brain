//! Fail-safe 看门狗：大脑“心跳”监督与强制安全兜底。

use brain_core::time::Timestamp;

/// 看门狗状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchdogStatus {
    /// 心跳正常，大脑健康。
    Armed,
    /// 心跳超时，已触发 fail-safe。
    Tripped,
    /// 曾经触发过（记录历史）。
    HadTrip,
}

/// 看门狗输出事件。
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FailsafeEvent {
    /// 心跳恢复，重新武装。
    ReArmed,
    /// 触发 fail-safe，强制进入 Loiter（自动悬停/一键返航）。
    Trip { missed_ms: u64 },
}

/// 独立的 fail-safe 看门狗模块。
///
/// 由主循环周期调用 `feed()` 喂狗。若超过 `timeout_ms` 未喂狗，
/// 判定大脑卡死，触发事件要求飞控进入自动悬停。
pub struct FailsafeWatchdog {
    timeout_ms: u64,
    last_heartbeat: Timestamp,
    status: WatchdogStatus,
    tripped_once: bool,
}

impl FailsafeWatchdog {
    /// 构造看门狗，指定判定阈值（毫秒，默认建议 50ms）。
    pub fn new(timeout_ms: u64) -> Self {
        Self {
            timeout_ms,
            last_heartbeat: 0,
            status: WatchdogStatus::Armed,
            tripped_once: false,
        }
    }

    /// 喂狗：由大脑每个心跳周期调用。
    pub fn feed(&mut self, now: Timestamp) {
        self.last_heartbeat = now;
        if self.status == WatchdogStatus::Tripped {
            self.status = WatchdogStatus::HadTrip;
            log::warn!("watchdog re-armed after trip");
        }
    }

    /// 周期检查：返回本周期是否发生了 fail-safe 事件。
    pub fn check(&mut self, now: Timestamp) -> Option<FailsafeEvent> {
        if self.tripped_once && self.status == WatchdogStatus::HadTrip {
            // 已恢复，复位 tripped_once 以便再次触发时能产生事件。
            self.tripped_once = false;
            return Some(FailsafeEvent::ReArmed);
        }

        let missed = now.saturating_sub(self.last_heartbeat);
        if !self.tripped_once && missed > self.timeout_ms {
            self.tripped_once = true;
            self.status = WatchdogStatus::Tripped;
            log::error!(
                "WATCHDOG TRIP: no heartbeat for {missed}ms (>{})",
                self.timeout_ms
            );
            return Some(FailsafeEvent::Trip { missed_ms: missed });
        }
        None
    }

    /// 当前状态。
    pub fn status(&self) -> WatchdogStatus {
        self.status
    }

    /// 阈值（毫秒）。
    pub fn timeout(&self) -> u64 {
        self.timeout_ms
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trip_when_starved() {
        let mut wd = FailsafeWatchdog::new(50);
        // 在 t=0 喂狗，随后心跳中断至 t=100（超过 50ms 阈值）。
        wd.feed(0);
        let ev = wd.check(100);
        assert!(matches!(ev, Some(FailsafeEvent::Trip { missed_ms: 100 })));
        assert_eq!(wd.status(), WatchdogStatus::Tripped);
    }

    #[test]
    fn no_trip_when_fed() {
        let mut wd = FailsafeWatchdog::new(50);
        wd.feed(1_000);
        assert!(wd.check(1_000).is_none());
        assert_eq!(wd.status(), WatchdogStatus::Armed);
    }

    #[test]
    fn rearms_after_trip() {
        let mut wd = FailsafeWatchdog::new(50);
        // 喂狗后超时 -> Trip。
        wd.feed(0);
        assert!(matches!(
            wd.check(100),
            Some(FailsafeEvent::Trip { missed_ms: 100 })
        ));
        assert_eq!(wd.status(), WatchdogStatus::Tripped);
        // 恢复喂狗 -> 状态转为 HadTrip，下一次 check 产生 ReArmed 事件。
        wd.feed(200);
        assert_eq!(wd.status(), WatchdogStatus::HadTrip);
        assert!(matches!(wd.check(200), Some(FailsafeEvent::ReArmed)));
        assert_eq!(wd.status(), WatchdogStatus::HadTrip);
        // 之后心跳正常则无事件。
        assert!(wd.check(210).is_none());
        assert!(wd.check(220).is_none());
    }

    #[test]
    fn can_trip_again_after_rearm() {
        let mut wd = FailsafeWatchdog::new(50);
        wd.feed(0);
        assert!(matches!(wd.check(100), Some(FailsafeEvent::Trip { .. })));
        wd.feed(150);
        assert!(matches!(wd.check(150), Some(FailsafeEvent::ReArmed)));
        // 再次超时 -> 再次 Trip。
        assert!(matches!(wd.check(250), Some(FailsafeEvent::Trip { .. })));
        assert_eq!(wd.status(), WatchdogStatus::Tripped);
    }
}
