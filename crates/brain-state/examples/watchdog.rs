//! `brain-state` 最小示例：Fail-safe 看门狗——心跳超时触发自动悬停。
//!
//! 运行：`cargo run -p brain-state --example watchdog`

use brain_state::{FailsafeEvent, FailsafeWatchdog, WatchdogStatus};

fn main() {
    // 心跳阈值 50ms
    let mut wd = FailsafeWatchdog::new(50);
    wd.feed(0); // 在 t=0 喂狗

    // 心跳中断到 t=100（>50ms）→ 应触发 fail-safe
    match wd.check(100) {
        Some(FailsafeEvent::Trip { missed_ms }) => {
            println!("WATCHDOG TRIPPED after {missed_ms}ms → force LOITER");
        }
        other => println!("unexpected: {other:?}"),
    }
    assert_eq!(wd.status(), WatchdogStatus::Tripped);

    // 恢复喂狗 → 重新武装
    wd.feed(200);
    println!("status after re-arm = {:?}", wd.status());
}
