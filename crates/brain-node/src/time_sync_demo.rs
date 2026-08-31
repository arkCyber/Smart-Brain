//! 时间同步（NTP 风格）演示。
//!
//! 模拟一台“服务器/飞控”（参考时钟）与一台“大脑”客户端之间的时钟握手：
//! 用 [`SyncDriver`] 自动执行多轮四时间戳交换，`TimeSync` 估计并收敛出真实
//! 时钟偏移，再用 `to_reference` 把本地时间换算成统一的参考时间。

use brain_core::time::{Clock, ManualClock, SyncDriver, SyncExchange, SyncSample, TimeSync};
use brain_core::Result;

/// 一个模拟“服务器（参考时钟）+ 往返链路”的握手实现。
struct DemoExchange {
    server: ManualClock,
    client: ManualClock,
    offset: i64,
}

impl SyncExchange for DemoExchange {
    fn exchange(&mut self) -> Result<SyncSample> {
        let t1 = self.client.now_ms();
        let t2 = (t1 as i128 + self.offset as i128 + 3) as u64; // 3ms 上行延迟
        let t3 = t2 + 1; // 服务端处理 1ms
        let t4 = (t3 as i128 - self.offset as i128 + 3) as u64; // 3ms 下行延迟
                                                                // 双方时钟各自推进 20ms，进入下一轮。
        self.client.advance(20);
        self.server.advance(20);
        Ok(SyncSample::new(t1, t2, t3, t4))
    }
}

/// 顶层入口。
pub fn run() {
    println!("\n=== 时间同步（NTP 风格四时间戳握手）===");

    // 参考（服务端）时钟比本地快 5000ms。
    let true_offset: i64 = 5_000;
    let server = ManualClock::new(1_000_000); // 参考时钟
    let client = ManualClock::new(995_000); // 本地时钟，落后 5000ms
    let driver = SyncDriver::new(TimeSync::new()); // 默认 min_samples = 3
    let mut ex = DemoExchange {
        server,
        client,
        offset: true_offset,
    };

    println!("  参考时钟领先本地 {true_offset} ms，开始自动握手…");
    let mut round = 0;
    while !driver.sync().is_synced() && round < 8 {
        let r = driver.run_round(&mut ex).expect("handshake ok");
        println!(
            "  第 {round} 轮: 估计偏移 = {r:?}  RTT = {} ms",
            driver.sync().last_rtt_ms()
        );
        round += 1;
    }

    assert_eq!(driver.sync().offset_ms(), true_offset, "应收敛到真实偏移");
    assert!(driver.sync().is_synced());

    // 用同步后的偏移把本地时间换算成统一参考时间，并与真实服务端时钟对比。
    let local = ex.client.now_ms();
    let reference = driver.sync().to_reference(local);
    println!(
        "  同步后: 本地时间 = {local} ms -> 参考时间 = {reference} ms (服务端实际 = {} ms)",
        ex.server.now_ms()
    );
    assert_eq!(reference, ex.server.now_ms(), "参考时间应与服务端一致");

    println!("  时间同步完成：本地时钟已对齐到参考时钟（误差为 0 ms）。");
}
