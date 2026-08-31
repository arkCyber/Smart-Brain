//! 小脑（飞控）经 Zenoh 桥接演示。
//!
//! 对应参考中“zenoh-pico 桥接底层单片机”的思路：
//! - 大脑一侧用 `ZenohFcuTransport`（发布 `fcu/command`、订阅 `fcu/telemetry`）。
//! - 小脑一侧用 `MockFcuZenoh` 模拟跑 zenoh-pico 的 STM32（订阅命令、上报遥测）。
//!
//! 二者共享同一个 `CommBackend` 键空间；真机上换成真实 zenoh 即可蜂群互通。

use std::sync::Arc;

use brain_message::{Command, CommandTarget, Mode};
use brain_transport::{FcuTransport, MockFcuZenoh, ZenohFcuTransport};
use brain_zenoh::LocalZenoh;

/// 顶层入口。
pub fn run() {
    println!("\n=== 小脑（飞控）经 Zenoh 桥接 ===");
    let backend: Arc<dyn brain_zenoh::CommBackend> = Arc::new(LocalZenoh::new());

    let mut brain_fcu = ZenohFcuTransport::new(backend.clone()).expect("brain side");
    let mut fcu = MockFcuZenoh::new(backend).expect("small brain (MCU) side");

    // 大脑下发起飞指令。
    let cmd = Command {
        timestamp: 1,
        mode: Mode::Takeoff,
        target: CommandTarget::Position {
            north: 0.0,
            east: 0.0,
            down: -30.0,
        },
    };
    println!("  大脑 → 小脑: send_command(Takeoff)");
    brain_fcu.send_command(&cmd).expect("send");

    // 小脑（zenoh-pico）收到指令并上报遥测。
    let handled = fcu.poll_and_respond().expect("respond");
    println!("  小脑收到并回传遥测: handled={handled}");

    // 大脑读取回传遥测。
    match brain_fcu.try_recv_telemetry() {
        Ok(Some(t)) => println!("  大脑 ← 小脑: alt={:.1}m", t.gps.alt),
        other => println!("  大脑未读到遥测: {other:?}"),
    }
}
