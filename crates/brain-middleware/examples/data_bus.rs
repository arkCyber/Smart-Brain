//! `brain-middleware` 最小示例：类型安全的数据总线（发布/订阅 + 保留最近值）。
//!
//! 运行：`cargo run -p brain-middleware --example data_bus`

use brain_middleware::{BusMessage, DataBus, Topic};
use std::sync::Arc;

fn main() {
    let bus = DataBus::new();

    // 注册话题并发布（timestamp 单调递增）
    let alt: Arc<Topic<f64>> = bus.register("altitude");
    alt.publish(30.5, 1);

    // 从任意一端按类型安全地取回
    let t: Arc<Topic<f64>> = bus.topic("altitude").expect("topic exists");
    println!(
        "altitude = {} (updated at ts {})",
        t.peek().unwrap(),
        t.last_updated()
    );

    // 推送订阅：发布即收到（含时间戳），订阅即拿到当前保留值
    let rx = bus.subscribe::<f64>("altitude");
    if let Ok(BusMessage { value, timestamp }) = rx.try_recv() {
        println!("subscriber got {value} at ts {timestamp} (retained latest)");
    }
    bus.publish::<f64>("altitude", 31.2, 3).unwrap();
    if let Ok(BusMessage { value, timestamp }) = rx.try_recv() {
        println!("subscriber got {value} at ts {timestamp} (pushed on publish)");
    }

    // 便捷 publish（自动注册），不同消息类型互不干扰
    bus.publish::<u32>("heartbeat", 42, 2).unwrap();
    println!("total topics on bus = {}", bus.len());
}
