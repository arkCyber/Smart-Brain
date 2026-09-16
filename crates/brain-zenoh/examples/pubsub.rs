//! `brain-zenoh` 最小示例：进程内 Zenoh 的三大支柱——Pub/Sub + Store/Query + Compute。
//!
//! 运行：`cargo run -p brain-zenoh --example pubsub`

use brain_zenoh::{CommBackend, LocalZenoh, QueryHandler};
use std::sync::Arc;

fn main() {
    let z = LocalZenoh::new();

    // 1) Pub/Sub
    let sub = z.subscribe("demo/telemetry").unwrap();
    z.put("demo/telemetry", b"30.5".to_vec()).unwrap();
    if let Ok(s) = sub.try_recv() {
        println!(
            "subscribed: {} = {}",
            s.key,
            String::from_utf8_lossy(&s.value)
        );
    }

    // 2) Store/Query：put 写入存储，get 全网查询
    z.put("map/cells", b"c1".to_vec()).unwrap();
    println!("query replies = {}", z.get("map/cells").unwrap().len());

    // 3) Compute：声明可查询的计算，get 时被触发
    let handler: QueryHandler =
        Arc::new(|key: &str| Ok(vec![format!("computed:{key}").into_bytes()]));
    let _handle = z.declare_queryable("service/distance", handler).unwrap();
    println!(
        "compute replies = {}",
        z.get("service/distance").unwrap().len()
    );
}
