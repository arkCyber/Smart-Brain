//! Zenoh 统一通信演示。
//!
//! 用 `LocalZenoh`（进程内，复刻 Zenoh 语义）演示三大支柱如何服务于大脑：
//!   1. Pub/Sub：遥测 / 检测结果的高并发低延迟分发
//!   2. Store/Query：把地图/配置存入“存储”，随时 `get` 全网透明查询
//!   3. Compute：在查询路径上注册“服务”（如碰撞检查 / 路径规划），`get` 触发计算
//!
//! 通过统一的 `CommBackend` trait，切换到真实 `zenoh` crate 只需换实现。

use std::sync::Arc;

use brain_mapping::{GridConfig, OccupancyGrid3D};
use brain_message::Telemetry;
use brain_zenoh::{decode, encode, CommBackend, LocalZenoh, QueryHandler};

fn demonstrate_pubsub() {
    println!("\n=== Zenoh Pub/Sub：遥测 / 检测分发 ===");
    let z = LocalZenoh::new();

    // 订阅方（如决策层）。
    let telemetry_sub = z.subscribe("fcu/telemetry").unwrap();
    let detect_sub = z.subscribe("perception/*").unwrap();

    // 发布方（如飞控桥接 / 感知层）。
    let telem = Telemetry::default_at(1);
    z.put("fcu/telemetry", encode(&telem).unwrap()).unwrap();
    z.put("perception/detections", b"yolo:person:0.95".to_vec())
        .unwrap();

    let s = telemetry_sub
        .recv_timeout(std::time::Duration::from_millis(200))
        .unwrap();
    let t: Telemetry = decode(&s.value).unwrap();
    println!(
        "  收到遥测: key={} battery={:.1}%",
        s.key, t.battery.remaining_pct
    );

    // 通配订阅收到感知结果。
    let d = detect_sub
        .recv_timeout(std::time::Duration::from_millis(200))
        .unwrap();
    println!(
        "  收到感知(通配): key={} value={:?}",
        d.key,
        String::from_utf8_lossy(&d.value)
    );
}

fn demonstrate_store_query() {
    println!("\n=== Zenoh Store/Query：地图存储与全网透明查询 ===");
    let z = LocalZenoh::new();
    // 建图模块把 3D 占据网格存进存储。
    let grid = OccupancyGrid3D::new(GridConfig::from_world_size(0.5, 8.0, 8.0, 8.0));
    let encoded = encode(&grid.counts()).unwrap();
    z.put("map/indoor1/occupancy", encoded).unwrap();
    z.put("map/indoor1/explored", b"72%".to_vec()).unwrap();
    z.put("config/failsafe_timeout_ms", b"50".to_vec()).unwrap();

    // 任意时刻查询（断网本地存，连网全网透明查）。
    let replies = z.get("map/indoor1/*").unwrap();
    println!("  查询 map/indoor1/* 命中 {} 条:", replies.len());
    for r in &replies {
        println!(
            "    key={} value={:?}",
            r.key,
            String::from_utf8_lossy(&r.value)
        );
    }
    let fs = z.get("config/failsafe_timeout_ms").unwrap();
    println!("  查询配置: {}", String::from_utf8_lossy(&fs[0].value));
}

fn demonstrate_compute() {
    println!("\n=== Zenoh Compute：查询路径上的边缘计算/服务 ===");
    let z = LocalZenoh::new();

    // 注册一个“路径规划服务”：查询即计算一条无碰撞路径。
    let handler: QueryHandler = Arc::new(|key: &str| {
        // 伪实现：返回一条直连路径的点数。
        vec![format!("{{\"path_points\": 12, \"service\": \"{key}\"}}").into_bytes()]
    });
    let _svc = z.declare_queryable("service/plan_path", handler).unwrap();

    // 客户端调用服务（类似远程过程调用）。
    let replies = z.get("service/plan_path").unwrap();
    println!(
        "  调用服务 service/plan_path → {}",
        String::from_utf8_lossy(&replies[0].value)
    );

    // 计算 + 存储聚合：查询同时拿到存储值 与 计算结果。
    z.put("sensor/front_range", b"1.2".to_vec()).unwrap();
    let check: QueryHandler = Arc::new(|_| vec![b"{\"safe\": false, \"min_dist\": 1.2}".to_vec()]);
    let _check_h = z
        .declare_queryable("service/collision_check", check)
        .unwrap();
    let agg = z.get("service/collision_check").unwrap();
    println!("  碰撞检查服务返回 {} 条", agg.len());
}

/// 顶层入口。
pub fn run() {
    println!("=== Zenoh 统一通信（Pub/Sub + Store/Query + Compute）演示 ===");
    demonstrate_pubsub();
    demonstrate_store_query();
    demonstrate_compute();
    println!("\n（默认使用进程内 LocalZenoh；启用 real-zenoh feature 可切换到真实 zenoh 会话）");
}
