//! `brain-ipc` 最小示例：零分配环形缓冲与线程安全共享缓冲。
//!
//! 运行：`cargo run -p brain-ipc --example ring`

use brain_ipc::{FixedRingBuffer, SharedRing};

fn main() {
    // 覆盖式环形缓冲：写满后覆盖最旧元素
    let mut ring = FixedRingBuffer::<f32>::new(4).unwrap();
    for v in [1.0, 2.0, 3.0, 4.0] {
        ring.push(v);
    }
    let evicted = ring.push(5.0); // 覆盖 1.0
    println!(
        "evicted = {evicted:?}, contents = {:?}",
        ring.iter().collect::<Vec<_>>()
    );

    // 线程安全共享缓冲
    let shared = SharedRing::new(3).unwrap();
    shared.push("a".to_string());
    shared.push("b".to_string());
    println!(
        "shared len = {}, oldest = {:?}",
        shared.len(),
        shared.pop_oldest()
    );
}
