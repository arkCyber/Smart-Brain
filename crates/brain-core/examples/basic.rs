//! `brain-core` 最小示例：统一错误类型 + 配置加载 + 数学原语。
//!
//! 运行：`cargo run -p brain-core --example basic`

use brain_core::{BrainConfig, Result, Vec3};

fn main() -> Result<()> {
    // 依次尝试候选路径加载配置，全部缺失时回退到内置默认值
    let cfg = BrainConfig::load_candidates(&["config.json", "config.example.json"]);
    println!("node_id = {}", cfg.node_id);
    println!("failsafe_timeout_ms = {}", cfg.failsafe_timeout_ms);
    println!("fcu.transport = {}", cfg.fcu.transport);

    // 通用数学原语
    let a = Vec3::new(1.0, 2.0, 3.0);
    let b = Vec3::new(4.0, -1.0, 0.0);
    println!(
        "|a| = {:.2}, a·b = {:.2}, |a − b| = {:.2}",
        a.norm(),
        a.dot(b),
        Vec3::distance(a, b)
    );
    Ok(())
}
