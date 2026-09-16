//! 模型后端工厂演示：按配置 `cfg.agent.backend` 选后端，用统一 `Agent` 跑一轮问答。
//!
//! - `mock`   —— 确定性 `EchoModel`（离线、零配置，默认）
//! - `ollama` —— 本地 Ollama（端口 11434，需 `--features ollama`）
//! - `hermes` —— Hermes 智能体 daemon（端口 11438，需 `--features hermes`）
//!
//! 运行：`cargo run -p brain-node -- --demo model`
//! 运行时把 `config.json` 的 `agent.backend` 改成 `ollama`/`hermes` 即可切换后端。

use brain_agent::{build_model, Agent};
use brain_core::config::BrainConfig;

/// 模型后端工厂演示入口。
pub fn run(cfg: &BrainConfig) {
    println!("\n=== Agent / LLM 思考层（模型后端工厂，按配置选择）===");
    println!("  配置后端: {}", cfg.agent.backend);

    match build_model(cfg) {
        Some(model) => {
            println!("  实际后端: {}", model.name());
            let mut agent = Agent::new(model, "你是一个简明的助手，用一句话回答。");
            let input = "你好，请回一句话。";
            println!("  用户指令: {input}");
            match agent.run(input) {
                Ok(answer) => println!("  最终答复: {answer}"),
                Err(e) => {
                    println!("  Agent 运行出错: {e}");
                    if e.to_string().contains("401") {
                        println!("  提示：后端要求鉴权，请检查对应配置（OLLAMA_API_KEY / HERMES_API_TOKEN）。");
                    }
                }
            }
        }
        None => {
            println!(
                "  后端 '{}' 不可用（feature 未启用或未知）。可选：mock / ollama / hermes。",
                cfg.agent.backend
            );
        }
    }
}
