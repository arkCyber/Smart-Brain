//! Ollama 推理演示：通过端口 11434 链接本地 Ollama 引擎，用真实 LLM 驱动 Agent。
//!
//! 演示：探测服务 → 列出已装模型 → 注册一个“导航”工具 → 用自然语言指令让
//! Ollama 决定是否调用工具，并最终给出答复（真实调用 /api/chat，含工具调用）。
//!
//! 运行：`cargo run -p brain-node --features ollama -- --demo ollama`
//! 需本机 Ollama 已运行（默认 `http://localhost:11434`）且已 `ollama pull <model>`。

use std::collections::HashMap;
use std::sync::Arc;

use brain_agent::{Agent, FnTool, OllamaModel};
use brain_core::config::BrainConfig;

/// Ollama 推理演示入口。
pub fn run(cfg: &BrainConfig) {
    println!("\n=== Agent / LLM 思考层（Ollama 推理引擎，端口 11434）===");

    let mut model = OllamaModel::from_config(&cfg.ollama);
    println!("  端点: {}  模型: {}", model.endpoint(), model.model_name());

    // 1) 探测服务是否可达；不可达则提示后优雅退出（不 panic）。
    match model.ping() {
        Ok(true) => println!("  服务可达 ✓"),
        Ok(false) => {
            println!(
                "  服务不可达：请确认 Ollama 已启动（{}）并 `ollama pull {}`。",
                model.endpoint(),
                model.model_name()
            );
            return;
        }
        Err(e) => {
            println!("  探测失败: {e}");
            return;
        }
    }

    // 2) 列出已装模型（便于确认模型名）。
    match model.list_models() {
        Ok(names) => println!("  已安装模型: {}", names.join(", ")),
        Err(e) => println!("  列出模型失败: {e}"),
    }

    // 3) 登记一个可供模型调用的工具。
    model = model.add_tool(brain_agent::ToolSchema::new(
        "navigate",
        "飞往指定杆塔（参数 wp 为杆塔编号，如 T-3）",
    ));

    // 4) 用真实 Ollama 驱动 Agent 循环。
    let mut agent = Agent::new(
        Box::new(model),
        "你是一名自主巡检无人机的任务大脑。若用户要求前往某个杆塔，请调用 navigate 工具；"
            .to_string(),
    );
    agent.add_tool(Arc::new(FnTool::new(
        "navigate",
        "飞往指定杆塔",
        |args: &HashMap<String, String>| {
            let wp = args.get("wp").cloned().unwrap_or_else(|| "T-3".into());
            Ok(format!("已规划前往 {wp} 的航线（12 个航点）"))
        },
    )));

    println!("  可用工具: {:?}", agent.available_tools());

    let user_input = "请去检查 3 号电线杆塔。";
    println!("  用户指令: {user_input}");
    match agent.run(user_input) {
        Ok(answer) => {
            println!("  最终答复: {answer}");
            // 打印 Agent 循环中的工具调用历史。
            for m in agent.history().iter().skip(1) {
                if m.content.contains("tool_call") || m.content.contains("[tool:") {
                    println!("    · {}", m.content);
                }
            }
        }
        Err(e) => {
            println!("  Agent 运行出错: {e}");
            if e.to_string().contains("401") {
                println!("  提示：Ollama 要求鉴权，请设置环境变量 OLLAMA_API_KEY 或在配置的 ollama.api_key 中填写 Bearer Token。");
            }
        }
    }
}
