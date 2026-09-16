//! `brain-agent` 最小示例：通过端口 11434 链接本地 Ollama 推理引擎，驱动 Agent。
//!
//! 运行：`cargo run -p brain-agent --features ollama --example ollama`
//! 需本机 Ollama 已运行（默认 `http://localhost:11434`）且已 `ollama pull qwen2.5`。
//!
//! 未启用 `ollama` feature 构建时，本示例仅打印提示（保证默认构建不破坏）。

#[cfg(feature = "ollama")]
mod impl_ollama {
    use brain_agent::{Agent, FnTool, OllamaModel};
    use std::sync::Arc;

    /// 真实 Ollama 推理：探测 → 注册工具 → 自然语言指令 → 工具调用闭环。
    pub fn run() {
        let model = OllamaModel::localhost("qwen2.5");
        println!("端点: {}  模型: {}", model.endpoint(), model.model_name());

        if !model.ping().unwrap_or(false) {
            println!("Ollama 不可达：请确认已启动并 `ollama pull qwen2.5`。");
            return;
        }

        let mut agent = Agent::new(
            Box::new(model),
            "你是一名自主巡检无人机的大脑；若要求前往某杆塔就调用 navigate 工具。",
        );
        agent.add_tool(Arc::new(FnTool::new(
            "navigate",
            "飞往指定杆塔",
            |args| {
                let wp = args.get("wp").map(String::as_str).unwrap_or("T-3");
                Ok(format!("已规划前往 {wp} 的航线"))
            },
        )));

        match agent.run("请去检查 3 号电线杆塔") {
            Ok(answer) => {
                println!("最终答复: {answer}");
                for m in agent.history().iter().skip(1) {
                    if m.content.contains("tool_call") || m.content.contains("[tool:") {
                        println!("  · {}", m.content);
                    }
                }
            }
            Err(e) => {
                println!("Agent 出错: {e}");
                if e.to_string().contains("401") {
                    println!("提示：Ollama 要求鉴权，请设置环境变量 OLLAMA_API_KEY。");
                }
            }
        }
    }
}

fn main() {
    #[cfg(feature = "ollama")]
    impl_ollama::run();
    #[cfg(not(feature = "ollama"))]
    {
        eprintln!("本示例需要 `ollama` feature：cargo run -p brain-agent --features ollama --example ollama");
    }
}
