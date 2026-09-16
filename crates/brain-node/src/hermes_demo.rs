//! Hermes 智能体 daemon 演示：通过端口 11438 链接本地 Hermes 引擎，驱动 Agent。
//!
//! 演示：探测服务 → 列出 `/v1/models` 广告的模型 → 用自然语言指令让 Hermes 给出答复。
//! Hermes 在其内部完成 ReAct 工具调用，故这里拿到的是最终答复文本。
//!
//! 运行：`cargo run -p brain-node --features hermes -- --demo hermes`
//! 需本机 Hermes daemon 已启动（`hermes-cli daemon start`，默认端口 11438）。

use brain_agent::{Agent, HermesModel};
use brain_core::config::BrainConfig;

/// Hermes 智能体演示入口。
pub fn run(cfg: &BrainConfig) {
    println!("\n=== Agent / LLM 思考层（Hermes 智能体 daemon，端口 11438）===");

    let model = HermesModel::from_config(&cfg.hermes);
    println!("  端点: {}  模型: {}", model.endpoint(), model.model_name());

    // 1) 探测服务是否可达；不可达则提示后优雅退出（不 panic）。
    match model.ping() {
        Ok(true) => println!("  服务可达 ✓"),
        Ok(false) => {
            println!(
                "  服务不可达：请先启动 Hermes daemon（`hermes-cli daemon start`，监听 {}）。",
                model.endpoint()
            );
            return;
        }
        Err(e) => {
            println!("  探测失败: {e}");
            return;
        }
    }

    // 2) 列出 daemon 广告的模型（确认请求体里的 model 名）。
    match model.list_models() {
        Ok(ids) => println!("  广告模型: {}", ids.join(", ")),
        Err(e) => println!("  列出模型失败: {e}"),
    }

    // 3) 用 Hermes daemon 驱动 Agent（Hermes 内部完成工具调用，这里取最终答复）。
    let mut agent = Agent::new(
        Box::new(model),
        "你是一名自主巡检无人机的大脑，把人类指令转成行动并给出简洁的中文答复。".to_string(),
    );
    println!("  可用工具（由 Hermes 内部驱动）: 请参考 hermes-cli tools");

    let user_input = "去检查 3 号电线杆塔，并汇报状态。";
    println!("  用户指令: {user_input}");
    match agent.run(user_input) {
        Ok(answer) => println!("  最终答复: {answer}"),
        Err(e) => {
            println!("  Agent 运行出错: {e}");
            if e.to_string().contains("401") {
                println!("  提示：Hermes daemon 配置了 api_token，请设置环境变量 HERMES_API_TOKEN 或在配置的 hermes.api_token 中填写 Bearer Token。");
            }
        }
    }
}
