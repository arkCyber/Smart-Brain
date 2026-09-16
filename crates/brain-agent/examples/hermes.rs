//! `brain-agent` 最小示例：通过端口 11438 链接 Hermes 智能体 daemon，驱动 Agent。
//!
//! 运行：`cargo run -p brain-agent --features hermes --example hermes`
//! 需本机已启动 Hermes daemon（`hermes-cli daemon start`，默认端口 11438）。
//!
//! 未启用 `hermes` feature 构建时，本示例仅打印提示（保证默认构建不破坏）。

#[cfg(feature = "hermes")]
mod impl_hermes {
    use brain_agent::{Agent, HermesModel};

    /// 真实 Hermes daemon：探测 → 列模型 → 自然语言指令 → 答复。
    pub fn run() {
        let model = HermesModel::localhost("hermes-rust");
        println!("端点: {}  模型: {}", model.endpoint(), model.model_name());

        if !model.ping().unwrap_or(false) {
            println!("Hermes daemon 不可达：请先 `hermes-cli daemon start`（端口 11438）。");
            return;
        }

        let mut agent = Agent::new(
            Box::new(model),
            "你是一名自主巡检无人机的大脑，负责把指令转成行动并给出简洁答复。",
        );
        match agent.run("去检查 3 号电线杆塔，并汇报状态") {
            Ok(answer) => println!("最终答复: {answer}"),
            Err(e) => {
                println!("Agent 出错: {e}");
                if e.to_string().contains("401") {
                    println!(
                        "提示：Hermes daemon 配置了 api_token，请设置环境变量 HERMES_API_TOKEN。"
                    );
                }
            }
        }
    }
}

fn main() {
    #[cfg(feature = "hermes")]
    impl_hermes::run();
    #[cfg(not(feature = "hermes"))]
    {
        eprintln!("本示例需要 `hermes` feature：cargo run -p brain-agent --features hermes --example hermes");
    }
}
