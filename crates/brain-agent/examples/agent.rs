//! `brain-agent` 最小示例：Agent 用 MockLLM 规划 → 调用工具 → 给出最终答复。
//!
//! 运行：`cargo run -p brain-agent --example agent`

use brain_agent::{Agent, FnTool, MockModel};
use std::sync::Arc;

fn main() {
    // MockLLM：先调用一次 "navigate"，随后给出最终答复 "done"
    let model = MockModel::new(vec!["navigate".into()], "done".to_string());
    let mut agent = Agent::new(Box::new(model), "you are a drone agent");

    // 把“飞控导航”封装成工具
    agent.add_tool(Arc::new(FnTool::new(
        "navigate",
        "navigate to a waypoint",
        |args| {
            let wp = args.get("wp").map(String::as_str).unwrap_or("?");
            Ok(format!("navigating to {wp}"))
        },
    )));

    let answer = agent.run("go to tower 3").unwrap();
    println!("agent reply: {answer}");
}
