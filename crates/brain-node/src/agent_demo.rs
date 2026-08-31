//! Agent / LLM"思考层"演示。
//!
//! 把大脑的底层能力（Zenoh 通信、规划、感知、存储查询）暴露为**工具**，
//! 由 Agent（这里用离线 MockLLM）执行"感知 → 推理 → 工具调用 → 控制"循环。
//! 演示：自然语言指令 → 依次调用 navigate/inspect/report → 最终答复，
//! 并把巡检报告存入 Zenoh 存储，再用 Store/Query 全网透明查询取回。

use std::collections::HashMap;
use std::sync::Arc;

use brain_agent::{Agent, FnTool, MemoryStore, MockEmbedder, MockModel, RetrieveTool};
use brain_zenoh::{CommBackend, LocalZenoh};

/// 顶层入口。
pub fn run() {
    println!("\n=== Agent / LLM 思考层（Rig 风格 + RAG）===");
    let backend: Arc<dyn CommBackend> = Arc::new(LocalZenoh::new());

    // 知识库：历史巡检记录（RAG 检索）。
    let store = Arc::new({
        let mut s = MemoryStore::new(Arc::new(MockEmbedder::default()));
        s.add(
            "r1",
            "tower 3 inspection on 2024-05-01 found minor corrosion on insulator",
        );
        s.add(
            "r2",
            "tower 3 inspection on 2024-08-15 all normal, no defect",
        );
        s.add("r3", "tower 5 insulator cracked, scheduled maintenance");
        s
    });

    // 用闭包把能力变成工具。
    let mut agent = Agent::new(
        Box::new(MockModel::new(
            vec!["navigate".into(), "retrieve".into(), "report".into()],
            "巡检完成：3 号杆塔无异常（已结合历史记录比对）。".to_string(),
        )),
        "你是一名自主巡检无人机的任务大脑，负责把人类指令拆成工具调用。",
    );

    let b = backend.clone();
    agent.add_tool(Arc::new(FnTool::new(
        "navigate",
        "飞往指定航点",
        move |args: &HashMap<String, String>| {
            let wp = args.get("wp").cloned().unwrap_or_else(|| "T-3".into());
            let msg = format!("已规划前往 {wp} 的航线（12 个航点）");
            let _ = b.put("agent/nav", msg.clone().into_bytes());
            Ok(msg)
        },
    )));

    let b = backend.clone();
    agent.add_tool(Arc::new(FnTool::new(
        "inspect",
        "对目标进行视觉巡检",
        move |args: &HashMap<String, String>| {
            let target = args
                .get("target")
                .cloned()
                .unwrap_or_else(|| "tower".into());
            let det = format!("检测到目标 {target}，置信度 0.95，未见明显缺陷");
            let _ = b.put("agent/inspection", det.clone().into_bytes());
            Ok(det)
        },
    )));

    // RAG 检索工具：查历史记录。
    agent.add_tool(Arc::new(RetrieveTool::new(store, 2)));

    let b = backend.clone();
    agent.add_tool(Arc::new(FnTool::new(
        "report",
        "把巡检报告存入存储，供全网查询",
        move |args: &HashMap<String, String>| {
            let summary = args.get("summary").cloned().unwrap_or_else(|| "ok".into());
            let json = format!("{{\"node\":\"brain-01\",\"summary\":\"{summary}\"}}");
            let _ = b.put("agent/report", json.into_bytes());
            Ok(format!("已存储报告: {summary}"))
        },
    )));

    println!("  可用工具: {:?}", agent.available_tools());

    // 自然语言指令 → Agent 拆解 → 工具调用 → 最终答复。
    let user_input = "去检查 3 号电线杆塔，并参考历史记录写一份巡检报告。";
    let answer = agent.run(user_input).expect("agent run");
    println!("  用户指令: {user_input}");
    println!("  最终答复: {answer}");

    // Agent 循环中的工具调用历史（含 RAG 检索结果）。
    for m in agent.history().iter().skip(1) {
        if m.content.contains("tool_call") || m.content.contains("[tool:") {
            println!("    · {}", m.content);
        }
    }

    // Store/Query：把 Agent 存的报告全网透明查询取回。
    let replies = backend.get("agent/*").expect("query");
    println!("  Store/Query agent/* 命中 {} 条:", replies.len());
    for r in &replies {
        println!(
            "    key={} value={}",
            r.key,
            String::from_utf8_lossy(&r.value)
        );
    }
}
