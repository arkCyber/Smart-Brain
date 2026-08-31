# brain-agent

> Smart-Brain agent layer: Rig-style LLM agent with tool-calling and MockLLM

**所属层**：第 4 层 · Agent/LLM 思考层 —— 工具调用 + 感知→推理→控制循环。

## 职责

大脑真正"思考与决策"的部分：把传感器数据与底层能力（行为树、建图、规划、飞控、Zenoh）统一暴露为**工具（Tool）**，由 Agent 通过"感知 → 推理 → 工具调用 → 控制"循环驱动。

- `types`：角色消息、工具调用、模型输出
- `model`：`Model` trait（LLM/SLM 抽象）+ 离线的 `MockModel`（确定性、可测）
- `http_model`：`HttpModel`（HTTP 调用真实 LLM 服务，带 `ToolSchema`）
- `tool`：`Tool` trait + `FnTool`（用闭包把任意能力变成工具）
- `rag`：`Embedder` / `MemoryStore` / `RetrieveTool` / `MockEmbedder`（检索增强）
- `agent`：`Agent` 主循环（生成 → 调用工具 → 回填结果 → 直到给出最终答复）

## 核心 API

```rust
pub use model::{Model, MockModel, ModelOutput};
pub use http_model::{HttpModel, ToolSchema};
pub use tool::{Tool, ToolRegistry, FnTool};
pub use rag::{Document, Embedder, MemoryStore, MockEmbedder, RetrieveTool};
pub use types::{Message, Role, ToolCall};
pub use agent::Agent;
```

## 用法

```rust
use brain_agent::{Agent, FnTool, MockModel};
use std::sync::Arc;

fn main() {
    let model = MockModel::new(vec!["navigate".into()], "done".to_string());
    let mut agent = Agent::new(Box::new(model), "you are a drone agent");
    agent.add_tool(Arc::new(FnTool::new("navigate", "navigate to a waypoint", |args| {
        let wp = args.get("wp").map(String::as_str).unwrap_or("?");
        Ok(format!("navigating to {wp}"))
    })));
    let reply = agent.run("go to tower 3").unwrap();
    println!("agent: {reply}");
}
```

## 依赖

- 外部：`log`、`serde`、`serde_json`
- 内部：`brain-core`

> **应用案例**：`brain-node/agent_demo.rs`（Agent 工具调用控制闭环）。
