# brain-agent

> Smart-Brain agent layer: Rig-style LLM agent with tool-calling and MockLLM

**所属层**：第 4 层 · Agent/LLM 思考层 —— 工具调用 + 感知→推理→控制循环。

## 职责

大脑真正"思考与决策"的部分：把传感器数据与底层能力（行为树、建图、规划、飞控、Zenoh）统一暴露为**工具（Tool）**，由 Agent 通过"感知 → 推理 → 工具调用 → 控制"循环驱动。

- `types`：角色消息、工具调用、模型输出、工具定义（`ToolSchema`）
- `model`：`Model` trait（LLM/SLM 抽象）+ 离线的 `MockModel`（确定性、可测）
- `http_model`：`HttpModel`（HTTP 调用任意 OpenAI 兼容 LLM 服务，`--features http-llm`）
- `ollama`：`OllamaModel`（本地端口 11434 原生 `/api/chat` 含工具调用，`--features ollama`）
- `hermes`：`HermesModel`（本地端口 11438 Hermes 智能体 daemon，OpenAI 兼容 `/v1`，`--features hermes`）
- `tool`：`Tool` trait + `FnTool`（用闭包把任意能力变成工具）
- `rag`：`Embedder` / `MemoryStore` / `RetrieveTool` / `MockEmbedder`（检索增强）
- `agent`：`Agent` 主循环（生成 → 调用工具 → 回填结果 → 直到给出最终答复）

## 核心 API

```rust
pub use model::{Model, MockModel, EchoModel, ModelOutput};
#[cfg(feature = "http-llm")] pub use http_model::HttpModel;
#[cfg(feature = "ollama")]   pub use ollama::OllamaModel;
#[cfg(feature = "hermes")]   pub use hermes::HermesModel;
pub use factory::build_model;   // 按 cfg.agent.backend 选 mock/ollama/hermes
pub use tool::{Tool, ToolRegistry, FnTool};
pub use rag::{Document, Embedder, MemoryStore, MockEmbedder, RetrieveTool};
pub use types::{Message, Role, ToolCall, ToolSchema};
pub use agent::Agent;
```

## 后端工厂（按配置选择）

`factory::build_model(&BrainConfig)` 依据 `cfg.agent.backend` 构造 `Box<dyn Model>`：
`mock`（确定性 `EchoModel`，默认）/ `ollama` / `hermes`。上层只需改配置即可切换后端，
无需改动装配代码。未启用对应 feature 时返回 `None`。

```rust
use brain_agent::{build_model, Agent};
use brain_core::config::BrainConfig;

fn main() {
    let cfg = BrainConfig::default(); // agent.backend == "mock"
    let mut agent = Agent::new(build_model(&cfg).unwrap(), "你是一个简明的助手。");
    println!("{}", agent.run("你好").unwrap()); // EchoModel 原样返回“你好”
}
```

## Hermes 智能体 daemon 后端（`--features hermes`，端口 11438）

Hermes-Rust 是一个独立的 Rust AI 智能体引擎（含 ReAct 工具调用、记忆、技能、沙箱），
其 daemon 暴露 **OpenAI 兼容**的 `/v1/chat/completions` 与 `/v1/models`。本后端把 Hermes
接入 Agent 框架（Hermes 在内部完成工具调用，`generate` 返回最终答复文本）。

```rust
use brain_agent::{Agent, HermesModel};

fn main() {
    // 默认端点 http://127.0.0.1:11438，模型 hermes-rust
    let model = HermesModel::localhost("hermes-rust");
    let mut agent = Agent::new(Box::new(model), "你是一名巡检无人机大脑");
    match agent.run("去检查 3 号杆塔") {
        Ok(answer) => println!("答复: {answer}"),
        Err(e) => println!("出错: {e}"),
    }
}
```

运行示例：`cargo run -p brain-agent --features hermes --example hermes`
（需 Hermes daemon 已启动：`hermes-cli daemon start`；服务要求鉴权时设置环境变量
`HERMES_API_TOKEN`）。

## Ollama 后端（`--features ollama`，端口 11434）

通过本机 Ollama 原生 `/api/chat` 调用真实模型（含工具调用），让 Agent 真正推理。

```rust
use brain_agent::{Agent, OllamaModel, ToolSchema};
use std::sync::Arc;

fn main() {
    // 默认端点 http://localhost:11434，模型 qwen2.5（需已 ollama pull qwen2.5）
    let mut model = OllamaModel::localhost("qwen2.5")
        .add_tool(ToolSchema::new("navigate", "飞往指定杆塔"));
    let mut agent = Agent::new(Box::new(model), "你是一名巡检无人机大脑");
    // agent.add_tool(...) 注册对应工具 ...
    match agent.run("去检查 3 号杆塔") {
        Ok(answer) => println!("答复: {answer}"),
        Err(e) => println!("出错: {e}"),
    }
}
```

运行示例：`cargo run -p brain-agent --features ollama --example ollama`
（需 Ollama 已启动；服务要求鉴权时设置环境变量 `OLLAMA_API_KEY`）。

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
