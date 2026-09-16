//! `brain-agent` — Agent / LLM"思考层"（参考 Rig / Open-Interpreter）。
//!
//! 这是大脑真正"思考与决策"的部分：把传感器数据与底层能力（行为树、建图、
//! 规划、飞控、Zenoh）统一暴露为**工具（Tool）**，由 Agent 通过"感知 → 推理 →
//! 工具调用 → 控制"循环驱动。
//!
//! 本 crate 提供：
//! - `types`：角色消息、工具调用、模型输出、工具定义
//! - `model`：`Model` trait（LLM/SLM 抽象）+ 离线的 `MockModel`（确定性、可测）
//! - `tool`：`Tool` trait + `FnTool`（用闭包把任意能力变成工具）
//! - `agent`：`Agent` 主循环（生成→调用工具→回填结果→直到给出最终答复）
//! - 真实后端（feature 开启）：`ollama`（端口 11434）/ `hermes`（端口 11438）/
//!   `http-llm`（任意 OpenAI 兼容端点）

pub mod agent;
pub mod factory;
#[cfg(feature = "hermes")]
pub mod hermes;
#[cfg(feature = "http-llm")]
pub mod http_model;
pub mod model;
#[cfg(feature = "ollama")]
pub mod ollama;
pub mod rag;
pub mod tool;
pub mod types;

pub use agent::Agent;
pub use factory::build_model;
#[cfg(feature = "hermes")]
pub use hermes::HermesModel;
#[cfg(feature = "http-llm")]
pub use http_model::HttpModel;
pub use model::{EchoModel, MockModel, Model, ModelOutput};
#[cfg(feature = "ollama")]
pub use ollama::OllamaModel;
pub use rag::{Document, Embedder, MemoryStore, MockEmbedder, RetrieveTool};
pub use tool::{FnTool, Tool, ToolRegistry};
pub use types::{Message, Role, ToolCall, ToolSchema};
