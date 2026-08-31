//! Agent 主循环：生成 → 调用工具 → 回填结果 → 直到给出最终答复。

use std::sync::Arc;

use brain_core::error::BrainError;
use brain_core::Result;

use crate::model::{Model, ModelOutput};
use crate::tool::ToolRegistry;
use crate::types::Message;

/// Agent 默认最大推理-工具循环步数。
const DEFAULT_MAX_ITERS: usize = 16;

/// 一个可复用的 Agent。
pub struct Agent {
    model: Box<dyn Model>,
    registry: ToolRegistry,
    history: Vec<Message>,
    max_iters: usize,
}

impl Agent {
    /// 用模型与系统提示创建 Agent。
    pub fn new(model: Box<dyn Model>, system_prompt: impl Into<String>) -> Self {
        Self {
            model,
            registry: ToolRegistry::new(),
            history: vec![Message::system(system_prompt)],
            max_iters: DEFAULT_MAX_ITERS,
        }
    }

    /// 注册一个工具。
    pub fn add_tool(&mut self, tool: Arc<dyn crate::tool::Tool>) {
        self.registry.add(tool);
    }

    /// 可用工具名。
    pub fn available_tools(&self) -> Vec<String> {
        self.registry.names()
    }

    /// 设置最大循环步数。
    pub fn set_max_iters(&mut self, n: usize) {
        self.max_iters = n;
    }

    /// 运行一次 Agent：输入人类指令，返回最终答复。
    pub fn run(&mut self, user_input: impl Into<String>) -> Result<String> {
        self.history.push(Message::user(user_input));
        for _ in 0..self.max_iters {
            let output = self.model.generate(&self.history)?;
            match output {
                ModelOutput::Text(answer) => {
                    self.history.push(Message::assistant(answer.clone()));
                    return Ok(answer);
                }
                ModelOutput::ToolCall(tc) => {
                    // 记录模型想调用的工具。
                    self.history.push(Message::assistant(format!(
                        "tool_call {} {}({:?})",
                        tc.id, tc.name, tc.arguments
                    )));
                    // 执行工具（找不到则记录错误，循环继续）。
                    let result = match self.registry.get(&tc.name) {
                        Some(tool) => tool
                            .run(&tc.arguments)
                            .unwrap_or_else(|e| format!("ERROR: {e}")),
                        None => format!("ERROR: unknown tool '{}'", tc.name),
                    };
                    log::info!("[agent] {} <- {}", tc.name, result);
                    self.history.push(Message::tool_result(&tc.id, result));
                }
            }
        }
        Err(BrainError::Agent(format!(
            "exceeded max iterations ({})",
            self.max_iters
        )))
    }

    /// 对话历史（用于调试）。
    pub fn history(&self) -> &[Message] {
        &self.history
    }

    /// 重置 Agent（保留模型，清空历史与系统提示）。
    pub fn reset(&mut self) {
        let system = self.history.first().cloned();
        self.history = vec![Message::system(
            system.map(|m| m.content).unwrap_or_default(),
        )];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tool::FnTool;
    use crate::types::Role;
    use std::collections::HashMap;
    use std::sync::atomic::{AtomicUsize, Ordering};

    fn nav_tool(calls: Arc<AtomicUsize>) -> Arc<dyn crate::tool::Tool> {
        Arc::new(FnTool::new(
            "navigate",
            "navigate to a waypoint",
            move |args: &HashMap<String, String>| {
                calls.fetch_add(1, Ordering::SeqCst);
                let wp = args.get("wp").map(|s| s.as_str()).unwrap_or("?");
                Ok(format!("navigating to {wp}"))
            },
        ))
    }

    #[test]
    fn agent_calls_tool_then_answers() {
        let calls = Arc::new(AtomicUsize::new(0));
        let mut agent = Agent::new(
            Box::new(crate::model::MockModel::new(
                vec!["navigate".into()],
                "done".to_string(),
            )),
            "you are a drone agent",
        );
        agent.add_tool(nav_tool(calls.clone()));

        let answer = agent.run("go to tower 3").unwrap();
        assert_eq!(answer, "done");
        assert_eq!(calls.load(Ordering::SeqCst), 1);
        // 历史里应包含工具调用与结果。
        assert!(agent
            .history()
            .iter()
            .any(|m| m.role == Role::Tool && m.content.contains("navigating to")));
    }

    #[test]
    fn agent_continues_after_unknown_tool() {
        let mut agent = Agent::new(
            Box::new(crate::model::MockModel::new(
                vec!["missing".into()],
                "fallback".to_string(),
            )),
            "sys",
        );
        // 不注册 "missing" 工具 → 应记录错误但仍给出最终答复。
        let answer = agent.run("hi").unwrap();
        assert_eq!(answer, "fallback");
        assert!(agent
            .history()
            .iter()
            .any(|m| m.role == Role::Tool && m.content.contains("unknown tool")));
    }

    #[test]
    fn agent_max_iters_errors() {
        let mut agent = Agent::new(
            Box::new(crate::model::MockModel::new(
                vec!["a".into(); 20],
                "never".to_string(),
            )),
            "sys",
        );
        agent.set_max_iters(3);
        assert!(matches!(agent.run("x"), Err(BrainError::Agent(_))));
    }
    #[test]
    fn returns_final_without_tools() {
        // 空计划 + 无工具：Agent 直接给最终答复。
        let mut agent = Agent::new(
            Box::new(crate::model::MockModel::new(vec![], "ok".to_string())),
            "sys",
        );
        assert_eq!(agent.run("hi").unwrap(), "ok");
    }

    #[test]
    fn max_iters_zero_errors() {
        let mut agent = Agent::new(
            Box::new(crate::model::MockModel::new(
                vec!["t".into()],
                "x".to_string(),
            )),
            "sys",
        );
        agent.set_max_iters(0);
        assert!(matches!(agent.run("go"), Err(BrainError::Agent(_))));
    }

    #[test]
    fn reset_clears_history() {
        let mut agent = Agent::new(
            Box::new(crate::model::MockModel::new(
                vec!["t".into()],
                "done".to_string(),
            )),
            "you are a drone",
        );
        agent.run("start").unwrap();
        assert!(agent.history().len() > 1); // 系统 + 用户 + 工具往返...
        agent.reset();
        // 重置后只保留系统提示。
        assert_eq!(agent.history().len(), 1);
        assert_eq!(agent.history()[0].role, Role::System);
    }

    #[test]
    fn stress_many_tool_calls() {
        let calls = Arc::new(AtomicUsize::new(0));
        let c2 = calls.clone();
        let mut agent = Agent::new(
            Box::new(crate::model::MockModel::new(
                vec!["t".into(); 30],
                "done".to_string(),
            )),
            "sys",
        );
        agent.add_tool(Arc::new(FnTool::new("t", "tool", move |_| {
            c2.fetch_add(1, Ordering::SeqCst);
            Ok("ok".to_string())
        })));
        agent.set_max_iters(40); // 计划 30 次工具调用，需 > 30
        assert_eq!(agent.run("go").unwrap(), "done");
        assert_eq!(calls.load(Ordering::SeqCst), 30);
    }
}
