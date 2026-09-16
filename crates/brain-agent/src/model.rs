//! 模型抽象与离线 Mock 实现。

use brain_core::Result;

use crate::types::{Message, Role, ToolCall};

/// 模型一次生成的结果：要么直接给最终答复，要么请求调用工具。
#[derive(Debug, Clone)]
pub enum ModelOutput {
    /// 最终答复（结束 Agent 循环）。
    Text(String),
    /// 请求调用一个工具。
    ToolCall(ToolCall),
}

/// LLM / SLM 抽象（Rig 风格）。
pub trait Model {
    /// 依据对话历史生成一个输出。
    fn generate(&mut self, history: &[Message]) -> Result<ModelOutput>;
    /// 模型名。
    fn name(&self) -> &str;
}

/// 离线的确定性"假模型"，用于仿真与单元测试。
///
/// 按给定剧本依次请求调用 `plan` 中的工具，全部调用完后给出 `final_answer`。
/// 这样 Agent 的工具调用循环可被完全确定性地测试。
pub struct MockModel {
    /// 依次要调用的工具名。
    plan: Vec<String>,
    /// 计划用尽后的最终答复。
    final_answer: String,
    step: usize,
}

impl MockModel {
    pub fn new(plan: Vec<String>, final_answer: impl Into<String>) -> Self {
        Self {
            plan,
            final_answer: final_answer.into(),
            step: 0,
        }
    }
}

impl Model for MockModel {
    fn generate(&mut self, _history: &[Message]) -> Result<ModelOutput> {
        if self.step < self.plan.len() {
            let name = self.plan[self.step].clone();
            self.step += 1;
            let arguments: std::collections::HashMap<String, String> =
                std::collections::HashMap::from([("_seq".to_string(), format!("{}", self.step))]);
            Ok(ModelOutput::ToolCall(ToolCall::new(
                format!("call-{}", self.step),
                name,
                arguments,
            )))
        } else {
            Ok(ModelOutput::Text(self.final_answer.clone()))
        }
    }

    fn name(&self) -> &str {
        "mock"
    }
}

/// 极简确定性后端：把“最后一条用户消息”原样作为最终答复。
///
/// 无需剧本、零配置、离线可用，用于接线/冒烟测试，以及 [`crate::factory::build_model`]
/// 里 `mock` 后端的默认实现。
#[derive(Debug, Clone, Copy, Default)]
pub struct EchoModel;

impl Model for EchoModel {
    fn generate(&mut self, history: &[Message]) -> Result<ModelOutput> {
        let reply = history
            .iter()
            .rev()
            .find(|m| m.role == Role::User)
            .map(|m| m.content.clone())
            .unwrap_or_else(|| "ok".to_string());
        Ok(ModelOutput::Text(reply))
    }

    fn name(&self) -> &str {
        "mock"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_emits_plan_then_final() {
        let mut m = MockModel::new(vec!["a".into(), "b".into()], "ok".to_string());
        assert!(matches!(m.generate(&[]).unwrap(), ModelOutput::ToolCall(_)));
        assert!(matches!(m.generate(&[]).unwrap(), ModelOutput::ToolCall(_)));
        assert!(matches!(m.generate(&[]).unwrap(), ModelOutput::Text(s) if s == "ok"));
    }

    #[test]
    fn echo_returns_last_user_message() {
        let mut e = EchoModel;
        assert_eq!(e.name(), "mock");
        let history = vec![
            Message::system("sys"),
            Message::user("hello"),
            Message::assistant("hi"),
            Message::user("again"),
        ];
        assert!(matches!(e.generate(&history).unwrap(), ModelOutput::Text(t) if t == "again"));
        // 无用户消息时回退到 "ok"。
        assert!(matches!(
            e.generate(&[Message::system("sys")]).unwrap(),
            ModelOutput::Text(t) if t == "ok"
        ));
    }
}
