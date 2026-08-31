//! 角色消息与工具调用类型。

use std::collections::HashMap;

/// 对话角色。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Role {
    /// 系统提示。
    System,
    /// 用户（人类指令）。
    User,
    /// 助手（模型）。
    Assistant,
    /// 工具执行结果。
    Tool,
}

/// 一条对话消息。
#[derive(Debug, Clone)]
pub struct Message {
    pub role: Role,
    pub content: String,
}

impl Message {
    pub fn system(content: impl Into<String>) -> Self {
        Self {
            role: Role::System,
            content: content.into(),
        }
    }
    pub fn user(content: impl Into<String>) -> Self {
        Self {
            role: Role::User,
            content: content.into(),
        }
    }
    pub fn assistant(content: impl Into<String>) -> Self {
        Self {
            role: Role::Assistant,
            content: content.into(),
        }
    }
    pub fn tool_result(tool_call_id: &str, content: impl Into<String>) -> Self {
        Self {
            role: Role::Tool,
            content: format!("[tool:{tool_call_id}] {}", content.into()),
        }
    }
}

/// 模型请求调用一个工具。
#[derive(Debug, Clone)]
pub struct ToolCall {
    pub id: String,
    pub name: String,
    pub arguments: HashMap<String, String>,
}

impl ToolCall {
    pub fn new(
        id: impl Into<String>,
        name: impl Into<String>,
        arguments: HashMap<String, String>,
    ) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            arguments,
        }
    }

    /// 读取一个参数；缺失时返回空字符串。
    pub fn arg(&self, key: &str) -> &str {
        self.arguments.get(key).map(|s| s.as_str()).unwrap_or("")
    }
}
