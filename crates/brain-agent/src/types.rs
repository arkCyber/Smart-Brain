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

/// 对话角色 → 传输层角色字符串（供 HTTP/Ollama 后端复用）。
pub fn role_str(role: Role) -> &'static str {
    match role {
        Role::System => "system",
        Role::User => "user",
        Role::Assistant => "assistant",
        Role::Tool => "tool",
    }
}

/// 一个 OpenAI/Ollama 风格的工具定义（JSON Schema 精简版）。
///
/// 传给模型，让它可以决定何时调用某个工具。`parameters` 为可选的 JSON Schema；
/// 缺省时许多模型也能自行生成 `arguments`（name/description 足够）。
#[derive(Debug, Clone, serde::Serialize)]
pub struct ToolSchema {
    name: String,
    description: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    parameters: Option<serde_json::Value>,
}

impl ToolSchema {
    /// 用名字与描述构造一个工具定义。
    pub fn new(name: impl Into<String>, description: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            description: description.into(),
            parameters: None,
        }
    }

    /// 追加一个 JSON Schema 作为参数定义。
    pub fn with_parameters(mut self, parameters: serde_json::Value) -> Self {
        self.parameters = Some(parameters);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_role_constructors() {
        let m = Message::system("sys");
        assert_eq!(m.role, Role::System);
        assert_eq!(m.content, "sys");

        let m = Message::user("你好");
        assert_eq!(m.role, Role::User);
        assert_eq!(m.content, "你好");

        let m = Message::assistant("ok");
        assert_eq!(m.role, Role::Assistant);

        let m = Message::tool_result("id-1", "42");
        assert_eq!(m.role, Role::Tool);
        assert_eq!(m.content, "[tool:id-1] 42");
    }

    #[test]
    fn tool_call_arg_lookup() {
        let mut args = HashMap::new();
        args.insert("goal".to_string(), "10.0".to_string());
        let tc = ToolCall::new("c1", "move", args);
        assert_eq!(tc.id, "c1");
        assert_eq!(tc.name, "move");
        assert_eq!(tc.arg("goal"), "10.0");
        // 缺失参数返回空字符串。
        assert_eq!(tc.arg("missing"), "");
    }

    #[test]
    fn role_equality_and_clone() {
        assert_eq!(Role::User, Role::User);
        assert_ne!(Role::System, Role::Tool);
        let a = Message::user("x");
        let b = a.clone();
        assert_eq!(a.content, b.content);
        assert_eq!(a.role, b.role);
    }
}
