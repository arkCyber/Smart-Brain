//! 工具抽象与注册表：把任意能力（行为树/建图/规划/飞控/Zenoh）变成可调用的工具。

use std::collections::HashMap;
use std::sync::Arc;

use brain_core::Result;

/// 一个可被 Agent 调用的工具。
pub trait Tool: Send + Sync {
    fn name(&self) -> &str;
    fn description(&self) -> &str;
    fn run(&self, args: &HashMap<String, String>) -> Result<String>;
}

/// 工具可调用的函数签名（接收参数字符串映射，返回文本结果）。
type ToolFn = Arc<dyn Fn(&HashMap<String, String>) -> Result<String> + Send + Sync>;

/// 用闭包把一个能力封装成工具（无需为每个能力写一个结构体）。
pub struct FnTool {
    name: String,
    description: String,
    f: ToolFn,
}

impl FnTool {
    pub fn new(
        name: impl Into<String>,
        description: impl Into<String>,
        f: impl Fn(&HashMap<String, String>) -> Result<String> + Send + Sync + 'static,
    ) -> Self {
        Self {
            name: name.into(),
            description: description.into(),
            f: Arc::new(f),
        }
    }
}

impl Tool for FnTool {
    fn name(&self) -> &str {
        &self.name
    }
    fn description(&self) -> &str {
        &self.description
    }
    fn run(&self, args: &HashMap<String, String>) -> Result<String> {
        (self.f)(args)
    }
}

/// 工具注册表：按名字索引。
pub struct ToolRegistry {
    tools: Vec<Arc<dyn Tool>>,
}

impl Default for ToolRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl ToolRegistry {
    pub fn new() -> Self {
        Self { tools: Vec::new() }
    }

    /// 注册一个工具。
    pub fn add(&mut self, tool: Arc<dyn Tool>) {
        self.tools.push(tool);
    }

    /// 按名字取工具。
    pub fn get(&self, name: &str) -> Option<&Arc<dyn Tool>> {
        self.tools.iter().find(|t| t.name() == name)
    }

    /// 是否为空。
    pub fn is_empty(&self) -> bool {
        self.tools.is_empty()
    }

    /// 所有可用工具名。
    pub fn names(&self) -> Vec<String> {
        self.tools.iter().map(|t| t.name().to_string()).collect()
    }

    /// 工具数量。
    pub fn len(&self) -> usize {
        self.tools.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fn_tool_runs() {
        let t = FnTool::new("add", "adds two numbers", |args| {
            let a: i32 = args.get("a").unwrap().parse().unwrap();
            let b: i32 = args.get("b").unwrap().parse().unwrap();
            Ok((a + b).to_string())
        });
        let mut args = HashMap::new();
        args.insert("a".to_string(), "2".to_string());
        args.insert("b".to_string(), "40".to_string());
        assert_eq!(t.run(&args).unwrap(), "42");
    }

    #[test]
    fn registry_lookup() {
        let mut reg = ToolRegistry::new();
        reg.add(Arc::new(FnTool::new("nav", "navigate", |_| {
            Ok("nav".into())
        })));
        reg.add(Arc::new(FnTool::new("detect", "detect", |_| {
            Ok("det".into())
        })));
        assert_eq!(reg.len(), 2);
        assert!(reg.get("nav").is_some());
        assert!(reg.get("missing").is_none());
        assert_eq!(reg.names(), vec!["nav".to_string(), "detect".to_string()]);
    }

    #[test]
    fn fn_tool_name_and_description() {
        let t = FnTool::new("calc", "does math", |_| Ok("0".into()));
        // 通过 Tool trait 暴露 name/description。
        assert_eq!(t.name(), "calc");
        assert_eq!(t.description(), "does math");
    }

    #[test]
    fn registry_is_empty_and_default() {
        assert!(ToolRegistry::new().is_empty());
        assert!(ToolRegistry::default().is_empty());
        assert_eq!(ToolRegistry::new().len(), 0);
        let mut reg = ToolRegistry::new();
        reg.add(Arc::new(FnTool::new("x", "x", |_| Ok("x".into()))));
        assert!(!reg.is_empty());
    }

    #[test]
    fn tool_run_error_propagates() {
        // 工具内部错误应通过 run 传播。
        let t = FnTool::new("failing", "always fails", |_| {
            Err(brain_core::BrainError::Agent("boom".into()))
        });
        assert!(t.run(&HashMap::new()).is_err());
    }
}
