//! Ollama 推理后端（本地端口 11434），`ollama` feature。
//!
//! 用同步 `ureq` 客户端调用 Ollama **原生 `/api/chat`**（而非 OpenAI 兼容层），
//! 使 Agent 能真正“请求工具调用 → 生成最终答复”，并复用本项目的 `Model` 抽象。
//!
//! 与 `http_model`（OpenAI 兼容 `/chat/completions`）的区别：Ollama 原生接口的
//! 工具参数 `arguments` 是 **JSON 对象**（值可为字符串/数字/布尔等），需要扁平化为
//! `HashMap<String,String>`；本模块还提供 `list_models` / `ping` 探测能力。
//!
//! 启用：`cargo build -p brain-agent --features ollama`
//! 运行：`cargo run -p brain-agent --features ollama --example ollama`

use std::collections::HashMap;
use std::fmt;
use std::time::Duration;

use brain_core::config::OllamaConfig;
use brain_core::error::BrainError;
use brain_core::Result;

use crate::model::{Model, ModelOutput};
use crate::types::{role_str, Message, ToolCall, ToolSchema};

/// 请求体中的一条对话消息（Ollama 接受 system/user/assistant/tool 角色）。
#[derive(Debug, Clone, serde::Serialize)]
struct ChatMsg {
    role: String,
    content: String,
}

/// Ollama 原生 `/api/chat` 后端。
///
/// 每次 `generate` 把对话历史 POST 到 `{endpoint}/api/chat`，解析响应里的
/// `message.content`（最终答复）或 `message.tool_calls`（工具请求）。
pub struct OllamaModel {
    endpoint: String,
    model: String,
    temperature: f32,
    num_predict: u32,
    tools: Vec<serde_json::Value>,
    api_key: Option<String>,
    agent: ureq::Agent,
}

impl OllamaModel {
    /// 以 `endpoint`（基础地址，如 `http://localhost:11434`）与 `model` 名创建。
    /// 默认 temperature=0.7、num_predict=1024、超时 120s。
    pub fn new(endpoint: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            endpoint: endpoint.into().trim_end_matches('/').to_string(),
            model: model.into(),
            temperature: 0.7,
            num_predict: 1024,
            tools: Vec::new(),
            api_key: None,
            agent: Self::build_agent(Duration::from_secs(120)),
        }
    }

    /// 用默认端点 `http://localhost:11434` 与给定模型名创建。
    pub fn localhost(model: impl Into<String>) -> Self {
        Self::new("http://localhost:11434", model)
    }

    /// 从 [`OllamaConfig`] 创建后端。
    pub fn from_config(cfg: &OllamaConfig) -> Self {
        let mut m = Self::new(&cfg.endpoint, &cfg.model)
            .with_temperature(cfg.temperature)
            .with_num_predict(cfg.num_predict);
        m.api_key = cfg.api_key.clone();
        m.agent = Self::build_agent(Duration::from_secs(cfg.timeout_secs.max(1)));
        m
    }

    /// 设置 Bearer Token（服务要求鉴权时使用，如 401）。
    pub fn with_api_key(mut self, api_key: impl Into<String>) -> Self {
        self.api_key = Some(api_key.into());
        self
    }

    /// 构造带连接/读超时的 HTTP 客户端。
    fn build_agent(timeout: Duration) -> ureq::Agent {
        ureq::AgentBuilder::new()
            .timeout_connect(Duration::from_secs(10))
            .timeout_read(timeout)
            .build()
    }

    /// 设置采样温度。
    pub fn with_temperature(mut self, t: f32) -> Self {
        self.temperature = t;
        self
    }

    /// 设置最大生成 token 数。
    pub fn with_num_predict(mut self, n: u32) -> Self {
        self.num_predict = n;
        self
    }

    /// 设置读取超时。
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.agent = Self::build_agent(timeout);
        self
    }

    /// 登记一个可供模型调用的工具（按 Ollama/OpenAI 函数调用格式 `{type, function}` 包装）。
    pub fn add_tool(mut self, tool: ToolSchema) -> Self {
        let schema = serde_json::to_value(tool).unwrap_or_default();
        self.tools
            .push(serde_json::json!({ "type": "function", "function": schema }));
        self
    }

    /// 已登记的基础端点。
    pub fn endpoint(&self) -> &str {
        &self.endpoint
    }

    /// 已登记的模型名。
    pub fn model_name(&self) -> &str {
        &self.model
    }

    /// 列出服务上已安装的模型（`GET /api/tags`）。
    pub fn list_models(&self) -> Result<Vec<String>> {
        let mut req = self.agent.get(&format!("{}/api/tags", self.endpoint));
        if let Some(key) = &self.api_key {
            req = req.set("Authorization", &format!("Bearer {key}"));
        }
        let resp: serde_json::Value = req
            .call()
            .map_err(|e| BrainError::Agent(format!("ollama tags error: {e}")))?
            .into_json()
            .map_err(|e| BrainError::Agent(format!("ollama tags parse error: {e}")))?;
        Ok(resp["models"]
            .as_array()
            .map(|arr| {
                arr.iter()
                    .filter_map(|m| m["name"].as_str().map(String::from))
                    .collect()
            })
            .unwrap_or_default())
    }

    /// 探测服务是否可达（`Ok(true)` 可达；连接失败返回 `Ok(false)`）。
    pub fn ping(&self) -> Result<bool> {
        Ok(self.list_models().is_ok())
    }
}

impl fmt::Debug for OllamaModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("OllamaModel")
            .field("endpoint", &self.endpoint)
            .field("model", &self.model)
            .field("temperature", &self.temperature)
            .field("num_predict", &self.num_predict)
            .field("tools", &self.tools.len())
            .field("api_key", &self.api_key.as_ref().map(|_| "<redacted>"))
            .finish_non_exhaustive()
    }
}

/// 把 Ollama 返回的 `arguments`（JSON 对象）扁平化为字符串映射。
///
/// 字符串原样；数字/布尔转成字符串；嵌套对象/数组序列化为 JSON 字符串。
fn flatten_arguments(v: &serde_json::Value) -> HashMap<String, String> {
    let mut out = HashMap::new();
    if let Some(obj) = v.as_object() {
        for (k, val) in obj {
            let s = match val {
                serde_json::Value::String(s) => s.clone(),
                serde_json::Value::Number(n) => n.to_string(),
                serde_json::Value::Bool(b) => b.to_string(),
                serde_json::Value::Null => String::new(),
                other => other.to_string(),
            };
            out.insert(k.clone(), s);
        }
    }
    out
}

impl Model for OllamaModel {
    fn generate(&mut self, history: &[Message]) -> Result<ModelOutput> {
        let messages: Vec<ChatMsg> = history
            .iter()
            .map(|m| ChatMsg {
                role: role_str(m.role).to_string(),
                content: m.content.clone(),
            })
            .collect();

        let mut body = serde_json::json!({
            "model": self.model,
            "messages": messages,
            "stream": false,
            "options": {
                "temperature": self.temperature,
                "num_predict": self.num_predict,
            },
        });
        if !self.tools.is_empty() {
            body["tools"] = serde_json::Value::Array(self.tools.clone());
        }

        let mut req = self
            .agent
            .post(&format!("{}/api/chat", self.endpoint))
            .set("Content-Type", "application/json");
        if let Some(key) = &self.api_key {
            req = req.set("Authorization", &format!("Bearer {key}"));
        }

        let resp: serde_json::Value = req
            .send_json(&body)
            .map_err(|e| BrainError::Agent(format!("ollama chat error: {e}")))?
            .into_json()
            .map_err(|e| BrainError::Agent(format!("ollama response parse error: {e}")))?;

        let message = &resp["message"];

        // 1) 工具调用优先：`message.tool_calls[].function.{name, arguments(对象)}`。
        if let Some(calls) = message["tool_calls"].as_array() {
            if let Some(call) = calls.first() {
                let name = call["function"]["name"].as_str().unwrap_or("").to_string();
                let arguments = flatten_arguments(&call["function"]["arguments"]);
                return Ok(ModelOutput::ToolCall(ToolCall::new(
                    format!("call-{name}"),
                    name,
                    arguments,
                )));
            }
        }

        // 2) 否则取文本答复。
        let text = message["content"].as_str().unwrap_or("").to_string();
        Ok(ModelOutput::Text(text))
    }

    fn name(&self) -> &str {
        "ollama"
    }
}

#[cfg(all(test, feature = "ollama"))]
mod tests {
    use super::*;
    use crate::types::Role;
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::sync::mpsc;
    use std::thread;

    /// 起一个本地 HTTP 服务器，返回 (url, 线程句柄, 收到的请求体)。
    fn serve(body: &'static str) -> (String, thread::JoinHandle<()>, mpsc::Receiver<String>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let url = format!("http://127.0.0.1:{port}");
        let (tx, rx) = mpsc::channel();
        let handle = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut buf = Vec::new();
            let mut tmp = [0u8; 4096];
            let header_end = loop {
                let n = stream.read(&mut tmp).unwrap();
                assert!(n > 0, "client closed before request finished");
                buf.extend_from_slice(&tmp[..n]);
                if let Some(pos) = find_subslice(&buf, b"\r\n\r\n") {
                    break pos + 4;
                }
            };
            let headers = String::from_utf8_lossy(&buf[..header_end]).to_string();
            let content_len = headers
                .lines()
                .find_map(|l| {
                    let mut it = l.splitn(2, ':');
                    let (k, v) = (it.next().unwrap(), it.next().unwrap_or(""));
                    (k.trim().eq_ignore_ascii_case("content-length"))
                        .then(|| v.trim().parse::<usize>().unwrap_or(0))
                })
                .unwrap_or(0);
            while buf.len() < header_end + content_len {
                let n = stream.read(&mut tmp).unwrap();
                assert!(n > 0, "client closed before request body finished");
                buf.extend_from_slice(&tmp[..n]);
            }
            let req_body =
                String::from_utf8_lossy(&buf[header_end..header_end + content_len]).to_string();
            let _ = tx.send(req_body);
            let resp = format!(
                "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                body.len(),
                body
            );
            stream.write_all(resp.as_bytes()).unwrap();
            stream.flush().unwrap();
            stream.shutdown(std::net::Shutdown::Write).unwrap();
        });
        (url, handle, rx)
    }

    fn find_subslice(haystack: &[u8], needle: &[u8]) -> Option<usize> {
        haystack.windows(needle.len()).position(|w| w == needle)
    }

    fn msg(role: Role, content: &str) -> Message {
        Message {
            role,
            content: content.to_string(),
        }
    }

    #[test]
    fn parses_text_response() {
        let body = r#"{"model":"qwen2.5","message":{"role":"assistant","content":"tower 3 is clear"},"done":true}"#;
        let (url, handle, _rx) = serve(body);
        let mut m = OllamaModel::new(url, "qwen2.5");
        let out = m.generate(&[msg(Role::User, "check tower 3")]).unwrap();
        assert!(matches!(out, ModelOutput::Text(t) if t == "tower 3 is clear"));
        assert_eq!(m.name(), "ollama");
        assert_eq!(m.model_name(), "qwen2.5");
        handle.join().unwrap();
    }

    #[test]
    fn parses_tool_call_with_object_arguments() {
        // Ollama 的 arguments 是 JSON 对象，值可为字符串/数字/布尔。
        let body = r#"{"model":"qwen2.5","message":{"role":"assistant","content":"","tool_calls":[{"function":{"name":"navigate","arguments":{"wp":"T-3","alt":30.5,"track":true}}}]},"done":true}"#;
        let (url, handle, _rx) = serve(body);
        let mut m = OllamaModel::new(url, "qwen2.5");
        let out = m.generate(&[]).unwrap();
        match out {
            ModelOutput::ToolCall(tc) => {
                assert_eq!(tc.name, "navigate");
                assert_eq!(tc.arg("wp"), "T-3");
                assert_eq!(tc.arg("alt"), "30.5");
                assert_eq!(tc.arg("track"), "true");
            }
            _ => panic!("expected tool call"),
        }
        handle.join().unwrap();
    }

    #[test]
    fn sends_chat_request_to_api_chat() {
        let body =
            r#"{"model":"qwen2.5","message":{"role":"assistant","content":"ok"},"done":true}"#;
        let (url, handle, rx) = serve(body);
        let mut m = OllamaModel::new(url, "qwen2.5").with_temperature(0.2);
        let mut history = vec![msg(Role::System, "be a drone agent")];
        history.push(msg(Role::User, "scan"));
        history.push(msg(Role::Tool, "[tool:c1] done"));
        let _ = m.generate(&history).unwrap();
        let req = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&req).unwrap();
        assert_eq!(v["model"], "qwen2.5");
        assert_eq!(v["stream"], false);
        // 温度是 f32，序列化后转 f64 有极小误差，用容差比较。
        assert!(
            (v["options"]["temperature"].as_f64().unwrap() - 0.2).abs() < 1e-6,
            "temperature={}",
            v["options"]["temperature"]
        );
        assert_eq!(v["options"]["num_predict"], 1024);
        let msgs = v["messages"].as_array().unwrap();
        assert_eq!(msgs.len(), 3);
        assert_eq!(msgs[0]["role"], "system");
        assert_eq!(msgs[2]["role"], "tool");
        handle.join().unwrap();
    }

    #[test]
    fn includes_tools_when_registered() {
        let body = r#"{"model":"m","message":{"role":"assistant","content":"ok"},"done":true}"#;
        let (url, handle, rx) = serve(body);
        let mut m = OllamaModel::new(url, "m").add_tool(ToolSchema::new("navigate", "go"));
        let _ = m.generate(&[msg(Role::User, "hi")]).unwrap();
        let req = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&req).unwrap();
        assert_eq!(v["tools"].as_array().unwrap().len(), 1);
        assert_eq!(v["tools"][0]["function"]["name"], "navigate");
        handle.join().unwrap();
    }

    #[test]
    fn lists_models_from_tags() {
        let body = r#"{"models":[{"name":"qwen2.5","size":1},{"name":"llama3.1","size":2}]}"#;
        let (url, handle, _rx) = serve(body);
        let m = OllamaModel::new(url, "x");
        let names = m.list_models().unwrap();
        assert_eq!(names, vec!["qwen2.5".to_string(), "llama3.1".to_string()]);
        handle.join().unwrap();
    }

    #[test]
    fn ping_true_when_reachable() {
        let body = r#"{"models":[]}"#;
        let (url, handle, _rx) = serve(body);
        let m = OllamaModel::new(url, "x");
        assert!(m.ping().unwrap());
        handle.join().unwrap();
    }

    #[test]
    fn list_models_sends_auth_header() {
        // 鉴权服务下，/api/tags 也需带 Bearer Token。
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let url = format!("http://127.0.0.1:{port}");
        let (tx, rx) = mpsc::channel();
        let resp_body = r#"{"models":[{"name":"qwen2.5","size":1}]}"#;
        let handle = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut buf = Vec::new();
            let mut tmp = [0u8; 4096];
            let header_end = loop {
                let n = stream.read(&mut tmp).unwrap();
                buf.extend_from_slice(&tmp[..n]);
                if let Some(pos) = find_subslice(&buf, b"\r\n\r\n") {
                    break pos + 4;
                }
            };
            let headers = String::from_utf8_lossy(&buf[..header_end]).to_string();
            let auth = headers
                .lines()
                .find(|l| l.to_ascii_lowercase().starts_with("authorization:"))
                .map(str::to_string)
                .unwrap_or_default();
            let _ = tx.send(auth);
            let resp = format!(
                "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                resp_body.len(),
                resp_body
            );
            stream.write_all(resp.as_bytes()).unwrap();
            stream.flush().unwrap();
            stream.shutdown(std::net::Shutdown::Write).unwrap();
        });
        let m = OllamaModel::new(url, "x").with_api_key("ollama-key");
        let _ = m.list_models().unwrap();
        let auth = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        assert_eq!(auth, "Authorization: Bearer ollama-key");
        handle.join().unwrap();
    }

    #[test]
    fn flatten_arguments_handles_various_types() {
        let v = serde_json::json!({"s":"str","n":3,"b":true,"o":{"k":1},"null":null});
        let map = flatten_arguments(&v);
        assert_eq!(map.get("s").map(String::as_str), Some("str"));
        assert_eq!(map.get("n").map(String::as_str), Some("3"));
        assert_eq!(map.get("b").map(String::as_str), Some("true"));
        assert_eq!(map.get("null").map(String::as_str), Some(""));
        assert!(map.contains_key("o"));
    }

    #[test]
    fn sends_bearer_header_when_key_set() {
        // 捕获请求的 Authorization 头，验证 api_key 生效。
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let url = format!("http://127.0.0.1:{port}");
        let (tx, rx) = mpsc::channel();
        let resp_body =
            r#"{"model":"m","message":{"role":"assistant","content":"ok"},"done":true}"#;
        let handle = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            let mut buf = Vec::new();
            let mut tmp = [0u8; 4096];
            let header_end = loop {
                let n = stream.read(&mut tmp).unwrap();
                buf.extend_from_slice(&tmp[..n]);
                if let Some(pos) = find_subslice(&buf, b"\r\n\r\n") {
                    break pos + 4;
                }
            };
            let headers = String::from_utf8_lossy(&buf[..header_end]).to_string();
            let content_len = headers
                .lines()
                .find_map(|l| {
                    let mut it = l.splitn(2, ':');
                    let (k, v) = (it.next().unwrap(), it.next().unwrap_or(""));
                    (k.trim().eq_ignore_ascii_case("content-length"))
                        .then(|| v.trim().parse::<usize>().unwrap_or(0))
                })
                .unwrap_or(0);
            while buf.len() < header_end + content_len {
                let n = stream.read(&mut tmp).unwrap();
                buf.extend_from_slice(&tmp[..n]);
            }
            let auth = headers
                .lines()
                .find(|l| l.to_ascii_lowercase().starts_with("authorization:"))
                .map(str::to_string)
                .unwrap_or_default();
            let _ = tx.send(auth);
            let resp = format!(
                "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                resp_body.len(),
                resp_body
            );
            stream.write_all(resp.as_bytes()).unwrap();
            stream.flush().unwrap();
            stream.shutdown(std::net::Shutdown::Write).unwrap();
        });
        let mut m = OllamaModel::new(url, "m").with_api_key("secret-123");
        let _ = m.generate(&[msg(Role::User, "hi")]).unwrap();
        let auth = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        assert_eq!(auth, "Authorization: Bearer secret-123");
        handle.join().unwrap();
    }

    #[test]
    fn debug_redacts_api_key() {
        let m = OllamaModel::new("http://127.0.0.1:9", "m").with_api_key("super-secret");
        let s = format!("{m:?}");
        assert!(s.contains("127.0.0.1"));
        assert!(s.contains("<redacted>"));
        assert!(!s.contains("super-secret"), "api_key must be redacted");
    }
}
