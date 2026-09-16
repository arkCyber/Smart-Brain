//! 真实 HTTP LLM 后端（OpenAI 兼容 `/chat/completions`），`http-llm` feature。
//!
//! 用同步、轻量的 `ureq` 客户端把本项目的 `Model` 抽象桥接到任意 OpenAI 兼容端点
//! （OpenAI / DeepSeek / Qwen / Ollama / vLLM / LM Studio / 自建网关…），
//! 使 Agent 能真正“请求工具调用 → 生成最终答复”，而不再仅依赖离线的 `MockModel` 剧本。
//!
//! 启用：`cargo build -p brain-agent --features http-llm`
//! （与 `brain-perception` 的 ONNX 后端、`brain-zenoh` 的真实 zenoh 后端同一策略：
//! 默认关闭以保持构建轻量，真机/接入真实模型时打开）。

use std::collections::HashMap;

use brain_core::error::BrainError;
use brain_core::Result;

use crate::model::{Model, ModelOutput};
use crate::types::{role_str, Message, ToolCall, ToolSchema};

/// 请求体中的一条对话消息（角色字符串序列化）。
#[derive(Debug, Clone, serde::Serialize)]
struct ChatMsg {
    role: String,
    content: String,
}

/// OpenAI `/chat/completions` 请求体。
#[derive(Debug, Clone, serde::Serialize)]
struct ChatRequest {
    model: String,
    messages: Vec<ChatMsg>,
    temperature: f32,
    max_tokens: u32,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    tools: Vec<serde_json::Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    tool_choice: Option<serde_json::Value>,
}

/// 真实 HTTP LLM 后端。
///
/// 每次 `generate` 都把对话历史 POST 到 OpenAI 兼容端点，解析响应里的
/// `content`（最终答复）或 `tool_calls`（工具请求）。
pub struct HttpModel {
    url: String,
    model: String,
    api_key: Option<String>,
    temperature: f32,
    max_tokens: u32,
    tools: Vec<serde_json::Value>,
    agent: ureq::Agent,
}

impl HttpModel {
    /// 以 `endpoint`（完整 URL，如 `https://api.openai.com/v1/chat/completions`）
    /// 与 `model` 名创建后端。默认 temperature=0.7、max_tokens=1024。
    pub fn new(endpoint: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            url: endpoint.into(),
            model: model.into(),
            api_key: None,
            temperature: 0.7,
            max_tokens: 1024,
            tools: Vec::new(),
            agent: ureq::AgentBuilder::new()
                .timeout_connect(std::time::Duration::from_secs(10))
                .timeout_read(std::time::Duration::from_secs(60))
                .build(),
        }
    }

    /// 设置 Bearer API Key（用于鉴权）。
    pub fn with_api_key(mut self, api_key: impl Into<String>) -> Self {
        self.api_key = Some(api_key.into());
        self
    }

    /// 设置采样温度。
    pub fn with_temperature(mut self, t: f32) -> Self {
        self.temperature = t;
        self
    }

    /// 设置最大生成长度（token）。
    pub fn with_max_tokens(mut self, n: u32) -> Self {
        self.max_tokens = n;
        self
    }

    /// 登记一个可供模型调用的工具（按 OpenAI 函数调用格式 `{type, function}` 包装）。
    pub fn add_tool(mut self, tool: ToolSchema) -> Self {
        let schema = serde_json::to_value(tool).unwrap_or_default();
        self.tools.push(serde_json::json!({
            "type": "function",
            "function": schema,
        }));
        self
    }

    /// 已登记的模型端点。
    pub fn endpoint(&self) -> &str {
        &self.url
    }

    /// 已登记的模型名。
    pub fn model_name(&self) -> &str {
        &self.model
    }
}

impl Model for HttpModel {
    fn generate(&mut self, history: &[Message]) -> Result<ModelOutput> {
        let messages: Vec<ChatMsg> = history
            .iter()
            .map(|m| ChatMsg {
                role: role_str(m.role).to_string(),
                content: m.content.clone(),
            })
            .collect();

        let body = ChatRequest {
            model: self.model.clone(),
            messages,
            temperature: self.temperature,
            max_tokens: self.max_tokens,
            tools: self.tools.clone(),
            tool_choice: if self.tools.is_empty() {
                None
            } else {
                Some(serde_json::json!("auto"))
            },
        };

        let mut req = self
            .agent
            .post(&self.url)
            .set("Content-Type", "application/json");
        if let Some(key) = &self.api_key {
            req = req.set("Authorization", &format!("Bearer {key}"));
        }

        let resp: serde_json::Value = req
            .send_json(&body)
            .map_err(|e| BrainError::Agent(format!("llm http error: {e}")))?
            .into_json()
            .map_err(|e| BrainError::Agent(format!("llm response parse error: {e}")))?;

        let message = &resp["choices"][0]["message"];

        // 1) 工具调用优先：`tool_calls[0].function.{name,arguments}`。
        if let Some(calls) = message["tool_calls"].as_array() {
            if let Some(call) = calls.first() {
                let id = call["id"].as_str().unwrap_or("call-0").to_string();
                let name = call["function"]["name"].as_str().unwrap_or("").to_string();
                let args_raw = call["function"]["arguments"].as_str().unwrap_or("{}");
                let arguments: HashMap<String, String> = serde_json::from_str(args_raw)
                    .map_err(|_| BrainError::Agent(format!("bad tool arguments: {args_raw}")))?;
                return Ok(ModelOutput::ToolCall(ToolCall::new(id, name, arguments)));
            }
        }

        // 2) 否则取文本答复。既无 tool_calls 也无 content 视为异常响应，返回错误。
        let content = message["content"].as_str();
        match content {
            Some(text) => Ok(ModelOutput::Text(text.to_string())),
            None => Err(BrainError::Agent(
                "llm: response has no content or tool_calls".into(),
            )),
        }
    }

    fn name(&self) -> &str {
        "http"
    }
}

#[cfg(all(test, feature = "http-llm"))]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::TcpListener;
    use std::sync::mpsc;
    use std::thread;

    /// 起一个本地 HTTP 服务器，返回 (url, 线程句柄, 收到的请求体)。
    fn serve(body: &'static str) -> (String, thread::JoinHandle<()>, mpsc::Receiver<String>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let url = format!("http://127.0.0.1:{port}/v1/chat/completions");
        let (tx, rx) = mpsc::channel();
        let handle = thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            // 读取完整请求：先读头部，再按 Content-Length 读满请求体。
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

    #[test]
    fn parses_text_reply() {
        let body = r#"{"choices":[{"message":{"role":"assistant","content":"fly to tower 3"}}]}"#;
        let (url, handle, _rx) = serve(body);
        let mut m = HttpModel::new(url, "gpt-test");
        let out = m.generate(&[Message::user("go to tower 3")]).unwrap();
        assert!(matches!(out, ModelOutput::Text(t) if t == "fly to tower 3"));
        assert_eq!(m.name(), "http");
        assert_eq!(m.model_name(), "gpt-test");
        handle.join().unwrap();
    }

    #[test]
    fn parses_tool_call() {
        let body = r#"{"choices":[{"message":{"role":"assistant","tool_calls":[{"id":"call-1","type":"function","function":{"name":"navigate","arguments":"{\"goal\":\"3\",\"mode\":\"cruise\"}"}}]}}]}"#;
        let (url, handle, _rx) = serve(body);
        let mut m = HttpModel::new(url, "gpt-test");
        let out = m.generate(&[]).unwrap();
        match out {
            ModelOutput::ToolCall(tc) => {
                assert_eq!(tc.id, "call-1");
                assert_eq!(tc.name, "navigate");
                assert_eq!(tc.arg("goal"), "3");
                assert_eq!(tc.arg("mode"), "cruise");
            }
            _ => panic!("expected tool call"),
        }
        handle.join().unwrap();
    }

    #[test]
    fn sends_role_history_and_model() {
        let body = r#"{"choices":[{"message":{"role":"assistant","content":"ok"}}]}"#;
        let (url, handle, rx) = serve(body);
        let mut m = HttpModel::new(url, "deepseek-test").with_temperature(0.2);
        let mut history = vec![Message::system("be a drone agent")];
        history.push(Message::user("scan area"));
        history.push(Message::assistant("tool_call c1 navigate({})"));
        history.push(Message::tool_result("c1", "done"));
        let _ = m.generate(&history).unwrap();
        let req = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&req).unwrap();
        assert_eq!(v["model"], "deepseek-test");
        assert_eq!(v["temperature"], 0.2);
        let msgs = v["messages"].as_array().unwrap();
        assert_eq!(msgs.len(), 4);
        assert_eq!(msgs[0]["role"], "system");
        assert_eq!(msgs[2]["role"], "assistant");
        assert_eq!(msgs[3]["role"], "tool");
        handle.join().unwrap();
    }

    #[test]
    fn includes_tools_and_auto_choice() {
        let body = r#"{"choices":[{"message":{"role":"assistant","content":"ok"}}]}"#;
        let (url, handle, rx) = serve(body);
        let m = HttpModel::new(url, "m").add_tool(ToolSchema::new("navigate", "go to a point"));
        let mut m = m.add_tool(ToolSchema::new("detect", "detect objects"));
        let _ = m.generate(&[Message::user("hi")]).unwrap();
        let req = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&req).unwrap();
        assert_eq!(v["tools"].as_array().unwrap().len(), 2);
        assert_eq!(v["tools"][0]["function"]["name"], "navigate");
        assert_eq!(v["tool_choice"], "auto");
        handle.join().unwrap();
    }

    #[test]
    fn tool_schema_parameters_serialize() {
        let t = ToolSchema::new("set_target", "set a target").with_parameters(
            serde_json::json!({ "type": "object", "properties": { "x": { "type": "number" } } }),
        );
        // ToolSchema 本身序列化为 {name, description, parameters}。
        let v = serde_json::to_value(t).unwrap();
        assert_eq!(v["name"], "set_target");
        assert_eq!(v["parameters"]["properties"]["x"]["type"], "number");
    }

    #[test]
    fn errors_on_malformed_response() {
        // 既无 tool_calls 也无 content 的响应视为异常，返回错误而非空答复。
        let body = r#"{"choices":[{"message":{"role":"assistant"}}]}"#;
        let (url, handle, _rx) = serve(body);
        let mut m = HttpModel::new(url, "m");
        assert!(m.generate(&[Message::user("hi")]).is_err());
        handle.join().unwrap();
    }
}
