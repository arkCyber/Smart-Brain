//! Hermes 智能体 daemon 后端（本地端口 11438），`hermes` feature。
//!
//! Hermes-Rust 是一个独立的 Rust AI 智能体引擎（自身含 ReAct 工具调用、记忆、技能、
//! 沙箱隔离），其 daemon 默认监听 `127.0.0.1:11438` 并暴露 **OpenAI 兼容** 的
//! `/v1/chat/completions` 与 `/v1/models`。本模块用同步 `ureq` 客户端桥接该接口，
//! 使 Smart-Brain 能把 Hermes 作为 `Model` 接入 Agent 框架。
//!
//! 注意：Hermes 在其**内部**完成 ReAct 工具调用，因此 `/v1/chat/completions`
//! 返回的是最终答复文本（无 OpenAI 风格 `tool_calls`）。故 [`HermesModel`]
//! 只产出 `ModelOutput::Text`。
//!
//! 启用：`cargo build -p brain-agent --features hermes`
//! 运行 daemon：`hermes-cli daemon start`（或 `hermes-cli run`，默认端口 11438）。

use std::fmt;
use std::time::Duration;

use brain_core::config::HermesConfig;
use brain_core::error::BrainError;
use brain_core::Result;

use crate::model::{Model, ModelOutput};
use crate::types::{role_str, Message};

/// 请求体中的一条对话消息。
#[derive(Debug, Clone, serde::Serialize)]
struct ChatMsg {
    role: String,
    content: String,
}

/// Hermes daemon（OpenAI 兼容 `/v1/chat/completions`）后端。
///
/// 每次 `generate` 把对话历史 POST 到 `{endpoint}/v1/chat/completions`，解析响应里的
/// `choices[0].message.content` 作为最终答复。
pub struct HermesModel {
    endpoint: String,
    model: String,
    temperature: f32,
    max_tokens: u32,
    api_token: Option<String>,
    session_id: Option<String>,
    agent: ureq::Agent,
}

impl HermesModel {
    /// 以 `endpoint`（基础地址，如 `http://127.0.0.1:11438`）与 `model` 名创建。
    /// 默认 temperature=0.7、max_tokens=2048、超时 180s。
    pub fn new(endpoint: impl Into<String>, model: impl Into<String>) -> Self {
        Self {
            endpoint: endpoint.into().trim_end_matches('/').to_string(),
            model: model.into(),
            temperature: 0.7,
            max_tokens: 2048,
            api_token: None,
            session_id: None,
            agent: Self::build_agent(Duration::from_secs(180)),
        }
    }

    /// 用默认端点 `http://127.0.0.1:11438` 与给定模型名创建。
    pub fn localhost(model: impl Into<String>) -> Self {
        Self::new("http://127.0.0.1:11438", model)
    }

    /// 从 [`HermesConfig`] 创建后端。
    pub fn from_config(cfg: &HermesConfig) -> Self {
        let mut m = Self::new(&cfg.endpoint, &cfg.model)
            .with_temperature(cfg.temperature)
            .with_max_tokens(cfg.max_tokens);
        m.api_token = cfg.api_token.clone();
        m.session_id = cfg.session_id.clone();
        m.agent = Self::build_agent(Duration::from_secs(cfg.timeout_secs.max(1)));
        m
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
    pub fn with_max_tokens(mut self, n: u32) -> Self {
        self.max_tokens = n;
        self
    }

    /// 设置读取超时。
    pub fn with_timeout(mut self, timeout: Duration) -> Self {
        self.agent = Self::build_agent(timeout);
        self
    }

    /// 设置 Bearer Token（daemon 配置了 `api_token` 时使用，如 401）。
    pub fn with_api_token(mut self, api_token: impl Into<String>) -> Self {
        self.api_token = Some(api_token.into());
        self
    }

    /// 设置会话 ID（Hermes 用它绑定对话 lineage，便于跨轮记忆）。
    pub fn with_session_id(mut self, session_id: impl Into<String>) -> Self {
        self.session_id = Some(session_id.into());
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

    /// 列出 daemon 广告的模型（`GET /v1/models`）。
    pub fn list_models(&self) -> Result<Vec<String>> {
        let mut req = self.agent.get(&format!("{}/v1/models", self.endpoint));
        if let Some(token) = &self.api_token {
            req = req.set("Authorization", &format!("Bearer {token}"));
        }
        let resp: serde_json::Value = req
            .call()
            .map_err(|e| BrainError::Agent(format!("hermes models error: {e}")))?
            .into_json()
            .map_err(|e| BrainError::Agent(format!("hermes models parse error: {e}")))?;
        Ok(resp["data"]
            .as_array()
            .map(|arr| {
                arr.iter()
                    .filter_map(|m| m["id"].as_str().map(String::from))
                    .collect()
            })
            .unwrap_or_default())
    }

    /// 探测服务是否可达（`Ok(true)` 可达；连接失败返回 `Ok(false)`）。
    pub fn ping(&self) -> Result<bool> {
        Ok(self.list_models().is_ok())
    }
}

impl fmt::Debug for HermesModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("HermesModel")
            .field("endpoint", &self.endpoint)
            .field("model", &self.model)
            .field("temperature", &self.temperature)
            .field("max_tokens", &self.max_tokens)
            .field("session_id", &self.session_id)
            .field("api_token", &self.api_token.as_ref().map(|_| "<redacted>"))
            .finish_non_exhaustive()
    }
}

impl Model for HermesModel {
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
            "temperature": self.temperature,
            "max_tokens": self.max_tokens,
        });
        if let Some(sid) = &self.session_id {
            body["session_id"] = serde_json::Value::String(sid.clone());
        }

        let mut req = self
            .agent
            .post(&format!("{}/v1/chat/completions", self.endpoint))
            .set("Content-Type", "application/json");
        if let Some(token) = &self.api_token {
            req = req.set("Authorization", &format!("Bearer {token}"));
        }

        let resp: serde_json::Value = req
            .send_json(&body)
            .map_err(|e| BrainError::Agent(format!("hermes chat error: {e}")))?
            .into_json()
            .map_err(|e| BrainError::Agent(format!("hermes response parse error: {e}")))?;

        // Hermes 的 OpenAI 兼容层仅返回文本（内部已跑完工具/ReAct 循环）。
        // 缺失 `content` 视为异常响应，返回错误而非静默空答复。
        let content = resp["choices"][0]["message"]["content"].as_str();
        match content {
            Some(text) => Ok(ModelOutput::Text(text.to_string())),
            None => Err(BrainError::Agent(
                "hermes: response has no message content".into(),
            )),
        }
    }

    fn name(&self) -> &str {
        "hermes"
    }
}

#[cfg(all(test, feature = "hermes"))]
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
        // Hermes OpenAI 兼容层只返回 content 文本。
        let body = r#"{"id":"s-1","object":"chat.completion","created":0,"model":"hermes-rust","choices":[{"index":0,"message":{"role":"assistant","content":"巡检完成：3 号杆塔无异常"},"finish_reason":"stop"}],"usage":{"prompt_tokens":0,"completion_tokens":0,"total_tokens":0}}"#;
        let (url, handle, _rx) = serve(body);
        let mut m = HermesModel::new(url, "hermes-rust");
        let out = m.generate(&[msg(Role::User, "去检查 3 号杆塔")]).unwrap();
        assert!(matches!(out, ModelOutput::Text(t) if t == "巡检完成：3 号杆塔无异常"));
        assert_eq!(m.name(), "hermes");
        assert_eq!(m.model_name(), "hermes-rust");
        handle.join().unwrap();
    }

    #[test]
    fn sends_chat_request_shape() {
        let body = r#"{"id":"s-1","object":"chat.completion","created":0,"model":"hermes-rust","choices":[{"index":0,"message":{"role":"assistant","content":"ok"},"finish_reason":"stop"}],"usage":{"prompt_tokens":0,"completion_tokens":0,"total_tokens":0}}"#;
        let (url, handle, rx) = serve(body);
        let mut m = HermesModel::new(url, "hermes-rust").with_temperature(0.3);
        let mut history = vec![msg(Role::System, "be a drone agent")];
        history.push(msg(Role::User, "scan"));
        history.push(msg(Role::Assistant, "ok"));
        let _ = m.generate(&history).unwrap();
        let req = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&req).unwrap();
        assert_eq!(v["model"], "hermes-rust");
        assert_eq!(v["stream"], false);
        assert!((v["temperature"].as_f64().unwrap() - 0.3).abs() < 1e-6);
        assert_eq!(v["max_tokens"], 2048);
        let msgs = v["messages"].as_array().unwrap();
        assert_eq!(msgs.len(), 3);
        assert_eq!(msgs[0]["role"], "system");
        assert_eq!(msgs[2]["role"], "assistant");
        handle.join().unwrap();
    }

    #[test]
    fn lists_models_from_models_endpoint() {
        let body = r#"{"object":"list","data":[{"id":"hermes-rust","object":"model","created":0,"owned_by":"hermes-rust"}]}"#;
        let (url, handle, _rx) = serve(body);
        let m = HermesModel::new(url, "x");
        let ids = m.list_models().unwrap();
        assert_eq!(ids, vec!["hermes-rust".to_string()]);
        handle.join().unwrap();
    }

    #[test]
    fn ping_true_when_reachable() {
        let body = r#"{"object":"list","data":[]}"#;
        let (url, handle, _rx) = serve(body);
        let m = HermesModel::new(url, "x");
        assert!(m.ping().unwrap());
        handle.join().unwrap();
    }

    #[test]
    fn sends_bearer_header_when_token_set() {
        // 捕获 Authorization 头，验证 api_token 生效（daemon 配置了 api_token 时）。
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let url = format!("http://127.0.0.1:{port}");
        let (tx, rx) = mpsc::channel();
        let resp_body = r#"{"id":"s-1","object":"chat.completion","created":0,"model":"m","choices":[{"index":0,"message":{"role":"assistant","content":"ok"},"finish_reason":"stop"}],"usage":{"prompt_tokens":0,"completion_tokens":0,"total_tokens":0}}"#;
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
        let mut m = HermesModel::new(url, "m").with_api_token("hermes-token");
        let _ = m.generate(&[msg(Role::User, "hi")]).unwrap();
        let auth = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        assert_eq!(auth, "Authorization: Bearer hermes-token");
        handle.join().unwrap();
    }

    #[test]
    fn list_models_sends_auth_header() {
        // 鉴权服务下，/v1/models 也需带 Bearer Token（否则 ping 会误判不可达）。
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let url = format!("http://127.0.0.1:{port}");
        let (tx, rx) = mpsc::channel();
        let resp_body = r#"{"object":"list","data":[{"id":"hermes-rust","object":"model","created":0,"owned_by":"hermes-rust"}]}"#;
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
        let m = HermesModel::new(url, "m").with_api_token("hermes-token");
        let _ = m.list_models().unwrap();
        let auth = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        assert_eq!(auth, "Authorization: Bearer hermes-token");
        handle.join().unwrap();
    }

    #[test]
    fn sends_session_id_in_request() {
        let body = r#"{"id":"s-1","object":"chat.completion","created":0,"model":"m","choices":[{"index":0,"message":{"role":"assistant","content":"ok"},"finish_reason":"stop"}],"usage":{"prompt_tokens":0,"completion_tokens":0,"total_tokens":0}}"#;
        let (url, handle, rx) = serve(body);
        let mut m = HermesModel::new(url, "m").with_session_id("sess-42");
        let _ = m.generate(&[msg(Role::User, "hi")]).unwrap();
        let req = rx.recv_timeout(std::time::Duration::from_secs(2)).unwrap();
        let v: serde_json::Value = serde_json::from_str(&req).unwrap();
        assert_eq!(v["session_id"], "sess-42");
        handle.join().unwrap();
    }

    #[test]
    fn errors_on_missing_content() {
        // 响应缺少 content 时返回错误，而非静默空答复。
        let body = r#"{"id":"s-1","object":"chat.completion","created":0,"model":"m","choices":[{"index":0,"message":{"role":"assistant"},"finish_reason":"stop"}],"usage":{"prompt_tokens":0,"completion_tokens":0,"total_tokens":0}}"#;
        let (url, handle, _rx) = serve(body);
        let mut m = HermesModel::new(url, "m");
        assert!(m.generate(&[msg(Role::User, "hi")]).is_err());
        handle.join().unwrap();
    }

    #[test]
    fn debug_redacts_api_token() {
        let m = HermesModel::new("http://127.0.0.1:9", "m").with_api_token("super-secret");
        let s = format!("{m:?}");
        assert!(s.contains("127.0.0.1"));
        assert!(s.contains("<redacted>"));
        assert!(!s.contains("super-secret"), "api_token must be redacted");
    }
}
