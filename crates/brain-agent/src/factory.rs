//! 模型后端工厂：按配置（`cfg.agent.backend`）构造 `Box<dyn Model>`。
//!
//! 让上层（`brain-node` 演示、业务装配）无需关心各后端的**具体类型与 feature**：
//! 只要在配置里写 `"agent": {"backend": "mock" | "ollama" | "hermes"}`，即可按名取到后端。
//!
//! - `mock`   —— [`EchoModel`]（确定性、离线、零配置，默认）
//! - `ollama` —— `OllamaModel`（feature `ollama`）
//! - `hermes` —— `HermesModel`（feature `hermes`）
//!
//! 未启用对应 feature / 未知后端时返回 `None`，由调用方决定回退策略。

use brain_core::config::BrainConfig;

use crate::model::{EchoModel, Model};

/// 按 `cfg.agent.backend` 构造一个模型后端；未知或未启用返回 `None`。
pub fn build_model(cfg: &BrainConfig) -> Option<Box<dyn Model>> {
    match cfg.agent.backend.as_str() {
        "mock" => Some(Box::new(EchoModel)),
        #[cfg(feature = "ollama")]
        "ollama" => Some(Box::new(crate::ollama::OllamaModel::from_config(
            &cfg.ollama,
        ))),
        #[cfg(feature = "hermes")]
        "hermes" => Some(Box::new(crate::hermes::HermesModel::from_config(
            &cfg.hermes,
        ))),
        other => {
            log::warn!("model backend '{other}' not available (feature not enabled or unknown)");
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::ModelOutput;
    use crate::types::Message;
    use brain_core::config::BrainConfig;

    fn cfg_with_backend(backend: &str) -> BrainConfig {
        BrainConfig {
            agent: brain_core::config::AgentConfig {
                backend: backend.to_string(),
            },
            ..BrainConfig::default()
        }
    }

    #[test]
    fn mock_backend_builds_and_echoes() {
        let mut m = build_model(&cfg_with_backend("mock")).expect("mock available");
        assert_eq!(m.name(), "mock");
        let out = m.generate(&[Message::user("hello")]).unwrap();
        assert!(matches!(out, ModelOutput::Text(t) if t == "hello"));
    }

    #[test]
    fn unknown_backend_returns_none() {
        assert!(build_model(&cfg_with_backend("nonexistent")).is_none());
    }

    #[test]
    fn empty_backend_returns_none() {
        assert!(build_model(&cfg_with_backend("")).is_none());
    }

    #[cfg(feature = "ollama")]
    #[test]
    fn ollama_backend_builds_when_feature_on() {
        assert!(build_model(&cfg_with_backend("ollama")).is_some());
    }

    #[cfg(feature = "hermes")]
    #[test]
    fn hermes_backend_builds_when_feature_on() {
        assert!(build_model(&cfg_with_backend("hermes")).is_some());
    }
}
