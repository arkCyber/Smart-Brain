//! 统一错误类型。

use std::io;

/// Smart-Brain 各模块共享的错误类型。
#[derive(Debug, thiserror::Error)]
pub enum BrainError {
    /// 配置解析/校验失败。
    #[error("configuration error: {0}")]
    Config(String),

    /// 与飞控（小脑）或外设的通信失败。
    #[error("transport error: {0}")]
    Transport(String),

    /// 底层 I/O 失败（串口/CAN/网络）。
    #[error("io error: {0}")]
    Io(#[from] io::Error),

    /// 数据总线/话题错误。
    #[error("bus error: {0}")]
    Bus(String),

    /// AI 推理错误。
    #[error("inference error: {0}")]
    Inference(String),

    /// 行为树执行错误。
    #[error("behavior tree error: {0}")]
    Behavior(String),

    /// 状态机非法迁移。
    #[error("invalid state transition: {0}")]
    State(String),

    /// 任务规划错误。
    #[error("mission error: {0}")]
    Mission(String),

    /// Agent / LLM 推理或工具调用错误。
    #[error("agent error: {0}")]
    Agent(String),

    /// 未知错误。
    #[error("unknown error: {0}")]
    Other(String),
}

/// 便捷的 crate 级 `Result` 别名。
pub type Result<T> = std::result::Result<T, BrainError>;
