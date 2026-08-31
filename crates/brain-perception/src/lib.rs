//! `brain-perception` — AI 感知与推理层。
//!
//! 对应参考架构第 4 层。设计上把“推理后端”抽象为 `ModelBackend` trait：
//! 桌面/SITL 使用 `MockModelBackend`（无需任何系统依赖），真机部署时启用
//! `onnx` feature 接入 ONNX Runtime（在 Jetson/RK3588 上配合 TensorRT/RKNN
//! 量化模型）。`VisionPipeline` 组织“取帧 -> 推理 -> 生成检测/跟踪”的流水线。

pub mod backend;
pub mod nms;
pub mod pipeline;

pub use backend::{MockModelBackend, ModelBackend, OnnxModelBackend};
pub use nms::{decode_yolov8, detect_and_nms, nms, DetectionBox};
pub use pipeline::{VisionConfig, VisionPipeline};
