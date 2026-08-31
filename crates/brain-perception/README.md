# brain-perception

> Smart-Brain AI perception: inference backend abstraction and vision pipeline

**所属层**：第 4 层 · AI 智能与感知层 —— 推理后端抽象 + 视觉流水线。

## 职责

把"推理后端"抽象为 `ModelBackend` trait：桌面/SITL 使用 `MockModelBackend`（无需任何系统依赖），真机部署时启用 `onnx` feature 接入 ONNX Runtime（Jetson/RK3588 上配合 TensorRT/RKNN 量化模型）。`VisionPipeline` 组织"取帧 → 推理 → 生成检测/跟踪"的流水线。

- `backend`：`ModelBackend` trait + `MockModelBackend` / `OnnxModelBackend`
- `nms`：YOLOv8 解码 + 非极大值抑制（`decode_yolov8` / `detect_and_nms` / `nms` / `DetectionBox`）
- `pipeline`：`VisionPipeline` / `VisionConfig`

## 核心 API

```rust
pub use backend::{MockModelBackend, ModelBackend, OnnxModelBackend};
pub use nms::{DetectionBox, decode_yolov8, detect_and_nms, nms};
pub use pipeline::{VisionConfig, VisionPipeline};
```

## 用法

```rust
use brain_perception::backend::{InferenceInput, InferenceOutput};
use brain_perception::{MockModelBackend, ModelBackend};

fn main() {
    let mut backend = MockModelBackend::new();
    backend.load("models/mock.onnx").unwrap(); // 加载模型
    let input = InferenceInput::new(vec![0.0; 6], vec![1, 1, 1, 6]);
    let out: InferenceOutput = backend.infer(&input).unwrap();
    println!("class = {}, conf = {:.2}", out.at(0, 0) as u32, out.at(0, 1));
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-message`、`brain-middleware`

> **应用案例**：`brain-node/agent_demo.rs`、`brain-node/comprehensive_demo.rs` 把检测结果接入决策闭环。
