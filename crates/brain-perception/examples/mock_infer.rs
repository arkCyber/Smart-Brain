//! `brain-perception` 最小示例：加载模型并做一次推理（Mock 后端，无系统依赖）。
//!
//! 运行：`cargo run -p brain-perception --example mock_infer`

use brain_perception::backend::{InferenceInput, InferenceOutput};
use brain_perception::{MockModelBackend, ModelBackend};

fn main() {
    let mut backend = MockModelBackend::new();
    backend.load("models/mock.onnx").expect("load model");

    // 模拟一帧图像张量（NCHW）
    let input = InferenceInput::new(vec![0.0; 6], vec![1, 1, 1, 6]);
    let out: InferenceOutput = backend.infer(&input).expect("infer");

    println!(
        "backend = {} | detected class = {} | confidence = {:.2}",
        backend.name(),
        out.at(0, 0) as u32,
        out.at(0, 1)
    );
}
