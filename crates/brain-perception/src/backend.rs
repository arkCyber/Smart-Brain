//! 推理后端抽象：模型加载与一次前向推理。

use brain_core::error::{BrainError, Result};

/// 一次推理的输入（原始张量视图）。
///
/// 对于视觉模型，通常为 NCHW 布局：`[batch, channels, height, width]`。
#[derive(Debug, Clone)]
pub struct InferenceInput {
    pub data: Vec<f32>,
    pub shape: Vec<usize>,
}

impl InferenceInput {
    pub fn new(data: Vec<f32>, shape: Vec<usize>) -> Self {
        Self { data, shape }
    }
}

/// 一次推理的输出：`[N, 1, 4+k]` 格式的检测框（x,y,w,h,cls 分数…），
/// 或更宽泛的特征向量。为演示起见，定义为浮点矩阵（行优先）。
#[derive(Debug, Clone)]
pub struct InferenceOutput {
    pub data: Vec<f32>,
    pub rows: usize,
    pub cols: usize,
}

impl InferenceOutput {
    /// 取第 `row` 行的第 `col` 个元素。
    pub fn at(&self, row: usize, col: usize) -> f32 {
        self.data[row * self.cols + col]
    }
}

/// 模型推理后端抽象。
pub trait ModelBackend: Send {
    /// 加载模型（ONNX 路径或内部标识）。
    fn load(&mut self, path: &str) -> Result<()>;
    /// 执行一次推理。
    fn infer(&mut self, input: &InferenceInput) -> Result<InferenceOutput>;
    /// 后端名称（用于日志）。
    fn name(&self) -> &str;
}

/// 仿真后端：不依赖任何系统库，根据输入尺寸生成确定性输出。
///
/// 用于 SITL 与单元测试，输出模拟一个“目标检测”结果。
pub struct MockModelBackend {
    loaded: bool,
}

impl MockModelBackend {
    pub fn new() -> Self {
        Self { loaded: false }
    }
}

impl Default for MockModelBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl ModelBackend for MockModelBackend {
    fn load(&mut self, path: &str) -> Result<()> {
        log::info!("mock backend loading model: {path}");
        self.loaded = true;
        Ok(())
    }

    fn infer(&mut self, input: &InferenceInput) -> Result<InferenceOutput> {
        if !self.loaded {
            return Err(BrainError::Inference("model not loaded".into()));
        }
        let _ = input;
        // 单行输出：[class_id, confidence, cx, cy, w, h]
        let cols = 6;
        let mut data = vec![0.0f32; cols];
        data[0] = 0.0; // class 0
        data[1] = 0.95; // confidence
        data[2] = 0.5; // cx
        data[3] = 0.5; // cy
        data[4] = 0.2; // w
        data[5] = 0.2; // h
        Ok(InferenceOutput {
            data,
            rows: 1,
            cols,
        })
    }

    fn name(&self) -> &str {
        "mock"
    }
}

/// 真实 ONNX Runtime 后端（可选，`onnx` feature）。
pub struct OnnxModelBackend {
    #[allow(dead_code)]
    session: Option<()>,
}

impl OnnxModelBackend {
    pub fn new() -> Self {
        Self { session: None }
    }
}

impl Default for OnnxModelBackend {
    fn default() -> Self {
        Self::new()
    }
}

impl ModelBackend for OnnxModelBackend {
    fn load(&mut self, path: &str) -> Result<()> {
        #[cfg(feature = "onnx")]
        {
            // 真实部署示例：使用 ort crate 创建 ONNX 会话。
            //   ort::Session::builder()?.commit_from_file(path) ...
            log::info!("onnx session would be created from {path}");
            self.session = Some(());
            Ok(())
        }
        #[cfg(not(feature = "onnx"))]
        {
            let _ = path;
            Err(BrainError::Inference(
                "onnx feature not enabled; build with --features onnx".into(),
            ))
        }
    }

    fn infer(&mut self, input: &InferenceInput) -> Result<InferenceOutput> {
        let _ = input;
        #[cfg(feature = "onnx")]
        {
            // 真实部署：运行会话并后处理 NMS。
            Err(BrainError::Inference("onnx run not wired in demo".into()))
        }
        #[cfg(not(feature = "onnx"))]
        {
            Err(BrainError::Inference(
                "onnx feature not enabled; build with --features onnx".into(),
            ))
        }
    }

    fn name(&self) -> &str {
        "onnx"
    }
}
