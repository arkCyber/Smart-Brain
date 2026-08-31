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
    /// 取第 `row` 行的第 `col` 个元素（**软失效**版本）。
    ///
    /// 越界（行/列超出 `rows`/`cols`，或索引超出底层数据）时返回
    /// [`BrainError::Inference`] 错误而非 panic，便于在推理下游把坏数据当作
    /// 一次可恢复的失败处理（日志告警 / 丢弃该帧 / 降级），而不至于让整个
    /// “大脑”进程崩溃。`at()` 是基于此的快速失败便捷封装。
    pub fn try_at(&self, row: usize, col: usize) -> Result<f32> {
        if row >= self.rows || col >= self.cols {
            return Err(BrainError::Inference(format!(
                "inference output ({row},{col}) outside shape {}x{}",
                self.rows, self.cols
            )));
        }
        // 用 checked 运算避免极端入参下 `row * cols` 溢出。
        let idx = row
            .checked_mul(self.cols)
            .and_then(|v| v.checked_add(col))
            .ok_or_else(|| BrainError::Inference("inference output index overflow".into()))?;
        self.data.get(idx).copied().ok_or_else(|| {
            BrainError::Inference(format!(
                "inference output index {idx} out of bounds ({})",
                self.data.len()
            ))
        })
    }

    /// 取第 `row` 行的第 `col` 个元素。
    ///
    /// 越界时给出明确的错误信息并 panic（生产环境宁可快速失败也不返回静默错误数据）。
    /// 若希望以错误而非 panic 的方式处理越界，请使用 [`Self::try_at`]。
    pub fn at(&self, row: usize, col: usize) -> f32 {
        match self.try_at(row, col) {
            Ok(v) => v,
            Err(e) => panic!("{e}"),
        }
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
///
/// 使用 `ort`（ONNX Runtime 的 Rust 绑定）加载 `.onnx` 模型并执行一次前向推理，
/// 输出按 YOLOv8 风格做“解码 + NMS”后处理，结果为 `[N, 6]` 矩阵
/// （`[class_id, confidence, cx, cy, w, h]`），可直接被 `VisionPipeline` 消费。
///
/// 依赖以 `load-dynamic` 方式接入：编译期无需下载 onnxruntime 二进制，
/// 运行时经 libloading 加载系统安装的 onnxruntime 共享库。
pub struct OnnxModelBackend {
    /// 检测类别数（YOLOv8 的 `nc`，用于输出张量 `[1, 4+nc, anchors]` 解码）。
    num_classes: usize,
    /// 置信度阈值。
    conf_threshold: f32,
    /// NMS IoU 阈值。
    iou_threshold: f32,
    #[cfg(feature = "onnx")]
    session: Option<ort::session::Session>,
}

impl OnnxModelBackend {
    pub fn new() -> Self {
        Self {
            num_classes: 80,
            conf_threshold: 0.25,
            iou_threshold: 0.45,
            #[cfg(feature = "onnx")]
            session: None,
        }
    }

    /// 设置检测类别数。
    pub fn with_num_classes(mut self, n: usize) -> Self {
        self.num_classes = n;
        self
    }

    /// 设置置信度与 NMS IoU 阈值。
    pub fn with_thresholds(mut self, conf: f32, iou: f32) -> Self {
        self.conf_threshold = conf;
        self.iou_threshold = iou;
        self
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
            let session = ort::session::Session::builder()
                .map_err(|e| BrainError::Inference(format!("session builder: {e}")))?
                .commit_from_file(path)
                .map_err(|e| BrainError::Inference(format!("load {path}: {e}")))?;
            self.session = Some(session);
            log::info!("onnx session loaded from {path}");
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
        #[cfg(feature = "onnx")]
        {
            // 1. 构造 NCHW 输入张量（batch, channel, height, width）。
            let shape = input.shape.as_slice();
            if shape.len() != 4 {
                return Err(BrainError::Inference(format!(
                    "expected NCHW input shape, got {shape:?}"
                )));
            }
            let dims = ndarray::IxDyn(shape);
            let arr = ndarray::Array::from_shape_vec(dims, input.data.clone())
                .map_err(|e| BrainError::Inference(format!("build input tensor: {e}")))?;
            let tensor = ort::value::Tensor::from_array(arr)
                .map_err(|e| BrainError::Inference(format!("tensor: {e}")))?;

            // 2. 运行推理（可变借用会话；随后在块末释放借用，避免与读取配置冲突）。
            let data = {
                let session = self
                    .session
                    .as_mut()
                    .ok_or_else(|| BrainError::Inference("onnx model not loaded".into()))?;
                let inputs = ort::inputs![tensor];
                let outputs = session
                    .run(inputs)
                    .map_err(|e| BrainError::Inference(format!("run: {e}")))?;
                let out = &outputs[0];
                let view = out
                    .try_extract_tensor::<f32>()
                    .map_err(|e| BrainError::Inference(format!("extract output: {e}")))?;
                // view = (&Shape, &[f32])，取数据切片复制为所有权的 Vec。
                view.1.to_vec()
            };

            // 3. YOLOv8 解码 + NMS，输出 [N,6]。
            let dets = crate::nms::detect_and_nms(
                &data,
                self.num_classes,
                self.conf_threshold,
                self.iou_threshold,
            )?;
            let cols = 6;
            let mut out_data = Vec::with_capacity(dets.len() * cols);
            for d in &dets {
                out_data.extend_from_slice(&[
                    d.class as f32,
                    d.score,
                    (d.x1 + d.x2) / 2.0,
                    (d.y1 + d.y2) / 2.0,
                    d.x2 - d.x1,
                    d.y2 - d.y1,
                ]);
            }
            Ok(InferenceOutput {
                data: out_data,
                rows: dets.len(),
                cols,
            })
        }
        #[cfg(not(feature = "onnx"))]
        {
            let _ = input;
            Err(BrainError::Inference(
                "onnx feature not enabled; build with --features onnx".into(),
            ))
        }
    }

    fn name(&self) -> &str {
        "onnx"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inference_input_new() {
        let i = InferenceInput::new(vec![1.0, 2.0], vec![1, 1, 1, 2]);
        assert_eq!(i.data.len(), 2);
        assert_eq!(i.shape, vec![1, 1, 1, 2]);
    }

    #[test]
    fn inference_output_at_and_bounds() {
        // 2 行 3 列。
        let o = InferenceOutput {
            data: vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            rows: 2,
            cols: 3,
        };
        assert_eq!(o.at(0, 0), 1.0);
        assert_eq!(o.at(1, 2), 6.0);
        // 越界应 panic 而非返回错误数据。
        assert!(std::panic::catch_unwind(|| o.at(5, 5)).is_err());
    }

    #[test]
    fn try_at_valid_returns_ok_and_matches_at() {
        let o = InferenceOutput {
            data: vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            rows: 2,
            cols: 3,
        };
        // 软失效版本在有效访问上与 at() 一致。
        assert_eq!(o.try_at(0, 0).unwrap(), o.at(0, 0));
        assert_eq!(o.try_at(1, 2).unwrap(), 6.0);
    }

    #[test]
    fn try_at_out_of_shape_is_err_not_panic() {
        let o = InferenceOutput {
            data: vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            rows: 2,
            cols: 3,
        };
        // 行越界。
        assert!(matches!(o.try_at(2, 0), Err(BrainError::Inference(_))));
        // 列越界。
        assert!(matches!(o.try_at(0, 3), Err(BrainError::Inference(_))));
        // 行、列同时越界。
        assert!(matches!(o.try_at(5, 5), Err(BrainError::Inference(_))));
        // 软失效：全程不应 panic。
    }

    #[test]
    fn try_at_out_of_data_is_err() {
        // 形状声明 2x4=8，但底层数据只有 6 个 => 形状内、数据外。
        let o = InferenceOutput {
            data: vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            rows: 2,
            cols: 4,
        };
        let err = o.try_at(1, 3).unwrap_err();
        assert!(matches!(err, BrainError::Inference(_)));
        assert!(err.to_string().contains("out of bounds"));
    }

    #[test]
    fn try_at_index_overflow_is_err() {
        let o = InferenceOutput {
            data: vec![1.0, 2.0, 3.0],
            rows: usize::MAX,
            cols: 2,
        };
        // row*cols 会溢出 usize => 返回错误而非 panic/包裹。
        assert!(matches!(
            o.try_at(usize::MAX - 1, 0),
            Err(BrainError::Inference(_))
        ));
    }

    #[test]
    fn mock_backend_rejects_infer_before_load() {
        let mut b = MockModelBackend::new();
        let input = InferenceInput::new(vec![0.0; 6], vec![1, 1, 1, 6]);
        assert!(matches!(b.infer(&input), Err(BrainError::Inference(_))));
        assert_eq!(b.name(), "mock");
    }

    #[test]
    fn mock_backend_load_then_infer() {
        let mut b = MockModelBackend::new();
        b.load("models/mock.onnx").unwrap();
        let input = InferenceInput::new(vec![0.0; 6], vec![1, 1, 1, 6]);
        let out = b.infer(&input).unwrap();
        assert_eq!(out.rows, 1);
        assert_eq!(out.cols, 6);
        // [class_id, confidence, cx, cy, w, h]
        assert_eq!(out.at(0, 0), 0.0);
        assert_eq!(out.at(0, 1), 0.95);
        assert_eq!(out.at(0, 2), 0.5);
    }

    #[test]
    fn mock_backend_default() {
        let b = MockModelBackend::default();
        assert_eq!(b.name(), "mock");
    }
}
