//! 感知流水线：组织“取帧 -> 推理 -> 检测/跟踪”并发布到数据总线。

use brain_core::time::Timestamp;
use brain_message::{Detection, TrackingStatus};
use brain_middleware::bus::topics;
use brain_middleware::DataBus;

use crate::backend::{InferenceInput, ModelBackend};

/// 感知流水线配置。
#[derive(Debug, Clone)]
pub struct VisionConfig {
    /// 推理间隔（毫秒）。
    pub infer_period_ms: u64,
    /// 输入图像边长（正方形，用于构造输入张量）。
    pub input_size: usize,
    /// 模拟目标丢失判定阈值（毫秒）。
    pub lock_lost_ms: u64,
}

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            infer_period_ms: 100,
            input_size: 640,
            lock_lost_ms: 500,
        }
    }
}

/// 感知流水线。内部持有推理后端，输出检测与跟踪状态。
pub struct VisionPipeline {
    backend: Box<dyn ModelBackend>,
    config: VisionConfig,
    last_infer: Timestamp,
    /// 当前跟踪状态。
    tracking: TrackingStatus,
    lock_acquired_at: Timestamp,
    last_seen: Timestamp,
}

impl VisionPipeline {
    /// 使用给定后端与配置创建流水线。
    pub fn new(backend: Box<dyn ModelBackend>, config: VisionConfig) -> Self {
        Self {
            backend,
            config,
            last_infer: 0,
            tracking: TrackingStatus::NoTarget,
            lock_acquired_at: 0,
            last_seen: 0,
        }
    }

    /// 使用真实 ONNX 后端创建流水线（`onnx` feature 时 `load_model` 才真正加载模型；
    /// 未启用时 `load_model` 会返回明确错误）。`num_classes` 为 YOLOv8 的类别数。
    pub fn onnx(config: VisionConfig, num_classes: usize) -> Self {
        let backend = crate::backend::OnnxModelBackend::new().with_num_classes(num_classes);
        Self::new(Box::new(backend), config)
    }

    /// 加载模型。
    pub fn load_model(&mut self, path: &str) -> brain_core::Result<()> {
        self.backend.load(path)
    }

    /// 当前跟踪状态。
    pub fn tracking(&self) -> &TrackingStatus {
        &self.tracking
    }

    /// 执行一次感知循环：按周期推理，更新跟踪状态，发布到总线。
    pub fn tick(&mut self, bus: &DataBus, now: Timestamp) {
        let due = now.saturating_sub(self.last_infer) >= self.config.infer_period_ms;
        if !due {
            self.update_tracking(now);
            return;
        }
        self.last_infer = now;

        // 1. 构造输入张量（仿真用全零帧）。
        let input = InferenceInput::new(
            vec![0.0f32; self.config.input_size * self.config.input_size * 3],
            vec![1, 3, self.config.input_size, self.config.input_size],
        );

        // 2. 推理。
        let output = match self.backend.infer(&input) {
            Ok(o) => o,
            Err(e) => {
                log::error!("perception infer failed: {e}");
                self.update_tracking(now);
                return;
            }
        };

        // 3. 把输出转成检测（仿真后端输出一行检测框）。
        if output.rows > 0 {
            let confidence = output.at(0, 1);
            let class_id = output.at(0, 0) as u32;
            let range_m = 5.0 + (now % 10) as f32; // 模拟深度估计
            let det = Detection {
                class_id,
                confidence,
                bearing_yaw: 0.15,
                bearing_pitch: 0.05,
                range_m,
            };
            if self.last_seen == 0 {
                self.lock_acquired_at = now;
            }
            self.last_seen = now;
            let lock_age = now.saturating_sub(self.lock_acquired_at);
            self.tracking = TrackingStatus::Locked {
                target: det.clone(),
                lock_age_ms: lock_age,
            };
            // 发布检测结果到数据总线。
            let _ = bus.publish(topics::DETECTIONS, det, now);
        } else {
            self.tracking = TrackingStatus::NoTarget;
        }

        self.update_tracking(now);
    }

    fn update_tracking(&mut self, now: Timestamp) {
        if matches!(self.tracking, TrackingStatus::Locked { .. }) {
            let since = now.saturating_sub(self.last_seen);
            if since > self.config.lock_lost_ms {
                self.tracking = TrackingStatus::Lost {
                    last_seen_ms: self.last_seen,
                };
            }
        }
    }

    /// 将当前跟踪状态发布到总线。
    pub fn publish_tracking(&self, bus: &DataBus, now: Timestamp) {
        let _ = bus.publish(topics::TRACKING, self.tracking.clone(), now);
    }
}

/// 便捷构造：默认仿真感知流水线。
pub fn default_pipeline() -> VisionPipeline {
    VisionPipeline::new(
        Box::new(crate::backend::MockModelBackend::new()),
        VisionConfig::default(),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::backend::MockModelBackend;
    use brain_middleware::bus::topics;

    /// 快速推理 + 快速失锁判定的配置，便于测试。
    fn fast_config() -> VisionConfig {
        VisionConfig {
            infer_period_ms: 1000,
            input_size: 32,
            lock_lost_ms: 30,
        }
    }

    #[test]
    fn loads_model_and_acquires_lock_on_first_tick() {
        let mut p = VisionPipeline::new(Box::new(MockModelBackend::new()), fast_config());
        p.load_model("mock.onnx").unwrap();
        let bus = DataBus::new();
        p.tick(&bus, 1000); // now>=infer_period -> 推理并锁定
        assert!(matches!(p.tracking(), TrackingStatus::Locked { .. }));
    }

    #[test]
    fn publishes_detection_and_tracking_to_bus() {
        let mut p = VisionPipeline::new(Box::new(MockModelBackend::new()), fast_config());
        p.load_model("mock.onnx").unwrap();
        let bus = DataBus::new();
        p.tick(&bus, 1000);
        p.publish_tracking(&bus, 1000);

        let det = bus.topic::<Detection>(topics::DETECTIONS).unwrap();
        let det = det.peek().expect("detection should be published");
        assert!(det.confidence > 0.9);

        let track = bus.topic::<TrackingStatus>(topics::TRACKING).unwrap();
        assert!(matches!(track.peek(), Some(TrackingStatus::Locked { .. })));
    }

    #[test]
    fn loses_lock_when_inference_stops() {
        let mut p = VisionPipeline::new(Box::new(MockModelBackend::new()), fast_config());
        p.load_model("mock.onnx").unwrap();
        let bus = DataBus::new();
        p.tick(&bus, 1000); // 锁定
        assert!(matches!(p.tracking(), TrackingStatus::Locked { .. }));
        // 50ms 后无新推理（50<infer_period），且超过失锁阈值（50>lock_lost_ms）
        p.tick(&bus, 1050);
        assert!(matches!(p.tracking(), TrackingStatus::Lost { .. }));
    }

    #[test]
    fn unloaded_model_reports_no_target() {
        let mut p = VisionPipeline::new(Box::new(MockModelBackend::new()), fast_config());
        let bus = DataBus::new();
        // 未 load_model -> 推理失败 -> 无目标
        p.tick(&bus, 1000);
        assert!(matches!(p.tracking(), TrackingStatus::NoTarget));
    }

    #[test]
    fn tracking_returns_no_target_before_any_tick() {
        let p = VisionPipeline::new(Box::new(MockModelBackend::new()), fast_config());
        assert!(matches!(p.tracking(), TrackingStatus::NoTarget));
    }

    #[test]
    fn onnx_factory_builds_backend_and_reports_no_target() {
        // onnx() 工厂始终可构造；未加载模型时推理应失败并保持 NoTarget。
        let mut p = VisionPipeline::onnx(fast_config(), 80);
        let bus = DataBus::new();
        p.tick(&bus, 1000);
        assert!(matches!(p.tracking(), TrackingStatus::NoTarget));
        // 未启用 onnx feature 时 load_model 应给出明确错误。
        let err = p.load_model("models/yolov8n.onnx").unwrap_err();
        let _ = err;
    }
}
