//! 感知流水线：组织“取帧 -> 推理 -> 检测/跟踪”并发布到数据总线。

use brain_core::time::{instant_now, Timestamp};
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

#[allow(dead_code)]
fn _usage(now: Timestamp) {
    let _ = instant_now();
    let _ = now;
}
