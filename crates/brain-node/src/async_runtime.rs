//! tokio 异步运行时演示：感知/决策作为 async 任务并发执行（`--features async`）。
//!
//! 与 [`super::parallel`] 的 std 线程版互补——这里用 tokio 的 async 任务在单一运行时上
//! 并发调度感知与决策，通过线程安全的 `Arc<DataBus>` 通信。tokio 的 `rt-multi-thread`
//! 会在多线程上并行执行这些任务，为将来把感知/决策/传输拆成独立异步服务提供起点。

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::Duration;

use brain_core::time::instant_now;
use brain_message::{Command, CommandTarget, Mode, TrackingStatus};
use brain_middleware::bus::topics;
use brain_middleware::DataBus;
use brain_perception::backend::MockModelBackend;
use brain_perception::pipeline::{VisionConfig, VisionPipeline};
use brain_transport::{FcuTransport, MockTransport};

/// 异步流水线运行结果（供断言/日志）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AsyncSummary {
    pub perception_ticks: usize,
    pub decisions: usize,
    pub locked_seen: usize,
}

/// 在给定的 tokio 运行时上运行异步流水线，返回汇总。
pub async fn run_async() -> AsyncSummary {
    let bus = Arc::new(DataBus::new());
    let stop = Arc::new(AtomicBool::new(false));
    let perception_ticks = Arc::new(AtomicUsize::new(0));
    let decisions = Arc::new(AtomicUsize::new(0));
    let locked_seen = Arc::new(AtomicUsize::new(0));

    // ---- 感知任务：推理并发布到总线。 ----
    let p_bus = bus.clone();
    let p_stop = stop.clone();
    let p_ticks = perception_ticks.clone();
    let perception_task = tokio::spawn(async move {
        let vcfg = VisionConfig {
            infer_period_ms: 1,
            input_size: 64,
            lock_lost_ms: 500,
        };
        let mut perception = VisionPipeline::new(Box::new(MockModelBackend::new()), vcfg);
        let _ = perception.load_model("models/yolov8n.onnx");
        while !p_stop.load(Ordering::Relaxed) {
            tokio::time::sleep(Duration::from_millis(2)).await;
            let now = instant_now();
            perception.tick(&p_bus, now);
            perception.publish_tracking(&p_bus, now);
            p_ticks.fetch_add(1, Ordering::Relaxed);
        }
    });

    // ---- 决策 + 传输任务：订阅感知结果并下发指令。 ----
    let d_bus = bus.clone();
    let d_stop = stop.clone();
    let d_decisions = decisions.clone();
    let d_locked = locked_seen.clone();
    let decision_task = tokio::spawn(async move {
        let mut transport = MockTransport::new();
        while !d_stop.load(Ordering::Relaxed) {
            tokio::time::sleep(Duration::from_millis(2)).await;
            let now = instant_now();
            let tracking = d_bus
                .topic::<TrackingStatus>(topics::TRACKING)
                .and_then(|t| t.peek());
            let mode = match tracking {
                Some(TrackingStatus::Locked { .. }) => {
                    d_locked.fetch_add(1, Ordering::Relaxed);
                    Mode::Track
                }
                _ => Mode::Cruise,
            };
            let _ = transport.send_command(&Command {
                timestamp: now,
                mode,
                target: CommandTarget::None,
            });
            d_decisions.fetch_add(1, Ordering::Relaxed);
        }
    });

    // 主协程等待一段时间后发出停止信号并回收任务。
    tokio::time::sleep(Duration::from_millis(40)).await;
    stop.store(true, Ordering::Relaxed);
    let _ = perception_task.await;
    let _ = decision_task.await;

    AsyncSummary {
        perception_ticks: perception_ticks.load(Ordering::Relaxed),
        decisions: decisions.load(Ordering::Relaxed),
        locked_seen: locked_seen.load(Ordering::Relaxed),
    }
}

/// 演示入口：构造 tokio 运行时并运行异步流水线。
pub fn run() {
    let rt = tokio::runtime::Runtime::new().expect("build tokio runtime");
    let s = rt.block_on(run_async());
    println!(
        "=== tokio 异步流水线 ===\n  感知 tick={}，决策下发指令 {}，观测锁定 {} 次",
        s.perception_ticks, s.decisions, s.locked_seen
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn async_pipeline_runs_and_communicates() {
        let s = run_async().await;
        assert!(s.perception_ticks > 0, "perception task should tick");
        assert!(s.decisions > 0, "decision task should issue commands");
        assert!(s.locked_seen > 0, "decision should observe a locked target");
    }
}
