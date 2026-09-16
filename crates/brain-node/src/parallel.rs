//! 并行（多线程）流水线演示：把“感知”与“决策+传输”拆到独立线程，经线程安全
//! 的 `DataBus` 解耦。对应 README 中“用 tokio/多线程把感知、决策、传输流水线
//! 拆成独立异步任务”的待办方向——这里用 std 线程 + 共享总线实现，无需额外运行时，
//! 且线程间数据流（感知发布 → 决策订阅）可被测试验证。

use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use brain_core::time::instant_now;
use brain_message::{Command, CommandTarget, Mode, TrackingStatus};
use brain_middleware::bus::topics;
use brain_middleware::DataBus;
use brain_perception::backend::MockModelBackend;
use brain_perception::pipeline::{VisionConfig, VisionPipeline};
use brain_transport::{FcuTransport, MockTransport};

/// 并行流水线配置。
#[derive(Debug, Clone)]
pub struct ParallelConfig {
    /// 主线程等待的周期数（随后发出停止信号并 join 线程）。
    pub iterations: usize,
    /// 感知推理周期（毫秒）。
    pub infer_period_ms: u64,
    /// 每个线程的调度周期（毫秒）。
    pub tick_period_ms: u64,
}

impl Default for ParallelConfig {
    fn default() -> Self {
        Self {
            iterations: 10,
            infer_period_ms: 1,
            tick_period_ms: 2,
        }
    }
}

/// 并行流水线运行结果（供断言/日志）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ParallelSummary {
    /// 感知线程完成的 tick 数。
    pub perception_ticks: usize,
    /// 决策线程下发的指令数。
    pub decisions: usize,
    /// 决策线程观测到“锁定目标”的次数（证明线程间数据真正流通）。
    pub locked_seen: usize,
}

/// 并行流水线：感知线程 + 决策/传输线程共享一个 `Arc<DataBus>`。
pub struct ParallelPipeline {
    cfg: ParallelConfig,
    bus: Arc<DataBus>,
}

impl ParallelPipeline {
    pub fn new(cfg: ParallelConfig) -> Self {
        Self {
            cfg,
            bus: Arc::new(DataBus::new()),
        }
    }

    /// 运行流水线：分别起感知线程与决策线程，跑若干周期后停止并汇总结果。
    pub fn run(&self) -> ParallelSummary {
        let stop = Arc::new(AtomicBool::new(false));
        let perception_ticks = Arc::new(AtomicUsize::new(0));
        let decisions = Arc::new(AtomicUsize::new(0));
        let locked_seen = Arc::new(AtomicUsize::new(0));

        // ---- 感知线程：推理并发布检测/跟踪到总线。 ----
        let p_bus = self.bus.clone();
        let p_stop = stop.clone();
        let p_ticks = perception_ticks.clone();
        let cfg = self.cfg.clone();
        let perception_handle = thread::spawn(move || {
            let vcfg = VisionConfig {
                infer_period_ms: cfg.infer_period_ms,
                input_size: 64,
                lock_lost_ms: 500,
            };
            let mut perception = VisionPipeline::new(Box::new(MockModelBackend::new()), vcfg);
            let _ = perception.load_model("models/yolov8n.onnx");
            while !p_stop.load(Ordering::Relaxed) {
                let now = instant_now();
                perception.tick(&p_bus, now);
                perception.publish_tracking(&p_bus, now);
                p_ticks.fetch_add(1, Ordering::Relaxed);
                thread::sleep(Duration::from_millis(cfg.tick_period_ms));
            }
        });

        // ---- 决策 + 传输线程：订阅感知结果，据此决策并下发指令。 ----
        let d_bus = self.bus.clone();
        let d_stop = stop.clone();
        let d_decisions = decisions.clone();
        let d_locked = locked_seen.clone();
        let cfg = self.cfg.clone();
        let decision_handle = thread::spawn(move || {
            let mut transport = MockTransport::new();
            while !d_stop.load(Ordering::Relaxed) {
                let now = instant_now();
                // 从总线读取感知的跟踪状态（线程间通信的“神经”）。
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
                let cmd = Command {
                    timestamp: now,
                    mode,
                    target: CommandTarget::None,
                };
                let _ = transport.send_command(&cmd);
                d_decisions.fetch_add(1, Ordering::Relaxed);
                thread::sleep(Duration::from_millis(cfg.tick_period_ms));
            }
        });

        // 主线程等待若干周期后发停止信号并回收线程。
        for _ in 0..self.cfg.iterations {
            thread::sleep(Duration::from_millis(self.cfg.tick_period_ms));
        }
        stop.store(true, Ordering::Relaxed);
        let _ = perception_handle.join();
        let _ = decision_handle.join();

        ParallelSummary {
            perception_ticks: perception_ticks.load(Ordering::Relaxed),
            decisions: decisions.load(Ordering::Relaxed),
            locked_seen: locked_seen.load(Ordering::Relaxed),
        }
    }
}

/// 演示入口：运行并行流水线并打印汇总。
pub fn run_parallel_demo() {
    let pipeline = ParallelPipeline::new(ParallelConfig::default());
    let s = pipeline.run();
    println!(
        "=== 并行流水线（多线程）===\n  \
         感知线程 tick={}，决策线程下发指令 {}，观测到锁定目标 {} 次",
        s.perception_ticks, s.decisions, s.locked_seen
    );
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parallel_pipeline_runs_all_threads() {
        // 周期数要足够长：这里是“两个线程 + 跨线程数据流”的断言，若窗口过短，
        // 在共享/慢速 CI runner 上可能因调度抖动出现“决策线程尚未观察到 Locked”
        // 的偶发失败（本地快机不易复现）。200 个 1ms 周期 ≈ 200ms 预算，
        // 远大于感知线程获取锁定（几 ms）所需时间。
        let cfg = ParallelConfig {
            iterations: 200,
            infer_period_ms: 1,
            tick_period_ms: 1,
        };
        let pipeline = ParallelPipeline::new(cfg);
        let s = pipeline.run();
        assert!(s.perception_ticks > 0, "perception thread should tick");
        assert!(s.decisions > 0, "decision thread should issue commands");
        assert!(
            s.locked_seen > 0,
            "decision should observe a locked target (ticks={}, decisions={}, locked={})",
            s.perception_ticks,
            s.decisions,
            s.locked_seen
        );
    }

    #[test]
    fn bus_is_shared_across_threads() {
        // 证明 DataBus 可跨线程共享：主线程发布，线程内订阅。
        let bus = Arc::new(DataBus::new());
        let b2 = bus.clone();
        let handle = thread::spawn(move || {
            let _ = b2.publish(topics::HEARTBEAT, 42u64, 0);
        });
        handle.join().unwrap();
        let t = bus.topic::<u64>(topics::HEARTBEAT).unwrap();
        assert_eq!(t.peek(), Some(42));
    }
}
