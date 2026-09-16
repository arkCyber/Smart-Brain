//! 统一通信后端抽象：Pub/Sub + Store/Query + Compute。

use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::sync::Arc;
use std::time::Duration;

use brain_core::Result;

use crate::core::{Reply, Sample, Value};

/// 计算/服务处理器：收到查询时返回若干应答值（由后端路由回查询方）。
///
/// 返回 `Err` 表示该次计算失败——后端会记录日志并跳过该应答，而不会崩溃
/// 查询方（与 handler panic 同样被隔离）。
pub type QueryHandler = Arc<dyn Fn(&str) -> Result<Vec<Value>> + Send + Sync>;

/// 一个订阅：从通道接收 `Sample`。
pub struct Subscription {
    rx: Receiver<Sample>,
    /// 对应键表达式（用于日志/校验）。
    key: String,
    /// 可选退订回调：调用后从后端移除该订阅（主动退订；`Drop` 时也会触发）。
    unsubscribe: Option<Arc<dyn Fn() + Send + Sync>>,
}

impl Subscription {
    /// 构造无退订回调的订阅（用于无法主动注销的后端/测试）。
    #[cfg_attr(not(feature = "real-zenoh"), allow(dead_code))]
    pub(crate) fn new(key: String, rx: Receiver<Sample>) -> Self {
        Self {
            rx,
            key,
            unsubscribe: None,
        }
    }

    /// 构造带退订回调的订阅（生产后端用它实现主动注销）。
    pub(crate) fn with_unsubscribe(
        key: String,
        rx: Receiver<Sample>,
        unsubscribe: Arc<dyn Fn() + Send + Sync>,
    ) -> Self {
        Self {
            rx,
            key,
            unsubscribe: Some(unsubscribe),
        }
    }

    /// 阻塞接收一条样本。
    pub fn recv(&self) -> Result<Sample> {
        self.rx
            .recv()
            .map_err(|_| brain_core::BrainError::Bus("subscription channel closed".into()))
    }

    /// 非阻塞接收。
    pub fn try_recv(&self) -> std::result::Result<Sample, TryRecvError> {
        self.rx.try_recv()
    }

    /// 带超时接收。
    pub fn recv_timeout(&self, dur: Duration) -> std::result::Result<Sample, RecvTimeoutError> {
        self.rx.recv_timeout(dur)
    }

    /// 订阅的键表达式。
    pub fn key(&self) -> &str {
        &self.key
    }

    /// 主动退订（幂等）。`Drop` 时也会自动调用。
    pub fn unsubscribe(&self) {
        if let Some(f) = &self.unsubscribe {
            f();
        }
    }
}

impl Drop for Subscription {
    fn drop(&mut self) {
        self.unsubscribe();
    }
}

/// 一个可查询的“计算/存储”句柄。`Drop` 时从后端注销。
pub struct QueryableHandle {
    key: String,
    // 通过共享退订标记实现自动注销。
    unregister: Arc<dyn Fn() + Send + Sync>,
}

impl QueryableHandle {
    pub(crate) fn new(key: String, unregister: Arc<dyn Fn() + Send + Sync>) -> Self {
        Self { key, unregister }
    }

    pub fn key(&self) -> &str {
        &self.key
    }

    /// 显式注销。
    pub fn unregister(&self) {
        (self.unregister)();
    }
}

impl Drop for QueryableHandle {
    fn drop(&mut self) {
        (self.unregister)();
    }
}

/// 统一通信后端：实现 Zenoh 三支柱（Pub/Sub + Store/Query + Compute）。
pub trait CommBackend: Send + Sync {
    /// 发布一条数据（同时写入“存储”以便被查询）。
    fn put(&self, key: &str, value: Value) -> Result<()>;

    /// 订阅一个键表达式，返回订阅通道（保留最近值：订阅即收到当前值）。
    fn subscribe(&self, key: &str) -> Result<Subscription>;

    /// 查询：从匹配的所有“存储”拉取数据，并在匹配的“计算”上触发计算。
    fn get(&self, key: &str) -> Result<Vec<Reply>>;

    /// 声明一个可查询的“计算/服务”，在 `get` 时被触发。
    fn declare_queryable(&self, key: &str, handler: QueryHandler) -> Result<QueryableHandle>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_core::time::Timestamp;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Mutex;
    use std::time::Duration;

    #[test]
    fn subscription_recv_try_recv_and_key() {
        let (tx, rx) = std::sync::mpsc::channel::<Sample>();
        let sub = Subscription::new("sensor/temp".to_string(), rx);
        assert_eq!(sub.key(), "sensor/temp");
        // 无数据 -> try_recv 失败。
        assert!(sub.try_recv().is_err());
        // 发送一条 -> recv / try_recv 均能取到。
        tx.send(Sample::new("sensor/temp", vec![1, 2, 3], 42))
            .unwrap();
        let s = sub.recv().unwrap();
        assert_eq!(s.timestamp, 42);
        assert_eq!(s.value, vec![1, 2, 3]);
    }

    #[test]
    fn subscription_recv_timeout() {
        let (_tx, rx) = std::sync::mpsc::channel::<Sample>();
        let sub = Subscription::new("a".to_string(), rx);
        assert!(matches!(
            sub.recv_timeout(Duration::from_millis(5)),
            Err(std::sync::mpsc::RecvTimeoutError::Timeout)
        ));
    }

    #[test]
    fn queryable_handle_key_and_unregister() {
        let called = Arc::new(AtomicBool::new(false));
        let flag = called.clone();
        let handle = QueryableHandle::new(
            "svc/add".to_string(),
            Arc::new(move || {
                flag.store(true, Ordering::SeqCst);
            }),
        );
        assert_eq!(handle.key(), "svc/add");
        assert!(!called.load(Ordering::SeqCst));
        handle.unregister();
        assert!(called.load(Ordering::SeqCst));
    }

    #[test]
    fn queryable_handle_drop_auto_unregisters() {
        // 用 Mutex 包裹计数，跨 Drop 检查自动注销。
        let count = Arc::new(Mutex::new(0u32));
        let inner = count.clone();
        {
            let _handle = QueryableHandle::new(
                "svc/x".to_string(),
                Arc::new(move || {
                    *inner.lock().unwrap() += 1;
                }),
            );
            assert_eq!(*count.lock().unwrap(), 0);
        } // 这里 Drop
        assert_eq!(*count.lock().unwrap(), 1, "Drop 应自动注销一次");
    }

    #[test]
    fn sample_and_reply_constructors() {
        let s = Sample::new("k", vec![9], Timestamp::default());
        assert_eq!(s.key, "k");
        assert_eq!(s.value, vec![9]);
        let r = Reply::new("k", vec![8]);
        assert_eq!(r.key, "k");
        assert_eq!(r.value, vec![8]);
    }
}
