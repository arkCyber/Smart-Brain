//! 统一通信后端抽象：Pub/Sub + Store/Query + Compute。

use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::sync::Arc;
use std::time::Duration;

use brain_core::Result;

use crate::core::{Reply, Sample, Value};

/// 计算/服务处理器：收到查询时返回若干应答值（由后端路由回查询方）。
pub type QueryHandler = Arc<dyn Fn(&str) -> Vec<Value> + Send + Sync>;

/// 一个订阅：从通道接收 `Sample`。
pub struct Subscription {
    rx: Receiver<Sample>,
    /// 对应键表达式（用于日志/校验）。
    key: String,
}

impl Subscription {
    pub(crate) fn new(key: String, rx: Receiver<Sample>) -> Self {
        Self { rx, key }
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
