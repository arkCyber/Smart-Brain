//! 进程内 Zenoh 语义实现（`LocalZenoh`）。
//!
//! 无需任何网络依赖，完整复刻 Zenoh 的统一通信语义：
//! - `put` 写入“存储”并通知订阅者（保留最近值）
//! - `subscribe` 订阅键表达式（订阅即收到当前值 = 保留语义）
//! - `get` 聚合所有匹配“存储”的值，并触发匹配的“计算/服务”
//! - `declare_queryable` 注册一个按需计算（Compute）
//!
//! 线程安全（内部 `Mutex`），可在多线程/多模块间共享；测试无需真实网络。

use std::collections::HashMap;
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, Mutex};

use brain_core::time::Timestamp;
use brain_core::Result;

use crate::backend::{CommBackend, QueryHandler, QueryableHandle, Subscription};
use crate::core::{key_matches, Reply, Sample, Value};

/// `LocalZenoh` 错误。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LocalZenohError {
    /// 内部状态损坏。
    Poisoned,
}

impl std::fmt::Display for LocalZenohError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "LocalZenoh error: {self:?}")
    }
}

impl std::error::Error for LocalZenohError {}

impl From<LocalZenohError> for brain_core::BrainError {
    fn from(e: LocalZenohError) -> Self {
        brain_core::BrainError::Bus(e.to_string())
    }
}

/// 存储中的一个条目：保留最近值 + 订阅者 + 可查询的计算。
#[derive(Default)]
struct Entry {
    value: Option<Value>,
    timestamp: Timestamp,
    subscribers: Vec<Sender<Sample>>,
    queryables: Vec<(u64, QueryHandler)>,
}

#[derive(Default)]
struct Registry {
    entries: HashMap<String, Entry>,
    next_queryable_id: u64,
}

/// 进程内 Zenoh 后端。
pub struct LocalZenoh {
    inner: Arc<Mutex<Registry>>,
}

impl Default for LocalZenoh {
    fn default() -> Self {
        Self::new()
    }
}

impl LocalZenoh {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(Registry::default())),
        }
    }

    /// 已存储的键数量（监控用）。
    pub fn store_count(&self) -> usize {
        self.inner.lock().map(|g| g.entries.len()).unwrap_or(0)
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Registry> {
        self.inner.lock().unwrap_or_else(|p| p.into_inner())
    }
}

impl CommBackend for LocalZenoh {
    fn put(&self, key: &str, value: Value) -> Result<()> {
        let mut reg = self.lock();
        // 写入存储（键 = key）。
        let entry = reg.entries.entry(key.to_string()).or_default();
        let ts = brain_core::time::instant_now();
        entry.value = Some(value.clone());
        entry.timestamp = ts;

        // 通知所有“键表达式匹配”的订阅者（支持通配订阅）。
        let sample = Sample::new(key, value, ts);
        for (expr, e) in reg.entries.iter_mut() {
            if key_matches(expr, key) {
                e.subscribers.retain(|tx| tx.send(sample.clone()).is_ok());
            }
        }
        log::trace!("[local-zenoh] put {key}");
        Ok(())
    }

    fn subscribe(&self, key: &str) -> Result<Subscription> {
        let mut reg = self.lock();
        let (tx, rx) = channel::<Sample>();
        // 保留语义：把当前所有匹配键的已存值发送给新订阅者。
        let mut retained = Vec::new();
        for (ek, e) in reg.entries.iter() {
            if key_matches(key, ek) {
                if let Some(v) = &e.value {
                    retained.push(Sample::new(ek.clone(), v.clone(), e.timestamp));
                }
            }
        }
        for s in retained {
            let _ = tx.send(s);
        }
        // 登记订阅者（用键表达式本身作为条目键，以便通配匹配）。
        let entry = reg.entries.entry(key.to_string()).or_default();
        entry.subscribers.push(tx);
        Ok(Subscription::new(key.to_string(), rx))
    }

    fn get(&self, key: &str) -> Result<Vec<Reply>> {
        let reg = self.lock();
        let mut replies = Vec::new();
        // 聚合所有匹配的“存储”值，并触发匹配的“计算/服务”。
        for (k, e) in reg.entries.iter().filter(|(k, _)| key_matches(key, k)) {
            if let Some(v) = &e.value {
                replies.push(Reply::new(k.clone(), v.clone()));
            }
            for (_, h) in &e.queryables {
                for out in h(k) {
                    replies.push(Reply::new(k.clone(), out));
                }
            }
        }
        Ok(replies)
    }

    fn declare_queryable(&self, key: &str, handler: QueryHandler) -> Result<QueryableHandle> {
        let mut reg = self.lock();
        let id = reg.next_queryable_id;
        reg.next_queryable_id += 1;
        let entry_key = key.to_string();
        let entry = reg.entries.entry(entry_key.clone()).or_default();
        entry.queryables.push((id, handler));

        // 注销闭包：Drop 时从条目移除该 queryable。
        let inner = self.inner.clone();
        let closure_key = entry_key.clone();
        let unregister: Arc<dyn Fn() + Send + Sync> = Arc::new(move || {
            if let Ok(mut g) = inner.lock() {
                if let Some(e) = g.entries.get_mut(&closure_key) {
                    e.queryables.retain(|(i, _)| *i != id);
                }
            }
        });
        Ok(QueryableHandle::new(entry_key, unregister))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicUsize, Ordering};

    #[test]
    fn pubsub_delivers() {
        let z = LocalZenoh::new();
        let sub = z.subscribe("fcu/telemetry").unwrap();
        z.put("fcu/telemetry", b"hi".to_vec()).unwrap();
        let s = sub
            .recv_timeout(std::time::Duration::from_millis(100))
            .unwrap();
        assert_eq!(s.value, b"hi");
        assert_eq!(s.key, "fcu/telemetry");
    }

    #[test]
    fn subscribe_retains_latest() {
        let z = LocalZenoh::new();
        z.put("sensor/temp", b"21.5".to_vec()).unwrap();
        // 订阅应立刻收到已存储的最近值（保留语义）。
        let sub = z.subscribe("sensor/temp").unwrap();
        let s = sub.try_recv().expect("retained value");
        assert_eq!(s.value, b"21.5");
    }

    #[test]
    fn store_query_returns_latest() {
        let z = LocalZenoh::new();
        z.put("map/cells", b"c1".to_vec()).unwrap();
        z.put("map/cells", b"c2".to_vec()).unwrap(); // 覆盖
        let replies = z.get("map/cells").unwrap();
        assert_eq!(replies.len(), 1);
        assert_eq!(replies[0].value, b"c2");
    }

    #[test]
    fn prefix_query_aggregates_stores() {
        let z = LocalZenoh::new();
        z.put("sensor/imu", b"accel".to_vec()).unwrap();
        z.put("sensor/temp", b"temp".to_vec()).unwrap();
        z.put("other/x", b"x".to_vec()).unwrap();
        let replies = z.get("sensor/*").unwrap();
        assert_eq!(replies.len(), 2);
        assert!(replies.iter().any(|r| r.key == "sensor/imu"));
        assert!(replies.iter().any(|r| r.key == "sensor/temp"));
    }

    #[test]
    fn compute_queryable_triggered_on_get() {
        let z = LocalZenoh::new();
        let calls = Arc::new(AtomicUsize::new(0));
        let calls2 = calls.clone();
        let handler: QueryHandler = Arc::new(move |key: &str| {
            calls2.fetch_add(1, Ordering::SeqCst);
            vec![format!("computed:{key}").into_bytes()]
        });
        let _handle = z.declare_queryable("service/distance", handler).unwrap();
        let replies = z.get("service/distance").unwrap();
        assert_eq!(replies.len(), 1);
        assert_eq!(replies[0].value, b"computed:service/distance");
        assert_eq!(calls.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn queryable_removed_on_drop() {
        let z = LocalZenoh::new();
        let handler: QueryHandler = Arc::new(|_| vec![b"x".to_vec()]);
        {
            let handle = z.declare_queryable("svc/f", handler).unwrap();
            assert_eq!(z.get("svc/f").unwrap().len(), 1);
            handle.unregister();
        }
        // 注销后不应再有应答。
        assert_eq!(z.get("svc/f").unwrap().len(), 0);
    }

    #[test]
    fn multithread_pubsub() {
        let z = Arc::new(LocalZenoh::new());
        let sub = z.subscribe("swarm/*").unwrap();
        let mut handles = Vec::new();
        for i in 0..4 {
            let z = z.clone();
            handles.push(std::thread::spawn(move || {
                for j in 0..10 {
                    z.put(&format!("swarm/node{i}"), format!("{j}").into_bytes())
                        .unwrap();
                }
            }));
        }
        for h in handles {
            h.join().unwrap();
        }
        // 至少应能收到若干样本（可能有丢弃，但一定 > 0）。
        let mut count = 0;
        while let Ok(_s) = sub.recv_timeout(std::time::Duration::from_millis(50)) {
            count += 1;
        }
        assert!(count > 0, "no samples received");
    }
}
