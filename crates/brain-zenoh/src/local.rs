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
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LocalZenohError {
    /// 内部状态损坏。
    Poisoned,
    /// 键或键表达式非法（见 [`crate::core::valid_key_expr`]）。
    InvalidKey(String),
}

impl std::fmt::Display for LocalZenohError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LocalZenohError::Poisoned => write!(f, "LocalZenoh state poisoned"),
            LocalZenohError::InvalidKey(k) => write!(f, "invalid key/key-expression: {k}"),
        }
    }
}

impl std::error::Error for LocalZenohError {}

impl From<LocalZenohError> for brain_core::BrainError {
    fn from(e: LocalZenohError) -> Self {
        brain_core::BrainError::Bus(e.to_string())
    }
}

/// 存储中的一个条目：保留最近值 + 订阅者（按 id）+ 可查询的计算。
#[derive(Default)]
struct Entry {
    value: Option<Value>,
    timestamp: Timestamp,
    /// 订阅者通道（`(订阅者 id, 发送端)`）；键表达式条目由订阅产生。
    subscribers: Vec<(u64, Sender<Sample>)>,
    queryables: Vec<(u64, QueryHandler)>,
}

#[derive(Default)]
struct Registry {
    entries: HashMap<String, Entry>,
    next_queryable_id: u64,
    next_subscriber_id: u64,
}

impl Registry {
    /// 若某键的条目已完全腾空（无值/无订阅/无计算），则移除，防止无界增长。
    fn prune(&mut self, key: &str) {
        let empty = self
            .entries
            .get(key)
            .map(|e| e.value.is_none() && e.subscribers.is_empty() && e.queryables.is_empty())
            .unwrap_or(false);
        if empty {
            self.entries.remove(key);
        }
    }
}

/// 进程内 Zenoh 后端。
#[derive(Clone)]
pub struct LocalZenoh {
    inner: Arc<Mutex<Registry>>,
}

impl Default for LocalZenoh {
    fn default() -> Self {
        Self::new()
    }
}

impl std::fmt::Debug for LocalZenoh {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LocalZenoh")
            .field("store_count", &self.store_count())
            .field("subscription_count", &self.subscription_count())
            .field("queryable_count", &self.queryable_count())
            .finish()
    }
}

impl LocalZenoh {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(Registry::default())),
        }
    }

    /// 已存储的**有值**键数量（监控用；不含仅订阅/仅计算的条目）。
    pub fn store_count(&self) -> usize {
        self.lock()
            .entries
            .values()
            .filter(|e| e.value.is_some())
            .count()
    }

    /// 查询某具体键的最近值（不触发计算）。
    pub fn value(&self, key: &str) -> Option<Value> {
        self.lock().entries.get(key).and_then(|e| e.value.clone())
    }

    /// 某具体键是否已存储值。
    pub fn contains(&self, key: &str) -> bool {
        self.value(key).is_some()
    }

    /// 移除某具体键的存储值（返回旧值）；订阅/计算条目在腾空后自动清理。
    pub fn remove(&self, key: &str) -> Result<Option<Value>> {
        if !crate::core::is_concrete_key(key) {
            return Err(LocalZenohError::InvalidKey(key.into()).into());
        }
        let mut reg = self.lock();
        let old = reg.entries.get_mut(key).and_then(|e| e.value.take());
        reg.prune(key);
        Ok(old)
    }

    /// 清空所有存储值（订阅/计算条目在腾空后自动清理），返回清空的键数。
    pub fn clear(&self) -> usize {
        let mut reg = self.lock();
        let keys: Vec<String> = reg
            .entries
            .iter()
            .filter(|(_, e)| e.value.is_some())
            .map(|(k, _)| k.clone())
            .collect();
        let n = keys.len();
        for k in keys {
            if let Some(e) = reg.entries.get_mut(&k) {
                e.value = None;
            }
            reg.prune(&k);
        }
        n
    }

    /// 当前活跃订阅者总数（监控用）。
    pub fn subscription_count(&self) -> usize {
        self.lock()
            .entries
            .values()
            .map(|e| e.subscribers.len())
            .sum()
    }

    /// 当前已注册的计算（queryable）总数（监控用）。
    pub fn queryable_count(&self) -> usize {
        self.lock()
            .entries
            .values()
            .map(|e| e.queryables.len())
            .sum()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Registry> {
        self.inner.lock().unwrap_or_else(|p| p.into_inner())
    }
}

impl CommBackend for LocalZenoh {
    fn put(&self, key: &str, value: Value) -> Result<()> {
        if !crate::core::is_concrete_key(key) {
            return Err(LocalZenohError::InvalidKey(key.into()).into());
        }
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
                e.subscribers
                    .retain(|(_, tx)| tx.send(sample.clone()).is_ok());
            }
        }
        log::trace!("[local-zenoh] put {key}");
        Ok(())
    }

    fn subscribe(&self, key: &str) -> Result<Subscription> {
        if !crate::core::valid_key_expr(key) {
            return Err(LocalZenohError::InvalidKey(key.into()).into());
        }
        let mut reg = self.lock();
        let (tx, rx) = channel::<Sample>();
        let id = reg.next_subscriber_id;
        reg.next_subscriber_id += 1;

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
        entry.subscribers.push((id, tx));

        // 退订闭包：主动退订/`Drop` 时按 id 移除该订阅者，腾空则清理条目。
        let inner = self.inner.clone();
        let unsub_key = key.to_string();
        let unsubscribe: Arc<dyn Fn() + Send + Sync> = Arc::new(move || {
            if let Ok(mut g) = inner.lock() {
                if let Some(e) = g.entries.get_mut(&unsub_key) {
                    e.subscribers.retain(|(sid, _)| *sid != id);
                }
                g.prune(&unsub_key);
            }
        });
        Ok(Subscription::with_unsubscribe(
            key.to_string(),
            rx,
            unsubscribe,
        ))
    }

    fn get(&self, key: &str) -> Result<Vec<Reply>> {
        if !crate::core::valid_key_expr(key) {
            return Err(LocalZenohError::InvalidKey(key.into()).into());
        }
        // 1) 持锁**只收集**存储值与匹配的 queryable 处理器（不在锁内执行用户代码）。
        let (mut replies, handlers): (Vec<Reply>, Vec<(String, QueryHandler)>) = {
            let reg = self.lock();
            let mut sr = Vec::new();
            let mut hs = Vec::new();
            for (k, e) in reg.entries.iter().filter(|(k, _)| key_matches(key, k)) {
                if let Some(v) = &e.value {
                    sr.push(Reply::new(k.clone(), v.clone()));
                }
                for (_, h) in &e.queryables {
                    hs.push((k.clone(), h.clone()));
                }
            }
            (sr, hs)
        };
        // 2) **锁外**执行用户 handler（避免 handler 内回调本后端导致死锁），
        //    并隔离 handler 的失败/panic——单个计算/服务异常不应击穿"大脑"进程。
        for (k, h) in handlers {
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| h(&k)));
            match result {
                Ok(Ok(vals)) => {
                    for v in vals {
                        replies.push(Reply::new(k.clone(), v));
                    }
                }
                Ok(Err(e)) => {
                    log::warn!("[local-zenoh] queryable handler failed on {k:?}: {e}");
                }
                Err(_) => {
                    log::error!("[local-zenoh] queryable handler panicked on key {k:?}");
                }
            }
        }
        Ok(replies)
    }

    fn declare_queryable(&self, key: &str, handler: QueryHandler) -> Result<QueryableHandle> {
        if !crate::core::is_concrete_key(key) {
            return Err(LocalZenohError::InvalidKey(key.into()).into());
        }
        let mut reg = self.lock();
        let id = reg.next_queryable_id;
        reg.next_queryable_id += 1;
        let entry_key = key.to_string();
        let entry = reg.entries.entry(entry_key.clone()).or_default();
        entry.queryables.push((id, handler));

        // 注销闭包：Drop 时从条目移除该 queryable，腾空则清理条目。
        let inner = self.inner.clone();
        let closure_key = entry_key.clone();
        let unregister: Arc<dyn Fn() + Send + Sync> = Arc::new(move || {
            if let Ok(mut g) = inner.lock() {
                if let Some(e) = g.entries.get_mut(&closure_key) {
                    e.queryables.retain(|(i, _)| *i != id);
                }
                g.prune(&closure_key);
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
            Ok(vec![format!("computed:{key}").into_bytes()])
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
        let handler: QueryHandler = Arc::new(|_| Ok(vec![b"x".to_vec()]));
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

    #[test]
    fn put_rejects_invalid_key() {
        let z = LocalZenoh::new();
        assert!(z.put("a*b", b"x".to_vec()).is_err());
        assert!(z.put("a//b", b"x".to_vec()).is_err());
        assert!(z.put("a.b", b"x".to_vec()).is_err());
        assert!(z.put("a/*", b"x".to_vec()).is_err()); // 具体键不得含通配
    }

    #[test]
    fn subscribe_and_get_reject_invalid_expr() {
        let z = LocalZenoh::new();
        assert!(z.subscribe("a*b").is_err());
        assert!(z.subscribe("a//b").is_err());
        assert!(z.get("a*b").is_err());
        assert!(z.get("").is_err());
    }

    #[test]
    fn declare_queryable_rejects_wildcard_key() {
        let z = LocalZenoh::new();
        assert!(z
            .declare_queryable("svc/*", Arc::new(|_| Ok(vec![])))
            .is_err());
    }

    #[test]
    fn queryable_handler_reentrancy_no_deadlock() {
        // 修复：`get` 不得在持锁时调用用户 handler（否则 handler 内回调本后端会死锁）。
        let z = Arc::new(LocalZenoh::new());
        let z2 = z.clone();
        let handler: QueryHandler = Arc::new(move |key: &str| {
            // handler 内再发布/查询同一后端（重入）。
            let _ = z2.put("inner", b"v".to_vec());
            Ok(vec![format!("out:{key}").into_bytes()])
        });
        let _handle = z.declare_queryable("svc/re", handler).unwrap();
        // 若死锁，这里会永久阻塞（用线程 join 兜底：能正常返回即未死锁）。
        let t = std::thread::spawn(move || z.get("svc/re").unwrap());
        let replies = t.join().expect("handler should not deadlock");
        assert_eq!(replies.len(), 1);
        assert_eq!(replies[0].value, b"out:svc/re");
    }

    #[test]
    fn double_star_aggregates_nested_stores() {
        let z = LocalZenoh::new();
        z.put("sensor/nav/gps", b"g".to_vec()).unwrap();
        z.put("sensor/imu", b"i".to_vec()).unwrap();
        z.put("other/x", b"x".to_vec()).unwrap();
        let replies = z.get("sensor/**").unwrap();
        assert_eq!(replies.len(), 2);
    }

    #[test]
    fn unsubscribe_stops_delivery_and_is_counted() {
        let z = LocalZenoh::new();
        let sub = z.subscribe("sensor/*").unwrap();
        assert_eq!(z.subscription_count(), 1);
        sub.unsubscribe();
        // 主动退订后订阅者计数归零。
        assert_eq!(z.subscription_count(), 0);
        // 退订后发布不再投递。
        z.put("sensor/temp", b"v".to_vec()).unwrap();
        assert!(sub.try_recv().is_err());
    }

    #[test]
    fn store_count_only_counts_stored_values() {
        let z = LocalZenoh::new();
        // 订阅本身产生一个条目（无值），不应计入 store_count。
        let _sub = z.subscribe("a/b").unwrap();
        assert_eq!(z.store_count(), 0);
        z.put("a/b", b"1".to_vec()).unwrap();
        assert_eq!(z.store_count(), 1);
        assert!(z.contains("a/b"));
        assert_eq!(z.value("a/b"), Some(b"1".to_vec()));
    }

    #[test]
    fn remove_and_clear_manage_storage() {
        let z = LocalZenoh::new();
        z.put("k/1", b"a".to_vec()).unwrap();
        z.put("k/2", b"b".to_vec()).unwrap();
        assert_eq!(z.remove("k/1").unwrap(), Some(b"a".to_vec()));
        assert!(!z.contains("k/1"));
        assert_eq!(z.store_count(), 1);
        assert_eq!(z.clear(), 1);
        assert_eq!(z.store_count(), 0);
    }

    #[test]
    fn clone_shares_registry_state() {
        // `LocalZenoh` 可克隆，克隆体共享同一份存储/订阅。
        let z = LocalZenoh::new();
        z.put("a/b", b"1".to_vec()).unwrap();
        let z2 = z.clone();
        let sub = z2.subscribe("a/*").unwrap();
        // 订阅即收到保留值（来自 a/b）。
        let s1 = sub
            .recv_timeout(std::time::Duration::from_millis(100))
            .unwrap();
        assert_eq!(s1.value, b"1");
        // 通过克隆体发布 → 与后端共享存储与投递。
        z.put("a/c", b"2".to_vec()).unwrap();
        let s2 = sub
            .recv_timeout(std::time::Duration::from_millis(100))
            .unwrap();
        assert_eq!(s2.value, b"2");
        assert_eq!(z2.value("a/c"), Some(b"2".to_vec()));
    }

    #[test]
    fn panicking_handler_is_isolated() {
        // 一个 handler panic 不应击穿 get 调用；其余正常 handler 的应答保留。
        let z = LocalZenoh::new();
        let panicker: QueryHandler = Arc::new(|_| panic!("compute exploded"));
        let _h1 = z.declare_queryable("svc/boom", panicker).unwrap();
        let ok_handler: QueryHandler = Arc::new(|k| Ok(vec![format!("ok:{k}").into_bytes()]));
        let _h2 = z.declare_queryable("svc/ok", ok_handler).unwrap();

        let replies = z.get("svc/*").unwrap();
        assert_eq!(replies.len(), 1);
        assert_eq!(replies[0].value, b"ok:svc/ok");
    }

    #[test]
    fn failing_handler_is_skipped() {
        // handler 返回 Err（计算失败）同样被记录并跳过，不崩溃、不污染应答。
        let z = LocalZenoh::new();
        let bad: QueryHandler =
            Arc::new(|_| Err(brain_core::BrainError::Bus("compute error".into())));
        let _h1 = z.declare_queryable("svc/bad", bad).unwrap();
        let ok: QueryHandler = Arc::new(|_| Ok(vec![b"fine".to_vec()]));
        let _h2 = z.declare_queryable("svc/good", ok).unwrap();

        let replies = z.get("svc/*").unwrap();
        assert_eq!(replies.len(), 1);
        assert_eq!(replies[0].value, b"fine");
    }

    #[test]
    fn debug_reports_live_counts() {
        // `Debug` 应反映当前存储/订阅/计算计数（生产排障用）。
        let z = LocalZenoh::new();
        z.put("a/b", b"1".to_vec()).unwrap();
        let _sub = z.subscribe("a/*").unwrap();
        let _h = z
            .declare_queryable("svc/x", Arc::new(|_| Ok(vec![])))
            .unwrap();
        let s = format!("{z:?}");
        assert!(s.contains("store_count: 1"), "{s}");
        assert!(s.contains("subscription_count: 1"), "{s}");
        assert!(s.contains("queryable_count: 1"), "{s}");
    }
}
