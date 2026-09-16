//! 真实 `zenoh` crate 适配（`real-zenoh` feature）。
//!
//! 把本项目的统一 `CommBackend` 桥接到 **Eclipse Zenoh 1.x**（异步，tokio）。
//! 启用方法：`cargo build --features real-zenoh`（默认关闭，保持构建轻量）。
//!
//! 部署时即可获得 Zenoh 全部能力：对等/树状/云混合拓扑、断网本地缓存后全网
//! 透明查询、无线多 AP / 4G 切换无缝会话迁移、零拷贝传输。机载端（大脑）用
//! 本适配器；单片机（小脑）可经 **zenoh-pico**（纯 C）接入同一键空间互通。

use std::any::Any;
use std::sync::mpsc::channel;
use std::sync::{Arc, Mutex};

use brain_core::{BrainError, Result};
use zenoh::key_expr::KeyExpr;

use crate::backend::{CommBackend, QueryHandler, QueryableHandle, Subscription};
use crate::core::{Reply, Sample, Value};

/// 持有已声明 `queryable` 的容器，保持其存活以对抗 GC。
///
/// 注销时把槽位置 `None`（该槽位的 `Queryable` 被 Drop → 自动 undeclare）。
/// 用类型别名避免 `clippy::type_complexity`，并让意图一目了然。
type QueryableStorage = Arc<Mutex<Vec<Option<Box<dyn Any + Send>>>>>;

/// 基于真实 Zenoh 会话的后端。
pub struct ZenohBackend {
    rt: tokio::runtime::Runtime,
    session: zenoh::Session,
    // 保持 queryable 存活；注销时置 None（Drop 即 undeclare）。
    queryables: QueryableStorage,
}

impl ZenohBackend {
    /// 用给定配置打开一个 Zenoh 会话（peer/router/client 由配置决定）。
    pub fn open(config: zenoh::Config) -> Result<Self> {
        let rt = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .map_err(|e| BrainError::Bus(e.to_string()))?;
        let session = rt
            .block_on(async { zenoh::open(config).await })
            .map_err(|e| BrainError::Bus(format!("open zenoh: {e}")))?;
        Ok(Self {
            rt,
            session,
            queryables: Arc::new(Mutex::new(Vec::new())),
        })
    }

    /// 以默认配置打开（peer 模式）。
    pub fn open_default() -> Result<Self> {
        Self::open(zenoh::Config::default())
    }
}

impl CommBackend for ZenohBackend {
    fn put(&self, key: &str, value: Value) -> Result<()> {
        let keyexpr =
            KeyExpr::try_from(key).map_err(|e| BrainError::Bus(format!("bad keyexpr: {e}")))?;
        let session = self.session.clone();
        self.rt
            .block_on(async move { session.put(keyexpr, value).await })
            .map_err(|e| BrainError::Bus(e.to_string()))?;
        Ok(())
    }

    fn subscribe(&self, key: &str) -> Result<Subscription> {
        let keyexpr =
            KeyExpr::try_from(key).map_err(|e| BrainError::Bus(format!("bad keyexpr: {e}")))?;
        let session = self.session.clone();
        let subscriber = self
            .rt
            .block_on(async move { session.declare_subscriber(keyexpr).await })
            .map_err(|e| BrainError::Bus(e.to_string()))?;

        let (tx, rx) = channel::<Sample>();
        let rt = self.rt.handle().clone();
        std::thread::spawn(move || {
            rt.block_on(async move {
                while let Ok(sample) = subscriber.recv_async().await {
                    let payload = sample
                        .payload()
                        .try_to_string()
                        .unwrap_or_default()
                        .into_owned()
                        .into_bytes();
                    let ts = sample
                        .timestamp()
                        .map(|t| t.get_time().as_nanos())
                        .unwrap_or(0);
                    let s = Sample::new(sample.key_expr().to_string(), payload, ts);
                    if tx.send(s).is_err() {
                        break;
                    }
                }
            });
        });
        Ok(Subscription::new(key.to_string(), rx))
    }

    fn get(&self, key: &str) -> Result<Vec<Reply>> {
        let keyexpr =
            KeyExpr::try_from(key).map_err(|e| BrainError::Bus(format!("bad keyexpr: {e}")))?;
        let session = self.session.clone();
        let replies = self
            .rt
            .block_on(async move { session.get(keyexpr).await })
            .map_err(|e| BrainError::Bus(e.to_string()))?;
        let mut out = Vec::new();
        self.rt.block_on(async {
            while let Ok(reply) = replies.recv_async().await {
                match reply.result() {
                    Ok(sample) => out.push(Reply::new(
                        sample.key_expr().to_string(),
                        sample
                            .payload()
                            .try_to_string()
                            .unwrap_or_default()
                            .into_owned()
                            .into_bytes(),
                    )),
                    Err(e) => log::warn!("zenoh get error: {e}"),
                }
            }
        });
        Ok(out)
    }

    fn declare_queryable(&self, key: &str, handler: QueryHandler) -> Result<QueryableHandle> {
        let keyexpr =
            KeyExpr::try_from(key).map_err(|e| BrainError::Bus(format!("bad keyexpr: {e}")))?;
        let session = self.session.clone();
        let key = key.to_string();
        let handler = handler.clone();
        let rt = self.rt.handle().clone();
        let key = key.to_string();
        let cb_key = key.clone();
        let handler = handler.clone();
        // Zenoh 的 queryable 回调是同步的；把“计算 + 应答”spawn 到运行时。
        let cb = move |query: zenoh::query::Query| {
            let values = match handler(&cb_key) {
                Ok(v) => v,
                Err(e) => {
                    log::warn!("[zenoh] queryable {cb_key:?} error: {e}");
                    Vec::new()
                }
            };
            let key = cb_key.clone();
            let rt = rt.clone();
            rt.spawn(async move {
                for v in values {
                    let _ = query.reply(key.clone(), v).await;
                }
            });
        };
        let queryable = self
            .rt
            .block_on(async move { session.declare_queryable(keyexpr).callback(cb).await })
            .map_err(|e| BrainError::Bus(e.to_string()))?;

        let mut qs = self.queryables.lock().unwrap_or_else(|p| p.into_inner());
        let idx = qs.len();
        qs.push(Some(Box::new(queryable) as Box<dyn Any + Send>));
        drop(qs);

        let qstore = self.queryables.clone();
        let unregister: Arc<dyn Fn() + Send + Sync> = Arc::new(move || {
            if let Ok(mut qs) = qstore.lock() {
                if let Some(slot) = qs.get_mut(idx) {
                    *slot = None; // Drop Queryable → undeclare
                }
            }
        });
        Ok(QueryableHandle::new(key, unregister))
    }
}

/// 真实 Zenoh 网络测试（`real-zenoh` 特性 + 两个 peer 会话）。
///
/// 默认 `#[ignore]`（需要真实网络/本机组播，非确定性）。运行方式：
/// `cargo test -p brain-zenoh --features real-zenoh -- --ignored --nocapture`
#[cfg(all(test, feature = "real-zenoh"))]
mod net_tests {
    use tokio::time::{sleep, Duration};
    use zenoh::key_expr::KeyExpr;

    #[tokio::test(flavor = "multi_thread")]
    #[ignore = "requires a real zenoh peer-to-peer session over the local network"]
    async fn peer_to_peer_three_pillars() {
        // 两个 peer 会话，经组播发现彼此（同一主机）。
        let s1 = zenoh::open(zenoh::Config::default())
            .await
            .expect("open s1");
        let s2 = zenoh::open(zenoh::Config::default())
            .await
            .expect("open s2");
        sleep(Duration::from_millis(800)).await; // 等待 peer 发现

        // 1) Compute：s1 声明一个可查询服务（同步回调，spawn 应答）。
        let handle = tokio::runtime::Handle::current();
        let _q = s1
            .declare_queryable(KeyExpr::try_from("svc/distance").unwrap())
            .callback(move |query: zenoh::query::Query| {
                let handle = handle.clone();
                handle.spawn(async move {
                    let _ = query.reply("svc/distance", "{\"d\":1.2}").await;
                });
            })
            .await
            .expect("queryable");

        // 2) Pub/Sub：s1 订阅遥测。
        let sub = s1
            .declare_subscriber(KeyExpr::try_from("data/temp").unwrap())
            .await
            .expect("subscriber");

        sleep(Duration::from_millis(300)).await;

        // 3) Store/Query：s2 查询 s1 上的服务（分布式调用）。
        let replies = s2
            .get(KeyExpr::try_from("svc/distance").unwrap())
            .await
            .expect("get");
        let mut got_reply = false;
        while let Ok(r) = replies.recv_async().await {
            if r.result().is_ok() {
                got_reply = true;
            }
        }
        assert!(got_reply, "s2 should receive the queryable reply from s1");

        // 4) Pub/Sub：s2 发布遥测，s1 收到。
        s2.put(KeyExpr::try_from("data/temp").unwrap(), "21.5")
            .await
            .expect("put");
        let sample = tokio::time::timeout(Duration::from_secs(2), sub.recv_async())
            .await
            .expect("timeout waiting for pub/sub")
            .expect("recv");
        assert_eq!(sample.key_expr().to_string(), "data/temp");
    }
}
