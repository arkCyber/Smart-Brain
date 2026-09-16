//! 类型安全的数据总线与话题。

use std::any::Any;
use std::collections::HashMap;
use std::fmt;
use std::sync::mpsc;
use std::sync::{Arc, Mutex};

use brain_core::error::Result;
use brain_core::time::Timestamp;

/// 一条总线消息：携带值与发布时间戳。
///
/// 推送给订阅者时附带时间戳，便于对端做时序/延迟判断。
#[derive(Debug, Clone, PartialEq)]
pub struct BusMessage<T> {
    /// 消息负载。
    pub value: T,
    /// 发布该消息时的时间戳。
    pub timestamp: Timestamp,
}

impl<T> BusMessage<T> {
    /// 构造一条总线消息。
    pub fn new(value: T, timestamp: Timestamp) -> Self {
        Self { value, timestamp }
    }
}

/// 话题内部状态：最新值、更新时间戳与订阅者列表。
///
/// 全部放在同一把锁下，保证“最新值 + 时间戳 + 订阅分发”的读写原子性：
/// 读者不可能看到新数据配旧时间戳的中间态。
struct TopicInner<T> {
    /// 最近一次发布的消息（保留最近值）。
    latest: Option<T>,
    /// 最近一次发布的时间戳。
    updated: Timestamp,
    /// 推送订阅者；已断开（Receiver 被 Drop）的在下次发布时被回收。
    subscribers: Vec<mpsc::Sender<BusMessage<T>>>,
}

/// 一个命名话题。
///
/// 每个话题保存一条“最新消息”（保留最近值），并支持两种消费方式：
/// - **拉取**：通过 [`Topic::peek`] 读取最近值；
/// - **推送订阅**：通过 [`Topic::subscribe`] 拿到接收端，发布即收到（含时间戳）。
///
/// `Arc<Mutex<...>>` 使得话题可在多个线程/循环间共享。
pub struct Topic<T: Clone + Send + 'static> {
    name: String,
    inner: Mutex<TopicInner<T>>,
}

impl<T: Clone + Send + 'static> Topic<T> {
    fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            inner: Mutex::new(TopicInner {
                latest: None,
                updated: 0,
                subscribers: Vec::new(),
            }),
        }
    }

    /// 发布一条消息：更新“最近值”，并向所有活跃订阅者推送。
    ///
    /// 发送给已断开订阅者的通道会自动被回收（`retain` 修剪）。
    pub fn publish(&self, msg: T, ts: Timestamp) {
        let mut inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        inner.latest = Some(msg.clone());
        inner.updated = ts;
        // 推送给所有订阅者；Receiver 已 Drop 的通道发送失败，随即被移除。
        inner
            .subscribers
            .retain(|tx| tx.send(BusMessage::new(msg.clone(), ts)).is_ok());
        log::trace!("topic[{}] published at ts {ts}", self.name);
    }

    /// 读取最近一条消息；若从未发布则返回 `None`。
    pub fn peek(&self) -> Option<T> {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .latest
            .clone()
    }

    /// 最近一次更新的时间戳。
    pub fn last_updated(&self) -> Timestamp {
        self.inner.lock().unwrap_or_else(|p| p.into_inner()).updated
    }

    /// 在**同一次加锁**内读取“最近消息 + 时间戳”，返回原子一致的对（若已发布过）。
    ///
    /// 与分两步调 [`Topic::peek`] + [`Topic::last_updated`] 不同，这里保证两者
    /// 来自同一次发布，避免并发发布导致的“新值配旧时间戳”错配。
    pub fn peek_message(&self) -> Option<BusMessage<T>> {
        let inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        inner
            .latest
            .clone()
            .map(|value| BusMessage::new(value, inner.updated))
    }

    /// 话题名。
    pub fn name(&self) -> &str {
        &self.name
    }

    /// 订阅本话题：返回一个接收端（`mpsc::Receiver`），发布时推送。
    ///
    /// 采用“保留最近值”语义：新订阅者会**立即**收到当前值（若已发布过），
    /// 与 Zenoh 订阅即取当前值的行为一致。Receiver 被 `Drop` 时，该订阅在下次
    /// 发布时被自动回收。
    ///
    /// 注意：通道为**无界**队列。发布在话题锁内非阻塞发送，故慢消费者若不及时
    /// `recv` 会累积消息、占用内存——高频话题下请保证消费足够快，或用对慢消费
    /// 更宽容的 [`Topic::peek`]/[`Topic::peek_message`] 拉取最新值。
    pub fn subscribe(&self) -> mpsc::Receiver<BusMessage<T>> {
        let mut inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        let (tx, rx) = mpsc::channel();
        // 保留最近值：订阅即收到当前最新值。
        if let Some(latest) = &inner.latest {
            let _ = tx.send(BusMessage::new(latest.clone(), inner.updated));
        }
        inner.subscribers.push(tx);
        rx
    }

    /// 当前活跃（未被回收）的订阅者数量。
    pub fn subscriber_count(&self) -> usize {
        self.inner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .subscribers
            .len()
    }
}

impl<T: Clone + Send + 'static> fmt::Debug for Topic<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let inner = self.inner.lock().unwrap_or_else(|p| p.into_inner());
        f.debug_struct("Topic")
            .field("name", &self.name)
            .field("has_latest", &inner.latest.is_some())
            .field("updated", &inner.updated)
            .field("subscribers", &inner.subscribers.len())
            .finish()
    }
}

/// 数据总线：管理一组命名话题。
///
/// 由于 Rust 没有异构容器，采用 `Any` 统一存放不同消息类型的话题。
/// 话题以 `Arc<Topic<T>>` 形式存放在 `Box<dyn Any>` 中，这样 `Any` 的具体
/// 类型就是 `Arc<Topic<T>>`（而非其内部 `Topic<T>`），可按类型安全取出并共享。
#[derive(Default)]
pub struct DataBus {
    topics: Mutex<HashMap<String, Box<dyn Any + Send + Sync>>>,
}

impl DataBus {
    /// 创建空数据总线。
    pub fn new() -> Self {
        Self::default()
    }

    /// 注册（或获取）一个 `Topic<T>`。重复注册同名话题返回同一个实例。
    pub fn register<T: Clone + Send + 'static>(&self, name: impl Into<String>) -> Arc<Topic<T>> {
        let name = name.into();
        let mut map = self.topics.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(existing) = map.get(&name) {
            if let Some(t) = existing.downcast_ref::<Arc<Topic<T>>>() {
                return t.clone();
            }
            log::warn!("topic[{}] type mismatch, replacing", name);
        }
        let topic: Arc<Topic<T>> = Arc::new(Topic::new(name.clone()));
        map.insert(name, Box::new(topic.clone()));
        topic
    }

    /// 获取已注册的 `Topic<T>`；若不存在或类型不匹配返回 `None`。
    pub fn topic<T: Clone + Send + 'static>(&self, name: &str) -> Option<Arc<Topic<T>>> {
        let map = self.topics.lock().unwrap_or_else(|p| p.into_inner());
        map.get(name)
            .and_then(|b| b.downcast_ref::<Arc<Topic<T>>>())
            .cloned()
    }

    /// 移除一个话题；仅当存在且类型匹配时才移除，返回是否移除成功。
    pub fn remove<T: Clone + Send + 'static>(&self, name: &str) -> bool {
        let mut map = self.topics.lock().unwrap_or_else(|p| p.into_inner());
        match map.get(name) {
            Some(b) if b.downcast_ref::<Arc<Topic<T>>>().is_some() => {
                map.remove(name);
                true
            }
            _ => false,
        }
    }

    /// 已注册的话题数量（用于监控）。
    pub fn len(&self) -> usize {
        self.topics.lock().unwrap_or_else(|p| p.into_inner()).len()
    }

    /// 总线是否为空。
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// 便捷：发布消息到指定话题，自动注册。
    pub fn publish<T: Clone + Send + 'static>(
        &self,
        name: &str,
        msg: T,
        ts: Timestamp,
    ) -> Result<()> {
        let topic = self.register::<T>(name);
        topic.publish(msg, ts);
        Ok(())
    }

    /// 便捷：订阅指定话题（不存在则自动注册），返回推送接收端。
    ///
    /// 与 [`Topic::subscribe`] 一致：立即收到当前保留值（若有），发布即推送。
    pub fn subscribe<T: Clone + Send + 'static>(
        &self,
        name: &str,
    ) -> mpsc::Receiver<BusMessage<T>> {
        let topic = self.register::<T>(name);
        topic.subscribe()
    }
}

/// 预定义的总线话题名常量（对应各层数据流）。
pub mod topics {
    /// 飞控遥测（小脑 → 大脑）。
    pub const TELEMETRY: &str = "fcu/telemetry";
    /// 大脑心跳。
    pub const HEARTBEAT: &str = "brain/heartbeat";
    /// 感知层检测结果。
    pub const DETECTIONS: &str = "perception/detections";
    /// 感知层跟踪状态。
    pub const TRACKING: &str = "perception/tracking";
    /// 大脑下发指令。
    pub const COMMAND: &str = "brain/command";
    /// 飞行状态机状态。
    pub const FLIGHT_STATE: &str = "state/flight";
    /// 当前任务航点。
    pub const ACTIVE_WAYPOINT: &str = "mission/active_waypoint";
    /// 蜂群共享态势。
    pub const SWARM_SHARE: &str = "swarm/share";

    // ---- 通用（身体无关）传感器与状态话题 ----
    /// 通用 IMU 样本。
    pub const SENSOR_IMU: &str = "sensor/imu";
    /// 通用里程计样本。
    pub const SENSOR_ODOMETRY: &str = "sensor/odometry";
    /// 通用测距扫描（激光/超声波）。
    pub const SENSOR_RANGE: &str = "sensor/range";
    /// 通用接触力样本。
    pub const SENSOR_CONTACT: &str = "sensor/contact";
    /// 通用机器人状态机状态。
    pub const ROBOT_STATE: &str = "state/robot";
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn topic_publish_and_peek() {
        let bus = DataBus::new();
        bus.publish::<u32>(topics::HEARTBEAT, 42, 1).unwrap();
        let t: Arc<Topic<u32>> = bus.topic(topics::HEARTBEAT).unwrap();
        assert_eq!(t.peek(), Some(42));
        assert_eq!(t.last_updated(), 1);
    }

    #[test]
    fn type_mismatch_returns_none() {
        let bus = DataBus::new();
        bus.register::<u32>("x");
        assert!(bus.topic::<String>("x").is_none());
    }

    #[test]
    fn register_same_name_returns_same_topic() {
        let bus = DataBus::new();
        let a: Arc<Topic<u64>> = bus.register("t");
        let b: Arc<Topic<u64>> = bus.register("t");
        assert!(Arc::ptr_eq(&a, &b));
        a.publish(7, 0);
        assert_eq!(b.peek(), Some(7));
    }

    #[test]
    fn multiple_topics_independent() {
        let bus = DataBus::new();
        bus.publish::<u8>(topics::HEARTBEAT, 1, 0).unwrap();
        bus.publish::<String>(topics::COMMAND, "go".into(), 0)
            .unwrap();
        assert_eq!(bus.len(), 2);
        assert_eq!(bus.topic::<u8>(topics::HEARTBEAT).unwrap().peek(), Some(1));
        assert_eq!(
            bus.topic::<String>(topics::COMMAND)
                .unwrap()
                .peek()
                .as_deref(),
            Some("go")
        );
    }

    #[test]
    fn is_empty_and_len() {
        let bus = DataBus::new();
        assert!(bus.is_empty());
        bus.publish::<u32>(topics::TELEMETRY, 0, 0).unwrap();
        assert_eq!(bus.len(), 1);
        assert!(!bus.is_empty());
    }

    #[test]
    fn sensor_topics_carry_typed_samples() {
        use brain_core::Vec3;
        use brain_message::sensor::ImuSample;
        let bus = DataBus::new();
        let sample = ImuSample::new(7, Vec3::new(0.0, 0.0, -9.81), Vec3::ZERO);
        bus.publish(topics::SENSOR_IMU, sample.clone(), 7).unwrap();
        let t: Arc<Topic<ImuSample>> = bus.topic(topics::SENSOR_IMU).unwrap();
        assert_eq!(t.peek(), Some(sample));
        assert_eq!(t.last_updated(), 7);
    }

    #[test]
    fn peek_message_returns_atomic_value_and_ts() {
        let bus = DataBus::new();
        // 未发布 -> None。
        let t: Arc<Topic<u32>> = bus.register("atomic");
        assert!(t.peek_message().is_none());
        // 发布后同一次读取返回一致的 (值, 时间戳)。
        t.publish(7, 42);
        let m = t.peek_message().expect("published");
        assert_eq!(m.value, 7);
        assert_eq!(m.timestamp, 42);
        // 再次发布，时间戳与值同步更新。
        t.publish(8, 43);
        let m = t.peek_message().unwrap();
        assert_eq!((m.value, m.timestamp), (8, 43));
    }

    #[test]
    fn subscribe_receives_retained_and_pushed() {
        let bus = DataBus::new();
        bus.publish::<u32>(topics::HEARTBEAT, 1, 10).unwrap();
        let rx: std::sync::mpsc::Receiver<BusMessage<u32>> = bus.subscribe(topics::HEARTBEAT);
        // 保留最近值：订阅即收到当前值。
        let first = rx.try_recv().unwrap();
        assert_eq!(first.value, 1);
        assert_eq!(first.timestamp, 10);
        // 后续发布即时推送。
        bus.publish::<u32>(topics::HEARTBEAT, 2, 11).unwrap();
        let second = rx.try_recv().unwrap();
        assert_eq!(second.value, 2);
        assert_eq!(second.timestamp, 11);
    }

    #[test]
    fn dropped_subscription_is_pruned_on_next_publish() {
        let topic = Arc::new(Topic::<u32>::new("t"));
        // 建立两个订阅后丢弃一个，再发布时活跃订阅应被修剪。
        let rx1 = topic.subscribe();
        let rx2 = topic.subscribe();
        topic.publish(1, 0);
        assert_eq!(topic.subscriber_count(), 2);
        // rx2 按序收到 1。
        assert_eq!(rx2.try_recv().unwrap().value, 1);
        drop(rx1);
        topic.publish(2, 1); // 发布时触发 retain，回收已断开者
        assert_eq!(topic.subscriber_count(), 1);
        // rx2 仍能收到后续推送的 2。
        assert_eq!(rx2.try_recv().unwrap().value, 2);
    }

    #[test]
    fn remove_topic_returns_success_and_frees_slot() {
        let bus = DataBus::new();
        bus.publish::<u32>("temp", 5, 0).unwrap();
        assert!(bus.remove::<u32>("temp"));
        assert!(bus.topic::<u32>("temp").is_none());
        // 已移除，再次移除失败。
        assert!(!bus.remove::<u32>("temp"));
        // 类型不匹配时不移除。
        bus.publish::<u32>("k", 1, 0).unwrap();
        assert!(!bus.remove::<String>("k"));
        assert!(bus.topic::<u32>("k").is_some());
    }

    #[test]
    fn topic_debug_is_available() {
        let topic = Arc::new(Topic::<u32>::new("dbg"));
        let s = format!("{topic:?}");
        assert!(s.contains("Topic"));
        assert!(s.contains("dbg"));
        topic.publish(3, 7);
        assert!(format!("{topic:?}").contains("updated: 7"));
    }
}
