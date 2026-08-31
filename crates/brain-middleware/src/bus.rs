//! 类型安全的数据总线与话题。

use std::any::Any;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use brain_core::error::Result;
use brain_core::time::Timestamp;

/// 一个命名话题。
///
/// 每个话题保存一个“最新消息”（保留最近值），并允许订阅者读取。
/// `Arc<Mutex<...>>` 使得话题可在多个线程/循环间共享。
pub struct Topic<T: Clone + Send + 'static> {
    name: String,
    /// 最近一次发布的消息。
    latest: Mutex<Option<T>>,
    /// 最近一次发布的时间戳。
    updated: Mutex<Timestamp>,
}

impl<T: Clone + Send + 'static> Topic<T> {
    fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            latest: Mutex::new(None),
            updated: Mutex::new(0),
        }
    }

    /// 发布一条消息（更新“最近值”）。
    pub fn publish(&self, msg: T, ts: Timestamp) {
        if let Ok(mut slot) = self.latest.lock() {
            *slot = Some(msg);
        }
        if let Ok(mut upd) = self.updated.lock() {
            *upd = ts;
        }
        log::trace!("topic[{}] published", self.name);
    }

    /// 读取最近一条消息；若从未发布则返回 `None`。
    pub fn peek(&self) -> Option<T> {
        self.latest.lock().ok().and_then(|g| g.clone())
    }

    /// 最近一次更新的时间戳。
    pub fn last_updated(&self) -> Timestamp {
        self.updated.lock().map(|g| *g).unwrap_or(0)
    }

    /// 话题名。
    pub fn name(&self) -> &str {
        &self.name
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
}
