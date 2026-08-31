//! 蜂群协同通信（Swarm Link）。

use brain_core::time::Timestamp;
use brain_message::telemetry::Vec3;
use brain_middleware::bus::topics;
use brain_middleware::DataBus;
use serde::{Deserialize, Serialize};

/// 蜂群中本机角色。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SwarmRole {
    /// 领航机。
    Leader,
    /// 跟随机。
    Follower,
    /// 侦察机。
    Scout,
}

/// 共享给蜂群的态势信息。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SwarmShare {
    pub node_id: String,
    pub timestamp: Timestamp,
    pub position: Vec3,
    pub heading: f32,
    /// 是否发现目标（供协同决策）。
    pub target_seen: bool,
    pub battery_pct: f32,
}

impl SwarmShare {
    /// 构造一条共享态势。
    pub fn new(
        node_id: &str,
        timestamp: Timestamp,
        position: Vec3,
        heading: f32,
        target_seen: bool,
        battery_pct: f32,
    ) -> Self {
        Self {
            node_id: node_id.to_string(),
            timestamp,
            position,
            heading,
            target_seen,
            battery_pct,
        }
    }
}

/// 蜂群链路：广播本机态势，并接收（读取）同伴共享态势。
///
/// 真实部署可基于 Zenoh / UDP 广播实现高并发低延迟通信；此处提供
/// 一个基于数据总线的仿真实现与一个 JSON 序列化接口，便于接入网络。
pub struct SwarmLink {
    node_id: String,
    role: SwarmRole,
    peer_shares: Vec<SwarmShare>,
}

impl SwarmLink {
    /// 创建蜂群链路。
    pub fn new(node_id: impl Into<String>, role: SwarmRole) -> Self {
        Self {
            node_id: node_id.into(),
            role,
            peer_shares: Vec::new(),
        }
    }

    /// 本机角色。
    pub fn role(&self) -> SwarmRole {
        self.role
    }

    /// 广播本机态势到数据总线（仿真/地面站可订阅）。
    pub fn broadcast(&self, share: &SwarmShare, bus: &DataBus, now: Timestamp) {
        let _ = bus.publish(topics::SWARM_SHARE, share.clone(), now);
        log::debug!("[swarm] {} broadcasting share", self.node_id);
    }

    /// 记录一帧来自同伴的共享态势。
    pub fn ingest(&mut self, share: SwarmShare) {
        self.peer_shares.push(share);
        // 保留最近若干帧，防止无界增长。
        if self.peer_shares.len() > 64 {
            let excess = self.peer_shares.len() - 64;
            self.peer_shares.drain(..excess);
        }
    }

    /// 最近收到的同伴态势。
    pub fn peers(&self) -> &[SwarmShare] {
        &self.peer_shares
    }

    /// 是否有任何同伴报告发现目标（用于协同确认）。
    pub fn any_peer_target_seen(&self) -> bool {
        self.peer_shares.iter().any(|s| s.target_seen)
    }

    /// 将共享态势序列化为 JSON（用于网络发送）。
    pub fn encode(share: &SwarmShare) -> Result<String, serde_json::Error> {
        serde_json::to_string(share)
    }

    /// 从 JSON 反序列化共享态势。
    pub fn decode(json: &str) -> Result<SwarmShare, serde_json::Error> {
        serde_json::from_str(json)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn share(node: &str, seen: bool) -> SwarmShare {
        SwarmShare::new(node, 123, Vec3::new(1.0, 2.0, -30.0), 0.5, seen, 80.0)
    }

    #[test]
    fn share_roundtrips_json() {
        let share = SwarmShare::new("a1", 123, Vec3::default(), 1.0, true, 80.0);
        let json = SwarmLink::encode(&share).unwrap();
        let back = SwarmLink::decode(&json).unwrap();
        assert_eq!(back.node_id, "a1");
        assert_eq!(back.timestamp, 123);
        assert!(back.target_seen);
        assert_eq!(back.battery_pct, 80.0);
        // 序列化形状稳定。
        let v: serde_json::Value = serde_json::from_str(&json).unwrap();
        assert_eq!(v["node_id"], "a1");
        assert_eq!(v["heading"], 1.0);
    }

    #[test]
    fn role_serde_roundtrip() {
        for r in [SwarmRole::Leader, SwarmRole::Follower, SwarmRole::Scout] {
            let j = serde_json::to_string(&r).unwrap();
            assert_eq!(serde_json::from_str::<SwarmRole>(&j).unwrap(), r);
        }
    }

    #[test]
    fn link_new_and_role() {
        let link = SwarmLink::new("n1", SwarmRole::Leader);
        assert_eq!(link.role(), SwarmRole::Leader);
        assert!(link.peers().is_empty());
        assert!(!link.any_peer_target_seen());
    }

    #[test]
    fn broadcast_publishes_to_bus() {
        let link = SwarmLink::new("n1", SwarmRole::Scout);
        let bus = DataBus::new();
        let s = share("n1", false);
        link.broadcast(&s, &bus, 9);
        let t: std::sync::Arc<brain_middleware::Topic<SwarmShare>> =
            bus.topic(topics::SWARM_SHARE).unwrap();
        let got = t.peek().unwrap();
        assert_eq!(got.node_id, "n1");
        assert_eq!(t.last_updated(), 9);
    }

    #[test]
    fn ingest_keeps_peers_and_caps_at_64() {
        let mut link = SwarmLink::new("n1", SwarmRole::Leader);
        // 超过 64 帧应裁剪到最近 64 帧。
        for i in 0..70 {
            link.ingest(share(&format!("peer{i}"), i % 3 == 0));
        }
        assert_eq!(link.peers().len(), 64);
        // 应保留最近的（peer6 之后的）。
        assert_eq!(link.peers().first().unwrap().node_id, "peer6");
        assert_eq!(link.peers().last().unwrap().node_id, "peer69");
    }

    #[test]
    fn any_peer_target_seen() {
        let mut link = SwarmLink::new("n1", SwarmRole::Leader);
        assert!(!link.any_peer_target_seen());
        link.ingest(share("a", false));
        link.ingest(share("b", true));
        assert!(link.any_peer_target_seen());

        let mut link2 = SwarmLink::new("n2", SwarmRole::Follower);
        link2.ingest(share("c", false));
        assert!(!link2.any_peer_target_seen());
    }

    #[test]
    fn ingest_preserves_order() {
        let mut link = SwarmLink::new("n1", SwarmRole::Follower);
        link.ingest(share("p1", false));
        link.ingest(share("p2", false));
        let ids: Vec<&str> = link.peers().iter().map(|s| s.node_id.as_str()).collect();
        assert_eq!(ids, vec!["p1", "p2"]);
    }
}
