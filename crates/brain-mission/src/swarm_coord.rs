//! 蜂群协同决策：Leader 选举与任务分配。
//!
//! 在 [`super::swarm::SwarmLink`]（态势共享）之上，提供确定性的协同逻辑：
//! - **Leader 选举**：基于全网可见的态势（node_id、电量）选出唯一 leader。
//!   由于所有成员基于同一份 peer 共享做**确定性**排序，因此无需中心协调即可收敛到
//!   一致的 leader（对应参考架构“蜂群协同”阶段）。
//! - **任务分配**：把一批任务（航点/区域）分发给各 agent（轮询 / 按优先级）。
//!
//! 纯逻辑、无外部依赖，可单测；经 SwarmLink 的 JSON 接口即可在多机间传递。

use serde::{Deserialize, Serialize};

use super::swarm::SwarmShare;

/// 一个可分配的蜂群任务（例如一片待巡检区域 / 一串航点）。
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SwarmTask {
    /// 任务唯一标识。
    pub id: String,
    /// 优先级（越大越优先）。
    pub priority: u32,
}

impl SwarmTask {
    pub fn new(id: impl Into<String>, priority: u32) -> Self {
        Self {
            id: id.into(),
            priority,
        }
    }
}

/// Leader 选举：`lowest node_id` 胜出（全网确定性一致）。
///
/// 所有成员把自己与 peer 的 `node_id` 一起按字典序排序取最小者，即可得到
/// 唯一且一致的 leader，无需中央协调。
pub struct LeaderElection {
    node_id: String,
}

impl LeaderElection {
    pub fn new(node_id: impl Into<String>) -> Self {
        Self {
            node_id: node_id.into(),
        }
    }

    /// 综合本机与 peer 态势，选出 leader 的 node_id（字典序最小者胜出）。
    pub fn elect(&self, peers: &[SwarmShare]) -> String {
        let mut ids: Vec<&str> = peers.iter().map(|p| p.node_id.as_str()).collect();
        ids.push(&self.node_id);
        ids.sort_unstable();
        ids[0].to_string()
    }

    /// 本机是否当选 leader。
    pub fn is_leader(&self, peers: &[SwarmShare]) -> bool {
        self.elect(peers) == self.node_id
    }

    /// 按“电量最高者胜出”的替代策略（更适合续航导向的选举）。
    ///
    /// `self_battery` 为本机剩余电量；在平局时保持本机为 leader（避免频繁切换）。
    pub fn elect_by_battery(&self, self_battery: f32, peers: &[SwarmShare]) -> String {
        let mut best_id = self.node_id.clone();
        let mut best = self_battery;
        for p in peers {
            if p.battery_pct > best {
                best = p.battery_pct;
                best_id = p.node_id.clone();
            }
        }
        best_id
    }
}

/// 任务分配器。
pub struct TaskAllocator;

impl TaskAllocator {
    /// 轮询分配：把任务依序循环分给各 agent。返回 `(agent, 其任务列表)`，含空列表。
    pub fn round_robin(tasks: &[SwarmTask], agents: &[String]) -> Vec<(String, Vec<SwarmTask>)> {
        let mut out: Vec<(String, Vec<SwarmTask>)> =
            agents.iter().map(|a| (a.clone(), Vec::new())).collect();
        for (i, t) in tasks.iter().enumerate() {
            out[i % agents.len()].1.push(t.clone());
        }
        out
    }

    /// 按优先级分配：先按优先级降序排序，再轮询分给各 agent。
    pub fn by_priority(tasks: &[SwarmTask], agents: &[String]) -> Vec<(String, Vec<SwarmTask>)> {
        let mut sorted = tasks.to_vec();
        sorted.sort_by_key(|t| std::cmp::Reverse(t.priority));
        Self::round_robin(&sorted, agents)
    }

    /// 取某 agent 应执行的任务（按优先级降序）。
    pub fn tasks_for<'a>(
        assignments: &'a [(String, Vec<SwarmTask>)],
        agent: &str,
    ) -> &'a [SwarmTask] {
        assignments
            .iter()
            .find(|(a, _)| a == agent)
            .map(|(_, t)| t.as_slice())
            .unwrap_or(&[])
    }
}

/// 蜂群协调器：持有本机身份 + Leader 选举，产出任务分配。
pub struct SwarmCoordinator {
    election: LeaderElection,
}

impl SwarmCoordinator {
    pub fn new(node_id: impl Into<String>) -> Self {
        Self {
            election: LeaderElection::new(node_id),
        }
    }

    /// 本机是否当选 leader。
    pub fn is_leader(&self, peers: &[SwarmShare]) -> bool {
        self.election.is_leader(peers)
    }

    /// 若本机是 leader，则把任务分发给全体成员（含自己），返回分配表。
    pub fn plan(
        &self,
        peers: &[SwarmShare],
        tasks: &[SwarmTask],
    ) -> Option<Vec<(String, Vec<SwarmTask>)>> {
        if !self.is_leader(peers) {
            return None;
        }
        // 全体成员 = 本机 + peers。
        let mut agents = vec![self.election.node_id.clone()];
        for p in peers {
            if !agents.contains(&p.node_id) {
                agents.push(p.node_id.clone());
            }
        }
        Some(TaskAllocator::round_robin(tasks, &agents))
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn peer(id: &str, battery: f32) -> SwarmShare {
        SwarmShare::new(id, 0, Default::default(), 0.0, false, battery)
    }

    #[test]
    fn elects_lowest_node_id() {
        let e = LeaderElection::new("node-b");
        // 无 peer：本机当选。
        assert_eq!(e.elect(&[]), "node-b");
        assert!(e.is_leader(&[]));
        // 有更小 node_id 的 peer：对方当选。
        let peers = vec![peer("node-a", 80.0)];
        assert_eq!(e.elect(&peers), "node-a");
        assert!(!e.is_leader(&peers));
    }

    #[test]
    fn election_is_deterministic_and_consistent() {
        // 任意成员在相同态势下选出的 leader 一致。
        let shares = vec![peer("n2", 50.0), peer("n1", 90.0), peer("n3", 60.0)];
        let e1 = LeaderElection::new("n2");
        let e2 = LeaderElection::new("n3");
        assert_eq!(e1.elect(&shares), e2.elect(&shares));
        assert_eq!(e1.elect(&shares), "n1"); // 字典序最小
    }

    #[test]
    fn elect_by_battery_prefers_highest() {
        let e = LeaderElection::new("self");
        let peers = vec![peer("other", 95.0)];
        assert_eq!(e.elect_by_battery(90.0, &peers), "other");
        // 平局时保持本机。
        let peers2 = vec![peer("other", 90.0)];
        assert_eq!(e.elect_by_battery(90.0, &peers2), "self");
    }

    #[test]
    fn round_robin_distributes_balanced() {
        let tasks: Vec<SwarmTask> = (0..5).map(|i| SwarmTask::new(format!("t{i}"), 1)).collect();
        let agents = vec!["a".to_string(), "b".to_string()];
        let plan = TaskAllocator::round_robin(&tasks, &agents);
        assert_eq!(plan[0].1.len(), 3); // a: t0,t2,t4
        assert_eq!(plan[1].1.len(), 2); // b: t1,t3
        assert_eq!(TaskAllocator::tasks_for(&plan, "a")[0].id, "t0");
    }

    #[test]
    fn by_priority_sorts_first() {
        let tasks = vec![
            SwarmTask::new("low", 1),
            SwarmTask::new("high", 9),
            SwarmTask::new("mid", 5),
        ];
        let agents = vec!["a".to_string()];
        let plan = TaskAllocator::by_priority(&tasks, &agents);
        let got = TaskAllocator::tasks_for(&plan, "a");
        assert_eq!(got[0].id, "high");
        assert_eq!(got[1].id, "mid");
        assert_eq!(got[2].id, "low");
    }

    #[test]
    fn coordinator_only_plans_when_leader() {
        let tasks = vec![SwarmTask::new("t1", 1)];
        // 全体成员态势（node-a 和 node-b 都广播了）。
        let peers = vec![peer("node-a", 80.0), peer("node-b", 70.0)];
        // node-b 不是 leader：不产出分配。
        let coord_b = SwarmCoordinator::new("node-b");
        assert!(coord_b.plan(&peers, &tasks).is_none());
        // node-a 是 leader：产出分配，且包含全体成员。
        let coord_a = SwarmCoordinator::new("node-a");
        let plan = coord_a.plan(&peers, &tasks).unwrap();
        let agents: Vec<&str> = plan.iter().map(|(a, _)| a.as_str()).collect();
        assert!(agents.contains(&"node-a"));
        assert!(agents.contains(&"node-b"));
    }
}
