# brain-mission

> Smart-Brain application layer: mission planning and swarm link

**所属层**：第 5 层 · 任务与应用层 —— 航线自主规划 + 蜂群协同通信。

## 职责

提供航点任务模型、任务执行器（把航点转成巡航指令），以及面向蜂群的广播链路与协同调度。

- `mission`：`Mission` / `Waypoint` / `MissionExecutor` / `MissionPhase` / `MissionProgress`（任务加载、执行与进度上报）
- `swarm`：`SwarmLink` / `SwarmRole` / `SwarmShare`（蜂群广播链路）
- `swarm_coord`：`SwarmCoordinator` / `LeaderElection` / `TaskAllocator` / `SwarmTask`（Leader 选举 + 全网确定性任务分配）

## 核心 API

```rust
pub use mission::{Mission, MissionExecutor, MissionPhase, MissionProgress, Waypoint};
pub use swarm::{SwarmLink, SwarmRole, SwarmShare};
pub use swarm_coord::{LeaderElection, SwarmCoordinator, SwarmTask, TaskAllocator};
```

## 用法

```rust
use brain_mission::{Mission, MissionExecutor, Waypoint};

fn main() {
    let mission = Mission::new(
        "survey-01",
        vec![
            Waypoint { sequence: 0, north: 0.0, east: 0.0, alt: 30.0, accept_radius: 2.0 },
            Waypoint { sequence: 1, north: 100.0, east: 0.0, alt: 30.0, accept_radius: 2.0 },
        ],
    );
    let mut exec = MissionExecutor::new(mission).unwrap();
    exec.start();
    while let Some(target) = exec.current_target() {
        println!("send {target:?}");
        exec.advance();
    }
}
```

## 依赖

- 外部：`log`、`serde`、`serde_json`
- 内部：`brain-core`、`brain-message`、`brain-middleware`

> **应用案例**：`brain-node/comprehensive_demo.rs`（任务加载 → 执行+进度上报 → 蜂群协同 → MAVLink 命令下发）、`swarm_coord_demo.rs`（Leader 选举 + 任务分配）。
