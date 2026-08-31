# brain-behavior-tree

> Smart-Brain decision layer: behavior tree framework and flight nodes

**所属层**：第 3/4 层 · 决策层 —— 行为树框架 + 飞行节点。

## 职责

将飞行行为拆解为可组合节点：组合节点（`Sequence` / `Selector`）与装饰器（`Inverter` / `Retry`）负责逻辑，行为节点（起飞/巡航/发现目标/跟踪/返航/降落/fail-safe）负责具体动作。相比 if-else / FSM，行为树更易维护与复用。

- `core`：`Node` / `Status` / `Tree` / `BehaviorContext` / `BrainOutput`（行为树框架）
- `control`：`Sequence` / `Selector` / `Inverter` / `Retry`（组合节点与装饰器）
- `drone_nodes`：飞行专用行为节点

## 核心 API

```rust
pub use control::{Inverter, Retry, Selector, Sequence};
pub use core::{BehaviorContext, BrainOutput, Node, Status, Tree};
pub use drone_nodes::DroneNode; // + 各行为节点
```

## 用法

```rust
use brain_behavior_tree::{BehaviorContext, BrainOutput, Node, Selector, Status, Tree};
use brain_middleware::DataBus;

struct Leaf(&'static str);
impl Node for Leaf {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        ctx.out.note = self.0.to_string();
        Status::Success
    }
}

fn main() {
    let mut tree = Tree::new(Box::new(Selector::new(vec![Box::new(Leaf("fly"))])));
    let bus = DataBus::new();
    let mut out = BrainOutput::idle();
    assert_eq!(tree.tick(&bus, 0, &mut out), Status::Success);
}
```

## 依赖

- 外部：`log`
- 内部：`brain-core`、`brain-message`、`brain-middleware`

> **应用案例**：`brain-node/tree_builder.rs` 用行为树装配完整飞行任务闭环。
