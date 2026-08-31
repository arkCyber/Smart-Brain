//! `brain-behavior-tree` 最小示例：用行为树组合节点装配决策逻辑。
//!
//! 运行：`cargo run -p brain-behavior-tree --example tree`

use brain_behavior_tree::{BehaviorContext, BrainOutput, Node, Selector, Status, Tree};
use brain_middleware::DataBus;

/// 一个叶子节点：把说明文字写入输出并返回 Success。
struct SetNote(&'static str);

impl Node for SetNote {
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
        ctx.out.note = self.0.to_string();
        Status::Success
    }
}

fn main() {
    // 根节点：Selector（任一个子节点成功即成功）
    let root = Selector::new(vec![Box::new(SetNote("flying"))]);
    let mut tree = Tree::new(Box::new(root));

    let bus = DataBus::new();
    let mut out = BrainOutput::idle();
    let status = tree.tick(&bus, 0, &mut out);

    println!("tree -> {status:?}, note = {}", out.note);
}
