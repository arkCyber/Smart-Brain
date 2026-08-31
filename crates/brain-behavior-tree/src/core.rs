//! 行为树核心抽象。

use brain_core::time::Timestamp;
use brain_message::{CommandTarget, Mode};
use brain_middleware::DataBus;

/// 节点执行状态。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Status {
    /// 节点完成。
    Success,
    /// 节点失败。
    Failure,
    /// 节点仍在运行（需要后续 tick 继续）。
    Running,
}

impl Status {
    pub fn is_running(&self) -> bool {
        matches!(self, Status::Running)
    }
}

/// 由树主循环写入的“大脑输出”：当前控制意图。
#[derive(Debug, Clone)]
pub struct BrainOutput {
    /// 当前飞行模式意图。
    pub mode: Mode,
    /// 指令目标。
    pub target: CommandTarget,
    /// 供日志/监控的文字说明。
    pub note: String,
}

impl Default for BrainOutput {
    fn default() -> Self {
        Self {
            mode: Mode::Idle,
            target: CommandTarget::None,
            note: String::new(),
        }
    }
}

impl BrainOutput {
    /// 创建默认输出（Idle）。
    pub fn idle() -> Self {
        Self::default()
    }

    /// 重置为默认。
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// 一次行为树 tick 的上下文。
pub struct BehaviorContext<'a> {
    pub bus: &'a DataBus,
    pub now: Timestamp,
    pub out: &'a mut BrainOutput,
}

/// 行为树节点 trait。
pub trait Node {
    /// 执行一次 tick。上下文提供数据总线与输出缓冲区。
    fn tick(&mut self, ctx: &mut BehaviorContext) -> Status;

    /// 重置节点内部状态（组合节点在重新开始时调用）。默认空实现：
    /// 无内部状态需要重置的节点可省略。
    fn reset(&mut self) {}
}

/// 行为树：持有根节点，周期驱动。
pub struct Tree {
    root: Box<dyn Node>,
}

impl Tree {
    /// 以根节点构建树。
    pub fn new(root: Box<dyn Node>) -> Self {
        Self { root }
    }

    /// 驱动一次 tick，返回根节点状态并填充输出。
    pub fn tick(&mut self, bus: &DataBus, now: Timestamp, out: &mut BrainOutput) -> Status {
        let mut ctx = BehaviorContext { bus, now, out };
        let status = self.root.tick(&mut ctx);
        log::trace!("tree tick -> {:?}", status);
        status
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_message::Mode;
    use brain_middleware::DataBus;

    /// 测试用假节点：写入指定模式并返回 Success。
    struct SetMode(Mode);
    impl Node for SetMode {
        fn tick(&mut self, ctx: &mut BehaviorContext) -> Status {
            ctx.out.mode = self.0;
            Status::Success
        }
    }

    #[test]
    fn status_semantics() {
        assert!(Status::Running.is_running());
        assert!(!Status::Success.is_running());
        assert!(!Status::Failure.is_running());
        assert_eq!(Status::Success, Status::Success);
    }

    #[test]
    fn brain_output_default_and_reset() {
        let mut out = BrainOutput::idle();
        assert_eq!(out.mode, Mode::Idle);
        assert_eq!(out.target, CommandTarget::None);
        assert_eq!(out.note, "");

        out.mode = Mode::Track;
        out.note = "tracking".into();
        out.reset();
        assert_eq!(out.mode, Mode::Idle);
        assert_eq!(out.note, "");
        assert_eq!(out.target, CommandTarget::None);
    }

    #[test]
    fn tree_tick_runs_root_and_writes_output() {
        let bus = DataBus::new();
        let mut tree = Tree::new(Box::new(SetMode(Mode::Cruise)));
        let mut out = BrainOutput::idle();
        let status = tree.tick(&bus, 123, &mut out);
        assert_eq!(status, Status::Success);
        assert_eq!(out.mode, Mode::Cruise);
    }
}
