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

    /// 重置节点内部状态（组合节点在重新开始时调用）。
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
