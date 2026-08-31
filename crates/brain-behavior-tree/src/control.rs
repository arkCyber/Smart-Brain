//! 组合节点与装饰器。

use super::core::{Node, Status};

/// 序列（Sequence）：从左到右依次执行子节点。
/// 全部成功 -> Success；任一失败 -> Failure（重置）；任一 Running -> Running。
pub struct Sequence {
    children: Vec<Box<dyn Node>>,
    /// 当前执行到的子节点下标。
    current: usize,
}

impl Sequence {
    pub fn new(children: Vec<Box<dyn Node>>) -> Self {
        Self {
            children,
            current: 0,
        }
    }
}

impl Node for Sequence {
    fn tick(&mut self, ctx: &mut super::core::BehaviorContext) -> Status {
        while self.current < self.children.len() {
            let status = self.children[self.current].tick(ctx);
            match status {
                Status::Success => self.current += 1,
                Status::Failure => {
                    self.current = 0;
                    return Status::Failure;
                }
                Status::Running => return Status::Running,
            }
        }
        self.current = 0;
        Status::Success
    }
}

/// 选择器（Selector / Fallback）：依次尝试子节点。
/// 任一成功 -> Success（重置）；全部失败 -> Failure；任一 Running -> Running。
pub struct Selector {
    children: Vec<Box<dyn Node>>,
    current: usize,
}

impl Selector {
    pub fn new(children: Vec<Box<dyn Node>>) -> Self {
        Self {
            children,
            current: 0,
        }
    }
}

impl Node for Selector {
    fn tick(&mut self, ctx: &mut super::core::BehaviorContext) -> Status {
        while self.current < self.children.len() {
            let status = self.children[self.current].tick(ctx);
            match status {
                Status::Success => {
                    self.current = 0;
                    return Status::Success;
                }
                Status::Failure => self.current += 1,
                Status::Running => return Status::Running,
            }
        }
        self.current = 0;
        Status::Failure
    }
}

/// 反转装饰器（Inverter）：成功 <-> 失败。
pub struct Inverter {
    child: Box<dyn Node>,
}

impl Inverter {
    pub fn new(child: Box<dyn Node>) -> Self {
        Self { child }
    }
}

impl Node for Inverter {
    fn tick(&mut self, ctx: &mut super::core::BehaviorContext) -> Status {
        match self.child.tick(ctx) {
            Status::Success => Status::Failure,
            Status::Failure => Status::Success,
            Status::Running => Status::Running,
        }
    }
}

/// 重试装饰器（Retry）：子节点失败时重试最多 `max_attempts` 次。
pub struct Retry {
    child: Box<dyn Node>,
    max_attempts: u32,
    attempts: u32,
}

impl Retry {
    pub fn new(child: Box<dyn Node>, max_attempts: u32) -> Self {
        Self {
            child,
            max_attempts,
            attempts: 0,
        }
    }
}

impl Node for Retry {
    fn tick(&mut self, ctx: &mut super::core::BehaviorContext) -> Status {
        loop {
            match self.child.tick(ctx) {
                Status::Success => {
                    self.attempts = 0;
                    return Status::Success;
                }
                Status::Failure => {
                    self.attempts += 1;
                    if self.attempts >= self.max_attempts {
                        self.attempts = 0;
                        return Status::Failure;
                    }
                    // 重试前重置子节点内部状态。
                    self.child.reset();
                }
                Status::Running => return Status::Running,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::core::{BehaviorContext, BrainOutput, Node, Status};
    use super::*;
    use brain_core::time::Timestamp;
    use brain_middleware::DataBus;

    /// 假节点：每次返回预设状态；可选记录被 tick 的次数与 reset 次数。
    struct Stub {
        result: Status,
        ticks: u32,
        resets: u32,
    }
    impl Stub {
        fn of(result: Status) -> Self {
            Self {
                result,
                ticks: 0,
                resets: 0,
            }
        }
    }
    impl Node for Stub {
        fn tick(&mut self, _ctx: &mut BehaviorContext) -> Status {
            self.ticks += 1;
            self.result
        }
        fn reset(&mut self) {
            self.resets += 1;
        }
    }

    fn mkctx<'a>(bus: &'a DataBus, out: &'a mut BrainOutput) -> BehaviorContext<'a> {
        BehaviorContext {
            bus,
            now: Timestamp::default(),
            out,
        }
    }

    #[test]
    fn sequence_success_when_all_succeed() {
        let mut seq = Sequence::new(vec![
            Box::new(Stub::of(Status::Success)),
            Box::new(Stub::of(Status::Success)),
        ]);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(seq.tick(&mut mkctx(&bus, &mut out)), Status::Success);
    }

    #[test]
    fn sequence_short_circuits_on_failure() {
        let mut seq = Sequence::new(vec![
            Box::new(Stub::of(Status::Failure)),
            Box::new(Stub::of(Status::Success)),
        ]);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(seq.tick(&mut mkctx(&bus, &mut out)), Status::Failure);
    }

    #[test]
    fn sequence_pauses_on_running_and_resumes() {
        let mut seq = Sequence::new(vec![
            Box::new(Stub::of(Status::Running)),
            Box::new(Stub::of(Status::Success)),
        ]);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        // 第一个子节点 Running -> 序列保持 Running，不推进到第二个。
        assert_eq!(seq.tick(&mut mkctx(&bus, &mut out)), Status::Running);
        assert_eq!(seq.tick(&mut mkctx(&bus, &mut out)), Status::Running);
    }

    #[test]
    fn selector_succeeds_when_any_succeeds() {
        let mut sel = Selector::new(vec![
            Box::new(Stub::of(Status::Failure)),
            Box::new(Stub::of(Status::Success)),
        ]);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(sel.tick(&mut mkctx(&bus, &mut out)), Status::Success);
    }

    #[test]
    fn selector_fails_when_all_fail() {
        let mut sel = Selector::new(vec![
            Box::new(Stub::of(Status::Failure)),
            Box::new(Stub::of(Status::Failure)),
        ]);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(sel.tick(&mut mkctx(&bus, &mut out)), Status::Failure);
    }

    #[test]
    fn inverter_flips_success_and_failure() {
        let mut inv = Inverter::new(Box::new(Stub::of(Status::Success)));
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(inv.tick(&mut mkctx(&bus, &mut out)), Status::Failure);

        let mut inv = Inverter::new(Box::new(Stub::of(Status::Failure)));
        assert_eq!(inv.tick(&mut mkctx(&bus, &mut out)), Status::Success);
    }

    #[test]
    fn retry_gives_up_after_max_attempts() {
        use std::cell::Cell;
        use std::rc::Rc;

        // 用共享计数器在 Box<dyn Node> 抹除类型后仍能断言被 tick 的次数。
        let ticks = Rc::new(Cell::new(0u32));
        struct Counter {
            ticks: Rc<Cell<u32>>,
        }
        impl Node for Counter {
            fn tick(&mut self, _ctx: &mut BehaviorContext) -> Status {
                self.ticks.set(self.ticks.get() + 1);
                Status::Failure
            }
        }
        let mut retry = Retry::new(
            Box::new(Counter {
                ticks: ticks.clone(),
            }),
            3,
        );
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(retry.tick(&mut mkctx(&bus, &mut out)), Status::Failure);
        // 失败 3 次后放弃：子节点恰被 tick 3 次。
        assert_eq!(ticks.get(), 3);
    }

    #[test]
    fn retry_succeeds_after_transient_failures() {
        // 子节点前两次失败、之后成功（max_attempts=5）。
        struct Flaky {
            fails: u32,
        }
        impl Node for Flaky {
            fn tick(&mut self, _ctx: &mut BehaviorContext) -> Status {
                if self.fails > 0 {
                    self.fails -= 1;
                    Status::Failure
                } else {
                    Status::Success
                }
            }
        }
        let mut retry = Retry::new(Box::new(Flaky { fails: 2 }), 5);
        let bus = DataBus::new();
        let mut out = BrainOutput::idle();
        assert_eq!(retry.tick(&mut mkctx(&bus, &mut out)), Status::Success);
    }
}
