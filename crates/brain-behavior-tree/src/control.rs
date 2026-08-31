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
