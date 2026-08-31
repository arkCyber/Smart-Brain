//! `RobotBody` trait：大脑与任意“身体”之间的统一契约。

use brain_core::Result;

use crate::command::EffectorCommand;
use crate::state::BodyState;

/// 大脑唯一访问身体（小脑）的入口。
///
/// 任何具身机器人（无人机/四足/轮式/机械臂/人形）都实现该 trait，
/// 使上层感知、决策、规划完全与具体身体解耦。`read_state` 提供当前身体
/// 状态（基座位姿 + 关节 + 接触），`send_command` 下发高层意图。
pub trait RobotBody: Send {
    /// 身体形态。
    fn kind(&self) -> crate::state::RobotKind;

    /// 读取当前身体状态。
    fn read_state(&mut self) -> Result<BodyState>;

    /// 下发一帧高层指令。
    fn send_command(&mut self, cmd: &EffectorCommand) -> Result<()>;

    /// 关闭连接并释放资源。
    fn shutdown(&mut self) {}
}

/// 对 `RobotBody` 的盒装别名（用于异构容器）。
pub type BodyCommandSink = Box<dyn RobotBody>;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::MockRobotBody;
    use crate::state::RobotKind;

    #[test]
    fn boxed_body_is_object_safe() {
        let mut body: BodyCommandSink = Box::new(MockRobotBody::new(RobotKind::Wheeled));
        assert_eq!(body.kind(), RobotKind::Wheeled);
        let st = body.read_state().unwrap();
        assert_eq!(st.kind, RobotKind::Wheeled);
        // 异构容器：可把不同形态装进同一 sink 集合。
        let _others: Vec<BodyCommandSink> = vec![
            Box::new(MockRobotBody::new(RobotKind::Car)),
            Box::new(MockRobotBody::new(RobotKind::Manipulator)),
        ];
    }
}
