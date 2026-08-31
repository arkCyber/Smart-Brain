//! `brain-robot` 最小示例：通过统一的 `RobotBody` 抽象驱动一辆仿真汽车。
//!
//! 运行：`cargo run -p brain-robot --example body`

use brain_core::Vec3;
use brain_robot::{
    BodyState, EffectorCommand, LocomotionMode, MockRobotBody, RobotBody, RobotKind, Task,
    TaskTarget,
};

fn main() {
    // 任意形态的仿真身体——这里是汽车
    let mut body = MockRobotBody::new(RobotKind::Car);
    let before: BodyState = body.read_state().expect("read state");
    println!(
        "kind = {} | wheel contacts = {}",
        body.kind().as_str(),
        before.contacts.len()
    );

    // 下发高层导航指令（大脑 → 身体，与具体形态解耦）
    let cmd = EffectorCommand {
        timestamp: 0,
        locomotion: LocomotionMode::Navigate,
        task: Task::NavigateTo(TaskTarget::Point(Vec3::new(10.0, 0.0, 0.0))),
    };
    body.send_command(&cmd).expect("send command");

    let after: BodyState = body.read_state().expect("read state");
    println!("position after move = {:?}", after.base.pose.position);

    // 停止
    body.send_command(&EffectorCommand::stop(1)).unwrap();
    let stopped: BodyState = body.read_state().expect("read state");
    println!("stopped, linear_vel = {:?}", stopped.base.linear_vel);
}
