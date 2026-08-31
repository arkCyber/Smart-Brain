//! 通用（身体无关）状态机 + 通用传感器话题演示。
//!
//! 展示两块新增能力：
//! 1. `RobotStateMachine` / `Fsm<S>`：身体无关的通用状态机（非飞行专用）。
//! 2. `brain-message` 的通用传感器消息（IMU/里程计/测距/接触力）+ `brain-middleware`
//!    的 `sensor/*` 与 `state/robot` 话题，演示在总线上发布/订阅。

use brain_core::Vec3;
use brain_message::sensor::{ContactSample, ImuSample, OdometrySample, RangeScan};
use brain_middleware::bus::topics;
use brain_middleware::{DataBus, Topic};
use brain_state::{RobotState, RobotStateMachine};

/// 顶层入口。
pub fn run() {
    println!("\n=== 通用状态机 + 通用传感器话题（身体无关）===");

    // ---- 1) 通用传感器话题：发布 + 读取 ----
    let bus = DataBus::new();

    let imu = ImuSample::new(1, Vec3::new(0.0, 0.0, -9.81), Vec3::new(0.01, 0.0, 0.0));
    bus.publish(topics::SENSOR_IMU, imu.clone(), 1).unwrap();

    let odom = OdometrySample::new(
        2,
        Vec3::new(1.5, 0.0, 0.0),
        brain_core::Quat::IDENTITY,
        Vec3::new(0.5, 0.0, 0.0),
        Vec3::ZERO,
    );
    bus.publish(topics::SENSOR_ODOMETRY, odom.clone(), 2)
        .unwrap();

    let scan = RangeScan::new(3, vec![0.0, 0.5], vec![3.2, 2.8], 10.0);
    bus.publish(topics::SENSOR_RANGE, scan.clone(), 3).unwrap();

    let contact = ContactSample::new(4, "foot_fl", true, 12.5);
    bus.publish(topics::SENSOR_CONTACT, contact.clone(), 4)
        .unwrap();

    // 读回 IMU 话题。
    let imu_topic: &std::sync::Arc<Topic<ImuSample>> =
        &bus.topic(topics::SENSOR_IMU).expect("imu topic");
    let latest = imu_topic.peek().expect("imu value");
    println!(
        "  [sensor/imu] accel=({:.2},{:.2},{:.2}) m/s², ts={}",
        latest.accel.x, latest.accel.y, latest.accel.z, latest.timestamp
    );
    println!(
        "  [sensor/odometry] pos=({:.2},{:.2},{:.2}) m, v={:.2} m/s",
        odom.position.x, odom.position.y, odom.position.z, odom.linear_vel.x
    );
    println!(
        "  [sensor/range] {} 束, 前向 {:.2} m",
        scan.len(),
        scan.ranges[0]
    );
    println!(
        "  [sensor/contact] {} {} ({:.1} N)",
        contact.frame,
        if contact.in_contact {
            "接触"
        } else {
            "离地"
        },
        contact.force
    );

    // ---- 2) 通用状态机：合法链 + 非法迁移 ----
    let mut sm = RobotStateMachine::new();
    let step = |from: &str, to: RobotState, sm: &mut RobotStateMachine| {
        let ok = sm.transition(to).is_ok();
        println!(
            "  [state/robot] {from} -> {} : {}",
            to.as_str(),
            if ok { "OK" } else { "非法" }
        );
    };
    step("standby", RobotState::Starting, &mut sm);
    step("starting", RobotState::Active, &mut sm);
    step("active", RobotState::Tracking, &mut sm);
    step("tracking", RobotState::Returning, &mut sm);
    step("returning", RobotState::Standby, &mut sm);
    // 非法：Standby -> Tracking 应被拒绝。
    step("standby", RobotState::Tracking, &mut sm);
    println!("  最终状态: {}", sm.current().as_str());

    // 同时把状态发布到总线。
    bus.publish(topics::ROBOT_STATE, sm.current().to_string(), 5)
        .unwrap();

    // 从总线读回并通过 `from_name` 反解析，验证 Display/FromStr 往返。
    let state_topic: &std::sync::Arc<Topic<String>> =
        &bus.topic(topics::ROBOT_STATE).expect("robot state topic");
    let raw = state_topic.peek().expect("robot state value");
    match RobotState::from_name(&raw) {
        Some(s) => println!(
            "  [state/robot] 总线读回并解析: {s} (is_safe={})",
            s.is_safe()
        ),
        None => println!("  [state/robot] 无法解析状态名: {raw}"),
    }
}
