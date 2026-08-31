//! `brain-transport` 最小示例：通过 Mock 飞控走一帧“指令 → 遥测”闭环。
//!
//! 运行：`cargo run -p brain-transport --example mock_fcu`

use brain_message::{Command, CommandTarget, Mode};
use brain_transport::{FcuTransport, MockTransport};

fn main() {
    // 内存中模拟的小脑：处于地面待命状态
    let mut fcu = MockTransport::new();

    // 大脑下发“起飞”指令
    let cmd = Command {
        timestamp: 0,
        mode: Mode::Takeoff,
        target: CommandTarget::None,
    };
    fcu.send_command(&cmd).expect("send takeoff");

    // 读取最新遥测：起飞后高度应抬升到 50m
    if let Some(telem) = fcu.try_recv_telemetry().expect("read telemetry") {
        println!(
            "sent {} command(s); altitude = {} m",
            fcu.command_count(),
            telem.gps.alt
        );
    }
}
