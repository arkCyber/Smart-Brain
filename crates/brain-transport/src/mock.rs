//! 仿真传输后端：内存中模拟飞控行为，用于 SITL 与单元测试。

use brain_core::Result;
use brain_message::{Command, Mode, Telemetry};

use crate::FcuTransport;

/// 一个在内存中模拟“小脑”响应的传输通道。
///
/// 收到指令后更新内部遥测状态（例如进入 Track 模式则 yaw 变化），
/// 供上层验证指令链路是否通畅。
pub struct MockTransport {
    telemetry: Telemetry,
    received_commands: Vec<Command>,
}

impl MockTransport {
    /// 创建仿真飞控，初始处于地面待命状态。
    pub fn new() -> Self {
        let mut telemetry = Telemetry::default_at(0);
        // 模拟具备 3D 定位与足够卫星，使行为树的 GPS 条件通过。
        telemetry.gps.fix_type = brain_message::telemetry::FixType::Fix3D;
        telemetry.gps.satellites = 12;
        Self {
            telemetry,
            received_commands: Vec::new(),
        }
    }

    /// 注入一帧外部遥测（例如来自仿真环境）。
    pub fn inject_telemetry(&mut self, telemetry: Telemetry) {
        self.telemetry = telemetry;
    }

    /// 已收到指令数量（用于断言）。
    pub fn command_count(&self) -> usize {
        self.received_commands.len()
    }
}

impl Default for MockTransport {
    fn default() -> Self {
        Self::new()
    }
}

impl FcuTransport for MockTransport {
    fn send_command(&mut self, cmd: &Command) -> Result<()> {
        self.received_commands.push(cmd.clone());
        // 简单地根据模式更新仿真状态。
        match cmd.mode {
            Mode::Takeoff | Mode::Cruise => self.telemetry.gps.alt = 50.0,
            Mode::Land => self.telemetry.gps.alt = 0.0,
            Mode::Track => self.telemetry.attitude.yaw += 0.05,
            _ => {}
        }
        Ok(())
    }

    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
        Ok(Some(self.telemetry.clone()))
    }

    fn shutdown(&mut self) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_message::telemetry::FixType;
    use brain_message::CommandTarget;

    fn cmd(mode: Mode) -> Command {
        Command {
            timestamp: 0,
            mode,
            target: CommandTarget::None,
        }
    }

    #[test]
    fn default_has_fix3d_on_ground() {
        let mut t = MockTransport::new();
        let telem = t.try_recv_telemetry().unwrap().unwrap();
        assert_eq!(telem.gps.fix_type, FixType::Fix3D);
        assert_eq!(telem.gps.satellites, 12);
        assert_eq!(telem.gps.alt, 0.0);
    }

    #[test]
    fn takeoff_raises_altitude_and_records_command() {
        let mut t = MockTransport::new();
        t.send_command(&cmd(Mode::Takeoff)).unwrap();
        assert_eq!(t.command_count(), 1);
        let telem = t.try_recv_telemetry().unwrap().unwrap();
        assert_eq!(telem.gps.alt, 50.0);
    }

    #[test]
    fn land_descends_back_to_ground() {
        let mut t = MockTransport::new();
        t.send_command(&cmd(Mode::Takeoff)).unwrap();
        t.send_command(&cmd(Mode::Land)).unwrap();
        let telem = t.try_recv_telemetry().unwrap().unwrap();
        assert_eq!(telem.gps.alt, 0.0);
    }

    #[test]
    fn track_sweeps_yaw() {
        let mut t = MockTransport::new();
        let yaw_before = t.try_recv_telemetry().unwrap().unwrap().attitude.yaw;
        t.send_command(&cmd(Mode::Track)).unwrap();
        let yaw_after = t.try_recv_telemetry().unwrap().unwrap().attitude.yaw;
        assert!(yaw_after > yaw_before);
    }

    #[test]
    fn inject_telemetry_overrides_state() {
        let mut t = MockTransport::new();
        let mut telem = Telemetry::default_at(7);
        telem.battery.remaining_pct = 33.0;
        t.inject_telemetry(telem.clone());
        let got = t.try_recv_telemetry().unwrap().unwrap();
        assert_eq!(got.timestamp, 7);
        assert_eq!(got.battery.remaining_pct, 33.0);
    }
}
