//! 小脑（飞控）通过 Zenoh 桥接。
//!
//! 参考 Zenoh 的“小脑（MCU）经 zenoh-pico 接入同一键空间”思路：
//! - 大脑（本 crate 的 `ZenohFcuTransport`）把控制指令 `put` 到 `fcu/command`，
//!   并订阅 `fcu/telemetry` 读取遥测。
//! - 小脑（单片机，跑 zenoh-pico）订阅 `fcu/command`、发布 `fcu/telemetry`。
//!
//! 这样大脑与小脑用同一个 `CommBackend`（LocalZenoh 或真实 zenoh）即可互通，
//! 天然支持 Pub/Sub、断网本地缓存后透明查询、以及蜂群多机共享键空间。

use std::sync::Arc;

use brain_core::Result;
use brain_message::{Command, Telemetry};
use brain_zenoh::{decode, encode, CommBackend, Subscription};

use crate::FcuTransport;

/// 桥接键空间常量。
pub mod keys {
    /// 大脑 → 小脑：控制指令。
    pub const COMMAND: &str = "fcu/command";
    /// 小脑 → 大脑：遥测。
    pub const TELEMETRY: &str = "fcu/telemetry";
}

/// 基于 Zenoh 键空间的小脑（飞控）链路。
///
/// 大脑一侧：`send_command` 发布指令，`try_recv_telemetry` 读取最近遥测。
/// 底层可换成 `LocalZenoh`（仿真）或真实 `zenoh`（真机/蜂群）。
pub struct ZenohFcuTransport {
    backend: Arc<dyn CommBackend>,
    telemetry_sub: Option<Subscription>,
}

impl ZenohFcuTransport {
    /// 用给定 Zenoh 后端创建飞控链路。
    pub fn new(backend: Arc<dyn CommBackend>) -> Result<Self> {
        let telemetry_sub = backend.subscribe(keys::TELEMETRY)?;
        Ok(Self {
            backend,
            telemetry_sub: Some(telemetry_sub),
        })
    }
}

impl FcuTransport for ZenohFcuTransport {
    fn send_command(&mut self, cmd: &Command) -> Result<()> {
        self.backend.put(keys::COMMAND, encode(cmd)?)
    }

    fn try_recv_telemetry(&mut self) -> Result<Option<Telemetry>> {
        match self.telemetry_sub.as_ref().map(Subscription::try_recv) {
            Some(Ok(sample)) => {
                let telem: Telemetry = decode(&sample.value)?;
                Ok(Some(telem))
            }
            _ => Ok(None),
        }
    }

    fn shutdown(&mut self) {
        // 释放遥测订阅通道（取消订阅），不再接收新的遥测。
        self.telemetry_sub = None;
    }
}

/// 模拟小脑：订阅 `fcu/command`，收到指令后发布一帧遥测回传。
///
/// 对应真实场景里跑 **zenoh-pico** 的 STM32：收到命令 → 执行 → 上报遥测。
pub struct MockFcuZenoh {
    backend: Arc<dyn CommBackend>,
    command_sub: Option<Subscription>,
    telemetry: Telemetry,
}

impl MockFcuZenoh {
    pub fn new(backend: Arc<dyn CommBackend>) -> Result<Self> {
        let command_sub = backend.subscribe(keys::COMMAND)?;
        Ok(Self {
            backend,
            command_sub: Some(command_sub),
            telemetry: Telemetry::default_at(0),
        })
    }

    /// 处理一条命令（若队列中有），并回传一帧遥测。
    pub fn poll_and_respond(&mut self) -> Result<bool> {
        let Some(sub) = self.command_sub.as_ref() else {
            return Ok(false); // 已 shutdown
        };
        if let Ok(sample) = sub.try_recv() {
            let cmd: Command = decode(&sample.value)?;
            // 简单响应：按模式更新高度。
            match cmd.mode {
                brain_message::Mode::Takeoff => self.telemetry.gps.alt = 30.0,
                brain_message::Mode::Land => self.telemetry.gps.alt = 0.0,
                _ => {}
            }
            self.telemetry.timestamp = sample.timestamp;
            self.backend
                .put(keys::TELEMETRY, encode(&self.telemetry)?)?;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// 当前遥测（只读）。
    pub fn telemetry(&self) -> &Telemetry {
        &self.telemetry
    }

    /// 释放命令订阅通道（取消订阅）。
    pub fn shutdown(&mut self) {
        self.command_sub = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brain_message::{Command, CommandTarget, Mode};
    use brain_zenoh::LocalZenoh;
    use std::sync::Arc;

    #[test]
    fn brain_to_fcu_roundtrip_over_zenoh() {
        let backend: Arc<dyn CommBackend> = Arc::new(LocalZenoh::new());
        // 大脑一侧的 Zenoh 飞控链路。
        let mut brain_fcu = ZenohFcuTransport::new(backend.clone()).unwrap();
        // 小脑一侧（模拟 zenoh-pico）。
        let mut fcu = MockFcuZenoh::new(backend).unwrap();

        // 大脑下发起飞指令。
        let cmd = Command {
            timestamp: 1,
            mode: Mode::Takeoff,
            target: CommandTarget::Position {
                north: 0.0,
                east: 0.0,
                down: -30.0,
            },
        };
        brain_fcu.send_command(&cmd).unwrap();

        // 小脑处理并回传遥测。
        assert!(fcu.poll_and_respond().unwrap());

        // 大脑读取回传遥测。
        let telem = brain_fcu.try_recv_telemetry().unwrap().expect("telemetry");
        assert!(telem.gps.alt > 20.0, "alt={}", telem.gps.alt);
    }

    #[test]
    fn retained_telemetry_immediately_available() {
        let backend: Arc<dyn CommBackend> = Arc::new(LocalZenoh::new());
        // 先让小脑发布一帧遥测。
        let mut fcu = MockFcuZenoh::new(backend.clone()).unwrap();
        backend
            .put(keys::TELEMETRY, encode(&Telemetry::default_at(5)).unwrap())
            .unwrap();
        // 大脑随后接入，应立刻读到保留的遥测。
        let mut brain_fcu = ZenohFcuTransport::new(backend).unwrap();
        assert!(brain_fcu.try_recv_telemetry().unwrap().is_some());
        let _ = &mut fcu;
    }

    #[test]
    fn brain_returns_none_when_no_telemetry() {
        let backend: Arc<dyn CommBackend> = Arc::new(LocalZenoh::new());
        let mut brain_fcu = ZenohFcuTransport::new(backend).unwrap();
        // 尚未有任何遥测发布 -> 返回 None（不 panic）。
        assert!(brain_fcu.try_recv_telemetry().unwrap().is_none());
        brain_fcu.shutdown();
    }

    #[test]
    fn fcu_poll_no_command_returns_false() {
        let backend: Arc<dyn CommBackend> = Arc::new(LocalZenoh::new());
        let mut fcu = MockFcuZenoh::new(backend).unwrap();
        // 队列为空 -> 返回 false。
        assert!(!fcu.poll_and_respond().unwrap());
    }

    #[test]
    fn fcu_telemetry_accessor() {
        let backend: Arc<dyn CommBackend> = Arc::new(LocalZenoh::new());
        let fcu = MockFcuZenoh::new(backend).unwrap();
        assert_eq!(fcu.telemetry().gps.alt, 0.0);
    }

    #[test]
    fn fcu_land_mode_resets_altitude() {
        let backend: Arc<dyn CommBackend> = Arc::new(LocalZenoh::new());
        let mut fcu = MockFcuZenoh::new(backend.clone()).unwrap();
        let mut brain_fcu = ZenohFcuTransport::new(backend).unwrap();
        // 先起飞（高度 30）。
        brain_fcu
            .send_command(&Command {
                timestamp: 1,
                mode: Mode::Takeoff,
                target: CommandTarget::None,
            })
            .unwrap();
        assert!(fcu.poll_and_respond().unwrap());
        assert!(fcu.telemetry().gps.alt > 20.0);
        // 降落（高度 0）。
        brain_fcu
            .send_command(&Command {
                timestamp: 2,
                mode: Mode::Land,
                target: CommandTarget::None,
            })
            .unwrap();
        assert!(fcu.poll_and_respond().unwrap());
        assert_eq!(fcu.telemetry().gps.alt, 0.0);
    }

    #[test]
    fn shutdown_stops_receiving_telemetry() {
        let backend: Arc<dyn CommBackend> = Arc::new(LocalZenoh::new());
        let mut brain_fcu = ZenohFcuTransport::new(backend).unwrap();
        // shutdown 释放订阅后，不再返回任何遥测。
        brain_fcu.shutdown();
        assert!(brain_fcu.try_recv_telemetry().unwrap().is_none());
    }
}
