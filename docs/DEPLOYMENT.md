# 真机部署指南（Deployment）

本项目遵循参考架构的建议：**真机上天/上路前，先在电脑里完成 90% 的测试**。
本文给出四阶段路线、配置说明与硬性安全要求。

> ⚠️ 免责声明：本项目是研究性与原型软件，**不构成适航、船级社或道路安全认证**。
> 真机测试请遵守当地法规，做好物理急停/遥控接管与场地隔离，安全责任由部署方承担。

## 1. 四阶段路线

### 阶段一 · 原型 / 仿真（当前所处阶段 ✅）

```bash
cargo run -p brain-node                     # 完整任务闭环（mock 飞控 + mock 推理）
cargo run -p brain-node -- --demo failsafe  # 看门狗：大脑卡死 → 强制 Loiter
cargo run -p brain-node -- --demo autopilot # 2D 世界自主探索（感知→建图→规划→驱动）
```

- 大脑逻辑与真机**完全一致**，仅硬件出口换成 `MockTransport` 与 `brain-sim::MockSimulator`。
- 下一步可基于 `brain-sim::Simulator` trait 接入 Gazebo / AirSim / Isaac：**大脑代码零改动**，
  只需新写一个 `Simulator` 实现并在装配处替换。

### 阶段二 · Rust 底层与中间件

- 硬件链路：`brain-transport` 已提供 `SerialTransport`（`serial` feature）与 Linux
  `CanTransport`（`can` feature，SocketCAN）；UDP 链路见 `UdpTransport`（可对接 SITL
  / 机载网口）；MAVLink 式编解码见 `brain-transport::mavlink`。
- 机载计算平台：建议 **Jetson Orin / RK3588** 一类的 ARM64 异构平台，系统用 Ubuntu，
  按需启用 RT 内核补丁（RT-Preempt）以提高实时性；`brain-ipc` 的零分配环形缓冲
  用于高频传感器数据的本地传递。
- 统一通信：`brain-zenoh` 的 `real-zenoh` feature 可切换到真实 Zenoh（Pub/Sub +
  Store/Query + Compute），用于多机/蜂群低延迟通信。

### 阶段三 · AI 模型工程化

1. 训练：PyTorch（检测/分割等），导出 `.onnx`。
2. 量化：Jetson → TensorRT（INT8）；RK3588 → RKNN。
3. 推理：启用 `brain-perception` 的 `onnx` feature（`ort` crate 使用 `load-dynamic`，
   运行时动态加载系统 onnxruntime）。YOLOv8 输出解码 + NMS 已实现。

```bash
cargo build -p brain-perception --features onnx
# 运行时确保可加载动态库：
export LD_LIBRARY_PATH=/usr/lib/onnxruntime:$LD_LIBRARY_PATH
```

模型权重不进仓库（`.gitignore` 已忽略 `*.onnx` / `/models/`）：请用外部存储、
对象存储或 Git LFS 显式管理，并在部署脚本中校验哈希。

### 阶段四 · 真机联调与边界测试

- **拉线测试**：先断电拆桨（无人机）/ 架起驱动轮（车辆）验证指令链路与心跳。
- **边界测试**：故意让大脑休眠/阻塞，验证 `FailsafeWatchdog` 在
  `failsafe_timeout_ms`（默认 50ms）后剥夺控制权进入自动悬停/返航。
- **安全监督**：`brain-state::safety` 的围栏（半径/高度）、电量分级
  （low/RTH/critical）与 pre-arm 检查（GPS 星数、3D Fix、电池、Home 点）会在
  任务指令之上做**兜底覆盖**。
- **降级策略**：飞控（小脑）自身的 failsafe、遥控接管通道与物理急停必须始终可用，
  大脑的 fail-safe 只是**最后一道防线**，不是唯一防线。

## 2. 配置说明（config.example.json）

启动时按优先级加载：`--config <path>` → 环境变量 `SMART_BRAIN_CONFIG` →
`./config.json` → `config.example.json` → 内置默认值。

```bash
cp config.example.json config.json   # config.json 已被 .gitignore 忽略
cargo run -p brain-node -- --config config.json
```

| 字段 | 默认 | 说明 |
|------|------|------|
| `node_id` | `smart-brain-01` | 节点标识，多机/蜂群场景用于区分节点 |
| `heartbeat_period_ms` | `10` | 心跳周期；与看门狗配合判定大脑活性 |
| `failsafe_timeout_ms` | `50` | 心跳超时阈值，超时强制进入安全模式（Loiter/返航） |
| `tick_period_ms` | `20` | 主循环周期（任务/决策节拍） |
| `fcu.transport` | `mock` | 飞控链路：`mock` / 串口 / `udp` 等（决定用哪个 `FcuTransport` 实现） |
| `fcu.serial_port` | `/dev/ttyS0` | `serial` feature 下的设备节点 |
| `fcu.baud_rate` | `921600` | 串口波特率 |
| `fcu.udp_target` | `127.0.0.1:14550` | UDP 目标地址（SITL 默认端口） |
| `safety.geofence_radius_m` | `500` | 水平围栏半径（超出触发返航/RTL） |
| `safety.geofence_max_altitude_m` | `200` | 高度上限 |
| `safety.battery_low_pct` | `40` | 低电告警阈值 |
| `safety.battery_rth_pct` | `30` | 触发返航的阈值 |
| `safety.battery_critical_pct` | `15` | 严重低电（就近降落） |
| `safety.prearm_*` | 见模板 | 起飞前检查：GPS 星数、3D Fix、电池、Home 点 |
| `ollama.endpoint` / `model` | `http://localhost:11434` / `qwen2.5` | 本地 Ollama 推理（`ollama` feature） |
| `hermes.endpoint` / `model` | `http://127.0.0.1:11438` / `hermes-rust` | Hermes 智能体 daemon（`hermes` feature） |
| `agent.backend` | `mock` | 模型后端工厂选择：`mock` / `ollama` / `hermes` |

**密钥不要写进配置文件**：Ollama / Hermes 的鉴权请用环境变量
`OLLAMA_API_KEY` / `HERMES_API_TOKEN`。

## 3. 构建部署产物

```bash
# 发布构建（LTO + 单 codegen unit，见 Cargo.toml 的 [profile.release]）
cargo build --locked --release -p brain-node
ls -lh target/release/brain-node
```

打 tag 时 `.github/workflows/release.yml` 会自动构建多平台二进制并附 SHA256 校验和：

```bash
git tag v0.1.0 && git push origin v0.1.0
# 校验下载产物
shasum -a 256 -c smart-brain-v0.1.0-x86_64-unknown-linux-gnu.tar.gz.sha256
```

## 4. 部署检查清单

- [ ] `cargo test --workspace` 与 `cargo clippy --workspace --all-targets -- -D warnings` 全绿
- [ ] 已在仿真（`brain-node` 演示 / SITL）中跑通目标任务闭环
- [ ] `config.json` 已按机型设置围栏、电量阈值与 pre-arm 检查，并**未包含密钥**
- [ ] `failsafe_timeout_ms` 已结合控制周期与通信时延调优，并做过“大脑卡死”注入测试
- [ ] 遥控接管 / 物理急停 / 飞控自身 failsafe 均可用且已验证
- [ ] 日志与遥测可回传（`brain-middleware` 总线 / 串口 / 网络），便于事故复盘
- [ ] 场地隔离、法规报备与保险已就绪

## 5. 相关文档

- [ARCHITECTURE.md](ARCHITECTURE.md) — 分层与大脑/小脑边界
- [DEVELOPMENT.md](DEVELOPMENT.md) — feature 矩阵与开发环境
- [ROADMAP.md](ROADMAP.md) — 各阶段剩余工作
- [SECURITY.md](../SECURITY.md) — 部署安全边界
