# 路线图（Roadmap）

本文是 Smart-Brain 的能力清单与演进计划。**✅ 表示已在 `main` 分支实现并有单测覆盖**。
想找活干？优先看「下一步」中标注了具体落点的条目。

## 当前状态

| 维度 | 状态 |
|------|------|
| 版本 | `0.1.0`（早期原型，API 可能变动） |
| 测试 | 570+ 单元测试，全部离线可跑（含 HTTP 后端的回环模拟） |
| 构建 | 默认 `cargo build` 零系统依赖；硬件/网络后端全部 feature 门控 |
| 部署阶段 | 阶段一（SITL 仿真闭环）完成，真机接入点已就绪 |

## ✅ 已完成

### 基础与通信
- ✅ `brain-core`：统一错误/配置、可注入时钟与单调计时器、**NTP 风格时间同步**
  （四时间戳握手 + 中位数滤波 + 健康判定）、数学原语。
- ✅ `brain-message`：遥测/指令消息（serde）+ **线缆帧编解码**（长度前缀 + CRC-16 +
  半包/粘包重组）。
- ✅ `brain-middleware`：类型安全话题总线 `DataBus`。
- ✅ `brain-transport`：`FcuTransport` 抽象 + mock/UDP/串口（`serial`）/CAN（`can`）+ MAVLink 式编解码。
- ✅ `brain-zenoh`：Zenoh 风格统一通信（Pub/Sub + Store/Query + Compute），
  可经 `real-zenoh` 切换到真实 Zenoh。
- ✅ `brain-ipc`：零分配环形缓冲。

### 感知、定位与建图
- ✅ `brain-perception`：推理后端抽象 + 流水线 + YOLOv8 解码/NMS；`onnx` feature 接 `ort`。
- ✅ `brain-odometry`：IMU、立体/RGB-D 视觉里程计、VIO 融合、Kalman、Kabsch。
- ✅ `brain-mapping`：占据/体素栅格 + raycast 更新。

### 规划与自主导航
- ✅ `brain-planning`：A*、RRT、DWA、**Ackermann-DWA**、Dubins、**Reeds-Shepp**。
- ✅ `brain-nav`：前沿探索选点 + 面包屑回溯。
- ✅ `brain-autopilot`：无人机/差速闭环 `Autopilot`、**汽车闭环 `CarAutopilot`**
  （Dubins 平滑 + Ackermann DWA + 倒车跟随）、**水面艇 `BoatAutopilot`**
  （时变水流/潮汐 + 逆流定泊）+ `ais`（`!AIVDM` 解码）+ `colregs`（对遇/交叉/追越、
  能见度受限 Rule 19、机动船让帆船、多目标协同避让）。

### 具身与运动控制
- ✅ `brain-robot`：`RobotBody` 身体无关抽象 + `CarBody` / `BoatBody`。
- ✅ `brain-kinematics`：FK/几何雅可比/IK（四足、机械臂、人形）+ 自行车/Ackermann 运动学。
- ✅ `brain-locomotion`：步态相位、足端轨迹、速度→落点、腿部 IK、**WBC**、
  **静力学/逆动力学（RNEA）/正向动力学**、**足-地接触动力学**。

### 决策、任务与智能
- ✅ `brain-state`：状态机 + `FailsafeWatchdog` + 安全监督（围栏/电量/pre-arm）。
- ✅ `brain-behavior-tree`：行为树框架 + 飞行节点。
- ✅ `brain-mission`：航点任务 + 蜂群 Link + **Leader 选举与任务分配**。
- ✅ `brain-agent`：Rig 风格 Agent（工具调用/ReAct）+ RAG + **模型后端工厂**
  （`mock` / `ollama` / `hermes`）。

### 仿真与工程化
- ✅ `brain-sim`：可插拔 `Simulator` 契约 + 确定性 `MockSimulator`（Gazebo/AirSim-agnostic）。
- ✅ `brain-node`：CLI（`--demo` / `--list` / `--config` / `--iterations`）+ 21 段演示。
- ✅ 工程化：CI（build/test/clippy/fmt/docs + feature 矩阵）、RustSec 依赖审计、
  打 tag 自动发布二进制、Dependabot、Issue/PR 模板、行为准则与安全策略。

## 🚧 下一步（按优先级）

| 优先级 | 目标 | 落点（crate / trait） | 验收标准 |
|--------|------|----------------------|----------|
| P0 | 接入 Gazebo / AirSim / Isaac | 新增 `brain-sim` 的 `Simulator` 实现 | 用新后端跑通 `brain-node` 的任务闭环演示，大脑代码零改动 |
| P0 | 真实飞控联调（MAVLink 完整协议） | `brain-transport::mavlink` + `FcuTransport` | 与 ArduPilot/PX4 SITL 双向收发模式/遥测；含单测 |
| P1 | 接触模型接入 `brain-sim` 物理后端 | `brain-locomotion::ContactModel` + `brain-sim` | 四足行走在仿真中保持稳定、不穿透地面 |
| P1 | 关节力矩在线校验 | `brain-locomotion::WholeBodyController` | 超限时自动限幅并上报，含单测 |
| P1 | 轨迹生成/平滑 | `brain-planning` | 输出满足曲率/加加速度约束的轨迹，含单测 |
| P2 | 泛化消息（去飞行专用类型） | `brain-message` → `brain-robot`（`RobotKind::Aerial`） | 飞行类型迁出，旧接口有兼容层 |
| P2 | 泛化状态机 | `brain-state`：`FlightState` → `RobotState`（站立/行走/操作/抓取） | 四足/机械臂场景可用，含单测 |
| P2 | 泛化行为树节点 | `brain-behavior-tree`：`Navigate` / `Grasp` / `Manipulate` | 飞行节点降级为一种具体身体实现 |
| P2 | 语义与场景理解 | `brain-planning` + `brain-mission`（车道、交通灯、航路规则） | 航点任务泛化为带语义约束的任务 |
| P3 | 扩展 AIS 报文类型 | `brain-autopilot::ais` | 支持 6/24/27 型报文，含样例单测 |
| P3 | 时变水流多点观测与建模 | `brain-autopilot::boat_autopilot` | 由多点观测估计水流场，含单测 |
| P3 | 传感器话题补全 | `brain-robot::BodyState`（contact/力矩等） | 接触力/IMU/里程计话题可用 |

## 💡 长期设想

- **蜂群与集群智能**：真实 Zenoh 之上的分布式任务分配、一致性共识与协同感知。
- **形式化安全**：为安全监督器/状态机引入可验证的不变量与在线监控（runtime verification）。
- **端到端学习**：感知→控制的学习型策略与现有经典规划器混合（如学习型局部规划）。
- **工具链**：接入更多平台（Windows/ROS2 桥接）、`cargo-deny` 许可证与供应链策略。

## 如何参与

1. 从「下一步」表中挑选一条（或先开 Issue 讨论新方向）；
2. 读 [ARCHITECTURE.md](ARCHITECTURE.md) 确认落点在正确的层；
3. 按 [DEVELOPMENT.md](DEVELOPMENT.md) 跑通质量门禁；
4. 按 [CONTRIBUTING.md](../CONTRIBUTING.md) 提 PR，并在 `CHANGELOG.md` 记录变更。
