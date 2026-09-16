# 架构说明（Architecture）

本文说明 Smart-Brain 的分层、依赖方向、数据流与关键抽象。架构的指导原则是
**软硬件解耦、安全隔离、异构计算**。

## 1. 一句话定位

Smart-Brain 是运行在“高算力机载计算机”上的**大脑**：它做感知、建图、规划、
决策与任务管理，但**不下沉到姿态控制**。姿态稳定由**小脑**（Pixhawk/STM32 等
飞控）以高频率闭环完成。两者通过 `brain-transport` 的 `FcuTransport` trait 通信。

```
        ┌──────────────────────────── 大脑（本仓库，Rust）────────────────────────────┐
        │  brain-mission（任务/蜂群）   brain-agent（LLM 推理/工具调用）              │
        │        │                            │                                      │
        │  brain-behavior-tree（决策）    brain-state（状态机 + Failsafe 看门狗）      │
        │        │                            │                                      │
        │  brain-autopilot / brain-planning / brain-nav（规划与闭环导航）              │
        │        │                            │                                      │
        │  brain-perception / brain-odometry / brain-mapping（感知/定位/建图）         │
        │        │                            │                                      │
        │  brain-robot / brain-kinematics / brain-locomotion（具身抽象/运动学/控制）    │
        │        │                            │                                      │
        │  brain-middleware（数据总线）   brain-zenoh（统一通信）/ brain-ipc（本地 IPC） │
        │        └──────────────┬─────────────┘                                      │
        │              brain-message / brain-core（协议与基础原语）                    │
        │                       │                                                     │
        │              brain-transport（唯一硬件出口，FcuTransport）                   │
        │              brain-sim（仿真后端，SITL 时替代硬件）                          │
        └───────────────────────┼─────────────────────────────────────────────────────┘
                                │  UART / CAN / UDP
                     ┌──────────┴──────────┐
                     │  小脑：飞控 / MCU    │  高频姿态闭环、电机混控、低层 failsafe
                     └─────────────────────┘
```

## 2. crate 分层与依赖规则

| 层 | crate | 职责 | 允许依赖 |
|----|-------|------|----------|
| L0 基础 | `brain-core` | 错误类型、配置、时钟与时间同步、数学原语 | 无内部依赖 |
| L1 协议 | `brain-message` | 遥测/指令消息 + 线缆帧编解码（CRC16） | `brain-core` |
| L2 通信/中间件 | `brain-middleware`、`brain-zenoh`、`brain-ipc` | 话题总线、统一通信三支柱、零分配环形缓冲 | L0–L1 |
| L2 硬件 | `brain-transport` | 串口 / CAN / UDP / MAVLink / mock 链路 | L0–L2 |
| L2 仿真 | `brain-sim` | 可插拔 `Simulator` 契约 + 确定性 mock 世界 | L0 |
| L3 具身与运动 | `brain-robot`、`brain-kinematics`、`brain-locomotion` | 身体抽象、FK/IK、步态/全身控制与接触动力学 | L0–L1 |
| L3 感知 | `brain-perception`、`brain-odometry`、`brain-mapping` | 推理后端、VIO、占据/体素栅格 | L0–L2 |
| L3 决策 | `brain-state`、`brain-behavior-tree` | 状态机 + 看门狗 + 安全原语、行为树 | L0–L2 |
| L4 规划与导航 | `brain-planning`、`brain-nav`、`brain-autopilot` | A*/RRT/DWA/Ackermann-DWA/Dubins/Reeds-Shepp、前沿探索、闭环导航（无人机/汽车/水面艇） | L0–L3 |
| L5 任务与智能 | `brain-mission`、`brain-agent` | 航点任务、蜂群协同、LLM Agent 工具调用 | L0–L4 |
| L6 装配 | `brain-node` | 主程序：把各层装配成闭环，提供 CLI 与演示 | 全部 |

**依赖方向规则（贡献代码时必须遵守）**：

1. 依赖只能**向下**（高层依赖低层），不允许出现反向依赖或环。
2. 跨领域逻辑放在对应的 crate，不要堆在 `brain-node`（它只做装配与演示）。
3. 通用逻辑优先放到 `brain-core`/`brain-robot`，避免在多个 crate 里重复实现。

## 3. 关键抽象（trait）

| trait | 位置 | 作用 | 真机替换对象 |
|-------|------|------|--------------|
| `FcuTransport` | `brain-transport` | 大脑访问飞控的唯一出口：发送 `Command`、接收遥测 | 串口 / CAN / UDP / MAVLink |
| `Simulator` | `brain-sim` | 仿真世界契约（步进、测距、目标检测、状态查询） | Gazebo / AirSim / Isaac |
| `RobotBody` | `brain-robot` | 身体无关的机器人接口（状态 / 指令 / 执行器设点） | 无人机、汽车、水面艇、四足 |
| `Model` | `brain-agent` | LLM 后端契约（生成 + 工具调用） | Ollama / Hermes / OpenAI 兼容 HTTP |
| `Clock` / `SyncExchange` | `brain-core` | 可注入时钟与时间同步握手，便于测试与分布式对时 | 系统时钟 / UDP / 串口 |

设计意图：**所有与硬件、外部服务、时间来源的耦合都收敛到少数 trait 上**，
因此在 SITL/mock 下写出的同一套业务代码可直接用于真机。

## 4. 数据流：一次完整任务闭环

以 `cargo run -p brain-node`（默认演示）为例：

```
任务（brain-mission: 航点表）
   │  下发 Mode + 目标
   ▼
状态机（brain-state）+ 行为树（brain-behavior-tree）
   │  选择当前动作：起飞 / 巡航 / 跟踪 / 返航 / 降落
   ▼
导航与规划（brain-autopilot + brain-planning + brain-nav）
   │  使用世界模型与局部障碍，输出速度/指令
   ▼
命令编码（brain-message::Command + frame CRC16）
   │
   ▼
传输（brain-transport::FcuTransport）──► 小脑（飞控）
   ▲                                        │
   └────────── 遥测（Attitude/GPS/Battery）◄─┘
   │
   ▼
数据总线（brain-middleware::DataBus）→ 感知（brain-perception）
                                       → 安全监督（brain-state::safety 围栏/电量/pre-arm）
                                       → Failsafe 看门狗（心跳超时 → Loiter/返航）
```

**安全隔离要点**：`brain-node` 的主循环中，安全监督器（围栏/电量/pre-arm）
可以**覆盖**任务下发的指令；`FailsafeWatchdog` 在心跳超时（默认 50ms）时
强制进入安全模式，优先级高于一切任务逻辑。

## 5. 室内/无 GPS 场景的链路

```
RangeSensor / Camera / IMU
   │
   ├─ brain-odometry（VIO：IMU + 视觉里程计 + Kalman/Kabsch 融合）→ 位姿
   ├─ brain-mapping（Voxel/Occupancy Grid + raycast 更新）→ 地图
   └─ brain-nav（前沿探索选择目标 + 面包屑回溯）
            │
            ▼
      brain-planning（A* 全局 / RRT 连续 / DWA 与 Ackermann-DWA 局部）
            │
            ▼
      brain-autopilot（无人机 Autopilot / CarAutopilot / BoatAutopilot 闭环）
```

## 6. 水面艇与车辆的特化

- **汽车（阿克曼）**：`brain-kinematics::BicycleModel`（运动学积分）+
  `AckermannDwaPlanner`（采样速度与前轮转角）+ `DubinsPlanner` /
  `ReedsSheppPlanner`（圆弧平滑与倒车泊车）→ `CarAutopilot` 输出
  `AckermannCommand`，可驱动 `brain-robot::CarBody`。
- **水面艇**：`BoatAutopilot`（差分 `(v, ω)` + 时变水流/潮汐 + 逆流定泊）+
  `ais`（`!AIVDM` 报文解码 → 局部东/北偏移）+ `colregs`（对遇/交叉/追越、
  能见度受限 Rule 19、机动船让帆船、多目标协同避让）。

## 7. 设计权衡记录

| 决策 | 理由 | 代价 |
|------|------|------|
| 硬件/网络后端全部 feature 门控（默认关闭） | 默认 `cargo build` 零系统依赖、可离线复现；避免误连真机 | 真机功能需要显式启用 feature
| `brain-node` 只做装配与演示 | 保证可复用逻辑都在库 crate 中，便于测试与复用 | 新增演示时需注意不要把业务逻辑写进 `brain-node`
| 同时提供 mock 与真实后端（如 Zenoh、LLM） | 无外部服务也能跑全量单测（CI 用本地回环服务器模拟 HTTP） | 需要维护两套实现的接口一致性（由共享 trait + 单测保证）
| 提交 `Cargo.lock` | workspace 含可执行文件，需要可复现构建（CI/Release 用 `--locked`） | 升级依赖需显式更新锁文件

## 8. 相关文档

- [DEVELOPMENT.md](DEVELOPMENT.md) — 环境、命令、feature 矩阵
- [DEPLOYMENT.md](DEPLOYMENT.md) — 真机部署路线与配置
- [ROADMAP.md](ROADMAP.md) — 演进方向
- 各 crate 的 `README.md` — 模块级 API 与示例
