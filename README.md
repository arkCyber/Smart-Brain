# Smart-Brain — 无人机 AI 大脑（智能任务计算机）

用 **Rust** 编写、以 **cargo workspace** 组织的飞行器“AI 大脑”原型。
它严格遵循“**软硬件解耦、安全隔离、异构计算**”的参考架构，将高频机动的
飞控（小脑）与高算力的 AI 决策（大脑）在逻辑上分层隔离。

> 这是一个**可编译、可运行、可测试**的工作区骨架。默认在 **SITL（mock 飞控）**
> 中运行完整任务闭环，无需任何系统级依赖，`cargo build` 开箱即用。
> 真机部署的接线点（真实串口、ONNX 模型）已在代码中以 feature/接口形式预留。

---

## 一、架构与参考分层映射

工作区的每个 crate 对应参考架构中的某一层：

| 层 | 参考架构 | 本工作区 crate |
|----|----------|----------------|
| 5. 任务与应用层 | 航线自主规划、蜂群协同通信 | `brain-mission`（航点任务 + Swarm Link） |
| 4. AI 智能与感知层 | 视觉SLAM / 避障 / 目标跟踪推理 | `brain-perception`（推理后端抽象 + 流水线） |
| 3. 决策/中间件层 | 数据总线 / 状态机 / 行为树 | `brain-middleware`（话题总线）、`brain-state`（状态机+看门狗）、`brain-behavior-tree`（行为树） |
| 3. 统一通信层 | **Zenoh 风格：Pub/Sub + Store/Query + Compute** | `brain-zenoh`（统一通信后端，可切换真实 zenoh） |
| 4. 闭环自主导航 | 感知→建图→规划→驱动→回溯 | `brain-autopilot`（2D 世界自主探索/避障仿真） |
| 4. Agent/LLM 思考层 | 工具调用 + 感知→推理→控制循环 | `brain-agent`（Agent + MockLLM，Rig 风格） |
| 3.5 具身抽象层 | **身体无关的机器人接口** | `brain-robot`（RobotKind / BodyState / EffectorCommand / RobotBody / CarBody / BoatBody） |
| 3.5 运动学层 | 正/逆运动学、雅可比、**车辆运动学** | `brain-kinematics`（FK/IK、四足/机械臂/人形、自行车/Ackermann 模型） |
| 3.5 室内空间感知 | 无 GPS 定位 + 3D 占据网格 + 局部避障 + 室内任务 | `brain-odometry`（VIO）、`brain-mapping`（Voxel Grid）、`brain-planning`（A*/DWA/Ackermann-DWA/Dubins/Reeds-Shepp）、`brain-nav`（探索/回溯）、`brain-ipc`（环形缓冲） |
| 2. 硬件接口层 | 与小脑（飞控）通信 | `brain-transport`（串口/UDP/mock） |
| 基础 | 通用原语 | `brain-core`（错误/配置/时钟/数学） |
| 装配 | 主程序 | `brain-node`（把各层串成闭环） |

**大脑 / 小脑解耦**：`brain-transport` 的 `FcuTransport` trait 是大脑唯一访问
飞控的入口。大脑只下发高层意图（`Command`），姿态稳定由小脑（STM32/Pixhawk）
负责，二者通过 UART/CAN 通信——正对应参考架构中“不要让 AI 和飞控算法跑在
同一处理器”的原则。

---

## 二、快速开始

```bash
cd Smart-Brain

# 构建整个 workspace（默认无系统级依赖，开箱即用）
cargo build

# 运行全部单元测试（236 项）
cargo test

# 运行完整任务演示（SITL）：起飞 → 巡航 → 发现并跟踪目标 → 降落 → 返回地面
# 随后演示 Fail-safe 看门狗在“大脑卡死”时强制进入自动悬停（Loiter）
cargo run -p brain-node
```

**配置加载**：`brain-node` 启动时依次尝试加载配置，优先级为
1. 环境变量 `SMART_BRAIN_CONFIG` 指定的路径
2. 工作目录下的 `config.json`
3. 仓库自带的 `config.example.json`
4. 以上均缺失时回退到内置默认值

主程序会用加载到的 `node_id`、`failsafe_timeout_ms`、`fcu.transport`（选择串口/CAN/
UDP/mock 传输）与 `safety`（围栏/电量/pre-arm 阈值）等配置。真机部署时复制
`config.example.json` 为 `config.json` 并按需修改即可（`config.json` 已被
`.gitignore` 忽略，避免提交本地私有配置）。

示例输出：

```
[tick   0] state=Ground      mode=Idle       note=mission armed, executing survey
[tick   1] state=TakingOff   mode=Takeoff    note=takeoff to 30m
[tick   2] state=Cruising    mode=Cruise     note=cruise waypoint 0
[tick   3] state=Cruising    mode=Cruise     note=cruise waypoint 1
>>> mission complete: landed & returned to Ground
=== Fail-safe demo (watchdog timeout = 50ms) ===
heartbeat ok, watchdog armed.
>>> WATCHDOG TRIPPED after 97ms -> forcing LOITER (auto-hover)
```

---

## 三、各模块说明

### brain-core（基础）
统一错误类型 `BrainError` / `Result`、`BrainConfig`（可 JSON 序列化）、
单调时间戳。是 workspace 的“契约底座”，零外部系统依赖。

### brain-message（通信协议）
仿 MAVLink 定义遥测（姿态/GPS/电池）与指令（模式 + 目标）消息，全部可
`serde` 序列化，便于经数据总线/串口/UDP/持久化传输。定义了大脑的
`Mode`（Idle/Takeoff/Cruise/Track/ReturnHome/Land/Loiter）。内置**线缆帧编解码**
（`frame`）：长度前缀 + CRC-16 + `FrameReader` 缓冲分帧器，正确处理半包/粘包，
供串口/UDP 做可靠传输。

### brain-middleware（数据总线 = “神经网”）
类型安全的话题发布/订阅 `DataBus` + `Topic<T>`，模块通过总线解耦
（感知发检测、决策订阅之），类似 ROS2 topic / Zenoh key-expression。

### brain-transport（与小脑的物理链路）
`FcuTransport` trait + `MockTransport`（SITL 仿真）、`UdpTransport`、
`SerialTransport`（真机接线，`serial` feature 启用）、`CanTransport`（Linux
SocketCAN，`can` feature 启用，仅 Linux；编解码为纯函数可离线测试）。UDP/串口
均使用 `brain-message::frame`（长度 + CRC-16 + `FrameReader`）做可靠分帧，修复了
"每条消息换一行/每次新建 BufReader 丢缓冲"的问题，正确处理半包/粘包。
内置 **MAVLink 风格二进制编解码**（`mavlink`）：把 `Command`/`Telemetry` 封装成
`[msgid, 定长小端负载]`，再经帧编解码走线缆；`MavLinkTransport` 实现 `FcuTransport`，
便于大脑与 Pixhawk/STM32 用标准协议互通。

### brain-state（状态机 + Fail-safe + 安全原语）
- `StateMachine`：大脑控制意图的合法迁移校验（防止非法状态跳转）。
- `FailsafeWatchdog`：心跳监督，**超过阈值（默认 50ms）未喂狗即触发**，
  强制小脑进入自动悬停/一键返航，实现安全兜底。
- `safety` 模块：**地理围栏**（水平半径 + 高度范围越界判定）、**电量监视**
  （低电/返航/临界分级告警）、**起飞前自检 pre-arm**（GPS 3D + 卫星数 + 电量 +
  home + 看门狗 + 链路），以及把三者归一为 `FlightPermission` 的统一飞行权限判定。

### brain-perception（AI 感知推理）
`ModelBackend` trait 抽象推理后端：`MockModelBackend`（仿真）开箱即用；`OnnxModelBackend`
（`onnx` feature）**已接通真实 ONNX Runtime**（`ort` crate，`load-dynamic` 方式：编译期不下载
onnxruntime、运行时加载系统库），并内置 **YOLOv8 输出解码 + 类别感知 NMS**（`nms` 模块，
纯函数、离线可测）。`VisionPipeline` 组织“取帧→推理→检测/跟踪”并发布到总线。

### brain-behavior-tree（决策层）
自研行为树框架：`Sequence`/`Selector` 组合节点、`Inverter`/`Retry` 装饰器，
以及飞行节点（`Takeoff`/`Cruise`/`DetectTarget`/`TrackTarget`/`ReturnHome`/
`Land`/`Failsafe`）。任务树在 `brain-node/src/tree_builder.rs` 装配。

### brain-mission（任务层）
航点任务模型、`MissionExecutor`（逐个下发航点）、`SwarmLink`（蜂群态势
广播，JSON 可序列化，便于接入 Zenoh/UDP）。支持**任务文件序列化**（`to_json`/
`from_json`/`save`/`load`）与**进度跟踪**（`MissionProgress`：航点完成数、百分比、
已飞/总/剩余距离，`update_position` 累计里程）。新增 **`swarm_coord`**：**Leader
选举**（按 `node_id` 或电量，全网确定性一致）与**任务分配**（`TaskAllocator`
轮询 / 按优先级，`SwarmCoordinator::plan` 由 leader 分发给全体成员）。

### brain-robot（具身抽象层）
让“大脑”适用于任意具身机器人的**关键 crate**：`RobotKind`（Aerial/Quadruped/
Humanoid/Wheeled/**Car**/Manipulator/Underwater/**SurfaceVessel**）、通用 `BodyState`
（基座位姿+关节+接触）、`EffectorCommand`（运动模式+高层任务）、`RobotBody` trait。
无人机只是众多“身体”之一——`brain-node/embodiment.rs` 演示了把 `FcuTransport`
包装成 `RobotBody`；`CarBody`（汽车）与 `BoatBody`（水面艇，双差速推进）把各自
运动学包装成 `RobotBody`，与驱动四足/机械臂的方式完全一致。

### brain-kinematics（运动学层）
`Pose`（四元数等距）等数学原语（放于 `brain-core::math`）、串联关节链的
正运动学（FK）、几何雅可比、雅可比转置逆运动学（IK）。四足/机械臂/人形
都依赖它；无人机虽用不到关节，但操作与步态规划必需。另含**车辆运动学**
`BicycleModel`/`BicycleState`（自行车/Ackermann 模型：轴距、转向限位与转向速率、
最小转弯半径、速率受限的 `step` 积分、速度↔转角换算），是汽车导航的运动学契约。

### brain-odometry（室内定位）
**视觉惯性里程计（VIO）**：替代 GPS 建立厘米级三维局部坐标系。含 IMU 模型与
积分、针孔相机/深度帧→点云、**多传感器时间戳对齐**（环形缓冲 + 线性插值，
高频 IMU ↔ 低频相机绝不丢包）、RGB-D 帧间 ICP（Kabsch/SVD）视觉里程计、
**卡尔曼滤波融合**（`KalmanFilter`/`KalmanFusion3d`：恒加速模型）已接入
`VisualInertialOdometry::update_frame/update_imu`——高频 IMU 用加速度预测卡尔曼，
低频 VO 用位置测量校正，融合后的位置/速度低噪声、抗漂移。真机可替换为成熟的
LIO/VIO（LIO-SAM/VINS）作为后端。含边界/压力测试：空点云、零步长 IMU、万帧 IMU、
千帧点云、`reset` 重初始化（并借此修复了 `reset` 未重置估计的 bug）。

### brain-mapping（室内避障核心）
**3D 占据网格（Voxel Grid）**：把点云经**光线投射（3D DDA）**实时写成
“已占据/空闲/未知”的概率网格（log-odds 融合），提供碰撞查询、**前沿探测**
（供探索）、以及**障碍膨胀 `inflate`**（为路径规划/避障预留安全边距）。
可接 OctoMap 类开源库做稠密建图。

### brain-planning（室内局部避障）
`AStar2D`（在占据网格某一高度层做全局无碰撞路径）+ `RrtPlanner`（**快速探索
随机树 RRT**：连续空间采样绕障，确定性种子可复现）+ `DwaPlanner`（**动态窗口法**：
结合最大加速度/转弯半径采样 `(v,ω)`、模拟轨迹、碰撞筛选，毫秒级输出速度指令；
不可避让时返回 `None` 拒绝前进，安全兜底）+ **`AckermannDwaPlanner`（阿克曼动态窗口）**：
面向汽车，采样 `(速度,前轮转角)`，尊重转向速率/最小转弯半径，用**矩形车身包络**
逐点扫掠碰撞检测，按**朝目标推进**打分（支持倒车+转向的 K 形掉头），并在接近目标时
**自动限速制动**，输出 `AckermannCommand`（纵向速度 + 方向盘转角）+ **`DubinsPlanner`**
（**Dubins 曲线**：只前进、受最小转弯半径约束的两构型最短路径，由圆弧+直线组成，
含 LSL/RSR/RSL/LSR/RLR/LRL 六种类型，并用自行车模型精确积分校验）+ **`ReedsSheppPlanner`**
（**Reeds-Shepp 曲线**：可前进/倒车的最短路径，覆盖掉头/侧方与垂直泊车等 48 种类型，
长度取负表示倒车段；比只前进的 Dubins 显著更短）。

### brain-nav（室内任务层）
`Explorer`（**前沿探索**：自动挑选最近的未知边界）、`Backtracker`（**面包屑
回溯**：前进时记录安全轨迹，SLAM 失败/全盲时沿原路返回脱困）。

### brain-ipc（本地低延迟传输）
预分配、零内存分配的**环形缓冲**（覆盖最旧数据），及线程安全封装，用于搬运
百万级点云/高帧率图像（对应参考架构第 3 层）。

### brain-zenoh（统一通信层）
复刻 **Eclipse Zenoh** 的“数据三态融合”统一 API（`CommBackend` trait）：
- **Pub/Sub**：`put` / `subscribe`（键表达式，支持通配，保留最近值）
- **Store/Query**：`put` 写入存储，`get` 聚合全网匹配数据（断网本地存、连网透明查）
- **Compute**：`declare_queryable` 在查询路径上触发计算/服务（RPC）

默认使用纯 Rust 进程内 `LocalZenoh`（离线、可测、与 Zenoh 语义一致）；启用
`real-zenoh` feature 时切换到真实 `zenoh` crate（异步/tokio），获得对等/树状/
云混合拓扑、无线多 AP / 4G 切换无缝会话迁移、零拷贝传输等完整能力。小脑
（单片机）可经 **zenoh-pico**（纯 C）接入同一键空间互通。

**小脑（飞控）桥接**：`brain-transport::ZenohFcuTransport` 把大脑的飞控链路
接到 Zenoh 键空间（大脑发布 `fcu/command`、订阅 `fcu/telemetry`）；`MockFcuZenoh`
模拟跑 zenoh-pico 的 STM32（订阅命令、上报遥测）。二者共享同一 `CommBackend`，
真机换成真实 zenoh 即可蜂群互通。提供 `brain-transport` 内确定性端到端测试。

**真实网络测试**：`brain-zenoh` 的 `net_tests`（`real-zenoh` 特性）用两个真实
peer 会话跑通 **Compute（queryable）+ Pub/Sub + Store/Query** 三支柱。默认
`#[ignore]`，运行：`cargo test -p brain-zenoh --features real-zenoh -- --ignored`。

### brain-agent（Agent / LLM 思考层）
Rig 风格的"感知→推理→工具调用→控制"循环：`Model` trait（LLM/SLM 抽象）+ 离线
确定性 `MockModel`、`Tool`/`FnTool`（用闭包把任意能力变成工具）、`Agent` 主循环
（生成→调用工具→回填结果→直到最终答复）。内置 **RAG**：`Embedder`/`MockEmbedder`
（确定性嵌入）、`MemoryStore`（向量存储 + 余弦检索）、`RetrieveTool`（检索工具）。
含边界/压力测试：空计划直出答复、`max_iters=0` 立即报错、`reset` 清历史、30 次
连续工具调用。真机可把 `MockModel`/`MockEmbedder` 换成 Rig / 端侧小模型与真实嵌入。

### brain-autopilot（闭环自主导航）
把 `brain-mapping`（占据网格/光线投射）、`brain-nav`（前沿探索/面包屑回溯）、
`brain-planning`（DWA + A*/RRT）串成"**感知→建图→规划→驱动→回溯**"的 2D 自主导航
闭环：`World`（地面真值）→ `RangeSensor`（多束测距）→ 局部建图 → 前沿探索选目标
→ **全局引导（A* 栅格 或 RRT 连续空间，`use_rrt` 可切换）** + DWA 局部避障 → 运动
积分 → 卡死时面包屑回溯。单元测试验证了**安全（绝不撞墙）、能移动、能建图、死胡同
可退出**。`Explorer` 会**避开紧贴墙体的前沿**。注：本实现为反应式导航，探索覆盖
有限（A* ~16% / RRT ~22%）；更高覆盖需更强全局规划与墙沿跟随。

除无人机/差速底盘的 `Autopilot` 外，还提供**汽车（阿克曼前轮转向）闭环导航**
`CarAutopilot`：全局引导 A*/RRT（可经 **Dubins 曲线平滑**成圆弧轨迹）+ **Ackermann
DWA 局部避障**（采样 `(速度,前轮转角)`，尊重最小转弯半径与转向速率、用矩形车身包络
做碰撞检测、目标前自动减速）+ **自行车运动学积分** + 纯追踪前瞻。支持 `set_goal`
点对点导航与前沿探索两种模式；单元测试验证能加速、绕障、回到车道并抵达目标。
控制器暴露 `current_command()` 下发 `AckermannCommand`（速度+前轮转角），可直接
`drive` 一辆 `CarBody`，实现“大脑→身体”的具身闭环（同模型同 dt 积分，偏差为 0）。
启用 `use_reeds_shepp` 并调用 `set_goal_pose(带朝向)` 后，会在距目标 `final_radius`
内进入“最终接近”阶段：规划 Reeds-Shepp 掉头/泊车轨迹并**切换倒车跟随**（按段下发
前进/倒车与满舵，漂移超限自动重规划），从而以任意朝向（含倒车入位）抵达。

另提供**水面艇** `BoatAutopilot`（双差速推进 `(v,ω)`）：差分 DWA 局部避障 + 全局引导、
每步叠加**水流漂移**（恒定向量 + 时变**潮汐**）、**多点巡航**（`set_track` 依次驶向一串
航点）、抵达后**逆流定泊保持 / 动力定位**（位置 P 控制 + 水流前馈）。并含 **`Colregs`**
（COLREGS 会遇避让规则引擎：对遇/交叉/追越 → 让路右转或保向）。

### brain-node（主程序）
把上述模块装配成闭环，驱动一次完整任务演示与 fail-safe 演示，并展示
`RobotBody` 具身抽象。含 **21 段演示**，其中 `comprehensive_demo` 串联"任务文件
加载 → 执行+进度上报 → 蜂群协同 → MAVLink 命令下发"的完整流水线，`car_driving_demo`
演示阿克曼汽车的闭环自主驾驶与 `CarBody` 具身抽象，`parallel` 演示**多线程并行
流水线**（感知线程 + 决策线程共享线程安全 `DataBus`，验证跨线程数据流通），
`safety_guard` 把**安全监督器**（围栏/电量/pre-arm）接入任务循环做指令兜底覆盖，
`async_runtime`（`--features async`）演示 **tokio 异步任务并发**（感知/决策作为
async 任务在单一运行时上调度），`swarm_coord_demo` 演示**蜂群 Leader 选举 +
任务分配**（全网确定性一致 + JSON 分配表）。

---

## 四、真机部署路线（对应参考架构四阶段）

参考架构建议：真机上天前先在电脑里完成 90% 测试。

- **第一阶段 · 原型/仿真**：本工作区已用 mock 飞控 + mock 推理跑通
  “起飞→巡航→发现目标→跟踪→降落”闭环。下一步可接入 Gazebo/AirSim SITL。
- **第二阶段 · Rust 底层与中间件**：`brain-transport` 已提供 `SerialTransport`
  （`serial`）与 Linux `CanTransport`（`can`，SocketCAN）；真机在 Jetson/RK3588
  上配置 Ubuntu + RT-Preempt 增强实时性。
- **第三阶段 · AI 模型工程化**：`brain-perception` 已接通 `ort`（`onnx` feature，
  YOLOv8 解码 + NMS）；地面用 PyTorch 训练 → 导出 `.onnx` → TensorRT/RKNN 量化
  为 INT8 → 运行时加载系统 onnxruntime。
- **第四阶段 · 真机联调与边界测试**：拉线测试、用 `FailsafeWatchdog` + `safety`
  模块（围栏/电量/pre-arm）兜底，一旦大脑延迟超过 50ms 立即剥夺控制权进入自动
  悬停/返航。

**当前原型所处阶段**：第一阶段（SITL 仿真闭环）已完成，真机接入点（串口/CAN/
ONNX）已就绪，可据此继续。

---

## 五、面向具身机器人的扩展（从“无人机大脑”到“通用机器人大脑”）

当前代码是“无人机优先”的，但已加入**具身抽象层**，向任意机器人泛化的
关键接口已经就位。要让它完整服务四足/轮式/机械臂/人形，建议按此路线推进：

### 已就位（本工作区已实现）
- `brain-robot`：`RobotBody` trait + 通用状态/指令（身体无关），含 `RobotKind::Car` 与 `CarBody`（汽车适配）。
- `brain-kinematics`：FK / 几何雅可比 / IK，以及**自行车/Ackermann 车辆运动学** `BicycleModel`。
- `brain-robot::MockRobotBody`：任意形态的 SITL 仿真身体（含汽车转向/车轮关节与接地点）。
- `brain-node/embodiment.rs`：把无人机包装成 `RobotBody` 的适配示例；`car_driving_demo.rs` 演示汽车。
- **汽车闭环导航**：`brain-planning::AckermannDwaPlanner`（阿克曼局部避障）+ `DubinsPlanner`（圆弧/直线最短路径）+ `ReedsSheppPlanner`（可倒车的掉头/泊车）+ `brain-autopilot::CarAutopilot`（感知→建图→A*/RRT 引导→Dubins 平滑→Ackermann DWA→自行车积分→回溯，支持 `set_goal` 点对点驾驶；`set_goal_pose` + `use_reeds_shepp` 实现**倒车跟随闭环**）。
- **水面艇闭环导航**：`brain-robot::BoatBody`（双差速推进 `RobotBody`）+ `brain-autopilot::BoatAutopilot`（差分 DWA 导航 + **水流漂移** + **逆流定泊保持/动力定位**）。

### 下一步（建议新增/泛化）
| 目标 | 需要的 crate / 改动 |
|------|---------------------|
| 步态/全身控制 | 新增 `brain-control`（或 `brain-locomotion`）：步态相位、足点轨迹、全身(WBC)命令 |
| 通用路径规划 | 已在 `brain-planning` 落地 A*/RRT/DWA/Ackermann-DWA/Dubins/**Reeds-Shepp**，并在 `CarAutopilot` 接入**倒车跟随闭环**；剩余：轨迹生成/平滑、车道/交通灯语义（从 `brain-mission` 的航点泛化） |
| 泛化消息 | 把 `brain-message` 的飞行专用类型（Attitude/GPS/Mode）迁到 `brain-robot`，飞行模式改由 `RobotKind::Aerial` 的 embodied 后端提供 |
| 泛化状态机 | `brain-state` 由 `FlightState` 扩展为通用 `RobotState`（站立/行走/操作/抓取） |
| 泛化行为树 | 把 `drone_nodes` 泛化为通用机器人节点（`Navigate`/`Grasp`/`Manipulate`），飞行节点降级为具体身体的一种 |
| 传感器 | 增加接触力/IMU/里程计话题类型（已在 `BodyState` 预留 contact） |
| 水面艇/航海 | 已有 `BoatBody` + `BoatAutopilot`（水流漂移/潮汐/多点巡航/定泊）+ `Colregs`（对遇/交叉/追越）；剩余：时变水流接入 DWA、AIS 报文解析、完整 COLREGS（含能见度受限/机动船让路）、多艇协同 |

> 审计要点：`brain-core`、`brain-middleware`、`brain-perception`、
> `brain-state::FailsafeWatchdog`、`brain-behavior-tree`（框架本身）已经是
> **身体无关**的；需要改造的主要是 `brain-message`、`brain-state`、
> `drone_nodes` 与 `brain-transport` 中“飞行专用”的部分。

## 六、后续可扩展方向

- 接入真实 **Zenoh** 或 **ros2-client**（Rust）做蜂群低延迟通信。
- 把行为树换成社区成熟的 **behavior-tree** crate，或接入状态机/规划器。
- 用 **tokio**/多线程把感知、决策、传输流水线拆成独立异步任务。
- 为 `FcuTransport` 增加 **MAVLink 协议**编解码与 CAN 帧封装。
