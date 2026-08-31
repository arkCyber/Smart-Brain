# brain-node

> Smart-Brain main executable: assembles and drives all modules

**所属层**：装配层 —— 主程序，把各层串成闭环。

## 职责

工作区的**可执行入口**：装配 `brain-core`…`brain-sim` 等全部模块，驱动一次完整任务演示与 fail-safe 演示，并展示 `RobotBody` 具身抽象。也是**应用案例最丰富**的 crate（21 段演示，见 `src/*.rs`）。

## 运行方式

```bash
# 完整任务演示（SITL）：起飞 → 巡航 → 发现并跟踪目标 → 降落 → 返回地面
# 随后演示 Fail-safe 看门狗在"大脑卡死"时强制进入自动悬停
cargo run -p brain-node

# 其余演示以 cfg 编译，启用对应模块即可运行
```

**配置加载**：依次尝试环境变量 `SMART_BRAIN_CONFIG` → `config.json` → `config.example.json` → 内置默认值。

## 主要演示（`src/`）

| 文件 | 内容 |
|------|------|
| `comprehensive_demo` | 任务加载→执行+进度上报→蜂群协同→MAVLink 命令下发 完整流水线 |
| `car_driving_demo` | 阿克曼汽车闭环自主驾驶 + `CarBody` 具身抽象 |
| `boat_demo` | 水面艇差分 DWA + 水流漂移 + 逆流定泊 |
| `parallel` | 多线程并行流水线（感知 + 决策共享 `DataBus`） |
| `async_runtime` | tokio 异步任务并发（`--features async`） |
| `swarm_coord_demo` | 蜂群 Leader 选举 + 任务分配 |
| `safety_guard` | 安全监督器（围栏/电量/pre-arm）接入任务循环 |
| `autopilot_demo` / `indoor` | 2D 世界自主探索/室内感知闭环 |
| `agent_demo` / `rag_demo` | Agent 工具调用 / 检索增强 |
| `time_sync_demo` | NTP 风格时间同步 |
| `zenoh_demo` / `zenoh_fcu_demo` | Zenoh Pub/Sub + Store/Query + Compute / 飞控链路 |
| `kalman_demo` / `stereo_demo` | VIO/卡尔曼融合 / 双目视觉 |
| `locomotion_sim_demo` | 步态 + 腿部 IK + 逆动力学仿真 |
| `embodiment` | 把无人机包装成 `RobotBody` 的适配示例 |

## 依赖

- 外部：`log`、`env_logger`、`serde_json`、可选 `tokio`（`async` feature）
- 内部：全部 workspace crate

## 特性

- 默认无 feature；`--features async` 启用 tokio 异步任务并发。
