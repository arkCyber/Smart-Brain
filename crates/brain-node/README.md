# brain-node

> Smart-Brain main executable: assembles and drives all modules

**所属层**：装配层 —— 主程序，把各层串成闭环。

## 职责

工作区的**可执行入口**：装配 `brain-core`…`brain-sim` 等全部模块，驱动一次完整任务演示与 fail-safe 演示，并展示 `RobotBody` 具身抽象。也是**应用案例最丰富**的 crate（21 段演示，见 `src/*.rs`）。

## 运行方式

```bash
# 默认：依序运行全部演示（SITL），最后打印总耗时
cargo run -p brain-node

# 只运行某个演示（见 --list）
cargo run -p brain-node -- --demo failsafe
cargo run -p brain-node -- --demo mission --iterations 30

# 列出可用演示 / 帮助 / 版本
cargo run -p brain-node -- --list
cargo run -p brain-node -- --help
cargo run -p brain-node -- --version

# 显式指定配置文件（优先于 $SMART_BRAIN_CONFIG 与 config.json）
cargo run -p brain-node -- --config /path/to/config.json

# tokio 异步流水线（需编译时启用 feature）
cargo run -p brain-node --features async -- --demo async
```

**CLI 参数**：

| 参数 | 说明 |
|------|------|
| `--demo <name>` | 只运行指定演示（见 `--list`）；未知名称退出码 2 |
| `--list` | 列出所有可用演示并退出 |
| `--config <path>` | 配置文件路径（优先级：`--config` > `$SMART_BRAIN_CONFIG` > `config.json` > `config.example.json` > 内置默认） |
| `--iterations <n>` | mission 演示的 tick 数（默认 30，必须 > 0） |
| `-h, --help` | 打印帮助 |
| `-V, --version` | 打印版本 |

**退出码**：`0` 成功；`1` 配置无效；`2` 用法错误/未知演示。

**配置加载**：依次尝试 `--config` → 环境变量 `SMART_BRAIN_CONFIG` → `config.json` → `config.example.json` → 内置默认值。

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
