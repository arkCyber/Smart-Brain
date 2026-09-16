# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

### Fixed
- **依赖安全（RustSec 审计红灯）**：升级 `rustls` 0.23.43→**0.23.45**
  （RUSTSEC-2026-0285：TLS 1.3 握手跨加密层校验缺陷）与 `serialport`
  4.10.0→**4.10.1**（原版本已被 crates.io yank）；新增 **`.cargo/audit.toml`**，
  对仅由可选 `real-zenoh` feature 间接引入、上游暂无修复的两条公告
  （RUSTSEC-2026-0041 `lz4_flex`、RUSTSEC-2023-0071 `rsa`）显式豁免并注明跟踪方式
  （评估记录见 `SECURITY.md`）。`cargo audit` 现在通过（仅剩 2 条未维护类提示）。
- **CI — 审计与测试失败难以定位**：`audit` job 改为直接运行 `cargo audit`
  （`taiki-e/install-action` 预编译安装）并把报告写入 **job summary**；CI 的
  `Run tests` 步骤改为 `tee` 到日志并在失败时把失败用例/panic 摘录写入 job summary，
  无需下载日志即可排查 Linux 与 macOS 的行为差异。
- **`brain-node` — 并行流水线测试存在竞态**：`parallel_pipeline_runs_all_threads`
  以 6ms 窗口断言“决策线程观测到感知线程发布的 Locked”，在共享/慢速 CI runner 上
  会因线程调度抖动偶发失败；改为 200 个 1ms 周期（≈200ms 预算）并在断言中输出计数，
  消除偶发失败（本地与 Linux 目标均验证）。
- **`brain-transport` — `can` feature 在 Linux 上无法编译（CI 长期红灯的根因）**：
  SocketCAN 后端按旧版 API 编写（`CanSocket::open`/`CanFrame::new`/`write_frame`/
  `read_frame`/`id()`），在 `socketcan` 3.x 上均不存在，且该模块被
  `cfg(all(feature = "can", target_os = "linux"))` 门控，macOS 本地构建看不到问题。
  现按 socketcan 3.x 修正：引入 `Socket`/`Frame`/`EmbeddedFrame` trait，
  发送用 `CanDataFrame::from_raw_id`，接收用 `raw_id()`（去掉 EFF/RTR/ERR 标志）
  与 `data()`；顺手消除 `clippy::while_let_loop`。
  验证：`cargo clippy -p brain-transport --features can --all-targets
  --target x86_64-unknown-linux-gnu -- -D warnings` 通过。
- **CI — `serial` feature 在 Linux 上缺少系统依赖**：`serialport` 通过 pkg-config
  探测系统 libudev，ubuntu runner 默认未安装 `libudev-dev`；已在
  `feature-backends` job 增加 `Install system dependencies` 步骤
  （`libudev-dev` + `pkg-config`；`socketcan` 3.x 为纯 Rust，无需系统库）。

### Added
- **项目工程化 / GitHub 标准化（面向开源发布）**：
  - 新增 **`rust-toolchain.toml`**（固定 stable + `rustfmt`/`clippy`），保证本地与 CI
    环境一致；新增 **`.editorconfig`**、**`.gitattributes`**（统一 LF、二进制与
    生成物标记），并扩充 **`.gitignore`**（覆盖率产物、IDE 文件、`.env`、模型权重等）。
  - 新增社区健康文件：**`CODE_OF_CONDUCT.md`**（Contributor Covenant 2.1）、
    **`SECURITY.md`**（漏洞报告流程 + 部署安全边界）、**`SUPPORT.md`**（获取帮助）、
    **`NOTICE`**（版权与第三方依赖许可）。
  - 新增 GitHub 模板与自动化：**Issue Forms**
    （`.github/ISSUE_TEMPLATE/{bug_report,feature_request}.yml` + `config.yml`）、
    **PR 模板**、**`CODEOWNERS`**、**Dependabot**（cargo + github-actions 每周检查）、
    **Security audit workflow**（RustSec 每周扫描）、
    **Release workflow**（打 `v*.*.*` tag 自动构建多平台二进制 + SHA256）。
  - **CI 增强**：新增 `docs` job（`RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps`）、
    **macOS（arm64）build+test job**（跨平台可移植性）、主 job 改用 `--locked` 与
    `clippy --all-targets`、最小权限 `permissions: contents: read` 与
    `concurrency` 取消旧运行。
  - **开发体验**：新增 **`Makefile`**（`make help`/`build`/`test`/`clippy`/`fmt`/
    `doc-check`/`check`/`demo`/`example`/`release`）与 **`.vscode/extensions.json`**
    （推荐扩展）。
  - **Cargo 元数据**：`[workspace.package]` 补齐 `authors`/`repository`/`homepage`/
    `keywords`/`categories`（修正原先的占位仓库地址），21 个 crate 全部继承并声明
    `readme`。
  - **文档体系**：新增 `docs/`（`ARCHITECTURE.md` 分层与依赖规则/关键 trait/数据流、
    `DEVELOPMENT.md` 环境与 feature 矩阵与 CI 门禁、`DEPLOYMENT.md` 四阶段真机路线与
    配置说明、`ROADMAP.md` 已完成/下一步、`RELEASE.md` 发布清单）；README 增加徽章、
    目录、环境要求、文档导航、贡献与联系、许可证章节（测试数量更新为 570）；
    CONTRIBUTING 重写为完整贡献流程。
  - **文档链接修复**：修正 rustdoc 报错（`brain-core::math`、`brain-odometry::buffer`、
    `brain-odometry::kalman`、`brain-kinematics::chain`、`brain-state::safety`、
    `brain-autopilot::sensor`、`brain-node::tree_builder`、`brain-sim::{lib,mock}` 中
    裸写的 `[0,1]` 等被误解析为 intra-doc link），使 `cargo doc -D warnings` 通过。
- **`brain-agent` / `brain-core` / `brain-node` — 模型后端工厂（应用框架装配）**：
  - `brain-agent`：新增 `factory::build_model(&cfg)`，按 `cfg.agent.backend`
    （`mock`/`ollama`/`hermes`）构造 `Box<dyn Model>`（feature 门控，未启用返回 `None`）；
    新增确定性离线后端 `EchoModel`（把最后一条用户消息原样作为答复，零配置可测）。
  - `brain-core`：新增 `AgentConfig { backend }`（默认 `mock`，serde 默认兼容旧配置），
    并入 `BrainConfig`；`validate` 校验非空。
  - `brain-node`：新增 `model_factory` 演示（按配置选后端跑一轮问答），`--demo model`。
  - 测试：factory 按后端构造/未知回退（含 feature 门控用例）、`EchoModel` 行为、
    `AgentConfig` 默认/往返/校验。brain-core 65→**66**、brain-agent 默认 20→**23**。
- **`brain-agent` / `brain-node` — 后端三轮回审计补全**：
  - `OllamaModel` / `HermesModel` 新增 `Debug` 实现（**鉴权密钥脱敏**，日志可观测）。
  - `HttpModel` 对“无 `tool_calls` 且无 `content`”的异常响应返回错误，与 Hermes 后端行为对齐
    （不再静默返回空答复）。
  - `brain-node` 的 `ollama_demo` / `hermes_demo` 现在**使用加载的配置**（`run(cfg)`），不再
    内部回退到默认配置，`config.json` 里的 `ollama`/`hermes` 设置生效。
  - 新增测试：两模型 `Debug` 脱敏、`HttpModel` 异常响应报错。
    brain-agent hermes 27→**28**、ollama 28→**29**、http-llm 24→**25**。
- **`brain-agent` / `brain-core` — Ollama / Hermes 后端二轮审计补全**：
  - `list_models`/`ping` 现在会带上鉴权头（Bearer），修复鉴权服务下探测被误判为“不可达”的问题
    （`HermesModel`、`OllamaModel` 一并修复）。
  - `HermesModel` 新增 `session_id`（绑定 Hermes 对话 lineage，含 `with_session_id` 与
    `HermesConfig.session_id`）；`generate` 对缺失 `content` 的异常响应返回错误而非静默空答复。
  - 新增测试：`list_models` 鉴权头（hermes/ollama）、Hermes `session_id` 请求、缺失 content 报错。
    brain-core 测试 64→**65**、brain-agent hermes 24→**27**、ollama 27→**28**。
- **`brain-agent` / `brain-core` / `brain-node` — 链接 Hermes 智能体 daemon（端口 11438）**：
  - `brain-core`：新增 `HermesConfig`（endpoint 默认 `http://127.0.0.1:11438`、model、
    temperature、max_tokens、timeout_secs、可选 `api_token`），并入 `BrainConfig`（serde 默认，
    兼容旧配置）；`validate` 增加参数自洽校验。
  - `brain-agent`：新增 `hermes` feature + `HermesModel`（桥接 Hermes daemon 的 **OpenAI 兼容
    `/v1/chat/completions`** 与 `/v1/models`；Hermes 内部完成 ReAct 工具调用、返回最终答复；
    支持 `HERMES_API_TOKEN` Bearer 鉴权、`from_config`、`list_models`/`ping`）。
  - `brain-node`：新增 `hermes` feature + `hermes_demo`（探测→列模型→指令答复），`--demo hermes`。
  - 测试：brain-core 配置校验 + brain-agent 离线单测（本地回环 HTTP 服务器模拟 `/v1/chat/completions`
    `/v1/models`，覆盖文本/请求形状/模型列表/鉴权头/探测）。CI 新增 `hermes` 后端 job。
- **`brain-agent` / `brain-core` / `brain-node` — 链接本地 Ollama 推理引擎（端口 11434）**：
  - `brain-core`：新增 `OllamaConfig`（endpoint 默认 `http://localhost:11434`、model、
    temperature、num_predict、timeout_secs、可选 `api_key`），并入 `BrainConfig`（serde 默认，
    兼容旧配置）；`validate` 增加参数自洽校验。
  - `brain-agent`：新增 `ollama` feature + `OllamaModel`（原生 `/api/chat`，含工具调用，
    `arguments` JSON 对象扁平化、`list_models`/`ping` 探测、`OLLAMA_API_KEY` Bearer 鉴权、
    `from_config`）；`ToolSchema` 上移到 `types` 供两后端复用；抽出共享 `role_str`。
  - `brain-node`：新增 `ollama` feature + `ollama_demo`（探测→列模型→工具调用闭环），
    `--demo ollama` 运行。
  - 测试：brain-core 配置校验 + brain-agent 离线单测（本地回环 HTTP 服务器模拟 `/api/chat`
    `/api/tags`，覆盖文本/工具调用/请求形状/鉴权头/模型列表）。CI 新增 `ollama` 后端 job。
  - 已在真实 Ollama（端口 11434）实测：连通性、列模型、工具注册、401 鉴权提示均正常。
- **`brain-middleware` / `brain-message` / `brain-transport` — 新代码二轮审计补全**：
  - `bus`：新增 `Topic::peek_message()`，在**单次加锁**内原子返回“值 + 时间戳”对，消除
    `peek`+`last_updated` 两次加锁间的错配；`subscribe` 文档化无界通道的慢消费权衡。
  - `frame`：新增 `max_pending()` 访问器，并明确 `with_max_pending` 需 ≥
    `MAX_FRAME_PAYLOAD + FRAME_OVERHEAD` 才能可靠重组最大帧的约束。
  - `can`：说明 `MAX_CAN_FRAMES` 防御性取值依据。新增测试（message 30→**31**、
    middleware 10→**11**）。
- **`brain-message` — 生产化补全（审计加固）**：
  - `frame::FrameReader` 增加**累积缓冲上界**（`MAX_FRAME_PAYLOAD + FRAME_OVERHEAD`），
    对“合法但巨大、负载迟迟不来”的退化输入自动裁剪重同步，杜绝无界内存增长（防 DoS）；
    新增 `with_max_pending`/`clear`；`verify_frame` 增加超限长度前缀防御。
  - `Mode`/`FixType` 增加 `Display`/`FromStr`/`name`/`from_name`（稳定名、大小写不敏感），
    便于日志/配置；`Telemetry` 实现 `Default`；`lib.rs` 导出 `FixType`。
  - `brain-message` 测试 24→**30**。
- **`brain-middleware` — 总线生产化（原子性 + 推送订阅）**：
  - `Topic` 将“最新值 + 时间戳 + 订阅者”收敛到**单把锁**，`publish`/`peek`/`last_updated`
    读写原子，消除“新数据配旧时间戳”的中间态；锁中毒统一用 `into_inner` 恢复（不丢消息）。
  - 新增**推送订阅**：`Topic::subscribe`/`DataBus::subscribe` 返回 `mpsc::Receiver<BusMessage<T>>`，
    发布即推送（含时间戳），订阅即收到当前保留值，Receiver Drop 后自动回收；新增 `BusMessage`、
    `subscriber_count`、`DataBus::remove`、`Topic` 的 `Debug`。
  - `brain-middleware` 测试 6→**10**。
- **`brain-transport` — 传输后端加固**：
  - `can`：新增纯函数 `ingest_telemetry_frame` 增量重组（**遇新首帧自动对齐**、超
    `MAX_CAN_FRAMES` 上限清空重来），`CanTransport` 改用之，避免分片跨消息混入/缓冲无界增长。
  - `udp`：新增 `local_addr`/`peer`/`set_peer`（本地/对端诊断与运行期重配）。
  - `open_transport` 支持 `"mavlink"` 内存桥后端。
  - `brain-transport` 测试 41→**46**。
- **`brain-ipc` — 环形缓冲测试补全（基础/线程安全）**：
  - `clear` 清空后可复用、空缓冲 `pop_oldest`/`get`/`iter` 安全、`get` 越界返回 `None`、
    **8 线程并发 push 不丢数据**（Mutex 保护）。`brain-ipc` 测试 6→**10**。
- **`brain-transport::can` — 重组鲁棒性修复**：
  - `decode_frames` 增加**分片序号连续性校验**（seq 必须为 1,2,3,…）：拒绝**重复段**
    （此前重复段可绕过长度检查、截断出损坏数据）与缺段，避免 CAN 丢帧/重发导致错误重组。
  - 新增 `decode_frames_rejects_duplicate_segment` 测试。`brain-transport` 测试 40→**41**。
- **`brain-transport` — 分帧/鲁棒性测试补全**：
  - `mavlink`：`decode_stream` 半包/粘包分帧、坏数据鲁棒性（非法 mode、未知 msgid、
    空/截断负载均返回 `Err` 而非 panic）。
  - `udp`：同一数据报多帧遥测的分帧解析。
  - `brain-transport` 测试 37→**40**。
- **`brain-autopilot` — 多目标协同避让（水面艇）**：
  - `Colregs::classify_many(own, targets, params)`：对多个目标船逐一分类，返回
    `Encounter`（目标下标 + 态势 + 动作）。
  - `Colregs::aggregate(&[Encounter])`：按优先级聚合多目标动作（任一让路 → 让路；
    否则受限能见度 → 安全航速；否则保向；否则减速；否则不动作）。
  - `ais::ais_to_vessel_pose`：把 AIS 位置报告换算为局部 `VesselPose`（COG 真北→
    本地 `atan2` 航向：`π/2 − COG`），打通"AIS → 多目标 COLREGS"链路。
  - 新增 3 项测试（多目标聚合让路、全无风险→不动作、AIS→VesselPose 位置/航向换算）。
    `brain-autopilot` 测试 46→**49**。
- **`brain-mapping` — 占据网格测试补全**：
  - `OccupancyGrid3D` 派生 `Debug`；新增 4 项测试（三态计数 `counts`、前沿探测
    `frontiers`、越界写入/读取返回 `false`/`None` 不 panic、`is_occupied`/`is_free`
    对未知与越界的语义）。`brain-mapping` 测试 7→**11**。
- **生产门槛对齐（clippy -D warnings + rustfmt）**：
  - 修复 `brain-locomotion::wbc`（`dynamic_torques` 加 `#[allow(too_many_arguments)]`、
    测试循环改用迭代器）、`brain-autopilot`（`ais` 模式区间 `1..=3` 与 `is_multiple_of`、
    `colregs` 合并相同分支）、示例 `ais_colregs`（`is_multiple_of`）。
  - 全工作区 `cargo clippy --all-targets -- -D warnings` 与 `cargo fmt --check` 通过，
    与 CI 门槛一致。
- **`brain-zenoh` — 生产化补全（模块功能审计与加固）**：
  - **键表达式增强**：`core` 支持单段通配 `*` 与任意深度通配 `**`（含 `**/gps` 之类
    前缀匹配）；新增 `valid_key_expr`（拒绝空段/部分通配 `sen*or`/保留字符 `. # + $`）
    与 `is_concrete_key`；`put`/`subscribe`/`get`/`declare_queryable` 均做键校验。
  - **深度上限**：`valid_key_expr` 限定段数 ≤ `MAX_KEY_DEPTH`（64），防止恶意深键/
    `**` 匹配指数爆炸与栈过深（`key_depth_is_capped`、`double_star_matches_deep_key`）。
  - **线性匹配**：`key_matches` 改用**动态规划**（`O(|query|×|stored|)`），彻底消除 `**`
    的指数回溯（`many_double_stars_still_linear`）。
  - **可观测性**：`LocalZenoh` 实现 `Debug`（输出存储/订阅/计算计数，`debug_reports_live_counts`）。
  - **死锁修复**：`LocalZenoh::get` 改为**锁外执行用户 `QueryHandler`**（先持锁收集
    处理器再释放锁调用），杜绝 handler 内回调本后端造成的重入死锁。
  - **panic 隔离**：`get` 用 `catch_unwind` 包裹 handler 执行，单个计算/服务异常被
    日志化并跳过，不击穿进程。
  - **错误表达**：`QueryHandler` 返回 `Result<Vec<Value>>`，handler 可显式报告计算
    失败；`get`/真实 zenoh 回调对 `Err` 记录日志并跳过应答（新增 `failing_handler_is_skipped`
    测试）。
  - **`Clone`**：`LocalZenoh` 支持克隆，克隆体共享同一份存储/订阅。
  - **订阅退订**：`Subscription` 新增 `unsubscribe()`（幂等，`Drop` 自动调用）；
    `LocalZenoh` 按订阅者 id 管理并支持通配订阅退订。
  - **存储管理**：新增 `value`/`contains`/`remove`/`clear`；`store_count` 只统计有值
    条目；新增 `subscription_count`/`queryable_count` 监控；空条目自动 `prune` 防无界增长。
  - **错误类型扩展**：`LocalZenohError` 新增 `InvalidKey`。
  - **真实 zenoh 后端**：`zenoh_impl` 订阅样本改用真实时间戳（`Timestamp::get_time()`
    读取，而非恒 0）。
  - 新增 17 项测试（键校验、非法表达式拒绝、`**` 聚合、退订、存储管理、**handler 重入
    不死锁**、**handler 失败/panic 隔离**、`Clone` 共享、深度上限、`Debug`、DP 多 `**`）。
    `brain-zenoh` 测试 14→**31**。
- **`brain-node` — 主程序生产化（CLI + 退出码 + 测试）**：
  - 新增**命令行接口**（无第三方依赖）：`--demo <name>` 单演示运行、`--list` 枚举、
    `--config <path>`（优先级高于 `$SMART_BRAIN_CONFIG`）、`--iterations <n>`、`--help`/`--version`。
  - **退出码**：`0` 成功 / `1` 配置无效 / `2` 用法错误或未知演示；配置无效不再 panic。
  - **死代码清理**：移除仅被 `let _ = &mission` 引用的未用 `Mission`/`Waypoint` 构造。
  - 新增 4 项 CLI 单元测试（默认值、标志/取值解析、非法输入拒绝、演示名唯一）。
    `brain-node` 测试 9→**13**。
- **`brain-autopilot::ais` — AIS 报文解析器**（水面艇"多艇会遇感知"输入层）：
  - 解析 NMEA `!AIVDM/!AIVDO` 6-bit 负载，解码类型 1/2/3（A 类位置报告）、
    18（B 类位置报告）、5（静态/航次数据：船名/呼号/船型/IMO/尺寸）。
  - 提供 `to_local_offset`（经纬度 → 局部东/北米偏移）与 `AisMessage::position()`，
    可直接喂给 COLREGS。错误类型 `AisError`（非 AIS 语句/多句分片/非法字符/截断）。
  - 11 项单元测试（6-bit 字母表、MSB 位读取、类型 1/18 往返、负经纬度、不可用哨兵、
    分片拒绝、截断、局部偏移）。`brain-autopilot` 测试 35→**46**。
- **`brain-autopilot::colregs` — 完整 COLREGS 扩展**：
  - 新增 `Visibility`（能见度受限 Rule 19：双方均须主动让路、安全航速）与
    `Propulsion`（机动船交叉相遇时让帆船，帆船优先通行权）。
  - `VesselPose` 增加动力类型；新增动作 `ProceedSafeSpeed` 与态势 `RestrictedVisibility`。
  - 新增 4 项测试（受限能见度让路/安全航速、机动船让左舷帆船、机动船对机动船保向）。
- **`brain-locomotion::contact` — 足-地接触动力学模型**：
  - 弹簧-阻尼地面反力 `F = k·x − c·ẋ`（钳制 ≥0 只推不拉），输出接触状态/穿透/法向力。
  - 5 项单元测试（离地无接触、刚度力、静平衡=体重、下压增力、上抬不粘附）。
- **`brain-locomotion::wbc::dynamic_torques` — 全身动力学力矩整合**：
  - 在既有静力学 `joint_torques` 之上，新增 `WholeBodyController::dynamic_torques`：
    接触门控（`ContactModel` 判定支撑相）+ 逐腿逆动力学（`LegDynamics::inverse_dynamics`）
    计入惯性/科氏/重力；新增 `LegDynamics::from_links` 与 `with_dynamics` 覆盖默认腿参数。
  - 3 项整合测试（静平衡退化为 `Jᵀf`、摆动腿无负载、长度不一致报错）。
    `brain-locomotion` 测试 49→**59**。
- 新增示例：`brain-autopilot/examples/ais_colregs.rs`（AIS→局部坐标→受限能见度避让）、
  `brain-locomotion/examples/dynamic_contact.rs`（接触+全身动力学力矩）。
  工作区总测试 **469→507**。

- **`brain-locomotion::wbc` — 全身控制（Whole-Body Control, WBC）完成**（补全 README
  此前标注的“全身 WBC 命令”缺口）：
  - `WholeBodyCommand`：期望躯干位姿（高度 + roll/pitch/yaw + 重心横向偏移 COM）+
    逐足体重分配（`force_weights`，空 = 均匀分配）。
  - `WholeBodyController`：把躯干位姿命令经 `Pose::inverse_transform_point`（世界→机体）
    换算成各足期望机体坐标，再经既有 `LegIK` 求 `[髋, 膝]` 关节角，并按权重把体重
    （`total_weight`）分摊为逐足法向力；输出 `WholeBodyTarget`（关节角 + 足端位置 +
    足底力）。
  - 健壮性校验：拒绝 NaN/Inf 输入、髋/足/力权重数量不一致、IK 不可达、
    权重非负且总和 > 0，错误类型 `WbcError`（`Invalid`/`LegCountMismatch`/`Unreachable`）
    实现 `std::error::Error`。
  - 16 项单元测试（恒等命令回名义足端、IK 正解一致、升降/俯仰/横滚/偏航/COM 偏移的
    几何、不可达、非有限输入、均匀/加权/非法力分配、构造器校验、力合力=体重、与
    `Quat` 独立交叉验证）。`brain-locomotion` 测试 20→**36**；工作区总测试 **449→465**。
  - `brain-node::locomotion_sim_demo` 新增 WBC 站姿命令演示（逐腿关节角 + 足底力，
    力合力=体重）。
- **`brain-perception::InferenceOutput::try_at` — 软失效的越界访问**：
  - 新增 `try_at(row, col) -> Result<f32>`：越界（行/列超出 `rows`/`cols`、索引超出
    底层数据，或 `row*cols` 发生 `usize` 溢出）时返回 [`BrainError::Inference`] 而非
    panic，便于推理下游把坏数据当作可恢复失败处理（告警/丢帧/降级）。
  - 既有 `at()` 改为委托 `try_at` 并在出错时 panic，保留“快速失败”行为不变。
  - 新增 4 项测试（有效访问与 `at` 一致、形状越界软失效、数据越界、索引溢出）。
    `brain-perception` 测试 18→**22**；工作区总测试 **465→469**。
- **`brain-locomotion` — 全身动力学（静力学）下沉**（补全 README 上一轮标注的
  “全身动力学：足端力 → 关节力矩”）：
  - `LegIK::jacobian`（几何雅可比 `J`）与 `LegIK::static_torques`（`τ = Jᵀ·f`），
    把平面双连杆腿的足端力映射为 `[髋, 膝]` 关节力矩。
  - `WholeBodyTarget::trunk_wrench`：聚合逐足力为躯干**净合力**与**净合力矩**
    （`Σ F`、`Σ r×F`），用于静态平衡/防倾覆判定。
  - `WholeBodyController::joint_torques`：把一次 WBC 求解的逐足力下沉为各腿关节力矩
    （含长度一致性校验）。
  - 新增 7 项测试（`brain-locomotion` 测试 36→**43**，工作区总测试 **469→476**）：
    `LegIK` 雅可比与有限差分交叉验证、水平腿下坠力力臂校验、虚功恒等式 `τ·δq = f·δp`；
    WBC 对称站姿合力矩≈0、非对称载荷产生倾覆矩、关节力矩与单腿静力一致、长度不匹配报错。
  - `brain-node::locomotion_sim_demo` 输出关节力矩与躯干合力矩。
- **`brain-locomotion::dynamics` — 逆动力学（Recursive Newton-Euler, RNEA）**（补全
  README 上一轮标注的“动态逆动力学：含惯量/加速度”）：
  - `LegDynamics`：平面双连杆腿的质量/惯量参数（`l`/`m`/`rc`/`I`）+ `cm_world`
    （质心位置）+ `inverse_dynamics(q, q̇, q̈, 足端外力, 重力)`，用平面 RNEA 求
    `[髋, 膝]` 关节力矩（含惯量、科氏/离心、重力）。
  - 用**六重独立校验**锁定正确性：①无重力无运动时严格等于 `static_torques`；
    ②静态重力矩与“质心雅可比转置”一致；③与解析平面 2R 质量矩阵一致；
    ④与“解析质心加速度 + 虚功投影”一致；⑤无重力下 `τ·q̇ = dKE/dt`（解析 M）；
    ⑥有重力下 `τ·q̇ = dKE/dt + dPE/dt`。
  - 新增 6 项测试（`brain-locomotion` 测试 43→**49**，工作区总测试 **476→482**）。
  - `brain-node::locomotion_sim_demo` 新增 RNEA 逆动力学演示（输出 τ_hip/τ_knee）。
- **`brain-locomotion::dynamics` — 正向动力学（仿真）**（补全 README 上一轮标注的
  “正向动力学/仿真：给定力矩求运动”）：
  - `LegDynamics::forward_dynamics(q, q̇, τ, 足端外力, 重力) -> [q̈0, q̈1]`：`inverse_dynamics`
    的逆运算——用其提取质量矩阵 `M(q)`（q̈ 取单位向量）与偏置项 `b = C·q̇+g+Jᵀf`，
    再解析求解 `M·q̈ = τ − b`。
  - 新增 2 项测试（`brain-locomotion` 测试 49→**51**，工作区总测试 **482→484**）：
    正逆回环（`forward(inverse(·)) == q̈`，多状态含重力/外力）、无重力无外力零力矩下
    自由运动回代逆动力学应为零力矩（能量守恒）。
  - `brain-node::locomotion_sim_demo` 新增正向动力学 + 欧拉积分回环演示。
- **`brain-core` completion pass on the newest additions**:
  - `Vec3`: added `Div<f32>`, `Neg`, scalar-left `Mul<Vec3> for f32` (i.e. `s * v`), and a
    named `component_mul` (Hadamard product).
  - `Pose`: added `from_rotation(q)` (translation-zero pose).
  - Edge-case tests for the new math: `to_euler` gimbal-lock (±90° pitch, no panic),
    `angle_to` at 0 and π, `slerp` for identical inputs and shortest-path (negative dot),
    the new `Vec3` operators, and `Pose::from_rotation`. `brain-core` total **61**;
    workspace total unit tests now **449**.
- **`brain-core` audit & completion** (`math` / `config`):
  - `math::Quat`: added `to_euler` (inverse of `from_euler`, reads a rotation back as
    roll/pitch/yaw), `dot`, `angle_to` (returns the actual rotation angle, 0..=π), and
    `slerp` (shortest-path spherical interpolation).
  - `math::Vec3`: added `lerp`, `distance`, `is_finite` (NaN/inf robustness).
  - `math::Pose`: added `transform_direction` (rotate only, no translate) and
    `inverse_transform_point` (world→local).
  - `config`: `FcuConfig` now has an independent `Default`; `BrainConfig::validate`
    additionally rejects a `serial` transport with a zero `baud_rate`.
  - 7 new tests (`brain-core` total **56**); workspace total unit tests now **444**.
- **Audit & completion pass on the newly added generic/sensor code**:
  - `brain-state::Fsm`: added `transition_to` (returns new state), `next` (reachable
    targets from a state), `reset`; `RobotState` gains `from_name`/`Display`/`FromStr`
    (round-trip with `as_str`); `RobotStateMachine` forwards the new `Fsm` methods.
    New tests: convenience methods, Display/parse round-trip, and an ALLOWED-table
    consistency check (no dead/unreachable states). Total unit tests now **437**.
  - `brain-message::RangeScan`: added bounds-checked `beam_range` and `clamped_ranges`;
    removed a dead `#[allow(clippy::too_many_arguments)]`; added a JSON shape-stability
    test to lock the wire format.
  - `brain-node::generic_demo`: demonstrates reading the `state/robot` topic back and
    parsing it via `RobotState::from_name`.
- **Generic (body-agnostic) state machine in `brain-state`** (`robot_state` module):
  - Reusable `Fsm<S>` finite-state machine (only pre-defined transitions, illegal ones
    rejected) — genuinely generic over any state enum.
  - `RobotState` (Standby/Starting/Active/Paused/Tracking/Returning/Fault/EmergencyStop/
    PowerOff) + `RobotStateMachine` with a body-agnostic transition table including
    safety paths (`EmergencyStop`/`Fault`). Complements the flight-specific
    `FlightState` (kept intact). 5 new tests; `brain-state` total 26.
- **Generic sensor messages in `brain-message`** (`sensor` module, serde-serializable for
  bus/transport): `ImuSample`, `OdometrySample` (pose+velocity, re-exports `Quat`),
  `RangeScan`, `ContactSample`. 4 new tests; `brain-message` total 21.
- **Generic topics in `brain-middleware`**: `sensor/imu`, `sensor/odometry`,
  `sensor/range`, `sensor/contact`, `state/robot` constants + a typed bus test.
- **`brain-node::generic_demo`**: demonstrates publishing/reading the sensor topics on a
  `DataBus` and driving `RobotStateMachine` through a valid chain while rejecting an
  illegal transition. Total unit tests now **432**.
- **Stereo vision (双目立体视觉) in `brain-odometry`** — two cameras obtain spatial depth:
  - `StereoCamera` (rectified stereo model: shared intrinsics + baseline) with
    `disparity_to_depth` (`z = f·b/d`) and `triangulate` (pixel + disparity → 3D point).
  - `compute_disparity` — real **SAD block-matching** along horizontal epipolar lines,
    plus `disparity_to_pointcloud` and a full `process_stereo` pipeline producing a
    `StereoResult{disparity, point_cloud}`.
  - 5 self-contained tests (synthetic textured-plane rendering): triangulation math,
    depth formula, block-matching recovers known disparity, full pipeline yields a
    point cloud at ground-truth depth, near/far depth ordering. `brain-odometry` total 36.
  - `brain-node::stereo_demo` runs in the main demo: two rendered depth planes (0.5 m /
    2.0 m) → recovered depth ≈ 0.51 m / 2.00 m.
- **Sensor layer enhancements** (`brain-odometry` / `brain-autopilot`):
  - `brain-odometry::sensor`:
    - `PinholeCamera::from_fov` + `fov_x`/`fov_y`/`aspect_ratio`/`contains` (bounds check).
    - `DepthFrame::new` + bounds-checked `depth_at`/`depth_at_index` (no out-of-bounds index).
    - `depth_to_pointcloud` is now robust to short depth buffers, plus a subsampled
      `depth_to_pointcloud_subsampled` (drops points for high-res depth frames).
    - Point-cloud helpers `cloud_centroid` / `cloud_transform`.
    - 5 new tests (sensor module 5 tests; `brain-odometry` total 31).
  - `brain-autopilot::sensor::RangeSensor`:
    - FOV support (`with_fov`, defaults to full 360° — existing behavior preserved),
      deterministic measurement noise (`with_noise`, reproducible via fixed seed),
      single-ray `ray_range`, raw `scan_ranges`, and `fov`/`noise` accessors.
    - Backward-compatible `scan()` (navigation integration unchanged; all
      `car_autopilot`/`boat_autopilot`/`autopilot` tests still pass).
    - 6 new tests (sensor module 7 tests; `brain-autopilot` total 31).
  - Total unit tests now **417**.

  bridges the `Model` trait to any **OpenAI-compatible** `/chat/completions` endpoint
  (OpenAI / DeepSeek / Qwen / Ollama / vLLM / LM Studio…) via a synchronous `ureq`
  client. Parses `content` (final answer) and `tool_calls` (requested tool), and
  exposes `ToolSchema` for registering OpenAI-style `{type, function}` tool
  definitions. Offline-tested against a local loopback HTTP server (text reply,
  tool-call parsing, role/history/model serialization, tools+`tool_choice`, schema
  parameters). 5 new tests (brain-agent total 24 with the feature). Matches the
  existing "offline mock by default, real backend behind a feature" policy of
  `OnnxModelBackend`/`ZenohBackend`.
- **Clippy-clean `real-zenoh` feature**: introduced a `QueryableStorage` type alias
  for `Arc<Mutex<Vec<Option<Box<dyn Any + Send>>>>>` so `cargo clippy
  -p brain-zenoh --features real-zenoh --all-targets -- -D warnings` passes. This
  previously failed, but the CI job only ran `cargo build` so it was silently masked.
- **CI hardening**:
  - `real-zenoh` job now also runs `clippy -- -D warnings` and offline lib tests
    (previously build-only, which hid the type-complexity warning).
  - `feature-backends` job now lints + tests `brain-agent --features http-llm`.
- **Audit findings documented**: the project was already a complete, runnable,
  405-test workspace (build clean, all optional features `onnx`/`can`/`serial`/
  `real-zenoh`/`async`/`http-llm` compile). Note: `cargo test --workspace
  --all-features` requires `libonnxruntime` to be installed (a pre-existing
  environment dependency; CI tests feature crates per-crate instead).

### Added
- **Real `shutdown()` implementations** (replaces the previous empty no-op bodies
  on transports & robot bodies):
  - `UdpTransport`/`SerialTransport`/`CanTransport` now hold their OS handle in an
    `Option`; `shutdown()` **releases it synchronously** (drops the UDP socket /
    flushes+closes the UART port / drops the SocketCAN socket) and clears internal
    buffers. Sends after shutdown return `"…closed"` errors.
  - `MavLinkTransport`/`MockTransport` clear their send/recv buffers on shutdown.
  - `ZenohFcuTransport`/`MockFcuZenoh` release their subscription channel.
  - `CarBody`/`BoatBody` `shutdown()` implements a real "power-off": clears the
    goal and stops motion; `MockRobotBody` resets to initial state.
  - Trait-default `shutdown()`/`Node::reset()` kept as documented default hooks.
  - Total unit tests now **405**.

### Added
- **Coverage-driven test expansion** (every public function now has a test;
  library crates reach **95.8%** line/region coverage via `cargo llvm-cov`):
  - `brain-transport::lib` — `open_transport` (mock/udp/serial-feature/unknown),
    trait-object roundtrip, default `shutdown`.
  - `brain-mission::swarm` — role serde, `broadcast`/`ingest` (64-cap)/`peers`/
    `any_peer_target_seen`; `mission` — validate error branches, distance metrics,
    `abort`/`index`/`current_target` down-mapping, save/load error paths.
  - `brain-robot::state` (all `as_str` variants + `BasePose::new`/`BodyState::new`/
    serde), `car` (`pose`/`drive`/`kind`/`read_state` joints+angular vel/`Reach`),
    `boat` (same + `Hold`/thrusters/rudder).
  - `brain-core::math` (Vec3 ops/dot/cross/norm/normalized, Quat euler/mul/inverse,
    Pose transform/operator), `brain-odometry::buffer` (`align_to`, past-last,
    scalar lerp), `brain-nav::backtrack` (reset/trail/rewind-to-home/empty),
    `brain-transport::udp` (invalid bind, empty recv, send), `serial_backend`
    (non-feature paths), `brain-state::failsafe` (rearm cycle, re-trip) + `safety`
    (defaults, custom pre-arm, `classify`, RTH branch), `brain-agent::tool`
    (name/description, empty, error propagation), `zenoh_fcu` (none, no-command,
    telemetry accessor, Land branch).
  - Total unit tests now **399**.
- **Leg IK end-to-end wiring in `brain-locomotion`**:
  - New `leg_ik` module: planar 2-link `LegIK` (`solve`/`forward`/
    `solve_from_hip`) with reachability `IkError`, plus 6 unit tests (round-trip,
    fully-extended, unreachable far/near, hip-relative solve).
  - `LocomotionController`/`LocomotionOutput` now emit **absolute `foot_targets`**
    (body frame) alongside `foot_offsets`, with a `with_foot_positions` builder and
    a `foot_targets_are_ik_solvable` end-to-end test.
  - `brain-node` `locomotion_sim_demo` now solves gait foot targets → leg IK →
    per-leg `[hip, knee]` joint angles, printing them each interval.
  - Total unit tests now 334.
- **Two new crates: `brain-locomotion` & `brain-sim`** (functional additions on
  top of existing hardening):
  - `brain-locomotion` — gait & whole-body locomotion layer for legged robots:
    `GaitConfig`/`GaitGenerator`/`GaitPhase` (Stand/Walk/Trot/Run, quadruped
    phase tables, biped anti-phase), `FootTrajectory` (stance back-sweep + sine
    swing-lift), and `LocomotionController` (velocity → per-leg foot targets +
    body pitch/height/turn). 13 unit tests.
  - `brain-sim` — pluggable `Simulator` backend contract (step/state/velocity
    command/range/detection/reset) plus a deterministic in-process
    `MockSimulator` (2D grid world, speed-limited velocity integration, ray-cast
    range sensor, target detection, collision counter). 8 unit tests.
  - Both registered in the workspace and wired into a `brain-node`
    `locomotion_sim_demo` that drives a quadruped forward through a mock world
    (range sensing + detection) as a closed loop.
  - Total unit tests now 327.
- **More production-hardening & test coverage (round 2)**:
  - `brain-zenoh::backend` tests for `Subscription` (`recv`/`try_recv`/
    `recv_timeout`/`key`), `QueryableHandle` (explicit `unregister` + `Drop`
    auto-unregister), and `Sample`/`Reply` constructors.
  - `brain-planning::rrt` hardens nearest-node selection (`min_by` now `?` instead
    of `.unwrap()`) so a hypothetical empty node set returns `None` instead of
    panicking.
  - Total unit tests now 327.
- **Production-hardening & test coverage**:
  - `brain-behavior-tree` gained its first unit tests (21): composite nodes
    (`Sequence`/`Selector`), decorators (`Inverter`/`Retry`), and all flight nodes
    (`Takeoff`/`Cruise`/`Land`/`BatteryCheck`/`GpsFixCheck`/`DetectTarget`/
    `TrackTarget`/`ReturnHome`/`Failsafe`/`LogNode`).
  - `brain-message` serde round-trip + JSON shape-stability tests for
    `Command`/`CommandTarget`/`Mode`/`WaypointCommand`/`Detection`/`TrackingStatus`
    and `Telemetry`/`Attitude`/`GpsFix`/`FixType`/`BatteryStatus`.
  - `brain-perception::backend` tests for `MockModelBackend` (load-gate) and
    `InferenceOutput` bounds; `brain-agent::types` tests for `Message`/`ToolCall`.
  - `cargo fmt --check` now part of the standard pre-commit discipline.

### Changed
- **Panic-proofing on safety-critical paths**:
  - `brain-core::time::TimeSync` recovers from mutex poisoning (`lock().unwrap()` →
    `unwrap_or_else(|p| p.into_inner())` via a `window()` helper) so a one-off
    panic can never permanently cripple the fail-safe/watchdog path.
  - `brain-zenoh::RealZenoh` queryable registration likewise recovers from
    poisoning instead of panicking.
  - `brain-odometry::TimestampAligned::align` uses safe `Option` access instead of
    raw indexing, degrading to `AlignmentError::InsufficientData` on any invariant
    edge rather than panicking.
  - `brain-perception::InferenceOutput::at` bounds-checks and panics with an
    explicit message on out-of-range access (fail fast, never silent bad data).
- README test count updated (262 → 301).

### Added (from prior hardening batch)
- **Car driving navigation**: Ackermann/bicycle kinematics, Ackermann DWA local
  avoidance, `CarBody`, `CarAutopilot` (closed-loop with reverse-parking via
  Reeds-Shepp final approach).
- **Path planning**: Dubins curves (forward-only) and Reeds-Shepp curves
  (forward/reverse, parking & U-turn) with simulation-validated candidates.
- **Surface vessel (ASV/USV)**: `RobotKind::SurfaceVessel`, `BoatBody`,
  `BoatAutopilot` (differential twin-thruster, water current + tide,
  multi-waypoint cruise, station-keeping / dynamic positioning), and a COLREGS
  encounter-avoidance rule engine.
- **Embodied closed loop**: `CarAutopilot::current_command()` drives a `CarBody`
  with the low-level Ackermann command (0.0m sync error).
- **Config file loading**: `BrainConfig::from_file` / `load_candidates`, and
  `brain-node` now loads config from `$SMART_BRAIN_CONFIG` → `config.json` →
  `config.example.json` → built-in defaults, honoring `node_id`,
  `failsafe_timeout_ms`, etc. Extended `validate()` (zero tick period, serial/udp
  field requirements).
- **Unit tests for previously-untested critical modules**: `MockTransport`
  (send/recv semantics), `VisionPipeline` (lock acquisition / loss / bus publish),
  `MockRobotBody` (navigate / stop / grasp / joints / per-kind body shape), and
  `RobotBody` trait-object safety.
- **Real ONNX backend** (`brain-perception`): `OnnxModelBackend` now wires `ort`
  (ONNX Runtime, `load-dynamic`) with **YOLOv8 decode + class-aware NMS** (`nms`
  module, pure functions with unit tests). Enabled via `--features onnx`.
- **CAN transport backend** (`brain-transport`): `CanTransport` (Linux SocketCAN,
  `--features can`) + pure CAN frame codec (`encode_frames`/`decode_frames`,
  Command/Telemetry round-trip) with offline tests.
- **Safety module** (`brain-state::safety`): geofence (radius/altitude), battery
  monitor (low/RTH/critical), pre-arm self-check, and a unified
  `flight_permission` verdict.
- **Parallel pipeline** (`brain-node::parallel`): multithreaded perception + decision
  threads sharing a thread-safe `DataBus`, demonstrating cross-thread data flow.
- **Safety integration** (`brain-node::safety_guard`): a `SafetySupervisor` wires the
  geofence/battery/pre-arm safety module into the mission decision loop, overriding
  commands to ReturnHome/Land when boundaries are violated.
- **tokio async runtime** (`brain-node::async_runtime`, `--features async`): perception
  and decision run as async tasks on a single tokio runtime, sharing the thread-safe
  `DataBus`; includes a `#[tokio::test]`.
- **ONNX into pipeline** (`brain-perception`): `VisionPipeline::onnx(config, nc)`
  factory builds a real ONNX-backed pipeline.
- **Swarm coordination** (`brain-mission::swarm_coord`): deterministic `LeaderElection`
  (lowest node_id / highest battery) and `TaskAllocator` (round-robin / by priority),
  with `SwarmCoordinator::plan` distributing tasks from the elected leader; demo in
  `brain-node::swarm_coord_demo`.
- **Configurable safety**: `BrainConfig` gains a `safety` section (geofence radius/
  altitude, battery RTH/critical/low thresholds, pre-arm GPS/battery/home), wired into
  `SafetySupervisor`; `brain-node` now honors `fcu.transport` to select the real
  transport backend (falling back to mock).
- **Robustness**: fixed a latent panic in `brain-nav::Backtracker::record` when
  `max == 0`; removed dead-code stubs in `safety`/`pipeline`.
- **Foundation modules**:
  - `brain-core::time`: added injectable `Clock` trait (`SystemClock`/`ManualClock`)
    and a truly-monotonic `Stopwatch` (wall-clock `instant_now` was not monotonic).
  - `brain-core::time`: added **NTP-style time synchronization** — `SyncSample`
    (four-timestamp offset/RTT computation), `TimeSync` (windowed **median**
    filter that rejects RTT outliers to converge on a stable clock offset),
    and `SyncedClock` (a `Clock` that reports reference time). All exposed via
    `brain-core` re-exports; offline unit-tested (offset correctness under
    symmetric/asymmetric delay, outlier rejection, negative-offset clamping,
    cross-thread sharing) and demonstrated in `brain-node::time_sync_demo`.
  - `brain-core::time`: completed the time-sync feature with a **health layer**
    (`TimeSync` gains `min_samples`/`is_synced`/`estimate_offset`/`last_update`/
    `age`/`is_stale` for watchdog-style freshness checks) and a transport-agnostic
    **handshake driver** `SyncDriver` + `SyncExchange` trait that automates
    multi-round sync and propagates link errors. Added tests: serde round-trip,
    sync-health/staleness transitions, driver end-to-end convergence, premature-stop
    at min samples, RTT-rejection reporting, and exchange-error propagation.
    `brain-node::time_sync_demo` now drives the sync via `SyncDriver`.
  - `brain-message::frame`: CRC-16/CCITT known-answer test and a max-payload guard
    in `FrameReader` so a bogus length prefix cannot stall the buffer (desync).
  - `brain-ipc::ring`: `FixedRingBuffer::iter()` + `SharedRing` `get`/`capacity`/
    `is_full`/`iter`.
  - `brain-state`/`brain-middleware`: extra unit tests (state `flight_mode`/`is_safe`,
    DataBus multi-topic & shared-topic semantics).
  - `brain-node` uses `Stopwatch` to report total demo runtime.
- CI workflow, LICENSE (Apache-2.0), CONTRIBUTING guide.

### Changed
- Cargo.lock is now committed for reproducible builds.
- Added `RobotKind::SurfaceVessel`, `Car`, and `BoatBody`/`CarBody` to the
  body-agnostic abstraction.
- **Clippy-clean across the workspace**: fixed `should_implement_trait`,
  `needless_range_loop`, `matches!`, `len_without_is_empty`,
  `field_reassign_with_default`, `too_many_arguments`, `manual_is_multiple_of`,
  `single_match`, and doc-comment lint issues so both
  `cargo clippy -- -D warnings` (CI) and `--all-targets` pass with zero warnings.
- **Robustness**: `Vec3`/`Quat` now expose both operator impls (`+`/`-`/`*`) and
  convenience methods; `brain-core` gains a `log` dependency for config diagnostics.
- `.gitignore` now excludes user-generated `config.json`.


## [0.1.0] - Initial

- Workspace skeleton ("AI 大脑") with drone-focused SITL loop, telemetry/command
  framing, state machine + failsafe watchdog, perception pipeline, behavior
  tree, mission/swarm, Zenoh transport, agent + RAG, VIO/odometry, indoor
  mapping/A*/DWA navigation.
