# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

### Added
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
