# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

### Added
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
