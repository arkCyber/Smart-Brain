# Smart-Brain — AI Brain for Autonomous Robots

**English** | [简体中文](README.zh-CN.md)

[![CI](https://github.com/arkCyber/Smart-Brain/actions/workflows/ci.yml/badge.svg)](https://github.com/arkCyber/Smart-Brain/actions/workflows/ci.yml)
[![Security audit](https://github.com/arkCyber/Smart-Brain/actions/workflows/audit.yml/badge.svg)](https://github.com/arkCyber/Smart-Brain/actions/workflows/audit.yml)
[![Release](https://github.com/arkCyber/Smart-Brain/actions/workflows/release.yml/badge.svg)](https://github.com/arkCyber/Smart-Brain/releases)
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Rust](https://img.shields.io/badge/rust-stable-orange.svg?logo=rust)](rust-toolchain.toml)
[![Tests](https://img.shields.io/badge/tests-570%20passing-brightgreen.svg)](#testing--quality-gates)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)
[![Conventional Commits](https://img.shields.io/badge/commits-conventional-fe5196.svg)](https://www.conventionalcommits.org/)

An **AI brain** (mission computer) for autonomous robots — drones, cars, surface
vessels and legged robots — written in **Rust** and organized as a **cargo
workspace**. It follows a *software/hardware-decoupled, safety-isolated,
heterogeneous-compute* architecture: the high-rate flight controller
(the **cerebellum**) and the high-compute AI decision layer (the **cerebrum**)
are separated by an explicit interface instead of sharing a processor.

> **Runs out of the box in SITL** (mock flight controller): `cargo build` needs no
> system dependencies, and the complete mission loop is covered by **570+ offline
> unit tests**. Every hardware/network backend (serial, CAN, ONNX Runtime,
> Ollama/Hermes/HTTP LLMs, real Zenoh, tokio) is **feature-gated** and off by default.

## Table of contents

- [Highlights](#highlights)
- [Architecture](#architecture)
- [Quick start](#quick-start)
- [Workspace layout](#workspace-layout)
- [Feature flags](#feature-flags)
- [Testing & quality gates](#testing--quality-gates)
- [Deployment path](#deployment-path)
- [Roadmap](#roadmap)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [Security](#security)
- [License & disclaimer](#license--disclaimer)

## Highlights

- 🧠 **21-crate layered workspace** — perception, mapping, planning (A\*, RRT, DWA,
  Ackermann-DWA, Dubins, Reeds-Shepp), closed-loop navigation, embodiment
  abstraction, legged locomotion/WBC, state machine, behavior tree, mission/swarm,
  Zenoh-style communication, LLM agent.
- 🚁 **Multi-embodiment** — drones, Ackermann cars, differential-drive surface
  vessels (with AIS + COLREGS) and legged robots share one `RobotBody` abstraction;
  drone-specific types stay in thin adapters.
- 🛰️ **Navigation stack with real algorithms** — 3D voxel mapping with ray-casting,
  frontier exploration, breadcrumb backtracking, VIO (IMU + RGB-D + Kalman fusion),
  Dubins/Reeds-Shepp path smoothing (including reverse parking).
- 🐕 **Legged control** — gait phases, foot trajectories, leg IK, whole-body
  control (WBC), static/recursive Newton-Euler dynamics and spring-damper
  foot-ground contact.
- 🤖 **Agent / LLM layer** — Rig-style tool-calling loop, RAG (deterministic offline
  embedder), pluggable model backends (`mock`, `ollama`, `hermes`, any
  OpenAI-compatible HTTP endpoint) via a single backend factory.
- 🛡️ **Safety by design** — `FailsafeWatchdog` (heartbeat supervision → forced
  loiter/RTL), geofence, battery tiers and pre-arm checks can override mission
  commands; the FCU stays the sole actuator authority.
- 🔌 **Offline-first engineering** — all tests (including HTTP LLM backends, via a
  loopback server) run without network or hardware; CI covers Linux + macOS,
  a feature matrix, rustdoc `-D warnings`, and weekly RustSec audits.

## Architecture

Every crate maps to one layer of the reference architecture:

| Layer | Reference architecture | Crates in this workspace |
|-------|------------------------|--------------------------|
| 5. Mission & application | Route planning, swarm coordination | `brain-mission` (waypoint missions + swarm link/leader election/task allocation) |
| 4. AI & perception | Visual SLAM / obstacle avoidance / tracking | `brain-perception` (inference backend + vision pipeline), `brain-odometry` (VIO), `brain-mapping` (voxel grid) |
| 4. Closed-loop autonomy | Sense → map → plan → drive → backtrack | `brain-autopilot` (drone/car/boat navigators, AIS + COLREGS) |
| 4. Agent / LLM reasoning | Tool calling, perceive → reason → control | `brain-agent` (agent loop, RAG, model backends) |
| 3. Decision / middleware | Data bus, state machine, behavior tree | `brain-middleware` (topic bus), `brain-state` (state machine + watchdog + safety), `brain-behavior-tree` |
| 3. Unified communication | **Zenoh-style: Pub/Sub + Store/Query + Compute** | `brain-zenoh` (in-process backend, swappable for real Zenoh), `brain-ipc` (zero-alloc ring buffers) |
| 3.5 Embodiment abstraction | Body-agnostic robot interface | `brain-robot` (`RobotKind` / `BodyState` / `EffectorCommand` / `RobotBody` / `CarBody` / `BoatBody`) |
| 3.5 Locomotion & whole-body control | Gait phase, foot trajectories, velocity → footholds | `brain-locomotion` (gait / foot trajectory / leg IK / WBC / contact dynamics) |
| 3.5 Kinematics | Forward/inverse kinematics, Jacobian, vehicle kinematics | `brain-kinematics` (FK/IK, quadruped/arm/humanoid, bicycle & Ackermann model) |
| 2. Hardware interface | Link to the cerebellum (FCU) | `brain-transport` (serial / CAN / UDP / mock / MAVLink-style) |
| Base | Shared primitives | `brain-core` (error / config / clock / time sync / math) |
| Assembly | Main executable | `brain-node` (wires everything into closed loops, CLI + 21 demos) |
| Simulation | Pluggable simulator backend | `brain-sim` (`Simulator` trait + deterministic `MockSimulator`) |

### Cerebrum / cerebellum decoupling

`brain-transport::FcuTransport` is the *only* path from the brain to the flight
controller. The brain sends high-level intent (`Command`); attitude stabilization
stays on the microcontroller (STM32/Pixhawk) and the two talk over UART/CAN —
which is exactly the "never run your AI and your flight-control loop on the same
processor" rule.

### Dependency rules

Dependencies only point **downwards** (higher layer → lower layer); there are no
cycles and no upward dependencies. Cross-cutting logic belongs in the matching
crate, never in `brain-node` demos. See
[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the dependency graph, the key
traits (`FcuTransport`, `Simulator`, `RobotBody`, `Model`, `Clock`), the data flow
of a full mission loop, and the design trade-offs.

## Quick start

### Requirements

- **Rust stable** with `rustfmt` + `clippy` (pinned by
  [`rust-toolchain.toml`](rust-toolchain.toml); `rustup` switches automatically)
- `git`, and optionally `make` (convenience targets)
- **No system dependencies for the default build** — serial/CAN/ONNX/LLM backends
  are feature-gated (see [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md))

### Build, test, run

```bash
# 0) Get the code
git clone https://github.com/arkCyber/Smart-Brain.git
cd Smart-Brain
rustup show                # should report "stable"

# 1) Build the whole workspace (--locked = strictly follow Cargo.lock)
cargo build --locked

# 2) Run all unit tests (570+, fully offline)
cargo test --workspace

# 3) Run the full mission demo (SITL): takeoff → cruise → detect & track → land
#    followed by the fail-safe demo (watchdog forces LOITER when the brain stalls)
cargo run -p brain-node

# 4) Quality gates, identical to CI (`make check` runs all four)
cargo fmt --all -- --check && \
cargo clippy --workspace --all-targets -- -D warnings && \
cargo test --workspace && \
RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps
```

Expected demo output:

```text
[tick   0] state=Ground      mode=Idle       note=mission armed, executing survey
[tick   1] state=TakingOff   mode=Takeoff    note=takeoff to 30m
[tick   2] state=Cruising    mode=Cruise     note=cruise waypoint 0
>>> mission complete: landed & returned to Ground
=== Fail-safe demo (watchdog timeout = 50ms) ===
heartbeat ok, watchdog armed.
>>> WATCHDOG TRIPPED after 97ms -> forcing LOITER (auto-hover)
```

### Configuration

`brain-node` resolves its configuration in this order (first hit wins):

1. `--config <path>` (CLI)
2. `$SMART_BRAIN_CONFIG`
3. `./config.json`
4. `./config.example.json` (shipped template)
5. built-in defaults

For real hardware, copy the template and edit it — `config.json` is git-ignored:

```bash
cp config.example.json config.json
cargo run -p brain-node -- --config config.json
```

Key fields: `node_id`, `heartbeat_period_ms`, `failsafe_timeout_ms`,
`tick_period_ms`, `fcu.transport` (`mock`/`serial`/`can`/`udp`) with
`serial_port`/`baud_rate`/`udp_target`, `safety.*` (geofence radius/altitude,
battery tiers, pre-arm thresholds, RTH), and `ollama`/`hermes`/`agent.backend`
(model backends). Secrets are **never** stored in the config file — use
`OLLAMA_API_KEY` / `HERMES_API_TOKEN`. Full field reference:
[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).

### Run a single crate example

Each library crate ships an `examples/` directory (minimal runnable demo) and a
`README.md` (role / layer / key API / usage / dependencies):

```bash
cargo run -p brain-core      --example basic      # config loading + math primitives
cargo run -p brain-ipc       --example ring       # zero-allocation ring buffer
cargo run -p brain-zenoh     --example pubsub     # Pub/Sub + Store/Query + Compute
cargo run -p brain-planning  --example astar      # A* obstacle-avoiding planning
cargo run -p brain-agent     --example agent      # agent tool-calling loop
cargo run -p brain-agent     --features ollama --example ollama  # local Ollama (11434)
cargo run -p brain-agent     --features hermes --example hermes  # Hermes daemon (11438)
cargo run -p brain-autopilot --example ais_colregs      # AIS → local frame → COLREGS
cargo run -p brain-locomotion --example dynamic_contact # contact + WBC torques
```

### Demo CLI

```bash
cargo run -p brain-node                      # all 21 demos, sequentially
cargo run -p brain-node -- --list            # list available demos
cargo run -p brain-node -- --demo mission --iterations 30
cargo run -p brain-node --features async -- --demo async
cargo run -p brain-node -- --help            # usage
cargo run -p brain-node -- --version
```

| Flag | Description |
|------|-------------|
| `--demo <name>` | Run a single demo (see `--list`); unknown name → exit code 2 |
| `--list` | List all demos and exit |
| `--config <path>` | Explicit config file (highest priority) |
| `--iterations <n>` | Tick count for the mission demo (default 30, must be > 0) |
| `-h, --help` / `-V, --version` | Help / version |

Exit codes: `0` success · `1` invalid configuration · `2` usage error or unknown
demo. An invalid configuration no longer panics.

## Workspace layout

| Crate | Role |
|-------|------|
| [`brain-core`](crates/brain-core/README.md) | Shared error type, config, injectable clock, monotonic stopwatch, **NTP-style time sync**, math primitives (`Vec3`/`Quat`/`Pose`) |
| [`brain-message`](crates/brain-message/README.md) | MAVLink-like telemetry/command messages plus wire framing (length prefix + CRC-16 + split/merge-safe `FrameReader`) |
| [`brain-middleware`](crates/brain-middleware/README.md) | Type-safe topic bus (`DataBus`, `Topic<T>`) — the robot's "nervous system" |
| [`brain-transport`](crates/brain-transport/README.md) | `FcuTransport` + mock/UDP/serial (`serial`)/SocketCAN (`can`) backends, MAVLink-style binary codec |
| [`brain-state`](crates/brain-state/README.md) | Control state machine, fail-safe watchdog, safety primitives (geofence, battery tiers, pre-arm, unified `FlightPermission`) |
| [`brain-perception`](crates/brain-perception/README.md) | `ModelBackend` abstraction, vision pipeline, ONNX Runtime backend (`onnx`) with YOLOv8 decode + class-aware NMS |
| [`brain-behavior-tree`](crates/brain-behavior-tree/README.md) | Behavior-tree framework (`Sequence`/`Selector`/decorators) and flight nodes |
| [`brain-mission`](crates/brain-mission/README.md) | Waypoint missions, mission files, progress tracking, swarm link + leader election + task allocation |
| [`brain-robot`](crates/brain-robot/README.md) | Body-agnostic `RobotBody` abstraction with `RobotKind`, `BodyState`, `EffectorCommand`, `CarBody`, `BoatBody` |
| [`brain-kinematics`](crates/brain-kinematics/README.md) | FK, geometric Jacobian, Jacobian-transpose IK, bicycle/Ackermann vehicle model |
| [`brain-ipc`](crates/brain-ipc/README.md) | Pre-allocated, zero-allocation ring buffers for high-rate sensor data |
| [`brain-mapping`](crates/brain-mapping/README.md) | 3D occupancy/voxel grid with probabilistic ray-casting updates, inflation, frontier queries |
| [`brain-odometry`](crates/brain-odometry/README.md) | VIO: IMU integration, RGB-D visual odometry (Kabsch/SVD), timestamp interpolation, Kalman fusion |
| [`brain-planning`](crates/brain-planning/README.md) | A\*, RRT, DWA, **Ackermann-DWA**, **Dubins** and **Reeds-Shepp** planners |
| [`brain-nav`](crates/brain-nav/README.md) | Frontier exploration (`Explorer`) and breadcrumb backtracking (`Backtracker`) |
| [`brain-zenoh`](crates/brain-zenoh/README.md) | Zenoh-style unified communication (Pub/Sub + Store/Query + Compute) with `real-zenoh` swap-in |
| [`brain-autopilot`](crates/brain-autopilot/README.md) | Closed-loop navigators: drone/differential `Autopilot`, `CarAutopilot` (Ackermann + reverse parking), `BoatAutopilot` (+ currents/tides, AIS, COLREGS) |
| [`brain-agent`](crates/brain-agent/README.md) | Agent/LLM layer: tool calling, RAG, model backends (`mock`/`ollama`/`hermes`/OpenAI-compatible HTTP) and backend factory |
| [`brain-locomotion`](crates/brain-locomotion/README.md) | Gait phases, foot trajectories, leg IK, WBC, RNEA inverse/forward dynamics, spring-damper ground contact |
| [`brain-sim`](crates/brain-sim/README.md) | Pluggable `Simulator` contract + deterministic in-process `MockSimulator` (Gazebo/AirSim/Isaac-agnostic) |
| [`brain-node`](crates/brain-node/README.md) | Main executable: assembly, production CLI, 21 demos |

## Feature flags

Everything hardware- or network-facing is opt-in, so the default build stays
dependency-free and reproducible offline.

| Crate | Feature | What it enables | Requires |
|-------|---------|-----------------|----------|
| `brain-agent` | `http-llm` | OpenAI-compatible HTTP LLM backend (`HttpModel`) | network (can target localhost) |
| `brain-agent` | `ollama` | Native Ollama backend (`/api/chat`, port 11434) with tool calling | local Ollama (optional) |
| `brain-agent` | `hermes` | Hermes agent daemon (OpenAI-compatible `/v1`, port 11438) | local Hermes (optional) |
| `brain-node` | `async` | tokio runtime: perception/decision as async tasks | — |
| `brain-node` | `ollama` / `hermes` | Re-export the matching demos (`--demo ollama` / `--demo hermes`) | as above |
| `brain-perception` | `onnx` | `ort` backend (ONNX Runtime) with YOLOv8 decode + NMS | system `onnxruntime` (loaded at runtime) |
| `brain-transport` | `serial` | `SerialTransport` (UART link to the FCU) | `libudev-dev` on Linux |
| `brain-transport` | `can` | `CanTransport` (Linux SocketCAN) | **Linux only** |
| `brain-zenoh` | `real-zenoh` | Real `zenoh` crate (Pub/Sub + Queryable) instead of the in-process backend | — (pure Rust) |

```bash
cargo run   -p brain-node --features async -- --demo async
cargo run   -p brain-node --features ollama -- --demo ollama
cargo build -p brain-perception --features onnx
cargo build -p brain-transport --features serial      # needs a serial device at runtime
cargo build -p brain-transport --features can          # Linux only
cargo test  -p brain-zenoh --features real-zenoh --lib
cargo test  -p brain-zenoh --features real-zenoh -- --ignored   # real two-peer network tests
```

## Testing & quality gates

```bash
make check     # fmt --check + clippy(-D warnings) + test + rustdoc(-D warnings)
```

| Gate | Command |
|------|---------|
| Formatting | `cargo fmt --all -- --check` |
| Lints | `cargo clippy --workspace --all-targets --locked -- -D warnings` |
| Tests | `cargo test --workspace --locked` (570+ tests, no network/hardware needed) |
| Docs | `RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps` |
| Linux cross-check | `cargo clippy --workspace --all-targets --target x86_64-unknown-linux-gnu -- -D warnings` |
| Supply chain | `cargo audit` (config in [`.cargo/audit.toml`](.cargo/audit.toml)) |

CI (see [`.github/workflows/ci.yml`](.github/workflows/ci.yml)) runs six jobs:
Linux build/test/clippy, macOS arm64 build/test, rustdoc, `real-zenoh`, the
feature matrix (`onnx`/`can`/`serial`/`http-llm`/`ollama`/`hermes`), and tokio
async. A separate weekly job runs RustSec `cargo audit`
([`audit.yml`](.github/workflows/audit.yml)), and
[`release.yml`](.github/workflows/release.yml) publishes binaries for tagged
releases.

## Deployment path

The rule of thumb: **finish 90% of your testing on a workstation before touching
real hardware.**

1. **Prototype / simulation (current stage ✅)** — the full "takeoff → cruise →
   detect & track → land" loop already runs against a mock FCU and mock inference;
   `brain-sim` provides the pluggable `Simulator` contract plus a deterministic
   `MockSimulator`. Implementing the same trait is all it takes to plug in
   Gazebo/AirSim/Isaac — brain code unchanged.
2. **Rust middleware on target** — `SerialTransport` (`serial`) and Linux
   `CanTransport` (`can`, SocketCAN) are ready; run Ubuntu with an RT-Preempt
   kernel on Jetson Orin / RK3588 class hardware.
3. **AI model engineering** — train with PyTorch → export `.onnx` → quantize
   (TensorRT/RKNN INT8) → enable the `onnx` feature and provide the system
   onnxruntime library at runtime.
4. **On-vehicle integration & boundary tests** — tethered/propped-down tests, then
   deliberate watchdog-stall injection: if the brain misses its heartbeat budget
   (`failsafe_timeout_ms`, default 50 ms) the FCU takes over into loiter/RTL.
   Always keep the RC override, the FCU's own failsafe and a physical kill switch.

The full checklist, configuration reference and safety notes live in
[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).

## Roadmap

Short-term priorities (details, acceptance criteria and the full list are in
[docs/ROADMAP.md](docs/ROADMAP.md)):

- **Gazebo / AirSim / Isaac integration** by implementing `brain-sim::Simulator`
  (no brain-side changes required).
- **Real MAVLink interoperability** with ArduPilot/PX4 (the frame-level codec
  exists; telemetry parsing and command acknowledgement are next).
- **Contact model inside the `brain-sim` physics backend** and online joint-torque
  limit checking for legged robots.
- **Trajectory generation/smoothing** (curvature and jerk limits) plus
  lane/traffic-semantics for ground vehicles.
- **Generalization** of the remaining flight-specific types
  (`brain-message`/`brain-state`/drone nodes) so the same brain drives quadrupeds,
  arms and humanoids unchanged.

Also planned: AIS message type 6/24/27, multi-point current estimation for
surface vessels, and richer sensor topics (contact/wrench) on `BodyState`.

## Documentation

| Document | Contents |
|----------|----------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Layering, dependency rules, key traits, data flow, design trade-offs |
| [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md) | Environment setup, commands, feature matrix, CI gates, code conventions, FAQ |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | Four-stage deployment path, configuration reference, checklists |
| [docs/ROADMAP.md](docs/ROADMAP.md) | Done / next / long-term |
| [docs/RELEASE.md](docs/RELEASE.md) | Versioning, release checklist, artifacts, repo settings |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to contribute |
| [SECURITY.md](SECURITY.md) | Vulnerability reporting + deployment security boundaries |
| [SUPPORT.md](SUPPORT.md) | Where to ask for help |
| [CHANGELOG.md](CHANGELOG.md) | Release history |
| `crates/*/README.md` | Per-crate role, layer, key API, usage, dependencies |

Generate and browse the API docs locally:

```bash
cargo doc --workspace --no-deps --open
```

> 🌏 **Language note**: this README is available in
> [English](README.md) and [简体中文](README.zh-CN.md). The deep-dive guides under
> `docs/`, the per-crate READMEs and the source comments are currently in Chinese —
> translation contributions are very welcome.

## Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) and
[CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) first.

```bash
git checkout -b feat/your-feature
make check        # fmt --check + clippy -D warnings + tests + rustdoc -D warnings
git commit -m "feat(planning): add hybrid A* global planner"
```

- Keep the workspace warning-free; new logic needs unit tests.
- Follow the layering rules (no upward dependencies, no business logic in demos).
- New hardware/network backends must be feature-gated and covered by a CI job.
- Never commit secrets, `config.json`, `.env` files or model weights.
- Use [Conventional Commits](https://www.conventionalcommits.org/); update
  [CHANGELOG.md](CHANGELOG.md) under `[Unreleased]`.

Bug reports and feature requests: use the
[issue forms](https://github.com/arkCyber/Smart-Brain/issues/new/choose). For usage
questions, start with [SUPPORT.md](SUPPORT.md).

## Security

Please **do not** open a public issue for security problems — follow
[SECURITY.md](SECURITY.md) (private vulnerability reporting or email). Two RustSec
advisories that only affect the optional `real-zenoh` dependency tree are
documented and explicitly accepted in [`.cargo/audit.toml`](.cargo/audit.toml).

Deployment reminders: this is research-grade software without airworthiness,
classification-society or road-safety certification; keep the RC override, the
FCU's own failsafe and a physical kill switch available, and never expose the
debug ports (UDP/CAN/Zenoh/local LLM endpoints) to untrusted networks.

## License & disclaimer

Licensed under the **Apache License 2.0** — see [LICENSE](LICENSE); third-party
dependency licenses and disclaimers are listed in [NOTICE](NOTICE).

```text
Copyright 2026 arkSong <arksong2018@gmail.com>

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
```

- **Author / maintainer**: arkSong — arksong2018@gmail.com
- **Repository**: https://github.com/arkCyber/Smart-Brain
- **Releases** (prebuilt Linux x86_64 / macOS arm64 binaries with SHA256):
  https://github.com/arkCyber/Smart-Brain/releases

> ⚠️ **Safety disclaimer**: this project is research/prototype software and is
> **not certified** for flight, maritime or road use. Real-world deployment
> (drones, cars, surface vessels, legged robots) is the operator's responsibility:
> validate thoroughly in simulation and in an isolated area, and always keep a
> manual takeover and physical emergency stop available.

If this project is useful to you, a ⭐ is appreciated.

