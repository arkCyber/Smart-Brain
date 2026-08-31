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
- CI workflow, LICENSE (Apache-2.0), CONTRIBUTING guide.

### Changed
- Cargo.lock is now committed for reproducible builds.
- Added `RobotKind::SurfaceVessel`, `Car`, and `BoatBody`/`CarBody` to the
  body-agnostic abstraction.

## [0.1.0] - Initial

- Workspace skeleton ("AI 大脑") with drone-focused SITL loop, telemetry/command
  framing, state machine + failsafe watchdog, perception pipeline, behavior
  tree, mission/swarm, Zenoh transport, agent + RAG, VIO/odometry, indoor
  mapping/A*/DWA navigation.
