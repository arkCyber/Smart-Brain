# Contributing to Smart-Brain

Thanks for your interest! This is a Rust workspace ("AI 大脑") for autonomous
navigation across robots, cars, and surface vessels.

## Getting started

```bash
cd Smart-Brain
cargo build          # build the whole workspace (no system deps needed)
cargo test           # run all unit tests
cargo run -p brain-node   # run the full demo (SITL, mock flight controller)
cargo clippy --workspace  # lint
cargo fmt            # format
```

## Before submitting

- Keep the workspace compiling with **zero warnings**: `cargo build` and
  `cargo clippy --workspace` must be clean.
- Add **unit tests** for new logic and make sure `cargo test` passes.
- Follow the existing layering: new cross-cutting logic belongs in a crate that
  matches the reference architecture (see README architecture table), not in
  `brain-node` demos.

## Code layout

- `brain-core` — error/config/time/math primitives (zero deps).
- `brain-robot` — body-agnostic `RobotBody` abstraction + concrete bodies.
- `brain-planning` — A*/RRT/DWA/Ackermann-DWA/Dubins/Reeds-Shepp.
- `brain-autopilot` — closed-loop navigators (drone, car, boat) + COLREGS.
- `brain-node` — assembly + demos.

## License

Apache-2.0. By contributing you agree your contributions are under this license.
