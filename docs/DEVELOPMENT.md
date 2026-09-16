# 开发指南（Development）

本文面向**要改代码的人**：从零搭环境、跑通构建测试、理解 feature 矩阵与 CI 门禁。

## 1. 环境要求

| 项目 | 要求 |
|------|------|
| Rust | **stable** 工具链（仓库用 [`rust-toolchain.toml`](../rust-toolchain.toml) 固定；`rustup` 会自动切换） |
| rustup 组件 | `rustfmt`、`clippy`（由 `rust-toolchain.toml` 自动安装） |
| C 链接器 | 系统自带 `cc`/`clang` 即可（默认可选依赖均为纯 Rust） |
| 可选系统依赖 | 仅在启用对应 feature 时需要：`serial`（串口设备）、`can`（Linux SocketCAN）、`onnx`（系统 onnxruntime 动态库） |
| 已验证平台 | macOS (arm64)、Linux x86_64；Windows 未在 CI 验证 |

**默认构建不需要任何系统级依赖**——`cargo build` 开箱即用，所有硬件/网络后端都是 feature 门控的。

```bash
git clone https://github.com/arkCyber/Smart-Brain.git
cd Smart-Brain
rustup show          # 应显示 stable（由 rust-toolchain.toml 指定）
cargo build          # 全 workspace 构建（首次会拉取依赖）
```

## 2. 常用命令

> 💡 已提供 **`Makefile`** 快捷命令（等价于下面的 cargo 命令，`make help` 查看全部）：
> `make build` / `make test` / `make clippy` / `make fmt` / `make doc-check` /
> `make check`（一次跑完提交前门禁）/ `make demo DEMO=car` /
> `make example CRATE=brain-core EXAMPLE=basic` / `make release`。

```bash
# 构建与测试
cargo build --workspace              # 全量构建
cargo build --workspace --locked     # 与 Cargo.lock 严格一致（CI/发布用）
cargo test  --workspace              # 全部单元测试
cargo test  -p brain-planning        # 单 crate 测试
cargo test  -p brain-core --lib -- lerp   # 按名字过滤（只测匹配的用例）

# 质量门禁（与 CI 一致，提交前请本地跑一遍）
cargo fmt --all                      # 格式化
cargo fmt --all -- --check           # 只检查不改
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy -p <crate> --features <feature> --all-targets -- -D warnings
RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps   # 文档链接受检

# 运行
cargo run -p brain-node                          # 全部演示（SITL）
cargo run -p brain-node -- --list                # 列出可用演示
cargo run -p brain-node -- --demo mission --iterations 30
cargo run -p brain-core --example basic          # 单 crate 示例
cargo doc --workspace --no-deps --open           # 生成并打开 API 文档
```

## 3. feature 矩阵

feature 的设计原则：**默认关闭一切硬件与网络后端**，保证零系统依赖、离线可复现。

| crate | feature | 作用 | 系统/外部依赖 | CI 是否验证 |
|-------|---------|------|----------------|-------------|
| `brain-agent` | `http-llm` | OpenAI 兼容 HTTP LLM 后端（`HttpModel`） | 网络（可指向本机） | ✅ clippy + 离线单测 |
| `brain-agent` | `ollama` | 本地 Ollama 原生后端（`/api/chat`，端口 11434） | 本地 Ollama 服务（可选） | ✅ clippy + 离线单测 |
| `brain-agent` | `hermes` | Hermes 智能体 daemon（OpenAI 兼容 `/v1`，端口 11438） | 本地 Hermes 服务（可选） | ✅ clippy + 离线单测 |
| `brain-node` | `async` | tokio 异步运行时，把感知/决策拆成 async 任务 | 无 | ✅ test + clippy |
| `brain-node` | `ollama` / `hermes` | 转发到 `brain-agent` 同名 feature 的演示入口 | 同上 | ✅ clippy |
| `brain-perception` | `onnx` | `ort` 推理后端（YOLOv8 解码 + NMS） | 系统 onnxruntime（`load-dynamic`，编译期不下载） | ✅ clippy |
| `brain-transport` | `serial` | 串口链路 `SerialTransport` | 串口设备 | ✅ build |
| `brain-transport` | `can` | Linux SocketCAN 链路 `CanTransport` | **仅 Linux** | ✅ clippy（ubuntu） |
| `brain-zenoh` | `real-zenoh` | 真实 `zenoh` crate 适配（Pub/Sub + Queryable） | 无（纯 Rust） | ✅ build + clippy + 离线单测 |

组合启用示例：

```bash
cargo run -p brain-node --features async -- --demo async
cargo run -p brain-node --features ollama -- --demo ollama
cargo run -p brain-agent --features hermes --example hermes
cargo build -p brain-perception --features onnx
cargo build -p brain-transport --features serial        # 需本机串口设备
cargo build -p brain-transport --features can           # 仅 Linux
```

> 提示：LLM 相关单测使用**本地回环 HTTP 服务器**模拟外部服务，因此 CI 无需真实模型即可验证请求形状、鉴权头与错误分支。

## 4. CI 门禁（.github/workflows/ci.yml）

| Job | 内容 |
|-----|------|
| `build-and-test` | `cargo build --workspace --locked` → `cargo test --workspace --locked` → `clippy --all-targets -D warnings` → `fmt --check` |
| `build-and-test-macos` | 在 `macos-14`（arm64）上做 build + test，验证跨平台可移植性（`can` 为 Linux 专属，不在本 job 验证） |
| `real-zenoh` | `brain-zenoh --features real-zenoh` 的 build + clippy + 离线单测 |
| `feature-backends` | `onnx` / `can` / `serial` / `http-llm` / `ollama` / `hermes` 的构建与单测 |
| `async-runtime` | `brain-node --features async` 的测试与 clippy |
| `docs` | `RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps` |
| `Security audit`（独立 workflow） | 每周一 + 依赖变更时跑 RustSec `cargo audit` |
| `Release`（独立 workflow） | 打 `v*.*.*` tag 时构建并发布 `brain-node` 二进制 + SHA256 |

**提交前请本地执行**（与 CI 相同的最小集合）：

```bash
make check          # 等价于下面四条
# 或手动执行：
cargo fmt --all -- --check && \
cargo clippy --workspace --all-targets -- -D warnings && \
cargo test --workspace && \
RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps
```

## 5. 代码规范

- **格式**：只使用 `cargo fmt` 默认配置，不引入自定义 rustfmt 规则，避免风格分歧。
- **Lint**：以 `clippy -D warnings` 为准；确需例外时用**带理由的 `#[allow(...)]`** 局部豁免。
- **错误处理**：库 crate 统一返回 `brain_core::Result<T>`（`BrainError`），不要 `unwrap()` 处理外部输入；binary/演示中允许对必然成功的路径 `expect("原因")`。
- **文档**：公共 API 必须有 `///` 文档；crate 根用 `//!` 说明职责、所属层、示例。文档中的 `[` 需用反引号包裹，否则会被当作 intra-doc link 而让 `docs` job 失败。
- **测试**：新增逻辑必须带 `#[cfg(test)]` 单测；涉及数值算法时断言容差而非精确相等；涉及随机性的逻辑必须**可复现**（种子固定），避免 flaky。
- **feature 门控**：新增硬件/网络后端一律 `optional = true` + feature，并在 CI 增加对应 job。
- **不提交**：真实密钥、`config.json`、`.env`、模型权重（见 `.gitignore`）。

## 6. 提交与 PR 流程

1. Fork 或以分支开发：`git checkout -b feat/你的功能`（分支名：`feat/` `fix/` `docs/` `refactor/` `test/`）。
2. 遵循 [Conventional Commits](https://www.conventionalcommits.org/)：
   `feat(planning): 增加 Hybrid A* 全局规划`。
3. 同步更新 `CHANGELOG.md` 的 `[Unreleased]` 段落，并在 PR 模板中勾选自查项。
4. 保持 PR **小而聚焦**，一个 PR 解决一件事；大改动请先开 Issue 对齐设计。
5. 等待 CI 全绿 + 维护者评审（见 `.github/CODEOWNERS`）。

## 7. 故障排查（FAQ）

| 现象 | 原因与处理 |
|------|------------|
| `error: package ID specification ... did not match` | 在错误的目录执行；`cargo run -p brain-node` 必须在 workspace 根执行 |
| `error: no such subcommand: clippy` | 未安装组件：`rustup component add clippy rustfmt` |
| `cargo build` 卡在下载依赖 | 网络受限；可配置镜像源后重试（本仓库默认依赖量很小） |
| 启用 `serial` 报找不到设备 | 检查 `config.json` 的 `fcu.serial_port` 与权限（Linux 需加入 `dialout` 组） |
| `can` feature 在 macOS 编译失败 | SocketCAN 仅 Linux 支持，请改在 Linux 上验证 |
| `onnx` 运行时找不到 `libonnxruntime` | `ort` 使用 `load-dynamic`，需把系统 onnxruntime 动态库放入 `LD_LIBRARY_PATH` |
| Ollama / Hermes 演示连接失败 | 先 `curl http://localhost:11434/api/tags` 确认服务与端口；鉴权时设置 `OLLAMA_API_KEY` / `HERMES_API_TOKEN` |
| `docs` job 报 `unresolved link` | 文档里裸写的 `[0,1]` 被当作链接；用反引号包裹 |
| 演示输出与 README 不一致 | 先确认 `git rev-parse --short HEAD` 与文档版本一致，再提 Issue |

## 8. 相关文档

- [ARCHITECTURE.md](ARCHITECTURE.md) — 分层与依赖规则
- [DEPLOYMENT.md](DEPLOYMENT.md) — 真机部署
- [CONTRIBUTING.md](../CONTRIBUTING.md) — 贡献流程
- [SUPPORT.md](../SUPPORT.md) — 提问渠道
