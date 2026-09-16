# 贡献指南（Contributing to Smart-Brain）

感谢你有兴趣参与！Smart-Brain 是一个 Rust workspace（“AI 大脑”），为**无人机、
汽车、水面艇、足式机器人**提供自主导航与决策能力。

- 📖 先读 [README.md](README.md) 与 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) 了解分层；
- 🛠 环境与命令见 [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md)；
- 🤝 参与即表示你同意遵守 [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md)。

## 快速开始

```bash
git clone https://github.com/arkCyber/Smart-Brain.git
cd Smart-Brain
rustup show                    # stable（由 rust-toolchain.toml 固定）
cargo build --locked           # 默认无系统级依赖，开箱即用
cargo test --workspace         # 全部 570+ 单测，离线可跑
cargo run -p brain-node        # 完整任务演示（SITL，mock 飞控）

# 质量门禁（与 CI 一致）
make check
# 等价的手动命令：
# cargo fmt --all -- --check
# cargo clippy --workspace --all-targets -- -D warnings
# cargo test --workspace
# RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps
```

## 提交前的要求

- **零警告**：`cargo clippy --workspace --all-targets -- -D warnings` 与
  `cargo fmt --all -- --check` 必须通过；文档链接也必须干净（`-D warnings`）。
- **有测试**：新增逻辑必须带 `#[cfg(test)]` 单测，并保证 `cargo test --workspace` 通过。
  涉及随机性的逻辑要**可复现**（固定种子），避免 flaky。
- **按层放置代码**：新逻辑放到匹配参考架构的 crate 里，而不是 `brain-node` 的演示中
  （它只做装配与演示）。依赖只能**向下**，禁止反向依赖或环。
- **feature 门控**：新增硬件/网络后端一律 `optional = true` + feature，默认构建保持
  零系统依赖；并在 `.github/workflows/ci.yml` 增加对应 job。
- **不提交密钥**：`config.json`、`.env`、真实 token、模型权重均不应进仓库
  （已在 `.gitignore` 中）。

## 分支与提交信息

- 分支命名：`feat/...`、`fix/...`、`docs/...`、`refactor/...`、`test/...`、`chore/...`。
- 提交信息遵循 [Conventional Commits](https://www.conventionalcommits.org/)：

  ```
  <type>(<scope>): <subject>

  <body：为什么这样改，以及取舍>

  <footer：Closes #123 / BREAKING CHANGE: ...>
  ```

  示例：`feat(planning): 增加 Hybrid A* 全局规划`、
  `fix(transport): 修复 CAN 分片重组的越界读`。

## 代码布局速查

- `brain-core` — 错误/配置/时钟/时间同步/数学原语（零内部依赖）。
- `brain-message` — 遥测与指令消息 + 帧编解码（CRC16）。
- `brain-middleware` / `brain-zenoh` / `brain-ipc` — 话题总线、统一通信、零分配环形缓冲。
- `brain-transport` — `FcuTransport`：串口/CAN/UDP/mock（大脑唯一硬件出口）。
- `brain-perception` / `brain-odometry` / `brain-mapping` — 推理、VIO、占据栅格。
- `brain-planning` / `brain-nav` / `brain-autopilot` — A*/RRT/DWA/Dubins/Reeds-Shepp、
  探索回溯、闭环导航（无人机/汽车/水面艇 + AIS/COLREGS）。
- `brain-robot` / `brain-kinematics` / `brain-locomotion` — 具身抽象、运动学、步态与 WBC。
- `brain-state` / `brain-behavior-tree` / `brain-mission` — 状态机与看门狗、行为树、任务与蜂群。
- `brain-agent` — Agent/LLM 层（mock/ollama/hermes 后端工厂）。
- `brain-sim` — 可插拔仿真后端契约 + 确定性 mock 世界。
- `brain-node` — 主程序：装配 + CLI + 演示。
- `docs/` — 架构、开发、部署、路线图、发布文档。

## Pull Request 流程

1. 先开 Issue 对齐设计（较大的改动尤其必要），或从
   [docs/ROADMAP.md](docs/ROADMAP.md) 的「下一步」表中挑一条。
2. Fork 并开分支开发，保持 PR **小而聚焦**（一个 PR 解决一件事）。
3. 按 [PR 模板](.github/PULL_REQUEST_TEMPLATE.md) 填写说明、测试方式与自查项。
4. 更新 [CHANGELOG.md](CHANGELOG.md) 的 `[Unreleased]` 段落（Added/Changed/Fixed…）。
5. 等待 CI 全绿与维护者评审（见 [.github/CODEOWNERS](.github/CODEOWNERS)）。

## 报告问题

| 类型 | 渠道 |
|------|------|
| Bug / 功能建议 | [Issue 模板](https://github.com/arkCyber/Smart-Brain/issues/new/choose) |
| 安全问题 | **不要开公开 Issue**，见 [SECURITY.md](SECURITY.md) |
| 使用疑问 | 先看 [SUPPORT.md](SUPPORT.md)，再联系维护者 |

## 联系维护者

- **arkSong** — arksong2018@gmail.com
- GitHub：[@arkCyber](https://github.com/arkCyber)

## 许可证

Apache-2.0（见 [LICENSE](LICENSE)）。提交贡献即表示你同意你的贡献以本许可证发布。

