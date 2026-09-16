# 发布流程（Release）

面向维护者。本文约定版本号、发布清单与产物形态。

## 1. 版本号约定

遵循 [语义化版本](https://semver.org/lang/zh-CN/)：`MAJOR.MINOR.PATCH`。

- `0.x` 阶段：`MINOR` 的变更**可能包含破坏性改动**（但仍需在
  `CHANGELOG.md` 中显式标注 «Breaking»）。
- 从 `1.0.0` 起：破坏性改动只能进 `MAJOR`；新增能力进 `MINOR`；修复进 `PATCH`。
- workspace 内所有 crate **版本统一**（由根 `Cargo.toml` 的
  `[workspace.package] version` 继承），便于整体发布与依赖对齐。

## 2. 发布前检查清单

```bash
# 1) 全量质量门禁（与 CI 一致）
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test  --workspace
RUSTDOCFLAGS='-D warnings' cargo doc --workspace --no-deps

# 2) feature 矩阵抽查
cargo clippy -p brain-zenoh --features real-zenoh --all-targets -- -D warnings
cargo clippy -p brain-agent -p brain-node --features ollama --all-targets -- -D warnings
cargo clippy -p brain-agent -p brain-node --features hermes --all-targets -- -D warnings
cargo test   -p brain-node --features async --all-targets

# 3) 发布构建（与 CI 使用 --locked 保持一致）
cargo build --locked --release -p brain-node
cargo run   --release -p brain-node -- --list
```

- [ ] `CHANGELOG.md` 的 `[Unreleased]` 已整理为对应的 `[x.y.z] - YYYY-MM-DD` 段落
- [ ] 根 `Cargo.toml` 的 `version` 与 `CHANGELOG` 一致
- [ ] `README.md` 中的测试数量、演示数量、能力描述与代码一致
- [ ] `Cargo.lock` 已提交（`--locked` 构建通过）
- [ ] 无密钥/私有配置被提交（`git status` 检查 `config.json`、`.env`）

## 3. 打标签与发布

```bash
# 1) 更新版本号（根 Cargo.toml 的 [workspace.package] version）
# 2) 整理 CHANGELOG
git add -A && git commit -m "chore(release): v0.1.0"
git tag -a v0.1.0 -m "Smart-Brain v0.1.0"
git push origin main --follow-tags
```

推送 tag 后，`.github/workflows/release.yml` 会自动：

1. 在 `ubuntu-latest`（`x86_64-unknown-linux-gnu`）与 `macos-14`
   （`aarch64-apple-darwin`）上执行 `cargo build --locked --release -p brain-node`；
2. 打包 `smart-brain-<tag>-<target>.tar.gz`（内含 `brain-node`、`README.md`、
   `LICENSE`、`NOTICE`、`CHANGELOG.md`、`config.example.json`）；
3. 生成 SHA256 校验和；
4. 附到 GitHub Release，并自动生成发布说明（`generate_release_notes`）。

> 也可在 Actions 页面手动 `Run workflow` 做一次冒烟构建（不创建 Release）。

## 4. 产物校验

```bash
shasum -a 256 -c smart-brain-v0.1.0-x86_64-unknown-linux-gnu.tar.gz.sha256
tar -xzf smart-brain-v0.1.0-x86_64-unknown-linux-gnu.tar.gz
./smart-brain-v0.1.0-x86_64-unknown-linux-gnu/brain-node --version
```

## 5. 发布后

- 在 Release 说明中标注**破坏性变更**与**迁移方法**；
- 如包含安全修复，按 [SECURITY.md](../SECURITY.md) 的流程披露并对报告者致谢；
- 开一个新的 `[Unreleased]` 段落，开始收集下一轮变更。

## 6. 首次发布到 GitHub 的仓库设置清单

代码侧的“社区健康文件”已就绪（README、LICENSE、CONTRIBUTING、CODE_OF_CONDUCT、
SECURITY、SUPPORT、Issue/PR 模板、CODEOWNERS、Dependabot、CI/Audit/Release 工作流）。
剩下的是**仓库设置**，需在 GitHub 网页端（或 `gh` CLI）完成一次：

- [ ] **About**：填写 Description、主页，并设置 Topics，便于被检索到：

  ```bash
  gh repo edit arkCyber/Smart-Brain \
    --description "Rust AI brain for autonomous drones, cars, surface vessels and legged robots" \
    --add-topic rust --add-topic robotics --add-topic autonomous-navigation \
    --add-topic drone --add-topic autonomous-driving --add-topic colregs \
    --add-topic zenoh --add-topic mavlink
  ```

- [ ] **分支保护（main）**：要求 PR 评审、要求 CI 通过、禁止强制推送；
      建议开启 “Require branches to be up to date before merging”。
- [ ] **Security**：开启 **Private vulnerability reporting**（`SECURITY.md` 与
      Issue 模板的私报链接依赖它）、Dependabot alerts、Secret scanning。
- [ ] **Actions**：允许 GitHub Actions；Release 工作流需要
      `permissions: contents: write`（已在 workflow 内声明）。
- [ ] **Labels**：创建 Issue 模板引用的标签 `bug`、`triage`、`enhancement`、
      `dependencies`、`rust`、`ci`，否则提交后不会自动打标。
- [ ] **Release**：首个版本打 `v0.1.0` tag 前，确认 `CHANGELOG.md` 与
      根 `Cargo.toml` 的 `version` 一致。
- [ ] 上传前确认没有提交密钥或私有配置：

  ```bash
  git status --short
  git ls-files | grep -E 'config\\.json$|\\.env$|\\.onnx$' || echo "no secrets tracked"
  ```

## 7. 尚未发布到 crates.io

当前 workspace 内部依赖使用 `path` 且未声明版本号，因此**暂不支持
`cargo publish`**。若后续要发布到 crates.io，需要：

1. 为 `[workspace.dependencies]` 中每个内部 crate 增加 `version`（与 workspace 版本同步）；
2. 按依赖拓扑顺序发布（`brain-core` → … → `brain-node`）；
3. 在 CI 增加 `cargo package --workspace` 与 `cargo publish --dry-run` 校验。
