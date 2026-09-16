# Pull Request 模板

## 变更类型

请勾选本次 PR 的主要类型（可多选）：

- [ ] 🐛 Bug 修复（fix）
- [ ] ✨ 新功能（feat）
- [ ] ♻️ 重构（refactor，无行为变化）
- [ ] ⚡ 性能优化（perf）
- [ ] ✅ 测试补充（test）
- [ ] 📝 文档（docs）
- [ ] 🔧 构建 / CI / 工程化（build / ci / chore）

## 关联 Issue

<!-- 例如：Closes #12 / Refs #34。无关联 Issue 时请简述背景。 -->

Closes #

## 变更说明

<!-- 这个 PR 做了什么、为什么这样做。若涉及分层/架构决策，请说明落在哪一层。 -->

## 影响范围

- 受影响的 crate：
- 是否涉及公共 API 破坏性变更：是 / 否（若是，请在下方说明迁移方式）
- 是否引入新依赖或新 feature：是 / 否

## 安全与合规自查（自主机器人项目必填）

- [ ] 未提交任何真实密钥/token（`config.json`、`.env` 均在 `.gitignore` 中）
- [ ] 未改变 fail-safe / 看门狗 / 围栏的安全语义；若改变，已在上方说明原因与风险
- [ ] 新增硬件/网络后端仍为 **feature 门控**，默认构建不连真机、不访问外网

## 测试方式

<!-- 请粘贴你实际执行的命令与结果，评审者会按此复现。 -->

```bash
cargo build --workspace
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo fmt --all -- --check
# 若改动涉及 feature（serial/can/onnx/ollama/hermes/real-zenoh/async），请一并执行：
# cargo clippy -p <crate> --features <feature> --all-targets -- -D warnings
```

结果：

<!-- 例如：test result: ok. 570 passed; 0 failed -->

## 检查清单

- [ ] 已阅读 [CONTRIBUTING.md](../CONTRIBUTING.md) 与 [CODE_OF_CONDUCT.md](../CODE_OF_CONDUCT.md)
- [ ] 新增逻辑附带单元测试，且 `cargo test --workspace` 全部通过
- [ ] `cargo clippy` **零警告**、`cargo fmt` 已格式化
- [ ] 新增/变更的公共 API 已补充文档注释与对应 crate 的 `README.md`
- [ ] 变更已记录到 [CHANGELOG.md](../CHANGELOG.md) 的 `[Unreleased]` 段落
- [ ] 面向用户的行为变化已在 README / docs 中说明

## 截图 / 输出（可选）

<!-- 演示程序输出、日志片段、性能对比数据等。 -->
