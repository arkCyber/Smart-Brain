# 获取帮助（Support）

感谢使用 **Smart-Brain**！请按下表选择最合适的渠道，以便我们更快地帮到你。

| 你的需求 | 推荐渠道 |
|----------|----------|
| 使用/构建/运行报错 | [GitHub Issues（Bug 报告模板）](https://github.com/arkCyber/Smart-Brain/issues/new/choose) |
| 新功能、新平台（Gazebo/AirSim、机械臂等）建议 | [Feature request 模板](https://github.com/arkCyber/Smart-Brain/issues/new/choose) |
| 安全漏洞 | **不要开 Issue**，见 [SECURITY.md](SECURITY.md) |
| 贡献代码 | 见 [CONTRIBUTING.md](CONTRIBUTING.md) 与 [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md) |
| 架构/分层/术语疑问 | 先读 [README.md](README.md)（英文）/ [README.zh-CN.md](README.zh-CN.md)（中文）与 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| 真机部署 | [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) |
| 其他 | 邮件联系维护者 **arkSong** — arksong2018@gmail.com |

## 提问前请先自查

1. 使用仓库锁定的工具链（`rust-toolchain.toml`），并执行
   `cargo build --workspace && cargo test --workspace`。
2. 确认问题在**最小复现**下仍然存在：优先用 `cargo run -p brain-node -- --demo <name>`
   或某个 crate 的 `examples/` 复现，而不是整机流程。
3. 附上环境信息：`rustc --version`、`cargo --version`、操作系统、
   启用的 feature（`serial`/`can`/`onnx`/`ollama`/`hermes`/`async` 等）。
4. 附上完整报错日志与你的 `config.json` 中**脱敏后**的相关字段。

## 响应预期

本项目由个人维护，属于业余时间投入：Issue 通常在数天内答复，但请理解**不承诺
SLA**。欢迎自带 PR —— 附上修复或测试的 Issue 会被优先处理。
