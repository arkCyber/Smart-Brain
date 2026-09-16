# 文档索引（docs/）

本目录存放 README 之外的深度文档。README 是入口与总览，这里给出可操作、可落地的细节。

| 文档 | 内容 | 适合谁读 |
|------|------|----------|
| [../README.md](../README.md) | 英文版 README（GitHub 默认展示）：定位、快速开始、crate 布局、feature 矩阵、质量门禁 | 所有人 |
| [../README.zh-CN.md](../README.zh-CN.md) | 中文版 README（含各模块的详细实现说明与真机部署路线） | 中文读者 |
| [ARCHITECTURE.md](ARCHITECTURE.md) | 分层架构、crate 依赖图、数据流、大脑/小脑边界、关键抽象（trait）清单 | 想理解设计、要做扩展的开发者 |
| [DEVELOPMENT.md](DEVELOPMENT.md) | 开发环境搭建、构建/测试/lint 命令、feature 矩阵、CI 说明、调试与故障排查 | 首次参与贡献的开发者 |
| [DEPLOYMENT.md](DEPLOYMENT.md) | 从 SITL 到真机的四阶段部署路线、配置项说明、安全与 fail-safe 注意事项 | 要在真机/仿真平台落地的人 |
| [ROADMAP.md](ROADMAP.md) | 已完成 / 进行中 / 计划中的能力，以及“下一步”的具体落点 | 想找活干的贡献者、评估项目成熟度的人 |
| [RELEASE.md](RELEASE.md) | 版本号约定、发布清单、产物与校验方式 | 维护者 |

## 阅读顺序建议

1. 先读 [README.md](../README.md) 了解项目定位与快速开始；
2. 再读 [ARCHITECTURE.md](ARCHITECTURE.md) 建立分层心智模型；
3. 动手前读 [DEVELOPMENT.md](DEVELOPMENT.md) 把环境跑通；
4. 准备上真机前读 [DEPLOYMENT.md](DEPLOYMENT.md)；
5. 参与开发前读 [CONTRIBUTING.md](../CONTRIBUTING.md) 与 [CODE_OF_CONDUCT.md](../CODE_OF_CONDUCT.md)。

## 各 crate 的模块级文档

每个 crate 都有自己的 `README.md`（职责 / 所属层 / 核心 API / 用法 / 依赖）与
`examples/`（最小可运行示例）。查看全部 crate 列表：

```bash
cargo metadata --no-deps --format-version 1 | python3 -c "import json,sys;print('\n'.join(p['name'] for p in json.load(sys.stdin)['packages']))"
ls crates/*/README.md
```

生成并在浏览器中查看 API 文档：

```bash
cargo doc --workspace --no-deps --open
```
