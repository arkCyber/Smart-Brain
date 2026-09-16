# 安全策略（Security Policy）

## 支持的版本

本项目处于 `0.x` 早期阶段，安全修复只保证应用在**最新的 `main` 分支**与最新
发布标签上。

| 版本 | 是否支持安全更新 |
|------|------------------|
| `main`（最新提交） | ✅ |
| `0.1.x` | ✅ |
| 更早版本 | ❌ |

## 报告安全漏洞

**请不要通过公开 Issue 报告安全漏洞。**

请使用 GitHub 的私有漏洞报告通道
（仓库页面 → **Security** → **Report a vulnerability**），或发送邮件至：

- **arkSong** — arksong2018@gmail.com
- 建议邮件标题加上 `[SECURITY][Smart-Brain]` 前缀，便于分流

为方便定位与修复，请尽量提供：

- 漏洞类型与影响（例如：拒绝服务、越权控制、注入、敏感信息泄露）
- 受影响的 crate / 文件 / 函数与版本或提交哈希
- 最小复现步骤（可运行代码片段、配置或报文样本更佳）
- 你的环境（OS、Rust 版本、是否启用 `serial` / `can` / `onnx` / `ollama` /
  `hermes` / `http-llm` / `async` 等 feature）
- 可能的话，附上你的处置建议或补丁

## 响应流程与时间预期

| 阶段 | 目标时间 |
|------|----------|
| 确认收到报告 | 3 个工作日内 |
| 初步评估（是否复现、影响范围） | 7 个工作日内 |
| 修复或缓解措施 | 视严重程度，约 30 天内 |
| 公开披露 | 修复发布后进行，并在 `CHANGELOG.md` 中致谢（如你同意署名） |

在修复发布前，请对漏洞细节保密；我们会在修复版本中同步发布安全说明。

## 已知的第三方公告（已评估）

`cargo audit` 在 CI 中运行（见 `.github/workflows/audit.yml`）。当前有两条公告被
**显式豁免**（配置见 [`.cargo/audit.toml`](.cargo/audit.toml)），原因是它们
**仅由可选的 `real-zenoh` feature 间接引入**、上游暂无可用修复，且不在默认构建中：

| 公告 | 依赖 | 影响 | 处置 |
|------|------|------|------|
| RUSTSEC-2026-0041 | `lz4_flex` 0.10（经 `zenoh-transport`） | 解压非法数据时可能读取未初始化内存 | 等待 zenoh 升级到 `lz4_flex` 0.11+ 后移除豁免 |
| RUSTSEC-2023-0071 | `rsa` 0.9.10（经 `zenoh`） | Marvin Attack（RSA 解密时序侧信道） | 上游标注 “No fixed upgrade is available”；仅在使用 zenoh 加密链路时涉及 |

另有两条**未维护**类提示（`paste`、`rustls-pemfile`，同为 zenoh 生态间接依赖）：
不阻断构建，保留在审计输出中以便跟踪。

> 如果你在部署中启用 `real-zenoh` 并对外暴露加密链路，请自行评估上述风险并优先
> 升级 zenoh 到已修复版本（升级后应同时删除 `.cargo/audit.toml` 中的豁免项）。

## 部署方必须注意的安全边界

本项目是**自主机器人大脑**，请务必记住以下与安全相关的设计约束：

1. **飞行安全责任由部署方承担。** 本仓库是研究/原型软件，不构成适航或安全
   认证。真机运行前请在 SITL 与地面拉线测试中充分验证。
2. **fail-safe 是最后一道防线，而非唯一防线。** `brain-state::FailsafeWatchdog`
   在心跳超时后强制进入 Loiter/安全模式；请同时保留飞控（小脑）自身的
   failsafe、物理遥控接管与围栏配置。
3. **不要提交密钥。** `config.json`、`.env` 已被 `.gitignore` 忽略；
   Ollama/Hermes 的 `api_token` 请通过环境变量
   （`OLLAMA_API_KEY` / `HERMES_API_TOKEN`）或本地未跟踪文件提供。
4. **网络暴露面。** `brain-transport` 的 UDP/CAN/串口后端、`brain-zenoh`
   与本地 LLM 端点（11434/11438）默认信任本机/局域网。请勿将调试端口直接
   暴露到公网；远程部署时使用防火墙、VPN 与鉴权。
5. **默认 mock 优先。** 所有硬件后端均为 feature 门控，默认构建不含真实设备
   驱动，避免误连真机。
