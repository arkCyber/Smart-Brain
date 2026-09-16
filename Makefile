# Smart-Brain 开发快捷命令（等价于 docs/DEVELOPMENT.md 中的命令）。
# 用法：make help
#
# 若未安装 make（例如部分 Windows 环境），请直接使用 DEVELOPMENT.md 中的 cargo 命令。

CARGO   ?= cargo
CRATE   ?= brain-core
EXAMPLE ?= basic
DEMO    ?= mission

.PHONY: help build test clippy fmt fmt-check doc doc-check check \
        run demo demo-async example release clean

help:  ## 显示可用目标
	@grep -E '^[a-zA-Z0-9_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN{FS=":.*?## "}{printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}'

build:  ## 构建整个 workspace（与 Cargo.lock 严格一致）
	$(CARGO) build --workspace --locked

test:  ## 运行全部单元测试
	$(CARGO) test --workspace --locked

clippy:  ## Clippy：警告即错误（含测试与示例）
	$(CARGO) clippy --workspace --all-targets --locked -- -D warnings

fmt:  ## 格式化代码
	$(CARGO) fmt --all

fmt-check:  ## 只检查格式，不修改
	$(CARGO) fmt --all -- --check

doc:  ## 生成并打开 API 文档
	$(CARGO) doc --workspace --no-deps --open

doc-check:  ## 文档构建以警告为错误（与 CI 的 docs job 一致）
	RUSTDOCFLAGS='-D warnings' $(CARGO) doc --workspace --no-deps

check: fmt-check clippy test doc-check  ## 提交前质量门禁（与 CI 一致）
	@echo ">>> all quality gates passed"

run:  ## 运行全部演示（SITL，mock 飞控）
	$(CARGO) run -p brain-node

demo:  ## 运行单个演示：make demo DEMO=car
	$(CARGO) run -p brain-node -- --demo $(DEMO)

demo-async:  ## 运行 tokio 异步演示（需编译 async feature）
	$(CARGO) run -p brain-node --features async -- --demo async

example:  ## 运行某 crate 示例：make example CRATE=brain-core EXAMPLE=basic
	$(CARGO) run -p $(CRATE) --example $(EXAMPLE)

release:  ## 发布构建（brain-node）
	$(CARGO) build --locked --release -p brain-node

clean:  ## 清理构建产物
	$(CARGO) clean
