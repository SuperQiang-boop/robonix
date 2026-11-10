# SPDX-License-Identifier: MulanPSL-2.0
# Robonix top-level Makefile. Orchestrates Cargo workspace build + install.
#
# The Cargo workspace lives at the repo root and references Rust system
# components under system/ plus the Rust dev tools under tools/. Python system
# components (system/world) and Python reference services (services/*) are
# managed by uv from this same root.

.PHONY: help build release install clean fmt check pyrightconfig \
        build-src build-planner build-executor build-gateway build-robotdesc build-monitor
.DEFAULT_GOAL := help

BUILD_MODE ?= debug
CARGO_FLAGS := $(if $(filter release,$(BUILD_MODE)),--release,)

help:
	@echo "Robonix Makefile"
	@echo ""
	@echo "  Build:"
	@echo "    make build           - Build all Cargo workspace crates (debug)"
	@echo "    make release         - Build all Cargo workspace crates (release)"
	@echo "    make build-src     - Install robonix-src to ~/.cargo/bin"
	@echo "    make build-planner   - Install robonix-planner to ~/.cargo/bin"
	@echo "    make build-executor  - Install robonix-executor to ~/.cargo/bin"
	@echo "    make build-gateway   - Install robonix-gateway to ~/.cargo/bin"
	@echo "    make build-robotdesc      - Install robonix-robotdesc to ~/.cargo/bin"
	@echo "    make build-monitor    - Install robonix-monitor to ~/.cargo/bin"
	@echo ""
	@echo "  Install:"
	@echo "    make install         - Install all binaries to ~/.cargo/bin and register this repo via rbnx setup"
	@echo ""
	@echo "  Format & Lint:"
	@echo "    make fmt             - cargo fmt --all"
	@echo "    make check           - cargo fmt --check + clippy -D warnings"
	@echo ""
	@echo "  Clean:"
	@echo "    make clean           - cargo clean"

build:
	cargo build --workspace $(CARGO_FLAGS)

release:
	@$(MAKE) BUILD_MODE=release build

build-src:
	cargo install --force --path system/src    --bin robonix-src    $(CARGO_FLAGS)

build-planner:
	cargo install --force --path system/planner  --bin robonix-planner  $(CARGO_FLAGS)

build-executor:
	cargo install --force --path system/executor --bin robonix-executor $(CARGO_FLAGS)

build-gateway:
	cargo install --force --path system/gateway  --bin robonix-gateway  $(CARGO_FLAGS)

build-robotdesc:
	cargo install --force --path system/robotdesc     --bin robonix-robotdesc     $(CARGO_FLAGS)

build-monitor:
	cargo install --force --path system/monitor   --bin robonix-monitor   $(CARGO_FLAGS)

fmt:
	cargo fmt --all

check:
	cargo fmt --all -- --check
	cargo clippy --workspace -- -D warnings

install:
	cargo install --force --path tools/codegen   --bin robonix-codegen  $(CARGO_FLAGS)
	cargo install --force --path system/src    --bin robonix-src    $(CARGO_FLAGS)
	cargo install --force --path system/planner  --bin robonix-planner  $(CARGO_FLAGS)
	cargo install --force --path system/executor --bin robonix-executor $(CARGO_FLAGS)
	cargo install --force --path system/gateway  --bin robonix-gateway  $(CARGO_FLAGS)
	cargo install --force --path system/robotdesc     --bin robonix-robotdesc     $(CARGO_FLAGS)
	cargo install --force --path system/monitor   --bin robonix-monitor   $(CARGO_FLAGS)
	cargo install --force --path tools/rbnx      --bin rbnx             $(CARGO_FLAGS)
	@# Register this clone as the robonix source tree so packages anywhere
	@# on disk can resolve capabilities/IDL via `rbnx path`. By default this
	@# updates ~/.robonix/config.yaml; set ROBONIX_HOME=/path/to/dir to keep
	@# install/setup state isolated for CI or temporary runner workspaces.
	@REPO_ROOT="$$(pwd)"; \
	echo ""; \
	echo "[make install] registering robonix_source_path → $$REPO_ROOT"; \
	"$$HOME/.cargo/bin/rbnx" setup "$$REPO_ROOT" || { \
		echo "[make install] WARNING: 'rbnx setup' failed — run it manually:"; \
		echo "                cd $$REPO_ROOT && rbnx setup"; \
	}

clean:
	cargo clean
