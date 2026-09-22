# List the available commands
help:
    @just --list --justfile {{justfile()}}

# Rust nightly toolchain version used across the workspace
nightly_toolchain := "nightly-2026-09-22"

_check_nextest_installed:
    #!/usr/bin/env bash
    cargo nextest --version >/dev/null 2>&1 || { echo "❌ cargo-nextest not found. Install binary from https://nexte.st/docs/installation/pre-built-binaries/"; exit 1; }

# Create the default conan profile if it doesn't exist.
_check_default_conan_profile:
    #!/usr/bin/env bash
    uvx conan profile list | grep "default" >/dev/null 2>&1
    if [ $? -ne 0 ]; then
        uvx conan profile detect
    fi

# Prepare the environment for development, installing all the dependencies and
# setting up the pre-commit hooks.
setup: && _check_default_conan_profile _check_nextest_installed
    uv tool install conan
    uv sync --all-extras --group pg-libs
    [[ -n "${TKET_JUST_INHIBIT_GIT_HOOKS:-}" ]] || uv run pre-commit install -t pre-commit

# Check both projects' Rust and Python code.
check: check-rust check-python

# Run all pre-commit hooks, including tests and repository hygiene checks.
check-pre-commit: _check_nextest_installed
    uv run pre-commit run --all-files

# Auto-fix all clippy warnings.
fix: fix-rust fix-python
# Auto-fix all rust clippy warnings.
fix-rust:
    uv run cargo clippy --all-targets --all-features --workspace --exclude 'pg-*' --fix --allow-staged --allow-dirty
    cargo clippy --all-targets -p 'pg-*' --fix --allow-staged --allow-dirty
# Auto-fix all python clippy warnings.
fix-python:
    uv run ruff check --fix

# Format the code.
format: format-rust format-python
# Format the rust code.
format-rust:
    cargo fmt --all
# Format the python code.
format-python:
    uv run ruff format

# Update hugrenv version, including discovery of new hashes.
# This change bumps the hugrenv version used in both devenv and CI.
update-hugrenv version:
    curl -L -o hugrenv.lock https://github.com/Quantinuum/hugrverse-env/releases/download/v{{version}}/hugrenv.lock

# Fetch hugrverse environment packages for the current platform and extract them
# to the provided directory.
fetch-hugrenv install_path='./target/hugrenv/':
    python scripts/fetch_hugrenv.py "{{install_path}}"

clean-env:
    uv clean
    cargo clean

# Run both projects' test suites (stable features for pg-libs).
test:
    @just --justfile tket/justfile test
    @just --justfile pg-libs/justfile test

# Run both projects' Rust tests, forwarding arguments to each runner.
[positional-arguments]
test-rust *TEST_ARGS:
    @just --justfile tket/justfile test-rust "$@"
    @just --justfile pg-libs/justfile test-rust "$@"

# Run Python tests (currently provided by tket).
[positional-arguments]
test-python *TEST_ARGS:
    @just --justfile tket/justfile test-python "$@"

# Check both projects' Rust code.
check-rust:
    @just --justfile tket/justfile check-rust
    @just --justfile pg-libs/justfile check-rust

# Check both projects' Python code.
check-python:
    @just --justfile tket/justfile check-python
    @just --justfile pg-libs/justfile check-python
