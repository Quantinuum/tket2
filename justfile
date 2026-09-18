# List the available commands
help:
    @just --list --justfile {{justfile()}}

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
    uv sync --all-extras
    [[ -n "${TKET_JUST_INHIBIT_GIT_HOOKS:-}" ]] || uv run pre-commit install -t pre-commit

# Run the pre-commit checks.
check: _check_nextest_installed
    uv run pre-commit run --all-files

# Auto-fix all clippy warnings.
fix: fix-rust fix-python
# Auto-fix all rust clippy warnings.
fix-rust:
    uv run cargo clippy --all-targets --all-features --workspace --fix --allow-staged --allow-dirty
# Auto-fix all python clippy warnings.
fix-python:
    uv run ruff check --fix

# Format the code.
format: format-rust format-python
# Format the rust code.
format-rust:
    uv run cargo fmt
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

# Build the tket Python wheels.
build:
    @just --justfile tket/justfile build

# Run the tket Rust and Python tests.
test:
    @just --justfile tket/justfile test

# Run the tket Rust tests.
[positional-arguments]
test-rust *TEST_ARGS:
    @just --justfile tket/justfile test-rust "$@"

# Run the tket Python tests.
[positional-arguments]
test-python *TEST_ARGS:
    @just --justfile tket/justfile test-python "$@"

# Build the tket Python API documentation.
build-pydocs:
    @just --justfile tket/justfile build-pydocs
