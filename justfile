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
    uv sync
    [[ -n "${TKET_JUST_INHIBIT_GIT_HOOKS:-}" ]] || uv run pre-commit install -t pre-commit

# Run the pre-commit checks.
check: _check_nextest_installed
    uv run pre-commit run --all-files

# Compile the wheels for the python package.
build:
    cd tket-py && uv run maturin build --release

# Run all the tests.
test: test-rust test-python
# Run all rust tests.
test-rust *TEST_ARGS: _check_nextest_installed
    uv run cargo nextest r --all-features {{TEST_ARGS}}
# Run all python tests.
test-python *TEST_ARGS:
    uv run maturin develop --uv
    uv run pytest {{TEST_ARGS}}

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

# Generate a test coverage report.
coverage: coverage-rust coverage-python
# Generate a test coverage report for the rust code.
coverage-rust *TEST_ARGS:
    uv run cargo llvm-cov --lcov >lcov.info {{TEST_ARGS}}
# Generate a test coverage report for the python code.
coverage-python *TEST_ARGS:
    uv run maturin develop
    uv run pytest --cov=./ --cov-report=html {{TEST_ARGS}}

# Run Rust unsoundness checks using miri
miri *TEST_ARGS:
    PROPTEST_DISABLE_FAILURE_PERSISTENCE=true MIRIFLAGS='-Zmiri-env-forward=PROPTEST_DISABLE_FAILURE_PERSISTENCE' cargo +nightly miri test {{TEST_ARGS}}

# Runs `compile-rewriter` on the ECCs in `test_files/eccs`
recompile-eccs:
    scripts/compile-test-eccs.sh

# Regenerates all hugr definitions inside `test_files/`
recompile-test-hugrs:
    @echo "---- Recompiling example guppy programs ----"
    just test_files/guppy_examples/recompile
    @echo "---- Recompiling optimization-target guppy programs ----"
    just test_files/guppy_optimization/recompile

# Generate serialized declarations for the tket extensions
gen-extensions:
    cargo run -p tket-qsystem gen-extensions -o tket-exts/src/tket_exts/data

# Update snapshot tests for both rust and python (requires `cargo-insta`)
update-snapshots: update-snapshots-rs update-snapshots-py
# Interactively update snapshot tests (requires `cargo-insta`)
update-snapshots-rs:
    cargo insta review
# Update python snapshot tests.
update-snapshots-py *TEST_ARGS:
    uv run maturin develop --uv
    uv run pytest --snapshot-update {{TEST_ARGS}}

# Build the sphinx API documentation
build-pydocs:
    cd tket-py/docs && uv run --group docs sphinx-build -b html . build

# Serve the docs html pages locally
serve-docs: build-pydocs
    npm exec serve tket-py/docs/build

# Clean up all generated files
clean-docs:
    rm -rf tket-py/docs/build
    rm -rf tket-py/docs/generated
    rm -rf tket-py/docs/jupyter_execute

clean-env:
    uv clean
    cargo clean
