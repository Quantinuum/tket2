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

# Update hugrenv version, including discovery of new hashes.
# This change bumps the hugrenv version used in both devenv and CI.
update-hugrenv version:
    curl -L -o hugrenv.lock https://github.com/Quantinuum/hugrverse-env/releases/download/v{{version}}/hugrenv.lock

# Fetch hugrverse environment packages for the current platform and extract them
# to the provided directory.
fetch-hugrenv install_path:
    python -c '
import io
import json
import platform
import posixpath
import tarfile
import urllib.request
from pathlib import Path

install_path = Path(r"""{{install_path}}""").expanduser().resolve()
install_path.mkdir(parents=True, exist_ok=True)
lock = json.loads(Path("hugrenv.lock").read_text(encoding="utf-8"))
version = lock["version"]

os_name = platform.system().lower()
arch = platform.machine().lower()
if os_name == "linux":
    platform_key = "manylinux_2_28"
    arch_key = {"x86_64": "x86_64", "amd64": "x86_64", "aarch64": "aarch64", "arm64": "aarch64"}.get(arch)
elif os_name == "darwin":
    platform_key = "macosx_11_0"
    arch_key = {"x86_64": "x86_64", "amd64": "x86_64", "aarch64": "aarch64", "arm64": "aarch64"}.get(arch)
elif os_name == "windows":
    platform_key = "win"
    arch_key = {"x86_64": "amd64", "amd64": "amd64"}.get(arch)
else:
    platform_key = None
    arch_key = None

if platform_key is None or arch_key is None:
    raise SystemExit(f"Unsupported platform: os={os_name} arch={arch}")

for package in ("llvm", "tket"):
    try:
        lock["hashes"][platform_key][arch_key][package]
    except KeyError as err:
        raise SystemExit(
            f"Unsupported hugrenv target in lockfile: platform={platform_key} arch={arch_key} package={package}"
        ) from err
    target = f"{platform_key}_{arch_key}"
    url = f"https://github.com/Quantinuum/hugrverse-env/releases/download/v{version}/hugrenv-{package}-{target}.tar.gz"
    print(f"Downloading {package} from {url}")
    data = urllib.request.urlopen(url).read()
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
        members = []
        for member in tar.getmembers():
            stripped = member.name.split("/", 1)
            if len(stripped) == 1:
                continue
            relative_name = posixpath.normpath(stripped[1])
            if (
                not relative_name
                or relative_name == "."
                or relative_name.startswith("../")
                or relative_name.startswith("/")
            ):
                continue
            member.name = relative_name
            members.append(member)
        tar.extractall(path=install_path, members=members)

print("")
print(f"hugrenv {version} installed in {install_path}")
print("")
if os_name == "windows":
    p = str(install_path)
    print("PowerShell:")
    print(f"$env:TKET_C_API_PATH = \"{p}\"")
    print(f"$env:LLVM_SYS_211_PREFIX = \"{p}\"")
    print(f"$env:LIBCLANG_PATH = \"{p}\\\\lib\"")
    print(f"$env:PATH = \"{p}\\\\bin;{p}\\\\lib;{p}\\\\lib64;$env:PATH\"")
else:
    p = str(install_path)
    print("Bash/Zsh:")
    print(f"export TKET_C_API_PATH=\"{p}\"")
    print(f"export LLVM_SYS_211_PREFIX=\"{p}\"")
    print(f"export LIBCLANG_PATH=\"{p}/lib\"")
    print(f"export PATH=\"{p}/bin:$PATH\"")
    if os_name == "darwin":
        print(f"export DYLD_LIBRARY_PATH=\"{p}/lib:{p}/lib64:$DYLD_LIBRARY_PATH\"")
    else:
        print(f"export LD_LIBRARY_PATH=\"{p}/lib:{p}/lib64:$LD_LIBRARY_PATH\"")
'

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
