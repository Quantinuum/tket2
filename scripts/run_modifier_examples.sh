#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "==> Running just r in test_files/modifier_examples"
(
    cd "$repo_root/test_files/modifier_examples"
    just r
)

echo "==> Running Rust test test_saved_hugr"
(
    cd "$repo_root"
    cargo test -p tket test_saved_hugr
)

echo "==> Running just r in test_files/run_modifier_examples"
(
    cd "$repo_root/test_files/run_modifier_examples"
    just r
)
