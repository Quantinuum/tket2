#!/usr/bin/env bash

# This script calls `compile-rewriter` to recompile all ECC sets in the
# `test_files/eccs/` directory.
#
# Note that the output is not deterministic, so the resulting `.rwr` will always
# be different even if the input and the compiler are the same.
set -euo pipefail

# Get the directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ECCS_DIR="$DIR/../test_files/eccs"

# Command to call compile-rewriter with cargo
COMPILE_REWRITER="cargo run --release -p compile-rewriter --"

# Compile all json files into `.rwr`
for ecc in $ECCS_DIR/*.json; do
    echo "-------- Compiling $ecc"
    $COMPILE_REWRITER -i "$ecc" -o "${ecc%.json}.rwr"
done

# Additional hard-coded step:
# The python package contains a copy of the nam_6_3 ecc,
# which must be manually copied.
PY_ECCS_DIR="$DIR/../tket-eccs/src/tket_eccs/data"
nam_6_3="$ECCS_DIR/nam_6_3.rwr"
PY_NAM_6_3="$PY_ECCS_DIR/nam_6_3.rwr"

echo "Copying $nam_6_3 to $PY_NAM_6_3"
cp -f "$nam_6_3" "$PY_NAM_6_3"

echo "Done!"
