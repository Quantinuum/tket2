# TKET1-Passes

A bridge between C++ legacy TKET and Rust land. The folder `tket-c-api`
provides a minimal C-interface to a subset of TKET1's functionality. The
crate in this directory then wraps this interface into a Rust-friendly
API.

## Building instructions

Building `tket1-passes` is done using `conan >= 2.0`. This will be installed
automatically by `uv` if using the build tools `just` or `uv` (recommended; see
[DEVELOPMENT.md](../../DEVELOPMENT.md) for more detailed instructions). If you wish
to run `cargo build` directly, make sure that `conan` is installed and available
in your `PATH`.

If conan is unable to fetch all dependencies as pre-compiled binaries, you will
also need standard C++ tooling to compile the dependencies (i.e. a reasonably
recent version of cmake and a C++ compiler).

You can also eschew using conan altogether by setting the `TKET_C_API_PATH`
environment variable to point to a pre-compiled version of the `tket-c-api`
library. Note that this must be a dynamic library (the pre-built binaries on
conan on the other hand are static libraries).

## Currently supported TKET1 features

This library is currently limited to legacy TKET passes (a small subset of them, in fact).
Specifically, the following passes are supported:

- **`two_qubit_squash`** - Squash sequences of two-qubit operations using KAK decomposition
- **`clifford_resynthesis`** - Resynthesize Clifford subcircuits and simplify using Clifford rules
