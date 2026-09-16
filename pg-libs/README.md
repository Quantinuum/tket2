# pg-libs

This workspace contains Rust crates for working with Pauli graphs.


## Development notes

- `pg-libs` has its own Rust and Python (`uv`) workspaces, separate from the repository root.
- Some Rust tests depend on `pytket` — see [`pg-tk`'s README](pg-converters/pg-tk/README.md) for details.
- **Python / uv**: To set up the Python environment, run `uv sync` from inside `pg-libs`. If you are using the devenv shell, prefer the `justfile` tasks (e.g. `just sync-uv`, `just fix-python`, `just format-python`) over invoking `uv` directly, since they unset the root environment variables before running so they don't pick up the repository root's Python environment.
- **Nightly toolchain**: The `simd` feature available to some crates requires a nightly Rust toolchain. If using the devenv shell, switch to the nightly profile (`devenv shell nightly`) so the toolchain is picked up automatically; otherwise use `cargo +nightly-2025-09-14 ...` as pinned in the `justfile`.
