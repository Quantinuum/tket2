# pg-libs

This directory contains Rust crates for working with Pauli graphs.


## Development notes

- `pg-libs` and tket share the repository-root Cargo and uv workspaces, lockfiles,
  `target/` directory, and Python environment.
- Select the pg-libs crates with `cargo check -p 'tk-pg-*'`. Plain Cargo commands at
  the repository root select the tket default packages; `--workspace` selects
  both projects. The local just recipes select pg-libs explicitly.
- Some Rust tests depend on `pytket` — see [`tk-pg-converter`'s README](pg-converters/tk-pg-converter/README.md) for details.
- **Python / uv**: Run `just sync-uv` from this directory to install the
  `pg-libs` dependency group in the shared environment. This does not build
  tket's Python extensions or remove existing packages. `just test` (or
  `just pg-libs/test` from the root) runs the pg-libs suite. These commands also work in the devenv shell.
- **Checks**: Run `just check` from this directory (or `just pg-libs/check`
  from the root) to check Rust formatting, compilation, Clippy, documentation,
  and Python formatting, linting, and types. Use `check-rust` or `check-python`
  to select one language. Rust checks use stable features. The `format-rust`
  recipe formats the shared workspace.
- **Nightly toolchain**: The `simd` feature requires the nightly toolchain selected by `nightly_toolchain` in the root [`justfile`](../justfile). Run `just test-rust-nightly`; in devenv, use `devenv shell --profile nightly`.
