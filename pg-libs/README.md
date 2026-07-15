# pg-libs

This workspace contains Rust crates for working with Pauli graphs.


## Development notes

- `pg-libs` has its own Rust and Python (`uv`) workspaces, separate from the repository root.
- Some Rust tests depend on `pytket` — see [`pg-tk`'s README](pg-converters/pg-tk/README.md) for details.
- To set up the Python environment, run `uv sync` from inside `pg-libs`.
