# tk-pg-converter

Conversions between `PauliGraph` and serialized TKET circuit format (JSON).

This crate provides:

- `pg_to_tk_json` / `pg_from_tk_json` — convert between `PauliGraph` and serialized TKET circuit.
- `compare_unitaries_via_tk` — verify that two `PauliGraph`s are unitarily equivalent by delegating to a `pytket` worker process.

The Python worker (`scripts/compare_unitaries_worker.py`) is embedded at compile time and spawned on demand. The interpreter is resolved from the `PG_TK_PYTHON` environment variable if set, falling back to `python` then `python3` on `PATH`.

In this repository, run tests with `uv run --only-group pg-libs cargo test -p tk-pg-converter`
to use the shared workspace environment with `pytket` installed. Set
`PG_TK_PYTHON` only when you want to use a different interpreter.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or http://www.apache.org/licenses/LICENSE-2.0).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
