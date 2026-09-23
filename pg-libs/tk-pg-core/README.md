# tk-pg-core
Core IR definitions for the `pg-libs` optimization stack.
This crate provides:

- A `PauliGraph` IR that also serves as a quantum circuit representation, supporting common circuit operations.
- The `PGPass` trait, which defines passes over a `PauliGraph`.

`tk-pg-core` provides the core types and pass trait used throughout the stack.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or http://www.apache.org/licenses/LICENSE-2.0).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
