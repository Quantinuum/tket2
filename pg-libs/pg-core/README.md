# pg-core
Core IR definitions for the `pg-libs` optimisation stack.
This crate provides:

- A `PauliGraph` IR that also serves as a quantum circuit representation, supporting common circuit operations.
- The `PGPass` trait, which defines passes over a `PauliGraph`.

`pg-core` provides the core types and pass trait used throughout the stack.