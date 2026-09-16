# pg-qm-tableau

A Clifford tableau implementation using a qubit-major memory layout.

## Memory layout

In a conventional tableau layout, each output Pauli string, often represented as a column, is stored contiguously in memory. This crate instead uses a row-oriented layout: for each output qubit, the corresponding Pauli letters from all output strings are stored contiguously in bit-packed form.

This layout is efficient for postcomposing Clifford gates because doing so updates only a small number of qubit sites. The trade-off is that image-oriented operations, such as conjugating Pauli strings and precomposing Clifford gates, are less efficient.

`converter.rs` implements the `PGTableau` trait from `pg-ir-kernels`, allowing `Tableau` to be used as the tableau backend for operations on `PauliGraph`.

## SIMD support

The optional `simd` feature provides the `SimdTableau` trait, which exposes SIMD implementations of the tableau operations using `std::simd`.

This feature requires the nightly Rust toolchain with the `portable_simd` feature enabled. Check the `README.md` in the `pg-libs/` directory for instructions on how to set up the nightly toolchain.
