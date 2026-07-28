# pg-bitpacked

This crate implements Clifford conjugation of Pauli operators using bit-packed representations.

- `kernels.rs`: template functions for conjugation.
- `slice.rs`: conjugation over slices of bit-packed Paulis.
- `simd.rs`: conjugation over SIMD vectors of bit-packed Paulis.

## SIMD (nightly)

The `simd` feature enables `std::simd` via the `portable_simd` nightly feature gate. This requires a nightly toolchain (tested with `nightly-2025-09-14`).
