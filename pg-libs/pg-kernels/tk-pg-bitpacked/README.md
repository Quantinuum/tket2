# tk-pg-bitpacked

This crate implements Clifford conjugation of Pauli operators using bit-packed representations.

- `kernels.rs`: template functions for conjugation.
- `slice.rs`: conjugation over slices of bit-packed Paulis.
- `simd.rs`: conjugation over SIMD vectors of bit-packed Paulis.

## SIMD (nightly)

The `unstable_simd` feature enables `std::simd` via the `portable_simd` nightly feature gate. This requires a nightly toolchain.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or <https://www.apache.org/licenses/LICENSE-2.0>).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
