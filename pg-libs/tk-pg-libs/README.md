# tk-pg-libs

An umbrella crate for pg-libs, TKET's Pauli graph libraries. It re-exports core
types, transformation and synthesis passes, and supporting utilities.

## Usage

This example uses Pauli graph passes to optimize and synthesize a two qubit circuit.

```rust
use tk_pg_libs::{GateData, GateType, Op, PauliGraph};
use tk_pg_libs::passes::{
    CanonicalFormPass, GreedySynthPass, GroupCommutingOpsPass, PGPass,
    RotationMergingPass,
};

let mut pg = PauliGraph::new(2);
for data in [
    GateData::new(GateType::RZ, vec![1]).with_params(vec![0.25]),
    GateData::new(GateType::ZX, vec![0, 1]), // CX(0, 1)
    GateData::new(GateType::ZX, vec![1, 0]), // CX(1, 0)
    GateData::new(GateType::RZ, vec![0]).with_params(vec![0.25]),
] {
    pg.add_op(Op::Gate { data });
}

let pg = CanonicalFormPass::new().transform(&pg);
let pg = RotationMergingPass::new().transform(&pg);
let pg = GroupCommutingOpsPass::new().transform(&pg);
let pg = GreedySynthPass::new().transform(&pg);
```

## Modules

| Module | Description |
| --- | --- |
| `core` | `PauliGraph` IR and the `PGPass` trait |
| `passes` | Canonical form, optimization, synthesis and rebasing passes |
| `tk_converter` | Conversions between `PauliGraph` and serialized TKET circuits |
| `qm_tableau` | Clifford tableaux using a qubit major memory layout |
| `bitpacked` | Clifford conjugation of Pauli operators using packed representations |
| `ir_kernels` | Basic rewrite operations and the `PGTableau` trait |
| `utils` | Utility functions for working with Pauli graphs |

## SIMD

Some experimental SIMD features can be enabled with `unstable_simd`. These require a nightly Rust toolchain. Build with
`cargo +nightly-2026-09-22 build`, or use `devenv shell --profile nightly` before building.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or <http://www.apache.org/licenses/LICENSE-2.0>).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
