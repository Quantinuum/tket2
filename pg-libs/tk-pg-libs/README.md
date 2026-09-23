# tk-pg-libs

Rust libraries for Pauli graph optimization and synthesis.

## Usage

Add `tk-pg-libs` to your `Cargo.toml`, using the path to your checkout:

```toml
[dependencies]
tk-pg-libs = { path = "/path/to/tket2/pg-libs/tk-pg-libs" }
```

Construct a two qubit circuit and apply the optimization and synthesis passes.
The two RZ rotations merge into a Clifford rotation and are folded into the tableau.
`GreedySynthPass` expects operations grouped into commuting sets by
`GroupCommutingOpsPass`.

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

The optional `unstable_simd` feature enables SIMD kernels and `passes::GreedySynthSimdPass`.
Use this pass in place of `GreedySynthPass` in the example above.

```toml
[dependencies]
tk-pg-libs = { path = "/path/to/tket2/pg-libs/tk-pg-libs", features = ["unstable_simd"] }
```

SIMD is experimental and requires a nightly toolchain. Build with
`cargo +nightly-2026-09-22 build`, or use `devenv shell --profile nightly` before building.
