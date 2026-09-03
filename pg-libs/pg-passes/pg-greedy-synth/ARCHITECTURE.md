# Architecture

## Overview

```text
GreedySynthPass or GreedySynthSimdPass
└── Reducer<P, C>
    ├── uses PackedPGSlice
    ├── uses Frontier
    ├── uses GatePool
    ├── P: PackedBackend
    └── C: TQECostBackend
        └── GreedyCostBackend<K, W>
            ├── K: CostKernel
            └── W: WeightedSumStrategy
```

The pass chooses a backend and weight strategy, then creates a `Reducer`. The
reducer manages the packed lookahead data, the current frontier, candidate
scoring and the output graph.

## Entry points

`GreedySynthPass` is the scalar entry point. `GreedySynthSimdPass` is available
with the `simd` feature and uses portable SIMD operations. Both implement
`PGPass`. They differ only in the backend used for packed operations and cost
calculations.

The scalar pass uses `ScalarBackend`. The SIMD pass uses `SimdBackend`. Each
backend implements both `PackedBackend` and `CostKernel`, so the same backend
type is used to update packed data and to calculate raw cost changes.

## Main structs

### `Reducer<P, C>`

`Reducer` runs the synthesis. `P` updates the packed data and `C` scores TQE
candidates. The reducer creates a `PackedPGSlice`, a `Frontier` and a
`GatePool`.

It loads commuting sets into the lookahead slice, reduces the first visible set
and appends the resulting gates to the output graph. It also handles black boxes
and recursively synthesises conditional boxes. After reducing every operation
set, it synthesises the remaining Clifford tableau.

### `PackedPGSlice`

`PackedPGSlice` stores the visible lookahead operations as packed Z bits, X
bits, sign bits and operation masks. It also stores operation metadata, set
boundaries, black box tableaux and the trailing Clifford tableau.

Candidate scoring reads packed views from this slice. A candidate stops at the
first black box that it cannot pass. Once selected, the TQE is applied up to
that point through `PackedBackend`.

### `Frontier`

`Frontier` stores the first visible set, called the front set. It tracks the
Pauli support of each operation, its current reduction cost and the TQEs that
can reduce it. Operations are grouped by reduction cost.

The frontier samples candidate TQEs from the lowest nonzero cost bucket. After a
TQE is selected, it updates the affected Paulis, support sets and cost buckets
using precomputed transition tables. Operations that reach zero cost are
removed and converted to output gates.

`Frontier` also reduces pairs of Clifford tableau generators.

### `GatePool`

`GatePool` stores scored TQE candidates in cost order. Each candidate records
the frontier operation that produced it, its cost, its stopping point and the
versions of its two qubits.

When a selected TQE changes a qubit, `GatePool` increments that qubit's version.
Stale candidates can then be discarded lazily without scanning the whole pool.
`GatePool` chooses between candidates using their calculated cost and projected
TQE depth.

### `GreedyCostBackend<K, W>`

`GreedyCostBackend` implements `TQECostBackend` by combining `K` and `W`. `K`
generates raw cost change masks and `W` turns those masks into a weighted cost.
The backend also stores weights for black boxes and the trailing tableau.

```text
packed Z and X bits
        │
        ▼
   CostKernel
        │
        ├── increase mask
        └── decrease mask
                │
                ▼
      WeightedSumStrategy
                │
                ▼
         final TQE cost
```

## Main traits

| Trait | Role | Implementations |
| --- | --- | --- |
| `PackedBackend` | Applies tableau operations and TQEs to packed Pauli data | `ScalarBackend`, `SimdBackend` |
| `CostKernel` | Calculates increase and decrease masks for a TQE candidate | `ScalarBackend`, `SimdBackend` |
| `WeightedSumStrategy` | Prepares slice weights and combines the cost masks | `ExpandedSparseWeightedSum`, `ExpandedSimdWeightedSum`, `GroupedWeightedSum` |
| `TQECostBackend` | Calculates the full cost of TQE candidates | `GreedyCostBackend<K, W>` |

The expanded strategies store one weight for each packed position.
`ExpandedSparseWeightedSum` visits only the set bits in each mask, while
`ExpandedSimdWeightedSum` uses SIMD for dense masks. `GroupedWeightedSum`
groups contiguous positions that have the same weight, then counts the set bits
in each group.

## Reduction flow

1. The pass chooses a backend and weight strategy, then creates a `Reducer`.
2. The reducer loads commuting sets into a `PackedPGSlice`.
3. `Frontier` is rebuilt from the first visible set, and the cost backend
   prepares weights for the current slice.
4. Operations with zero cost are removed and converted to output gates.
5. `Frontier` samples candidate TQEs from the lowest nonzero cost bucket.
6. `TQECostBackend` scores each candidate up to its stopping point and
   `GatePool` stores the scored candidates.
7. The best candidate is selected using its cost and expected TQE depth.
8. `PackedPGSlice` and `Frontier` apply the selected TQE, and `GatePool`
   invalidates candidates that touch the changed qubits.
9. These steps repeat until the current set is complete, then the packed slice
   advances to the next set.
10. The remaining Clifford tableau is reduced with the same `Frontier`,
    `GatePool`, `PackedBackend` and `TQECostBackend` types.
