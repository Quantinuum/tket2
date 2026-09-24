# tk-pg-greedy-synth

`tk-pg-greedy-synth` provides `GreedySynthPass`, which uses a greedy heuristic to
synthesize a canonical Pauli graph. It processes commuting operation sets in
order and selects two qubit entangling gates (TQEs) that reduce the current
operations while taking nearby operations into account.

This pass is a performant reimplementation of pytket's
[`GreedyPauliSimp`](https://docs.quantinuum.com/tket/api-docs/passes.html#pytket.passes.GreedyPauliSimp),
which is based on
[PCOAST: A Pauli-based Quantum Circuit Optimization Framework](https://arxiv.org/abs/2305.10966).

The pass expects a canonical Pauli graph whose operations are grouped into
commuting sets, as produced by `CanonicalFormPass` followed by
`GroupCommutingOpsPass`.

The pass can output:

1. Single qubit Clifford gates
2. TQE gates
3. Single qubit rotation gates
4. Measurements
5. Resets
6. Conditional gates
7. Black boxes, which are preserved

`ParallelMode` controls whether Rayon evaluates candidate costs in parallel.

See [ARCHITECTURE.md](https://github.com/Quantinuum/tket2/blob/main/pg-libs/pg-passes/tk-pg-greedy-synth/ARCHITECTURE.md) for an overview of the internal design.
See [TUNING.md](https://github.com/Quantinuum/tket2/blob/main/pg-libs/pg-passes/tk-pg-greedy-synth/TUNING.md) for internal tuning constants.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or http://www.apache.org/licenses/LICENSE-2.0).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
