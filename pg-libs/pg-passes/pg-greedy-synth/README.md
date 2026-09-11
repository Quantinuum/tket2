# pg-greedy-synth

`pg-greedy-synth` provides `GreedySynthPass`, which uses a greedy heuristic to
synthesise a canonical Pauli graph. It processes commuting operation sets in
order and selects TQEs that reduce the current operations while taking nearby
operations into account.

The pass can output:

1. Single qubit Clifford gates
2. TQE gates
3. Single qubit rotation gates
4. Measurements
5. Resets
6. Conditional gates
7. Black boxes, which are preserved

See [ARCHITECTURE.md](ARCHITECTURE.md) for an overview of the internal design.
