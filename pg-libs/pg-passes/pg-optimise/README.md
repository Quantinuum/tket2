# pg-optimise

This crate contains 2 optimisation passes for `PauliGraph`s.

## Passes

- `GroupCommutingOpsPass`
Groups mutually commuting operations by inserting `SetBoundary` operations.
Commutation is checked for rotations, measurements, resets, and conditional
boxes; other operation types are treated as non-commuting.

- `RotationMergingPass`
Greedily merges Pauli rotations with the same Pauli string when they commute
with every operation between them. Clifford rotations are folded into
tableaux.

Consider running `CanonicalFormPass` before either optimisation pass so that
the input operations are in the expected form.
