# tk-pg-optimize

This crate contains 2 optimization passes for `PauliGraph`s.

## Passes

- `GroupCommutingOpsPass`

  Groups mutually commuting operations by inserting `SetBoundary` operations.
  Commutation is checked for rotations, measurements, resets, and conditional
  boxes; other operation types are treated as non-commuting.

- `RotationMergingPass`

  Greedily merges Pauli rotations with the same Pauli string when they commute
  with every operation between them. Clifford rotations are folded into
  tableaux.

Consider running `CanonicalFormPass` before either optimization pass so that
the input operations are in the expected form.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or <https://www.apache.org/licenses/LICENSE-2.0>).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
