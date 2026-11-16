# tk-pg-canonical-form

Transforms a `PauliGraph` into canonical form: non-Clifford gates are rewritten as Pauli rotations, and Clifford gates are commuted towards the end of the circuit (or the start, if running backward), merging into a tableau whenever they hit the boundary or an obstruction such as a `BlackBox`. As a result, the output may contain multiple tableaux, interleaved with any obstructions and remaining non-Clifford ops.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or <https://www.apache.org/licenses/LICENSE-2.0>).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
