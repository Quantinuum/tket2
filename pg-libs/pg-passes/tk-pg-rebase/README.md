# tk-pg-rebase

`tk-pg-rebase` provides `RebaseTQEToZXPass`, which rewrites two qubit entangling
gates as `ZX` gates and single qubit Clifford gates. Other operations are left
unchanged.

By default, `ZX` is the only two qubit entangling gate kept unchanged. Other
gate types can also be allowed when creating the pass.

## License

This project is licensed under Apache License, Version 2.0 ([LICENCE][] or <https://www.apache.org/licenses/LICENSE-2.0>).

  [LICENCE]: https://github.com/quantinuum/tket2/blob/main/LICENCE
