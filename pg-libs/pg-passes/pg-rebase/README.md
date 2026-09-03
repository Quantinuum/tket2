# pg-rebase

`pg-rebase` provides `RebaseTQEToZXPass`, which rewrites two qubit entangling
gates as `ZX` gates and single qubit Clifford gates. Other operations are left
unchanged.

By default, `ZX` is the only two qubit entangling gate kept unchanged. Other
gate types can also be allowed when creating the pass.
