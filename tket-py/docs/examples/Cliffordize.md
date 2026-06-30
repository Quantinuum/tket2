# Cliffordize for debugging

`Cliffordize` replaces selected non-Clifford operations with deterministic
Clifford operations. It is intended for testing and debugging workflows that
need input suitable for Clifford-only tooling.

The transformation is **not semantics-preserving**.

The initial replacement table is:

| Input operation | Replacement |
| --------------- | ----------- |
| `tket.quantum.T` | `tket.quantum.S` |
| `tket.quantum.Tdg` | `tket.quantum.Sdg` |

Other operations are left unchanged. In particular, this pass does not replace
arbitrary or symbolic rotations, `PhasedX`, or `ZZPhase`.

```python
from tket._state.build import from_coms
from tket.passes import Cliffordize
from tket_exts import quantum

hugr = from_coms(quantum.T(0), quantum.H(0), quantum.Tdg(0)).to_python().modules[0]
result = Cliffordize().run(hugr, inplace=False)

print(result.results[-1][1])  # 2 replacements
```

By default, `Cliffordize` uses `GlobalScope.PRESERVE_PUBLIC`. Use
`with_scope(...)` to restrict which regions are rewritten:

```python
from hugr.passes.scope import LocalScope

local_result = Cliffordize().with_scope(LocalScope.FLAT).run(
    hugr, inplace=False
)
```
