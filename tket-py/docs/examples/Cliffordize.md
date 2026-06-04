# Cliffordize for debugging

`Cliffordize` is a debug-only pass that replaces selected non-Clifford
operations with Clifford operations. The transformed program is not
semantically equivalent to the input program, but it is useful when a workflow
needs a Clifford-only approximation, such as testing Stim-backed simulation
tooling.

The initial replacement set covers these `tket.quantum` operations:

| Input operation | Replacement |
| --------------- | ----------- |
| `T`             | `S`         |
| `Tdg`           | `Sdg`       |

Parameterized `tket.qsystem` rotations such as `Rz`, `PhasedX`, and `ZZPhase`
are not rewritten by this initial pass. Their Cliffordization requires choosing
replacement angles, so they are left unchanged rather than silently applying a
broader approximation.

```python
from pytket import Circuit
from tket._state import CompilationState
from tket.passes import Cliffordize
from tket_exts import tket_registry
from hugr import Hugr

state = CompilationState.from_tket1(Circuit(1).T(0).Tdg(0))
hugr = Hugr.from_str(state.to_str(), tket_registry())

result = Cliffordize().run(hugr, inplace=False)
print(result.results[-1][1])  # Number of replacements applied.
```
