# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import array, owned, result
from guppylang.std.quantum import cx, h, measure, qubit


@guppy
def main() -> None:
    q1, q2, q3, q4 = qubit(), qubit(), qubit(), qubit()
    q1, q2, q3, q4 = f(array(q1, q2, q3, q4))
    b1 = measure(q1)
    b2 = measure(q2)
    b3 = measure(q3)
    b4 = measure(q4)

    result("b1", b1)
    result("b2", b2)
    result("b3", b3)
    result("b4", b4)


# Version of f with flattened control flow but retaining array accesses
@guppy
def f(qs: array[qubit, 4] @ owned) -> array[qubit, 4]:
    h(qs[0])
    h(qs[3])
    h(qs[2])
    h(qs[1])

    cx(qs[0], qs[1])
    cx(qs[1], qs[2])
    cx(qs[2], qs[3])

    cx(qs[2], qs[3])
    cx(qs[1], qs[2])
    cx(qs[0], qs[1])

    h(qs[1])
    h(qs[0])
    h(qs[3])
    h(qs[2])
    return qs


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
