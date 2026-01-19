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


@guppy
def f(qs: array[qubit, 4] @ owned) -> array[qubit, 4]:
    for i in range(4):
        h(qs[(-i) % 4])
    for i in range(3):
        cx(qs[i], qs[i + 1])
    for i in range(3):
        cx(qs[2 - i], qs[3 - i])
    for i in range(4):
        h(qs[(3 * i + 1) % 4])
    return qs


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
