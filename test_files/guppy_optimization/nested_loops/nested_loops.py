# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang >=0.21.6",
# ]
# ///
"""A Guppy Bell-state example with no explicit function inputs or outputs."""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import array, owned, result
from guppylang.std.quantum import cz, h, measure, qubit


@guppy
def main() -> None:
    q1, q2, q3 = qubit(), qubit(), qubit()
    # Apply loops and unpack the result array
    q1, q2, q3 = f(array(q1, q2, q3))
    # Measure and provide results
    result("b1", measure(q1))
    result("b2", measure(q2))
    result("b3", measure(q3))


@guppy
def f(qs: array[qubit, 3] @ owned) -> array[qubit, 3]:
    """
    Demonstrates extended use of Python control flow constructs
    like loops inside Guppy functions. In total, this function
    applies the identity.

    The function must own the input qubits to be able to return
    them again.
    """
    for i in range(3):
        h(qs[i])
    for i in range(2):
        for j in range(3):
            cz(qs[j], qs[(j + 1) % 3])
    for i in range(3):
        h(qs[i])
    return qs


program = main.compile_function()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
