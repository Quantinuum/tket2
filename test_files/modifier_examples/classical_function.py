# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a5",
#    "guppylang-internals==1.0.0a5",
# ]
# ///
"""Test the use of a classical function inside modifiers"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import array, control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, angle, measure
from guppylang.std.quantum import h, rx, x

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def fuu(i: int) -> int:
    q = qubit()
    x(q)
    if measure(q):
        i = i + 1
    return i


@guppy
def main() -> None:
    t = qubit()
    c1 = qubit()
    c2 = qubit()
    arr = array(1, 1, 1, 1, 1)

    with control(t), dagger:
        arr[0] += 1
        arr[0] *= 2

    # testing that array operations are happening in the correct order
    if arr[0] == 4:
        h(c1)
    h(c2)

    with control(c1):
        d1 = fuu(2)
        with dagger:
            i = 2
            d2 = fuu(i)
            d3 = fuu(i)
            with control(c2):
                d = (d1 + d2 + d3) / (i + 1)
                rx(t, angle(1 / d))

    state_result("r", c1, c2, t)
    discard(c1)
    discard(c2)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
