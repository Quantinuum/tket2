# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a8",
# ]
# ///
"""Test the use of a classical function inside modifiers"""

from pathlib import Path
from sys import argv
from collections.abc import Callable

from guppylang import enable_experimental_features, guppy
from guppylang.std.array import array_swap
from guppylang.std.builtins import array, control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard, h, measure, qubit, rx, x

enable_experimental_features()


@guppy
def fuu(i: int) -> int:
    q = qubit()
    x(q)
    if measure(q):
        i = i + 1
    return i


@guppy
def inner(mk_struct: Callable[[int], int], x: int) -> int:
    return mk_struct(x)


@guppy
def foo(i: int) -> int:
    return i + 1


@guppy
def main() -> None:
    t = qubit()
    c1 = qubit()
    c2 = qubit()
    arr = array(1, 1, 2, 1, 1)

    # Testing that a classical higher order function can be called inside a modified context
    with dagger, control(c1):
        inner(foo, 2)

    # Testing that array operations are happening in the correct order
    with control(t), dagger:
        arr[1] += 1
        arr[1] *= 2
    if arr[1] == 4:
        h(c1)

    # Test that array swap in a dagger and control context works correctly
    with dagger:
        array_swap(arr, 2, 4)
        with control(c2):
            array_swap(arr, 0, 4)
    if arr[0] == 2:
        h(c2)

    # Test that dagger and control does not affect the classical function
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
