# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.1",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", rev = "edd34171996d16f6aeafc77fbb463146b8a62a22"}
# ///
"""Test the use of a classical function inside modifiers"""

from collections.abc import Callable
from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.array import array_swap
from guppylang.std.builtins import array, control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard, h, measure, qubit, rx, x, z


@guppy
def fuu(i: int) -> int:
    q = qubit()
    x(q)
    if measure(q):
        i = i + 1
    return i


@guppy
def dummy_fuu(i: int) -> int:
    q = qubit()
    c = qubit()
    h(c)
    with control(c), dagger:
        x(q)
        z(q)
    if measure(q):
        i = i + 1
    discard(c)
    return i


@guppy
def inner(mk_struct: Callable[[int], int], x: int) -> int:
    return mk_struct(x)


@guppy(daggerable=True)
def foo(i: int) -> int:
    return i + 1


@guppy
def main() -> None:
    t = qubit()
    c1 = qubit()
    c2 = qubit()
    arr = array(1, 1, 2, 1, 1)

    # Testing that a classical higher order function can be called inside a modified context
    with control(c1):
        inner(foo, 2)

    # Testing nested with_block with no quantum input
    # (issue: https://github.com/Quantinuum/tket2/issues/1814)
    a = 3
    with control(c1), dagger:
        pass
    a = 3
    with control(c1):
        dummy_fuu(a)

    # Testing that array operations are happening in the correct order
    with control(t), dagger:
        arr[1] += 1
        arr[1] *= 2
    if arr[1] == 4:
        h(c1)

    # Test that array swap in control context works correctly
    array_swap(arr, 2, 4)
    with control(c2):
        array_swap(arr, 0, 4)
    if arr[0] == 2:
        h(c2)

    # Test that dagger and control does not affect the classical function
    with control(c1):
        d1 = fuu(2)
        i = 2
        d2 = fuu(i)
        d3 = fuu(i)
        with dagger, control(c2):
            d = (d1 + d2 + d3) / (i + 1)
            rx(t, angle(1 / d))

    state_result("r", c1, c2, t)
    discard(c1)
    discard(c2)
    discard(t)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
