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
def inner(mk_struct: "Callable[[int], int]", x: int) -> int:
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

    # testing that a classical higher order function can be called inside a modified context
    with dagger, control(c1):
        inner(foo, 2)

    # Testing that array operations are happening in the correct order
    # with control(t), dagger:
    #     arr[1] += 1
    #     arr[1] *= 2
    # if arr[1] == 4:
    #     h(c1)

    # Test that array swap in a dagger and control context works correctly
    # with dagger:
    #     array_swap(arr, 2, 4)
    #     with control(c2):
    #         array_swap(arr, 0, 4)
    # if arr[0] == 2:
    #     h(c2)

    # Test that dagger and control does not affect the classical function
    # with control(c1):
    #     d1 = fuu(2)
    #     with dagger:
    #         i = 2
    #         d2 = fuu(i)
    #         d3 = fuu(i)
    #         with control(c2):
    #             d = (d1 + d2 + d3) / inner(foo, i)
    #             rx(t, angle(1 / d))

    state_result("r", c1, c2, t)
    discard(c1)
    discard(c2)
    discard(t)


program = main.compile()
from tket.passes import NormalizeGuppy

normalize = NormalizeGuppy(
    remove_tuple_untuple=False,
    remove_dead_funcs=False,
    remove_redundant_order_edges=False,
    squash_borrows=False,
    inline_funcs=False,
    resolve_modifiers=False,
    # Causes errors in the notebook examples;
    # Emulation succeeds but shots lose expected result entries
    # (`eigenvalue` / `attempts`), producing `KeyError` in the
    # plotting cells.
    simplify_cfgs=True,
    # fails `test_arithmetic.py::test_float_to_int`
    # Selene reports package validation error:
    # `Node(...) has an unconnected port Port(Outgoing, 0)`
    constant_folding=False,
    # when combined with `inline_funcs=True` fails
    # `test_qsystem_sol_functional`: tket/portgraph panic
    # with `Outgoing port count exceeds maximum`
    inline_dfgs=False,
)
program = normalize(program.modules[0])
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
