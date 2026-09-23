# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.1.1",
# ]
# ///
"""Some simple nested higher order functions inside modifiers"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import (
    Unitary,
    control,
    dagger,
)
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, h, qubit, s, x


@guppy(unitary=True)
def apply(f: Unitary[[qubit], None], q: qubit) -> None:
    apply1(f, q)


@guppy(unitary=True)
def apply1(f: Unitary[[qubit], None], q: qubit) -> None:
    apply2(f, q)


@guppy(unitary=True)
def apply2(f: Unitary[[qubit], None], q: qubit) -> None:
    f(q)


@guppy(controllable=True)
def recursive_apply_1(f: Unitary[[qubit], None], q: qubit, n: int) -> None:
    if n == 0:
        apply(f, q)
    else:
        recursive_apply_2(f, q, n)


@guppy(controllable=True)
def recursive_apply_2(f: Unitary[[qubit], None], q: qubit, n: int) -> None:
    recursive_apply_1(f, q, n - 1)


@guppy(controllable=True)
def apply_if(f: Unitary[[qubit], None], q: qubit, b: bool) -> None:
    if b:
        recursive_apply_1(f, q, 3)


@guppy
def main() -> None:
    q = qubit()
    c = qubit()
    x(c)
    flag = True
    with control(c):
        apply_if(x, q, flag)
        apply_if(h, q, not flag)

    h(c)
    with control(c), dagger:
        apply(s, q)
        apply(h, q)

    state_result("r", c, q)
    discard(q)
    discard(c)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
