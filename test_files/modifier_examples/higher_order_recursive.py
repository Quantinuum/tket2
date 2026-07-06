# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a7",
# ]
# ///
"""Some simple nested higher order functions inside modifiers"""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import (
    Unitary,
    control,
    dagger,
)
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, h, qubit, s, x

enable_experimental_features()


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
def apply_if(f: Unitary[[qubit], None], q: qubit, b: bool) -> None:
    if b:
        apply(f, q)


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


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
