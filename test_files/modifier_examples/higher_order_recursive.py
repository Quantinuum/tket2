# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a5",
#    "guppylang-internals==1.0.0a5",
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
from guppylang.std.quantum import discard, qubit
from guppylang.std.quantum import h, s, x
from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy(unitary=True)
def apply(f: Unitary[[qubit], None], q: qubit, c1: qubit) -> None:
    apply1(f, q, c1)


@guppy(unitary=True)
def apply1(f: Unitary[[qubit], None], q: qubit, c1: qubit) -> None:
    f(q)
    with control(c1):
        apply2(f, q)


@guppy(unitary=True)
def apply2(f: Unitary[[qubit], None], q: qubit) -> None:
    f(q)


@guppy
def main() -> None:
    q = qubit()
    c = qubit()
    c1 = qubit()
    h(c)
    h(c1)

    with control(c), dagger:
        apply(s, q, c1)
        apply(h, q, c1)

    state_result("r", c, c1, q)
    discard(q)
    discard(c)
    discard(c1)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
