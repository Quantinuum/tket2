# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a8",
# ]
# ///
"""Testing a dagger modifier on multiple functions, to ensure that the dagger is
reversing the order of quantum operations"""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.angles import angle
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, rx, s

enable_experimental_features()


@guppy
def get_f() -> float:
    return 1 / 3


@guppy(unitary=True)
def foo1(q: qubit) -> None:
    rx(q, angle(1 / 2))


@guppy(unitary=True)
def foo2(q: qubit) -> None:
    s(q)
    rx(q, angle(1 / 6))


@guppy(unitary=True)
def foo3(q: qubit, f: float) -> None:
    rx(q, angle(f / 2))


@guppy
def main() -> None:
    c = qubit()
    t = qubit()

    with dagger:
        with control(c):
            f = get_f()
            foo2(t)
            foo3(t, f)
        foo1(c)

    state_result("r", c, t)
    discard(t)
    discard(c)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
