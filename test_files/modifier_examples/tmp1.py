# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a8",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "main"}
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
def apply_unitary(f: Unitary[[qubit], None], ctrl: qubit, q: qubit) -> None:
    with dagger:
        with control(ctrl):
            f(q)


@guppy(unitary=True)
def foo(q: qubit) -> None:
    pass


@guppy
def main() -> None:
    q1 = qubit()
    q2 = qubit()
    apply_unitary(h, q1, q2)
    apply_unitary(foo, q1, q2)

    state_result("r", q1, q2)
    discard(q1)
    discard(q2)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
