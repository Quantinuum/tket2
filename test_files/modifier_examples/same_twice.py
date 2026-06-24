# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a5",
#    "guppylang-internals==1.0.0a5",
# ]
# ///
"""Testing modifying same function twice"""

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
from guppylang.std.angles import angle


enable_experimental_features()


@guppy(unitary=True)
def fun(q: qubit) -> None:
    x(q)


@guppy
def main() -> None:
    q = qubit()
    c = qubit()
    c1 = qubit()
    h(c)
    h(c1)

    with control(c):
        fun(q)
        with control(c1):
            fun(q)

    state_result("r", c, c1, q)
    discard(q)
    discard(c)
    discard(c1)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
