# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a5",
#    "guppylang-internals==1.0.0a5",
# ]
# ///
"""An example with an even number of daggers, which should cancel out"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import dagger, control
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, angle
from guppylang.std.quantum import rx, h
from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy(unitary=True)
def rotation(q: qubit) -> None:
    rx(q, angle(1 / 4))


@guppy
def main() -> None:
    c = qubit()
    q = qubit()

    with dagger:
        with dagger:
            rotation(c)

    with dagger, dagger, dagger:
        rotation(c)

    h(c)
    with dagger:
        with control(c):
            with dagger:
                rx(q, angle(1 / 3))

    state_result("r", c, q)

    discard(q)
    discard(c)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
