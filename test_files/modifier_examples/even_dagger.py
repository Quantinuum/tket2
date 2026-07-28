# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0rc1",
# ]
# ///
"""Test that an even number of daggers is equivalent to no dagger at all"""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard, h, qubit, rx

enable_experimental_features()


@guppy(controllable=True)
def rotation(q: qubit, f: float) -> None:
    rx(q, angle(f))


@guppy
def main() -> None:
    c = qubit()
    q = qubit()
    flag = True

    with dagger, dagger:
        # cfg is normally forbidden in a dagger context
        if flag:
            rotation(c, 1 / 4)

    with dagger, dagger:
        f = 1 / 4
        with dagger:
            rx(c, angle(f))

    h(c)
    with dagger:
        with control(c):
            with dagger:
                # rotation is only `controllable`: fine since we have 2 daggers
                rotation(q, 1 / 3)

    state_result("r", c, q)

    discard(q)
    discard(c)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
