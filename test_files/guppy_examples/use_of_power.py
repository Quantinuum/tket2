# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang==1.1.3",
#     "guppylang-internals==1.1.3",
# ]
# ///
"""Example program that uses the `power` modifier (expected to be rejected by tket)."""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import control, power
from guppylang.std.quantum import angle, discard, h, qubit, rx

enable_experimental_features()


@guppy
def main() -> None:
    c1 = qubit()
    t = qubit()
    h(c1)
    with control(c1):
        a = angle(1 / 3)
        with power(2):
            rx(t, a)

    discard(c1)
    discard(t)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
