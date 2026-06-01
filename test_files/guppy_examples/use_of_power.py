# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.15",
# ]
# ///
"""Example program that uses the `power` modifier (expected to be rejected by tket)."""

from pathlib import Path
from sys import argv
import sys

from guppylang import guppy
from guppylang.std.builtins import control, power
from guppylang.std.quantum import discard, qubit, angle
from guppylang.std.quantum import h, rx

sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

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


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
