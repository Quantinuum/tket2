# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a5",
#    "guppylang-internals==1.0.0a5",
# ]
# ///
"""Nested modifiers with multiple control qubits"""

from pathlib import Path
from sys import argv
import sys

from guppylang import guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, angle
from guppylang.std.quantum import h, rz, x

sys.path.append(str(Path(__file__).resolve().parents[1]))


from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def main() -> None:
    t = qubit()
    c1 = qubit()
    c2 = qubit()
    c3 = qubit()
    h(c1)
    x(c2)
    h(c3)
    x(t)
    with control(c1, c2):
        f = 1 / 3
        with dagger:
            a = angle(-f)
            with control(c3):
                x(t)
                rz(t, a)
                h(t)

    state_result("r", c1, c2, c3, t)
    discard(c1)
    discard(c2)
    discard(c3)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
