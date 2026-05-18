# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.14",
# ]
# ///
"""Controlling a function with internal while-loop control flow"""

from pathlib import Path
from sys import argv
import sys

from guppylang import guppy
from guppylang.std.angles import angle
from guppylang.std.builtins import control
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, h, qubit, rx

sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy(control=True)
def while_loop(q: qubit, count: int) -> None:
    """Exercise a control modifier over a while-loop CFG."""
    while count > 0:
        rx(q, angle(1 / 4))
        count = count - 1


@guppy
def main() -> None:
    c = qubit()
    t = qubit()
    count = 3
    h(c)
    with control(c):
        while_loop(t, count)

    state_result("r", c, t)
    discard(c)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
