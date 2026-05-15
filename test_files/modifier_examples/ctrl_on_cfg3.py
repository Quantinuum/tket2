# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.14",
# ]
# ///
"""Controlling a function with internal control flow"""

from pathlib import Path
from sys import argv
import sys

from guppylang import guppy
from guppylang.std.builtins import control
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, h, qubit, rx
from guppylang.std.angles import angle

sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy(control=True)
def looppy(q: qubit, flag: int) -> None:
    for _ in range(flag):
        rx(q, angle(1 / 4))


@guppy
def main() -> None:
    c = qubit()
    t = qubit()
    flag = 3
    h(c)
    with control(c):
        looppy(t, flag)

    state_result("r", c, t)
    discard(c)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
