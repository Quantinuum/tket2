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
from guppylang.std.quantum import discard, h, measure, qubit, x

sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy(unitary=True)
def branchy(q: qubit, flag: bool) -> None:
    if flag:
        x(q)
    else:
        h(q)


@guppy
def main() -> None:
    c = qubit()
    t = qubit()
    flag = False
    h(c)
    with control(c):
        branchy(t, flag)

    state_result("r", c, t)
    discard(c)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
