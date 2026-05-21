# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.15",
# ]
# ///
"""Testing a dagger modifier on acyclic control flow"""

from pathlib import Path
from sys import argv
import sys

from guppylang import guppy
from guppylang.std.builtins import dagger, control
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, rz, s, qubit, rx, h, ry
from guppylang.std.angles import angle

sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def foo(x: int) -> float:
    return 1 / x


@guppy(unitary=True)
def funx(t: qubit, a: angle) -> None:
    rz(t, a)
    rx(t, a)


@guppy
def main() -> None:
    c = qubit()
    t = qubit()
    h(c)
    flag = True

    with control(c):
        with dagger:
            if flag:
                ry(t, angle(1 / 2))
                f = foo(2)
                funx(t, angle(f))
            else:
                s(t)

    state_result("r", c, t)
    discard(t)
    discard(c)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
