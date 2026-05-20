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
from guppylang.std.builtins import dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, s, rx
from guppylang.std.angles import angle

sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def foo(x: int) -> float:
    return 1 / x


@guppy
def main() -> None:
    t = qubit()
    flag = False

    f = -foo(2)
    with dagger:
        if flag:
            rx(t, angle(f))
        else:
            s(t)
            rx(t, angle(f))

    state_result("r", t)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
