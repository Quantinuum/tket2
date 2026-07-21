# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0rc1",
# ]
# ///
"""Testing nested modifiers

The hugr generated from this script is also used to benchmark the performance of modifier passes resolver
"""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard, h, qubit, ry

enable_experimental_features()


@guppy
def main() -> None:
    c1 = qubit()
    t = qubit()
    h(c1)
    with control(c1):
        with dagger:
            ry(t, angle(1 / 3))

    state_result("r", c1, t)
    discard(c1)
    discard(t)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
