# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang==1.0.1",
#     "guppylang-internals==1.0.1",
# ]
# ///

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.angles import angle
from guppylang.std.quantum import h, measure, qubit, rz, discard


@guppy
def main() -> None:
    q1 = qubit()
    q2 = qubit()
    h(q1)
    theta = angle(0.5)
    if measure(q1):
        rz(q2, theta)
    else:
        rz(q2, -theta)
    discard(q2)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
