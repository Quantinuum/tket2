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
from guppylang.std.quantum import discard, qubit, rz


@guppy
def apply_rz(q: qubit, theta: float) -> None:
    rz(q, angle(theta))


@guppy
def main() -> None:
    theta = 0.5
    q = qubit()
    apply_rz(q, theta)
    discard(q)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
