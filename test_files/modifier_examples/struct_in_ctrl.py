# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.1.1",
# ]
# ///
"""A struct containing a qubit passed to a controlled function"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import control
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard, h, qubit, rx


@guppy.struct
class S:
    q: qubit


@guppy(controllable=True)
def use(s: S) -> None:
    rx(s.q, angle(1 / 3))


@guppy
def main() -> None:
    c = qubit()
    s = S(qubit())
    h(c)
    with control(c):
        use(s)

    state_result("r", c, s.q)

    discard(c)
    discard(s.q)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
