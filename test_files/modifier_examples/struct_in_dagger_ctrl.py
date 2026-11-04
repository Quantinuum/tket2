# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.1.1",
# ]
# ///
"""A struct containing a qubit passed to a daggered and controlled function"""

from pathlib import Path
from sys import argv

from guppylang import guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard, h, qubit, rx, s


@guppy.struct
class S:
    q: qubit


@guppy(unitary=True)
def use(st: S) -> None:
    s(st.q)
    rx(st.q, angle(1 / 3))


@guppy
def main() -> None:
    c = qubit()
    st = S(qubit())
    h(c)
    with dagger, control(c):
        use(st)

    state_result("r", c, st.q)

    discard(c)
    discard(st.q)


program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
