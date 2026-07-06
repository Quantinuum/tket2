# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang"
# ]
# ///
"""Testing modifier on a loaded pytket circuit"""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, h, qubit
from pytket import Circuit

enable_experimental_features()

# PyTket circuit
circ = Circuit(2)
circ.Rz(-0.5, 0)
circ.Ry(-0.5, 1)
circ.H(0)

guppy_circ = guppy.load_pytket("guppy_circ_2", circ, use_arrays=False)


@guppy
def main() -> None:
    q1 = qubit()
    q2 = qubit()
    c = qubit()
    h(c)
    with control(c), dagger:
        guppy_circ(q1, q2)

    state_result("r", c, q1, q2)
    discard(q1)
    discard(q2)
    discard(c)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
