# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang @ git+https://github.com/Quantinuum/guppylang.git@na/allowing-assigment-in-dagger#subdirectory=guppylang",
# ]
# ///
"""A simple controlled gate using modifiers"""

from pathlib import Path
from sys import argv
import sys

from guppylang import array, guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, discard_array, qubit, angle
from guppylang.std.quantum import h, rx

sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def main() -> None:
    q = qubit()
    array_qubits: array[qubit, 2] = array(qubit(), qubit())

    with dagger:
        h(array_qubits[1])

    state_result("r", array_qubits[0], array_qubits[1], q)
    discard_array(array_qubits)
    discard(q)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
