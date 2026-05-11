# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang @ git+https://github.com/Quantinuum/guppylang.git@main#subdirectory=guppylang",
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
from hugr import Hugr
from hugr.hugr.render import RenderConfig

sys.path.append(str(Path(__file__).resolve().parents[1]))

from guppylang.experimental import enable_experimental_features

enable_experimental_features()

hugr_pdf_directory = Path(__file__).resolve().parents[1] / "0_hugr_pdf"
hugr_pdf_directory.mkdir(exist_ok=True)


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
hugr_path = Path(argv[0]).with_suffix(".hugr")
hugr_bytes = program.to_bytes()
hugr_path.write_bytes(hugr_bytes)
Hugr.from_bytes(hugr_bytes).render_dot(
    RenderConfig(display_node_id=True, max_node_label_length=None)
).render(f"{hugr_path.stem}", directory=hugr_pdf_directory, cleanup=True)
