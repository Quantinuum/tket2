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

from guppylang import guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, angle
from guppylang.std.quantum import h, rz

sys.path.append(str(Path(__file__).resolve().parents[1]))


from guppylang.experimental import enable_experimental_features

enable_experimental_features()

hugr_pdf_directory = Path(__file__).resolve().parents[1] / "0_hugr_pdf"
hugr_pdf_directory.mkdir(exist_ok=True)


@guppy
def main() -> None:
    t = qubit()
    c1 = qubit()
    c2 = qubit()
    c3 = qubit()
    h(c1)
    h(c2)
    h(c3)
    h(t)
    with control(c1, c2):
        with control(c3):
            with dagger:
                rz(t, angle(1 / 2))

    state_result("r", c1, c2, c3, t)
    discard(c1)
    discard(c2)
    discard(c3)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
