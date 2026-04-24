# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang @ git+https://github.com/Quantinuum/guppylang.git@na/fix-modifier-metadata#subdirectory=guppylang",
# ]
# ///
"""A simple controlled gate using modifiers"""

from pathlib import Path
from sys import argv
import sys

from hugr import Hugr
from hugr.hugr.render import RenderConfig
from guppylang import guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, angle, measure
from guppylang.std.quantum import h, rx, x

sys.path.append(str(Path(__file__).resolve().parents[1]))
from utility import hugr_pdf_directory

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def fuu(i: int) -> int:
    # q = qubit()
    # x(q)
    # if measure(q):
    #     i = i + 1
    return i


@guppy
def main() -> None:
    t = qubit()
    c1 = qubit()
    c2 = qubit()
    # h(c1)
    # x(c2)
    with control(c1):
        d = fuu(2)
        with control(c2):
            with dagger:
                x(t)  # rx(t, angle(1 / d))

    state_result("r", c1, c2, t)
    discard(c1)
    discard(c2)
    discard(t)


program = main.compile()


hugr_path = Path(argv[0]).with_suffix(".hugr")
hugr_bytes = program.to_bytes()
hugr_path.write_bytes(hugr_bytes)
Hugr.from_bytes(hugr_bytes).render_dot(RenderConfig(display_node_id=True)).render(
    hugr_path.stem, directory=hugr_pdf_directory, cleanup=True
)
