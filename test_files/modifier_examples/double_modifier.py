# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.11",
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
from guppylang.std.quantum import h, rx, x
from hugr.hugr.render import RenderConfig

sys.path.append(str(Path(__file__).resolve().parents[1]))
from utility import hugr_pdf_directory

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy
def main() -> None:
    c1 = qubit()
    t = qubit()
    h(c1)
    with control(c1):
        with dagger:
            rx(t, angle(1 / 3))

    state_result("r", c1, t)
    discard(c1)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())


program.modules[0].render_dot(RenderConfig(display_node_id=True)).render(
    argv[0].removesuffix(".py") + "_before", directory=hugr_pdf_directory, cleanup=True
)
