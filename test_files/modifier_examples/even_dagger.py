# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.13",
# ]
# ///
"""An example with an even number of daggers, which should cancel out"""

from pathlib import Path
from sys import argv
import sys

from hugr.hugr.render import RenderConfig
from guppylang import guppy
from guppylang.std.builtins import dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, angle
from guppylang.std.quantum import rx

sys.path.append(str(Path(__file__).resolve().parents[1]))
from utility import hugr_pdf_directory

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy(unitary=True)
def rotation(q: qubit) -> None:
    rx(q, angle(1 / 4))


@guppy
def main() -> None:
    t = qubit()

    # Double dagger cancels out: dagger(dagger(rotation)) == rotation
    with dagger:
        with dagger:
            rotation(t)

    state_result("r", t)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
program.modules[0].render_dot(RenderConfig(display_node_id=True)).render(
    argv[0].removesuffix(".py") + "_before", directory=hugr_pdf_directory, cleanup=True
)
