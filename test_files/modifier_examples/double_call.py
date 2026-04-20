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

sys.path.append(str(Path(__file__).resolve().parents[1]))
from utility import hugr_pdf_directory

from guppylang.experimental import enable_experimental_features

enable_experimental_features()


@guppy(unitary=True)
def bar(q: qubit) -> None:
    rx(q, angle(1 / 3))


@guppy
def main() -> None:
    c1 = qubit()
    t = qubit()
    c2 = qubit()
    c3 = qubit()
    h(c1)
    x(c2)
    x(c3)
    with control(c1, c2, c3):
        # with dagger:
        bar(t)

    state_result("r", c1, c2, c3, t)
    discard(c1)
    discard(t)
    discard(c3)
    discard(c2)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())


program.modules[0].render_dot().render(
    argv[0].removesuffix(".py") + "_before", directory=hugr_pdf_directory, cleanup=True
)
