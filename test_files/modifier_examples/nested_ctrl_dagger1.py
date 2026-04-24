# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.13",
# ]
# ///
"""Nested control and dagger modifiers in various combinations"""

from pathlib import Path
from sys import argv
import sys

from hugr.hugr.render import RenderConfig
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
def rotation(q: qubit) -> None:
    rx(q, angle(1 / 4))


@guppy(unitary=True)
def flip(q: qubit) -> None:
    x(q)


@guppy
def main() -> None:
    c1 = qubit()
    c2 = qubit()
    c3 = qubit()
    t1 = qubit()
    t2 = qubit()

    h(c1)
    h(c2)
    x(c3)

    with control(c1):
        with control(c2):
            flip(t1)

    with control(c3):
        with dagger:
            rotation(t2)

    with dagger:
        with control(c1):
            rotation(t1)

    state_result("r", c1, c2, c3, t1, t2)
    discard(c1)
    discard(c2)
    discard(c3)
    discard(t1)
    discard(t2)


program = main.compile()

from hugr.ops import FuncDefn

hugr = program.modules[0]
for node, data in hugr.nodes():
    if isinstance(data.op, FuncDefn):
        if data.op.f_name.startswith("__WithBlock__"):
            data.metadata["unitary"] = 7

Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
program.modules[0].render_dot(RenderConfig(display_node_id=True)).render(
    argv[0].removesuffix(".py") + "_before", directory=hugr_pdf_directory, cleanup=True
)
