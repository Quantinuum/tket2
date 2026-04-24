# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "guppylang ==0.21.13",
# ]
# ///
"""A simple controlled gate using modifiers"""

from pathlib import Path
from sys import argv
import sys

from guppylang import guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, qubit, angle, measure
from guppylang.std.quantum import h, rx, x
from hugr.hugr.render import RenderConfig

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
    h(c1)
    h(c2)
    with control(c1):
        d = fuu(2)
        with control(c2):
            # with dagger:
            rx(t, angle(1 / d))

    state_result("r", c1, c2, t)
    discard(c1)
    discard(c2)
    discard(t)


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
