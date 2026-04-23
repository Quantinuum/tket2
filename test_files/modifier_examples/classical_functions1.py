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
from guppylang.std.quantum import discard, qubit, angle
from guppylang.std.quantum import h, rx, x

sys.path.append(str(Path(__file__).resolve().parents[1]))
from utility import hugr_pdf_directory

from guppylang.experimental import enable_experimental_features
from hugr.hugr.render import RenderConfig

enable_experimental_features()


@guppy
def fuu(i: int) -> int:
    return i + 1


@guppy
def main() -> None:
    q = qubit()
    h(q)
    with dagger:
        rx(q, angle(1 / fuu(2)))

    state_result("r", q)
    discard(q)


program = main.compile()

from hugr.ops import FuncDefn


# hugr = program.modules[0]
# for node, data in hugr.nodes():
#     if isinstance(data.op, FuncDefn):
#         if data.op.f_name.startswith("__WithBlock__"):
#             data.metadata["unitary"] = 7


Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())


program.modules[0].render_dot(RenderConfig(display_node_id=True)).render(
    argv[0].removesuffix(".py") + "_before", directory=hugr_pdf_directory, cleanup=True
)
