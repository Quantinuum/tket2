# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", rev = "cc0977af918569ef9747bb07ca17fdfcbe8376e9"}
# ///
"""Test the use of a classical function inside modifiers"""

from pathlib import Path
from sys import argv
from collections.abc import Callable

from guppylang import enable_experimental_features, guppy
from guppylang.std.array import array_swap
from guppylang.std.builtins import array, control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard, h, measure, qubit, rx, x, z

enable_experimental_features()


@guppy
def dummy_fuu(i: int) -> int:
    # q = qubit()
    # c = qubit()
    # h(c)
    # # with control(c), dagger:
    # #     x(q)
    # #     z(q)
    # # if measure(q):
    # #     i = i + 1
    # discard(c)
    return i


@guppy
def main() -> None:
    t = qubit()
    c1 = qubit()
    c2 = qubit()
    arr = array(1, 1, 2, 1, 1)

    # Testing nested with_block with no quantum input
    # (issue: https://github.com/Quantinuum/tket2/issues/1814)
    # a = 3
    # with control(c1):
    #     with dagger:
    #         pass
    a = 3
    with control(c1):
        with dagger:
            dummy_fuu(a)

    state_result("r", c1, c2, t)
    discard(c1)
    discard(c2)
    discard(t)


program = main.with_minimal_opt().compile()
from hugr.hugr.render import RenderConfig

program.modules[0].render_dot(
    RenderConfig(
        max_node_label_length=None,
    )
).view("tmp0_base")
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
