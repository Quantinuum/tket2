# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", rev = "cc0977af918569ef9747bb07ca17fdfcbe8376e9"}
# ///
"""Test the use of a higher-order function with complex control flow inside modifiers"""

from pathlib import Path
from sys import argv
from collections.abc import Callable

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import Controllable, Unitary, array, control, dagger
from guppylang.std.debug import state_result
from guppylang.std.lang import Function
from guppylang.std.quantum import angle, discard, h, qubit, rx, rz

enable_experimental_features()


@guppy
def get_angle(f: float) -> angle:
    return angle(f)


@guppy
def get_get_angle() -> Function[[float], angle]:
    return get_angle


@guppy(unitary=True)
def apply_c(
    g: Unitary[[qubit, angle], None],
    classic_fun: Function[[], Function[[float], angle]],
    q: qubit,
) -> None:
    get_a = classic_fun()
    # BUG if using:
    # angle = get_a(0.25)
    # # for _ in range(2):
    # # g(q, get_a(0.25))
    # # g(q, angle)


@guppy
def main() -> None:
    qs1 = qubit()
    with dagger:
        apply_c(rx, get_get_angle, qs1)

    state_result("r", qs1)
    discard(qs1)


program = main.compile()
from hugr.hugr.render import RenderConfig

# program.modules[0].render_dot(RenderConfig(max_node_label_length=None)).view(
#     "tmp0_base", quiet=True
# )
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
