# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a8",
# ]
# ///
"""Multiple modifiers nested"""

from pathlib import Path
from sys import argv
from hugr.hugr.render import RenderConfig

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import angle, discard, h, qubit, rz, x

enable_experimental_features()


@guppy
def main() -> None:
    t = qubit()
    c1 = qubit()
    c2 = qubit()
    c3 = qubit()
    h(c1)
    x(c2)
    h(c3)
    x(t)
    with control(c1, c2):
        f = 1 / 6
        with dagger:
            a = angle(-f)
            with control(c3):
                x(t)
                rz(t, a)
                rz(t, angle(-1 / 6))
                h(t)

    state_result("r", c1, c2, c3, t)
    discard(c1)
    discard(c2)
    discard(c3)
    discard(t)


program = main.compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
program.modules[0].render_dot(RenderConfig(max_node_label_length=None)).view("tmp2")
