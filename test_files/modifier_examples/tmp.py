# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0rc1",
# ]
# ///
from pathlib import Path
from sys import argv

from guppylang import guppy, qubit
from guppylang.std.builtins import Controllable, Unitary, control, dagger
from guppylang.std.quantum import h, measure
from hugr.hugr.render import RenderConfig


@guppy
def foo(q: int) -> bool:
    return measure(qubit()).read()


@guppy
def main() -> None:
    # apply_unitary(h, q1, q2)
    q1 = qubit()
    q2 = 2
    with control(q1):
        a = foo(q2)
        b = foo(5)
    measure(q1)


program = main.with_minimal_opt().compile()


# program.modules[0].render_dot(
#     RenderConfig(
#         max_node_label_length=None,
#     )
# ).view("a")
# Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
