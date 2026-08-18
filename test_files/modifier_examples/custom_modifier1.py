# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.1",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", rev = "e177e340b8680deee77a145bf416caf6a8da0f6c"}
# ///
"""Test the use of a higher-order function with complex control flow inside modifiers"""

from pathlib import Path
from sys import argv

from guppylang.std.builtins import array, control, dagger, nat, qubit
from guppylang import guppy
from guppylang.std.quantum import discard
from guppylang.std.debug import state_result

from hugr.hugr.render import RenderConfig
from guppylang import enable_experimental_features

enable_experimental_features()


@guppy.unitary
class foo:
    @guppy(unitary=True)
    def __call__(x: int) -> None:
        pass

    @guppy
    def daggered(x: int) -> None:
        pass

    @guppy
    def controlled[c: nat](x: int, _controls: array[qubit, c]) -> None:
        _x = 4


@guppy
def main() -> None:
    q = qubit()
    c = qubit()
    with control(q, c):
        foo(2)

    state_result("r", q, c)
    discard(q)
    discard(c)


main.with_minimal_opt().compile().modules[0].render_dot(
    RenderConfig(max_node_label_length=None)
).view("aaaa")
program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
