# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.1",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", rev = "51a990e6e26170f3d814391cc8a7dc9a2fc17eff"}
# ///
"""Test the use of a higher-order function with complex control flow inside modifiers"""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import array, control, dagger, nat, qubit
from guppylang.std.debug import state_result
from guppylang.std.quantum import discard, discard_array, h, x

enable_experimental_features()

# NICOLA: TODO: Design a proper test


@guppy.unitary
class foo:
    @guppy(unitary=True)
    def __call__(q: qubit) -> None:
        h(q)

    # @guppy
    # def daggered(x: int) -> None:
    #     _x = 2

    @guppy
    def controlled[c: nat](q: qubit, _controls: array[qubit, c]) -> None:
        # with control(_controls):
        x(q)

    # @guppy
    # def ctrl_daggered[c: nat](q: qubit, _controls: array[qubit, c]) -> None:
    #     h(q)


@guppy
def main() -> None:
    q = qubit()
    c = qubit()
    cs = array(qubit())
    # with control(cs):  # noqa: SIM117
    #     with control(c):
    #         foo(q)
    # with dagger:
    #     foo(3)
    with control(cs):  # noqa: SIM117
        with control(c):
            foo(q)
    # testing that cached adapter are used properly
    with control(cs):  # noqa: SIM117
        with control(c):
            foo(q)

    state_result("r", q, c)
    discard(q)
    discard(c)
    discard_array(cs)


# from hugr.hugr.render import RenderConfig

# main.with_minimal_opt().compile().modules[0].render_dot(
#     RenderConfig(max_node_label_length=None)
# ).view("aaaa")
program = main.with_minimal_opt().compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
