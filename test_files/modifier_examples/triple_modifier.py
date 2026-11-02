# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.1.1",
# ]
# ///
"""Testing nested modifiers

The hugr generated from this script is also used to benchmark the performance of modifier passes resolver
"""

from pathlib import Path
from sys import argv

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import control, dagger, power
from guppylang.std.debug import state_result
from guppylang.std.num import nat
from guppylang.std.quantum import angle, discard, h, qubit, ry

enable_experimental_features()


@guppy
def fun(q: qubit, i: nat) -> None:
    pass


@guppy
def main(c1: qubit, i: nat) -> None:  # , c2: qubit, t: qubit) -> None:
    # c1 = qubit()
    # t = qubit()
    # h(c1)
    with power(i):  # , power(4):
        j = nat(i + 1)
        with power(j):
            fun(c1, j)

    # with control(c1):
    #     # with control(c2):  # , dagger:
    #     h(t)

    # with dagger:
    #     h(t)

    # state_result("r", c1, t)
    # discard(c1)
    # discard(t)


program = main.with_minimal_opt().compile_function()
# from hugr.hugr.render import RenderConfig

# program.modules[0].render_dot(RenderConfig(max_node_label_length=None)).view(
#     "triple_modifier"
# )
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
