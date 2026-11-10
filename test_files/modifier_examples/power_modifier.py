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
from guppylang.std.quantum import angle, discard, h, qubit, x

enable_experimental_features()


@guppy(daggerable=True)
def fun(q: qubit, i: nat, p: qubit) -> None:
    pass


@guppy
def inner(
    c1: qubit, c2: qubit, i: nat, q: qubit
) -> None:  # , c2: qubit, t: qubit) -> None:
    # c1 = qubit()
    # t = qubit()
    # h(c1)
    with power(i), power(2):  # dagger, power(i):
        # j = nat(i + 1)
        # with power(j):
        # i = nat(i + 1)
        fun(c1, i, q)

    with control(c1):
        x(q)
        with power(i):
            x(c2)
            with control(c2):  # , dagger:
                x(q)

    # with dagger:
    #     h(t)


@guppy
def main() -> None:
    q1 = qubit()
    q2 = qubit()
    q3 = qubit()
    inner(q1, q2, 3, q3)

    state_result("r", q1)
    discard(q1)
    discard(q2)
    discard(q3)
    # discard(t)


program = main.with_minimal_opt().compile_function()
# from hugr.hugr.render import RenderConfig

# program.modules[0].render_dot(RenderConfig(max_node_label_length=None)).view(
#     "triple_modifier"
# )
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
