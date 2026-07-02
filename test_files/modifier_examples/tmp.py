# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a8",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "main"}
# ///
"""Subscript indexing in dagger and control context"""

from pathlib import Path
from sys import argv
import tket

from guppylang import enable_experimental_features, guppy
from guppylang.std.builtins import control, dagger
from guppylang.std.debug import state_result
from guppylang.std.quantum import h, qubit, cx, discard

enable_experimental_features()


@guppy(unitary=True)
def foo(q: qubit) -> None:
    h(q)
    with dagger:
        h(q)
    with dagger:
        h(q)


@guppy
def main() -> None:
    q1 = qubit()
    q2 = qubit()
    # with dagger:
    #     foo(q1)
    #     with control(q1):
    #         foo(q2)

    # the problem is here:
    cx(q1, q2)
    with control(q1):
        foo(q2)
    # ------

    # with dagger:
    #     h(q1)
    # with dagger:
    #     h(q1)

    state_result("r", q1, q2)

    discard(q1)
    discard(q2)


modifier_resolver = tket.passes.ModifierResolverPass()
program = (
    main.with_minimal_opt().compile()
)  # .with_optimization(modifier_resolver).compile()
Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
program.modules[0].render_dot().view("tmp")
