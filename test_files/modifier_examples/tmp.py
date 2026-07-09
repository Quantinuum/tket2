# /// script
# requires-python = ">=3.13"
# dependencies = [
#    "guppylang==1.0.0a8",
# ]
# [tool.uv.sources]
# guppylang = {git = "https://github.com/quantinuum/guppylang", subdirectory = "guppylang", branch = "main"}
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
def foo() -> None:
    pass
    q = qubit()
    c = qubit()
    with control(c), dagger:
        x(q)
        z(q)
    discard(q)
    discard(c)


@guppy
def bar(q: qubit) -> None:
    # with control(q):
    #     with dagger:
    #         pass

    with control(q):
        with dagger:
            foo()


from tket.passes import NormalizeGuppy

normalize = NormalizeGuppy(
    resolve_modifiers=False,
    simplify_cfgs=True,
    remove_tuple_untuple=False,
    constant_folding=False,
    remove_dead_funcs=False,
    inline_funcs=False,
    inline_dfgs=False,
    remove_redundant_order_edges=False,
    squash_borrows=False,
)
program = bar.with_minimal_opt().with_optimization(normalize).compile_function()
from hugr.hugr.render import RenderConfig

# program.modules[0].render_dot(RenderConfig(max_node_label_length=None))
# .view(
#     "1", quiet=True
# )
normalize = NormalizeGuppy(
    resolve_modifiers=True,
    simplify_cfgs=True,
    remove_tuple_untuple=False,
    constant_folding=False,
    remove_dead_funcs=False,
    inline_funcs=False,
    inline_dfgs=False,
    remove_redundant_order_edges=False,
    squash_borrows=False,
)
normalize(program.modules[0]).render_dot(
    RenderConfig(max_node_label_length=None)
)  # ß.view(
# program.modules[0].render_dot(RenderConfig(max_node_label_length=None))
# .view(
#     "2", quiet=True
# )

# Path(argv[0]).with_suffix(".hugr").write_bytes(program.to_bytes())
