import pytest
from hugr import ops, tys
from hugr.build.function import Function

from tket.passes import Normalize


def test_normalize_used_and_unused_unpacks() -> None:
    """An unused unpack must not prevent removing the used pack/unpack pair."""
    function = Function("main", [tys.Bool, tys.Bool])
    packed = function.add(ops.MakeTuple()(*function.inputs()))
    used = function.add(ops.UnpackTuple()(packed))
    function.add(ops.UnpackTuple()(packed))
    function.set_outputs(used[1])

    result = Normalize(
        resolve_modifiers=False,
        simplify_cfgs=False,
        inline_funcs=False,
        remove_dead_funcs=False,
        inline_dfgs=False,
        constant_folding=False,
        squash_borrows=False,
        remove_redundant_order_edges=False,
    )(function.hugr)
    assert not any(
        isinstance(data.op, (ops.MakeTuple, ops.UnpackTuple))
        for _, data in result.nodes()
    )
    (input_node,) = [n for n, data in result.nodes() if isinstance(data.op, ops.Input)]
    (output_node,) = [
        n for n, data in result.nodes() if isinstance(data.op, ops.Output)
    ]
    assert list(result.linked_ports(output_node.inp(0))) == [input_node.out(1)]


@pytest.mark.parametrize("remove_tuple_untuple", [None, True, False])
@pytest.mark.parametrize("extra_field_use", [False, True])
def test_normalize_unpack_repack(
    remove_tuple_untuple: bool | None, extra_field_use: bool
) -> None:
    """Normalize removes redundant pairs when enabled, preserving linearity."""
    function = Function("main", [tys.Tuple(tys.Qubit, tys.Bool)])
    (tuple_input,) = function.inputs()
    unpack = function.add(ops.UnpackTuple()(tuple_input))
    repack = function.add(ops.MakeTuple()(*unpack))
    if extra_field_use:
        function.set_outputs(repack, unpack[1])
    else:
        function.set_outputs(repack)

    normalize = (
        Normalize()
        if remove_tuple_untuple is None
        else Normalize(remove_tuple_untuple=remove_tuple_untuple)
    )
    result = normalize(function.hugr)
    tuple_ops = [
        data.op
        for _, data in result.nodes()
        if isinstance(data.op, (ops.MakeTuple, ops.UnpackTuple))
    ]
    assert len(tuple_ops) == (
        2 if remove_tuple_untuple is False or extra_field_use else 0
    )
