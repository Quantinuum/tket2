import pytest
from hugr import ops, tys
from hugr.build.function import Function

from tket.passes import Normalize


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
