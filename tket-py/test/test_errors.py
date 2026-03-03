import pytest
from hugr import ops, tys
from hugr.build.dfg import Function
from tket.circuit import Tk2Circuit


def test_unresolved_op() -> None:
    """Define a function with an unresolved op, and try to convert it to a Tk2Circuit."""
    fn = Function("unresolved_op", [tys.Qubit])
    [q] = fn.inputs()
    [q] = fn.add_op(
        ops.Custom(
            "unresolved",
            signature=tys.FunctionType([tys.Qubit], [tys.Qubit]),
            extension="unknown",
        )
    ).outputs()
    fn.set_outputs(q)
    hugr = fn.hugr

    # Loading the tk2circuit errors out since it cannot find the extension.
    with pytest.raises(RuntimeError) as excinfo:
        Tk2Circuit.from_bytes(hugr.to_bytes())

    # The error contains the full traceback, rather than just the top-level error message.
    err = str(excinfo.value)
    assert "Could not read tk2circuit from bytes" in err
    assert "Error reading package payload in envelope." in err
    assert "OpaqueOp:unknown.unresolved" in err
    assert "requires extension unknown" in err
