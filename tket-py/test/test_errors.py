import pytest
from hugr import ops, tys
from hugr.build.dfg import Function
from tket._state import CompilationState


def test_unresolved_op() -> None:
    """Define a function with an unresolved op, and try to convert it to a CompilationState."""
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

    # Loading the CompilationState errors out since it cannot find the extension.
    with pytest.raises(RuntimeError) as excinfo:
        CompilationState.from_bytes(hugr.to_bytes())

    # The error contains the full traceback, rather than just the top-level error message.
    err = str(excinfo.value)
    assert "Could not read CompilationState from bytes" in err
    assert "Error reading package payload in envelope." in err
    assert "unknown.unresolved" in err
    assert "requires extension unknown" in err
