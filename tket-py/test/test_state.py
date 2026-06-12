from semver import Version

from hugr import tys
from hugr.ext import Extension, OpDef, OpDefSig
from hugr.build.dfg import Function
from hugr.hugr import Hugr
from tket._state import (
    CompilationState,
)


def _custom_extension_hugr() -> Hugr:
    """Build a small HUGR using a Python-defined extension op."""
    extension = Extension("test.custom", Version.parse("0.1.0"))
    op_def = extension.add_op_def(
        OpDef(
            "gate",
            OpDefSig(tys.FunctionType([tys.Qubit], [tys.Qubit])),
        )
    )

    fn = Function("custom_op", [tys.Qubit])
    [q] = fn.inputs()
    [q] = fn.add_op(op_def.instantiate()).outputs()
    fn.set_outputs(q)
    return fn.hugr


def test_custom_ext_roundtrip() -> None:
    state = CompilationState.from_python(_custom_extension_hugr())

    binary_roundtrip = CompilationState.from_bytes(state.to_bytes()).to_python()
    text_roundtrip = CompilationState.from_str(state.to_str()).to_python()

    assert "test.custom" in binary_roundtrip.used_extensions().ids()
    assert "test.custom" in text_roundtrip.used_extensions().ids()
