from semver import Version

from hugr import tys
from hugr.ext import Extension, OpDef, OpDefSig
from hugr.build.dfg import Function
from hugr.hugr import Hugr
from hugr.std import _std_extensions
from tket._state import (
    CompilationState,
    embedded_extensions,
)
from tket_exts import tket_registry


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


def test_tket_exts_registry_matches_embedded_tket_extensions() -> None:
    """Keep tket-py's embedded extension registry in sync with tket_exts."""
    python_tket_ids = set(tket_registry().ids())
    prelude = set(_std_extensions().ids())

    rust_tket_ids = set(
        extension_id
        for extension_id in embedded_extensions()
        if extension_id not in prelude
    )

    # Currently missing from tket_exts
    # TODO: Add to tket_exts
    # <https://github.com/Quantinuum/tket2/issues/1693>
    rust_tket_ids.discard("TKET1")

    assert python_tket_ids == rust_tket_ids
