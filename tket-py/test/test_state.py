import json

from hugr import tys
from hugr.build.dfg import Function
from hugr.ext import Extension, OpDef, OpDefSig
from hugr.hugr import Hugr
from hugr.std import _std_extensions
from semver import Version
from tket_exts import tket_registry

from tket._state import CompilationState
from tket._tket import state as native_state


def _custom_extension_hugr() -> Hugr:
    """Build a small HUGR using a Python-defined extension op."""
    return _extension_hugr("test.custom", Version.parse("0.1.0"))


def _extension_hugr(name: str, version: Version) -> Hugr:
    """Build a small HUGR using the requested extension name and version."""
    extension = Extension(name, version)
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


def _bundled_extensions(state: CompilationState) -> set[tuple[str, Version]]:
    """Read extension identities from the default text envelope payload."""
    payload = state.to_str()[10:]
    extensions, _ = json.JSONDecoder().raw_decode(payload)
    return {(ext["name"], Version.parse(ext["version"])) for ext in extensions}


def _compatibility_band(version: Version) -> tuple[int, ...]:
    """Return the caret-semver compatibility band for a stable version."""
    if version.major != 0:
        return (version.major,)
    if version.minor != 0:
        return (version.major, version.minor)
    return (version.major, version.minor, version.patch)


def test_custom_ext_roundtrip() -> None:
    state = CompilationState.from_python(_custom_extension_hugr())

    binary_roundtrip = CompilationState.from_bytes(state.to_bytes()).to_python()
    text_roundtrip = CompilationState.from_str(state.to_str()).to_python()

    assert "test.custom" in binary_roundtrip.used_extensions().ids()
    assert "test.custom" in text_roundtrip.used_extensions().ids()


def test_newer_embedded_tket_extension_is_bundled() -> None:
    """A newer Python extension cannot be replaced by Rust's older definition."""
    embedded = tket_registry().get_extension("tket.globals")
    newer_version = embedded.version.bump_patch()
    hugr = _extension_hugr(str(embedded.name), newer_version)

    state = CompilationState.from_python(hugr)

    assert (str(embedded.name), newer_version) in _bundled_extensions(state)


def test_newer_embedded_std_extension_is_bundled() -> None:
    """Newer standard extensions must also be present in the wire package."""
    embedded = _std_extensions().get_extension("logic")
    newer_version = embedded.version.bump_patch()
    hugr = _extension_hugr(str(embedded.name), newer_version)

    state = CompilationState.from_python(hugr)

    assert (str(embedded.name), newer_version) in _bundled_extensions(state)


def test_tket_exts_registry_matches_embedded_tket_extensions() -> None:
    """Keep Python and Rust tket definitions in compatible version bands."""
    std_ids = _std_extensions().ids()
    python_extensions = {
        (str(ext.name), _compatibility_band(ext.version))
        for ext in tket_registry().all_extensions
    }
    rust_extensions = {
        (name, _compatibility_band(Version.parse(version)))
        for name, version in native_state.embedded_extensions()
        if name not in std_ids
    }

    assert python_extensions == rust_extensions
