from pathlib import Path
from typing import Literal

import pytest
from hugr import tys
from hugr.build.dfg import Function
from hugr.ext import Extension, OpDef, OpDefSig
from hugr.hugr import Hugr
from hugr.ops import CFG
from hugr.package import Package
from pytest_snapshot.plugin import Snapshot
from selene_hugr_qis_compiler import (
    EmulatorState,
    HugrReadError,
)
from selene_hugr_qis_compiler import selene_hugr_qis_compiler as _native
from semver import Version
from tket_exts import modifier

resources_dir = Path(__file__).parent / "resources"

triples = [
    "x86_64-unknown-linux-gnu",
    "x86_64-apple-darwin",
    # TODO: The test doesn't seem to like Apple Silicon, it throws a warning
    # > 'aarch64' is not a recognized processor for this target (ignoring processor)
    "aarch64-apple-darwin",
    "x86_64-windows-msvc",
]

Platform = Literal["helios", "sol"]

platforms: list[Platform] = ["helios", "sol"]


def load(name: str) -> bytes:
    hugr_file = resources_dir / f"{name}.hugr"
    return hugr_file.read_bytes()


def contains_modifiers(hugr_envelope: bytes) -> bool:
    package = Package.from_bytes(hugr_envelope)
    for module in package.modules:
        for _, node_data in module.nodes():
            if (
                modifier.control.qualified_name() in node_data.op.name()
                or modifier.dagger.qualified_name() in node_data.op.name()
            ):
                return True
    return False


def extension_hugr(name: str, version: Version) -> Hugr:
    """Build a HUGR using an operation from a specific extension version."""
    extension = Extension(name, version)
    op_def = extension.add_op_def(
        OpDef("gate", OpDefSig(tys.FunctionType([tys.Qubit], [tys.Qubit])))
    )

    fn = Function("custom_op", [tys.Qubit])
    [q] = fn.inputs()
    [q] = fn.add_op(op_def.instantiate(), q).outputs()
    fn.set_outputs(q)
    return fn.hugr


def test_check() -> None:
    """Test that state construction loads and validates a HUGR envelope."""
    hugr_envelope = load("check")
    EmulatorState.from_bytes(hugr_envelope)

    bad_number = hugr_envelope[1:]
    with pytest.raises(HugrReadError, match="Bad magic number"):
        EmulatorState.from_bytes(bad_number)

    bad_end = hugr_envelope[:-1]
    with pytest.raises(HugrReadError, match="Premature end of file"):
        EmulatorState.from_bytes(bad_end)

    package = Package.from_bytes(hugr_envelope)
    hugr = package.modules[0]
    hugr.add_node(CFG([], []))
    with pytest.raises(ValueError, match="has no entry block"):
        EmulatorState.from_bytes(package.to_str().encode("utf-8"))


def test_emulator_state_validates_on_construction() -> None:
    with pytest.raises(
        HugrReadError,
        match="Pytket op 'CSXdg' is not currently "
        "supported by the Selene HUGR-QIS compiler",
    ):
        EmulatorState.from_bytes(load("unsupported_pytket_ops"))


def test_hugr_input_bundles_newer_extension() -> None:
    name, version = next(
        (name, Version.parse(version))
        for name, version in _native.embedded_extensions()
        if name == "logic"
    )
    newer_version = version.bump_patch()
    assert not _native.has_compatible_extension(name, str(newer_version))

    state = EmulatorState.from_python(extension_hugr(name, newer_version))

    assert state is not None


def test_emulator_state_accepts_package_input() -> None:
    package = Package.from_bytes(load("check"))

    ir = EmulatorState.from_python(package).compile_to_llvm_ir()

    assert "define i64 @qmain" in ir


def normalize_ir_snapshot(ir: str) -> str:
    """Remove unstable or localized output from IR snapshots."""
    # remove debug file entries with absolute paths
    new_lines = filter(lambda line: "DIFile" not in line, ir.split("\n"))
    return "\n".join(new_lines)


@pytest.mark.parametrize(
    "hugr_file",
    [
        "no_results",
        "flip_some",
        "discard_qb_array",
        "measure_qb_array",
        "postselect_exit",
        "postselect_panic",
        "rus",
        "print_current_shot",
        "rng",
        "simple_modifier",
    ],
)
@pytest.mark.parametrize("target_triple", triples)
@pytest.mark.parametrize("platform", platforms)
def test_llvm(
    snapshot: Snapshot, hugr_file: str, target_triple: str, platform: Platform
) -> None:
    hugr_envelope = load(hugr_file)
    state = EmulatorState.from_bytes(hugr_envelope, platform=platform)
    ir = state.compile_to_llvm_ir(target_triple=target_triple, emit_debug=True)
    ir = normalize_ir_snapshot(ir)
    snapshot.assert_match(ir, f"{hugr_file}_{target_triple}_{platform}")


def test_entry_args() -> None:
    with pytest.raises(
        RuntimeError,
        match="Entry point function must have no input parameters",
    ):
        _ = EmulatorState.from_bytes(load("entry_args")).compile_to_llvm_ir()


@pytest.mark.parametrize("platform", platforms)
def test_compile_modifiers(platform: Platform) -> None:
    hugr_envelope = load("simple_modifier")
    assert contains_modifiers(hugr_envelope)

    ir = EmulatorState.from_bytes(hugr_envelope, platform=platform).compile_to_llvm_ir(
        target_triple="x86_64-unknown-linux-gnu",
    )
    assert "define i64 @qmain" in ir
    assert "ControlModifier" not in ir
    assert "DaggerModifier" not in ir


# TODO: The stored hugr compiles to an empty function. It is likely missing
# visibility information on the main function.
@pytest.mark.skip(reason="Stored example .hugr is outdated, needs to be re-created.")
@pytest.mark.parametrize("target_triple", triples)
def test_gpu(snapshot: Snapshot, target_triple: str) -> None:
    # when we get GPU support in guppy, we might write something like:
    #
    # @gpu_module("example_module.so", None)
    # class Decoder:
    #     @gpu
    #     @no_type_check
    #     def fn_returning_int(
    #         self: "Decoder", a: int, b: float
    #     ) -> int: ...
    #
    #     @gpu
    #     def fn_returning_float(self: "Decoder", x: int) -> float: ...
    #
    # @guppy
    # def main() -> None:
    #     decoder = Decoder()
    #     a = decoder.fn_returning_int(42, 2.71828)
    #     b = decoder.fn_returning_float(a)
    #     result("a", a)
    #     result("b", b)
    #     decoder.discard()
    #
    # hugr_envelope = main.compile().to_bytes()

    # resources/example_gpu.hugr contains the equivalent HUGR to the
    # above, using the tket_qsystem::extension::gpu entities.
    hugr_file = resources_dir / "example_gpu.hugr"
    hugr_envelope = hugr_file.read_bytes()
    ir = EmulatorState.from_bytes(hugr_envelope).compile_to_llvm_ir(
        target_triple=target_triple
    )
    snapshot.assert_match(ir, f"gpu_{target_triple}")
