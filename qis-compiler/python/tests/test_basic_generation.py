from pathlib import Path

import pytest
from hugr.ops import CFG
from hugr.package import Package
from pytest_snapshot.plugin import Snapshot
from selene_hugr_qis_compiler import HugrReadError, check_hugr, compile_to_llvm_ir

resources_dir = Path(__file__).parent / "resources"

triples = [
    "x86_64-unknown-linux-gnu",
    "x86_64-apple-darwin",
    # TODO: The test doesn't seem to like Apple Silicon, it throws a warning
    # > 'aarch64' is not a recognized processor for this target (ignoring processor)
    "aarch64-apple-darwin",
    "x86_64-windows-msvc",
]


def load(name: str) -> bytes:
    hugr_file = resources_dir / f"{name}.hugr"
    return hugr_file.read_bytes()


def test_check() -> None:
    """Test the check_hugr function to ensure it can load a HUGR envelope."""
    hugr_envelope = load("check")
    check_hugr(hugr_envelope)  # guppy produces a valid HUGR envelope!

    bad_number = hugr_envelope[1:]
    with pytest.raises(HugrReadError, match="Bad magic number"):
        check_hugr(bad_number)

    bad_end = hugr_envelope[:-1]
    with pytest.raises(HugrReadError, match="Premature end of file"):
        check_hugr(bad_end)

    package = Package.from_bytes(hugr_envelope)
    hugr = package.modules[0]
    hugr.add_node(CFG([], []))
    with pytest.raises(HugrReadError, match="CFG must have children"):
        check_hugr(package.to_str().encode("utf-8"))


def test_unsupported_pytket_ops() -> None:
    """Test the check_hugr function to ensure it flags unsupported pytket ops."""
    hugr_envelope = load("unsupported_pytket_ops")
    with pytest.raises(
        HugrReadError,
        match="Pytket op 'CSXdg' is not currently "
        "supported by the Selene HUGR-QIS compiler",
    ):
        check_hugr(hugr_envelope)


@pytest.mark.parametrize(
    "hugr_file",
    [
        "no_results",
        "flip_some",
        "discard_qb_array",
        "measure_qb_array",
        "print_array",
        "postselect_exit",
        "postselect_panic",
        "rus",
        "print_current_shot",
        "rng",
    ],
)
@pytest.mark.parametrize("target_triple", triples)
def test_llvm(snapshot: Snapshot, hugr_file: str, target_triple: str) -> None:
    hugr_envelope = load(hugr_file)
    ir = compile_to_llvm_ir(hugr_envelope, target_triple=target_triple)  # type: ignore[call-arg]
    snapshot.assert_match(ir, f"{hugr_file}_{target_triple}")


def test_entry_args() -> None:
    with pytest.raises(
        RuntimeError,
        match="Entry point function must have no input parameters",
    ):
        _ = compile_to_llvm_ir(load("entry_args"))


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
    ir = compile_to_llvm_ir(hugr_envelope, target_triple=target_triple)  # type: ignore[call-arg]
    snapshot.assert_match(ir, f"gpu_{target_triple}")
