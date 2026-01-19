from pathlib import Path

import pytest
from pytest_snapshot.plugin import Snapshot
from selene_hugr_qis_compiler import compile_to_llvm_ir #, HugrReadError, check_hugr

resources_dir = Path(__file__).parent / "resources"

triples = [
    "x86_64-unknown-linux-gnu",
    "x86_64-apple-darwin",
    # TODO: The test doesn't seem to like Apple Silicon, it throws a warning
    # > 'aarch64' is not a recognized processor for this target (ignoring processor)
    "aarch64-apple-darwin",
    "x86_64-windows-msvc",
]

platforms = ["Helios", "Sol"]


def load(name: str) -> bytes:
    hugr_file = resources_dir / f"{name}.hugr"
    return hugr_file.read_bytes()


@pytest.mark.parametrize(
    "hugr_file",
    [
        "no_results",
        "flip_some",
        "discard_qb_array",
        "measure_qb_array",
        "postselect_exit",
        "postselect_panic",
        #"rus", - 2q gates need some work
        "print_current_shot",
        "rng",
    ],
)
@pytest.mark.parametrize("target_triple", triples)
@pytest.mark.parametrize("platform", platforms)
def test_llvm_multiplatform(snapshot: Snapshot, hugr_file: str, target_triple: str, platform: str) -> None:
    hugr_envelope = load(hugr_file)
    ir = compile_to_llvm_ir(hugr_envelope, target_triple=target_triple, platform=platform)  # type: ignore[call-arg]
    snapshot.assert_match(ir, f"{hugr_file}_{target_triple}_{platform}")