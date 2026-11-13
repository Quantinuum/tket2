"""Tests optimizing Hugrs generated offline by Guppy programs."""

from pathlib import Path
from typing import Literal

import pytest
from hugr.build.base import Hugr
from hugr.ops import AsExtOp
from hugr.package import Package
from tket_exts import tket_registry

from tket._state import CompilationState
from tket.passes import Normalize, PytketHugrPass, QSystemRebasePass, _QSystemLLVMPass

# Import the pytket passes, if the `pytket` extra has been installed.
# If not, skip all tests in this file.
pytket = pytest.importorskip("pytket")
from pytket.passes import CliffordSimp, RemoveBarriers, RemoveRedundancies

guppy_opt_path = Path("guppy_optimization")


def _test_files_hugr(path: Path) -> Hugr:
    with open(Path(__file__).parents[3] / "test_files" / path, "rb") as f:
        h = Package.from_bytes(f.read(), tket_registry())
    return h.modules[0]


def _load_guppy_circuit(
    name: str,
    version_suffix: str = "",
    file_type: Literal["original", "flat", "optimized"] = "original",
) -> Hugr:
    """Load a Hugr from test_files / guppy_optimization.

    Args:
        name: Base name of the circuit (e.g., "angles", "nested_array")
        version_suffix: Version suffix (e.g., "1.1" for angles-v1.1)
        file_type: One of "original", "flat", or "optimized"

    Returns:
        Loaded HUGR module
    """
    suffix_map = {
        "original": "",
        "flat": ".flat",
        "optimized": ".opt",
    }
    suffix = suffix_map[file_type]

    dir_suffix = f"-v{version_suffix}" if version_suffix else ""

    return _test_files_hugr(
        guppy_opt_path / f"{name}{dir_suffix}" / f"{name}{suffix}.hugr"
    )


def _count_gates(hugr: Hugr) -> dict[str, int]:
    """Returns mapping from qualified ID to count for gates and result ops only"""
    counts: dict[str, int] = {}
    for _, data in hugr.nodes():
        if isinstance(data.op, AsExtOp):
            op_name = data.op.op_def().qualified_name()
            if any(
                op_name.startswith(prefix)
                for prefix in ["tket.quantum.", "TKET1.", "tket.result."]
            ):
                counts[op_name] = counts.get(op_name, 0) + 1
    return counts


def _run_clifford_simp(hugr: Hugr) -> Hugr:
    """Apply CliffordSimp pass to a HUGR, returning the optimized HUGR."""
    clifford_pass = PytketHugrPass(CliffordSimp(allow_swaps=True))
    result = clifford_pass.run(hugr)
    return result.hugr


@pytest.mark.parametrize(
    "version_suffix,expected",
    [
        ("1.1", 0),  # previously working as desired
        ("", 10),  # now: https://github.com/Quantinuum/tket2/issues/2130
    ],
)
def test_angles_rotations(version_suffix, expected) -> None:
    def count_from_halfturns(hugr: Hugr) -> int:
        return sum(
            int(data.op.name().startswith("tket.rotation.from_halfturns"))
            for _, data in hugr.nodes()
        )

    hugr = _load_guppy_circuit("angles", version_suffix, "flat")
    assert count_from_halfturns(hugr) == 2

    # We don't need Normalize to "flatten" control flow here, but we still want
    # to get rid of other guppy artifacts.
    hugr = Normalize()(hugr)
    hugr = _run_clifford_simp(hugr)

    optimized = _load_guppy_circuit("angles", version_suffix, "optimized")
    assert count_from_halfturns(optimized) == 0
    assert count_from_halfturns(hugr) == expected


def test_optimize_guppy_ranges_array() -> None:
    """Demonstrates we can fully optimize the array operations in ranges.

    (i.e. after control flow is flattened, but the array ops are not)"""
    hugr = _test_files_hugr(guppy_opt_path / "ranges" / "ranges.flat.array.hugr")

    hugr = Normalize()(hugr)
    hugr = _run_clifford_simp(hugr)

    expected_counts = _count_gates(_load_guppy_circuit("ranges", "", "optimized"))
    assert _count_gates(hugr) == expected_counts


@pytest.mark.parametrize(
    ("name", "version_suffix", "start", "expected"),
    [
        ("nested_array", "", "original", None),  # There is no nested_array.flat
        # There was a significant change to angles between guppylang v1.1.x and v1.2,
        # hence the second testcase, but this only shows up when looking at rotation ops
        # - see test_angles_rotations.
        (
            "angles",
            "1.1",
            "flat",
            {
                "tket.quantum.MeasureFree": 1,
                "tket.quantum.QAlloc": 1,
                "tket.result.result_bool": 1,
            },
        ),  # Since "flat" fails, do not bother testing "original"
        (
            "angles",
            "",
            "flat",
            {
                "tket.quantum.MeasureFree": 1,
                "tket.quantum.QAlloc": 1,
                "tket.result.result_bool": 1,
            },
        ),  # Since "flat" fails, do not bother testing "original"
        (
            "simple_cx",
            "",
            "original",  # There is no "flat"
            {
                "tket.quantum.QAlloc": 2,
                "tket.quantum.MeasureFree": 2,
                "tket.result.result_bool": 2,
            },
        ),
        ("nested", "", "flat", None),
        (
            "nested",
            "",
            "original",
            {
                "tket.quantum.QAlloc": 3,
                "tket.quantum.MeasureFree": 3,
                "tket.result.result_bool": 3,
                "TKET1.tk1op": 4,
                "tket.quantum.CX": 1,
            },
        ),
        ("ranges", "", "flat", None),
        (
            "ranges",
            "",
            "original",
            {
                "tket.quantum.QAlloc": 4,
                "tket.quantum.MeasureFree": 4,
                "tket.result.result_bool": 4,
                "TKET1.tk1op": 2,
                "tket.quantum.CX": 2,
            },
        ),
        (
            "false_branch",
            "",
            "original",
            {
                "tket.quantum.QAlloc": 1,
                "tket.quantum.MeasureFree": 1,
                "tket.result.result_bool": 1,
            },
        ),
        # ALAN maybe just delete func_decls.flat altogether, given "original" passes?
        ("func_decls", "", "flat", None),
        ("func_decls", "", "original", None),  # Note special-case optimizations below.
    ],
)
def test_optimize_guppy(
    name: str,
    version_suffix: str,
    start: Literal["flat", "original"],
    expected: dict[str, int] | None,
) -> None:
    """Test optimization pipeline on Guppy circuits.

    Check that each example (perhaps starting from manually-flattened) optimizes
     to the full extent given by the .opt .hugr files, and can be lowered to QSystem.
    """

    hugr = _load_guppy_circuit(name, version_suffix, start)
    hugr = Normalize()(hugr)
    if start == "original" and expected is None:
        # Check that we are at least flattening correctly.
        # If the manually-flattened version doesn't optimize, we don't expect
        #    the original to, so we don't test it.
        # If flattening fails (but the manually-flattened version optimizes),
        #   then overall will fail.
        try:
            flat_counts = _count_gates(
                _load_guppy_circuit(name, version_suffix, "flat")
            )
            assert _count_gates(hugr) == flat_counts
        except FileNotFoundError:
            pass

    if name == "func_decls":
        hugr = PytketHugrPass(RemoveRedundancies())(hugr)
    else:
        hugr = _run_clifford_simp(hugr)

    actual_counts = _count_gates(hugr)
    optimized_hugr = _load_guppy_circuit(name, version_suffix, "optimized")
    optimized_counts = _count_gates(optimized_hugr)

    QSystemRebasePass().run(hugr)
    _QSystemLLVMPass().run(hugr)

    CompilationState.from_python(hugr).validate()

    if expected is not None:
        assert actual_counts != optimized_counts
        assert actual_counts == expected
        pytest.xfail()
    else:
        assert actual_counts == optimized_counts


def test_issue_1577_remove_barriers_reassembles_nested_circuits() -> None:
    """Regression test for http://github.com/CQCL/tket2/issues/1577

    RemoveBarriers should properly reassemble nested circuits.
    """
    hugr = _test_files_hugr(Path("guppy_examples") / "issue_1577.hugr")

    result = PytketHugrPass(RemoveBarriers()).run(hugr)

    CompilationState.from_python(result.hugr).validate()
