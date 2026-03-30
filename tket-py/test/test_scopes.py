from pathlib import Path
from hugr.hugr.base import Hugr

from pytket.passes import FullPeepholeOptimise
from tket.passes import NormalizeGuppy, PytketHugrPass
from hugr.passes.scope import GlobalScope

normalize = NormalizeGuppy()


def _hugr_from_path(str_path: str) -> Hugr:
    with open(Path(str_path), "rb") as f:
        h = Hugr.from_bytes(f.read())

    return h


def _count_ops(hugr: Hugr, op_string_name: str) -> int:
    count = 0
    for _, data in hugr.nodes():
        if op_string_name in data.op.name():
            count += 1

    return count


def test_nested_function_opt() -> None:
    h = _hugr_from_path("test_files/guppy_optimization/nested/nested.flat.hugr")

    h_normalized = normalize(h)

    fpo = PytketHugrPass(FullPeepholeOptimise())
    fpo_preserve_entrypoint = fpo.with_scope(GlobalScope.PRESERVE_ENTRYPOINT)

    opt_hugr = fpo_preserve_entrypoint(h_normalized)
    # Assert that FullPeepholeOptimise cancels every CZ and H gate.
    assert _count_ops(opt_hugr, "H") == 0
    assert _count_ops(opt_hugr, "CZ") == 0
