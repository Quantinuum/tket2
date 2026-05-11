from .test_pass import _hugr_from_path, _count_ops


from pytket.passes import FullPeepholeOptimise
from tket.passes import NormalizeGuppy, PytketHugrPass
from hugr.passes.scope import GlobalScope, LocalScope

normalize = NormalizeGuppy()


def test_nested_function_opt_global() -> None:
    h = _hugr_from_path("test_files/guppy_optimization/nested/nested.flat.hugr")
    h_normalized = normalize(h)

    fpo = PytketHugrPass(FullPeepholeOptimise())
    fpo_preserve_entrypoint = fpo.with_scope(GlobalScope.PRESERVE_ENTRYPOINT)

    opt_hugr = fpo_preserve_entrypoint(h_normalized)
    # Assert that FullPeepholeOptimise cancels every CZ and H gate.
    assert _count_ops(opt_hugr, "H") == 0
    assert _count_ops(opt_hugr, "CZ") == 0


def test_nested_function_opt_local() -> None:
    h = _hugr_from_path("test_files/guppy_optimization/nested/nested.flat.hugr")
    h_normalized = normalize(h)

    fpo = PytketHugrPass(FullPeepholeOptimise())
    fpo_local_flat = fpo.with_scope(LocalScope.FLAT)

    flat_opt_hugr = fpo_local_flat(h_normalized)
    # Assert that no optimization is applied to the internal function.
    assert _count_ops(flat_opt_hugr, "H") == 6
    assert _count_ops(flat_opt_hugr, "CZ") == 6
