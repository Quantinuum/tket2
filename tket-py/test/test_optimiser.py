import pytest

from tket._state import CompilationState
from tket._rewrite import ECCRewriter
from tket._optimiser import BadgerOptimiser


def test_simple_optimiser():
    """a simple circuit matching test"""
    pytket = pytest.importorskip("pytket")
    tk = CompilationState.from_tket1(pytket.Circuit(3).CX(0, 1).CX(0, 1).CX(1, 2))
    opt = BadgerOptimiser.compile_eccs("test_files/eccs/cx_cx_eccs.json")

    opt.optimise(tk._inner, max_circuit_count=3)
    cc = tk.to_tket1()
    exp_c = pytket.Circuit(3).CX(1, 2)

    assert cc == exp_c


def test_compose_rewriter():
    """test composing rewriters."""
    pytket = pytest.importorskip("pytket")
    tk = CompilationState.from_tket1(
        pytket.Circuit(3).CX(0, 1).CX(0, 1).H(0).H(0).CX(0, 2)
    )
    cx_rewriter = ECCRewriter.compile_eccs("test_files/eccs/cx_cx_eccs.json")
    h_rewriter = ECCRewriter.compile_eccs("test_files/eccs/h_h_eccs.json")

    opt = BadgerOptimiser([cx_rewriter, h_rewriter])
    opt.optimise(tk._inner, max_circuit_count=3)
    cc = tk.to_tket1()
    exp_c = pytket.Circuit(3).CX(0, 2)

    assert cc == exp_c
