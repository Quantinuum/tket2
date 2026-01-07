from pytket import Circuit
from tket.rewrite import ECCRewriter
from tket.optimiser import BadgerOptimiser


def test_simple_optimiser():
    """a simple circuit matching test"""
    c = Circuit(3).CX(0, 1).CX(0, 1).CX(1, 2)
    opt = BadgerOptimiser.compile_eccs("test_files/eccs/cx_cx_eccs.json")

    cc = opt.optimise(c, 3)
    exp_c = Circuit(3).CX(1, 2)

    assert cc == exp_c


def test_compose_rewriter():
    """test composing rewriters."""
    c = Circuit(3).CX(0, 1).CX(0, 1).H(0).H(0).CX(0, 2)
    cx_rewriter = ECCRewriter.compile_eccs("test_files/eccs/cx_cx_eccs.json")
    h_rewriter = ECCRewriter.compile_eccs("test_files/eccs/h_h_eccs.json")

    cc = BadgerOptimiser([cx_rewriter, h_rewriter]).optimise(c, 3)
    exp_c = Circuit(3).CX(0, 2)

    assert cc == exp_c
