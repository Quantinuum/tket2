import tket
from tket._ops import TketOp, Pauli


def test_ops_roundtrip():
    for op in TketOp:
        assert TketOp._from_rs(op._to_rs()) == op

    for op in tket._tket.ops.TketOp.values():
        assert TketOp._from_rs(op)._to_rs() == op


def test_pauli_roundtrip():
    for pauli in Pauli:
        assert Pauli._from_rs(pauli._to_rs()) == pauli

    for pauli in tket._tket.ops.Pauli.values():
        assert Pauli._from_rs(pauli)._to_rs() == pauli
