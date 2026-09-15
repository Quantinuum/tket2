"""Pauli exponential signatures, static argument validation and registration."""

import pytest
from hugr.ext import Extension
from hugr.tys import BoundedNatArg, FunctionType, ListArg, Qubit
from tket_exts import pauli_exp, rotation, tket_registry


@pytest.mark.parametrize("paulis", [[], [0], [1], [2], [3], [0, 1, 2, 3]])
def test_signature(paulis):
    n = len(paulis)
    op = pauli_exp.pauli_exp(n, paulis)
    assert op.outer_signature() == FunctionType(
        [Qubit] * n + [rotation.rotation], [Qubit] * n
    )
    assert op.args == [BoundedNatArg(n), ListArg([BoundedNatArg(p) for p in paulis])]


@pytest.mark.parametrize(
    "n, paulis",
    [
        (2, [1]),
        (0, [0]),
        (1, [4]),
        (1, [-1]),
        (1, ["X"]),
        (1, [1.0]),
        (1, [True]),
        (-1, []),
        (2**64, []),
        (1.0, [1]),
        (True, [1]),
    ],
)
def test_invalid_arguments(n, paulis):
    with pytest.raises(ValueError):
        pauli_exp.pauli_exp(n, paulis)


def test_registration_and_json():
    extension = pauli_exp()
    assert extension.name == "tket.pauli_exp"
    assert pauli_exp.TYPES() == []
    assert pauli_exp.OPS() == [extension.get_op("PauliExp")]
    loaded = Extension.from_json(extension.to_json())
    assert loaded.get_op("PauliExp").name == "PauliExp"
    assert any(e.name == extension.name for e in tket_registry().extensions)
