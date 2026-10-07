"""Pauli exponential signatures, static argument validation and registration."""

import pytest
from hugr import Hugr
from hugr.build.dfg import Dfg
from hugr.ext import Extension
from hugr.ops import ExtOp
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


def test_hugr_with_pauli_exp():
    """Build and round-trip a graph, checking the linear qubit wiring."""
    inputs = [Qubit] * 4 + [rotation.rotation]
    dfg = Dfg(*inputs)
    op = pauli_exp.pauli_exp(4, [0, 1, 2, 3])
    node = dfg.add(op(*dfg.inputs()))
    dfg.set_outputs(*node)
    hugr = dfg.hugr

    assert hugr[hugr.entrypoint].op.outer_signature() == FunctionType(
        inputs, [Qubit] * 4
    )
    assert hugr[node].op == op
    for i, ty in enumerate(inputs):
        source = dfg.input_node.out(i)
        target = node.inp(i)
        assert list(hugr.linked_ports(source)) == [target]
        assert hugr.port_type(source) == hugr.port_type(target) == ty
    for i in range(4):
        source = node.out(i)
        target = dfg.output_node.inp(i)
        assert list(hugr.linked_ports(source)) == [target]
        assert hugr.port_type(source) == hugr.port_type(target) == Qubit

    loaded = Hugr.from_bytes(hugr.to_bytes(), extensions=tket_registry())
    loaded_ops = [loaded[n].op for n in loaded if isinstance(loaded[n].op, ExtOp)]
    assert len(loaded_ops) == 1
    assert loaded_ops[0].op_def().name == "PauliExp"
    assert loaded_ops[0].args == op.args
    assert loaded_ops[0].outer_signature() == op.outer_signature()
    assert loaded[loaded.entrypoint].op.outer_signature() == FunctionType(
        inputs, [Qubit] * 4
    )
