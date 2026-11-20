"""Check the semantics of QSystem lowerings with Selene."""

from math import pi
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np
import pytest
from hugr import tys
from hugr.build.function import Function
from hugr.std.collections.array import EXTENSION as ARRAY_EXTENSION
from hugr.std.collections.array import Array
from hugr.std.float import FloatVal
from hypothesis import example, given, settings
from hypothesis import strategies as st
from selene_hugr_qis_compiler import EmulatorState
from tket_exts import debug, qsystem_helios, qsystem_sol, quantum

selene_sim = pytest.importorskip("selene_sim")


@settings(max_examples=12, deadline=None)
@given(theta=st.floats(-2 * pi, 2 * pi), phi=st.floats(-2 * pi, 2 * pi))
@example(theta=0.7, phi=-0.3)
@example(theta=0.0, phi=pi / 4)
@example(theta=-pi, phi=pi / 2)
@example(theta=2 * pi, phi=0.0)
def test_phased_xx_lowering(theta: float, phi: float) -> None:
    from selene_quest_plugin import QuestPlugin
    from tket.passes import QSystemRebasePass

    # Entangle each input with a reference so one state result contains every
    # column of the unitary, including the relative phases between columns.
    main = Function("main", [])
    q0, q1, r0, r1 = [main.add_op(quantum.qAlloc) for _ in range(4)]
    r0, q0 = main.add_op(quantum.CX, main.add_op(quantum.H, r0), q0)
    r1, q1 = main.add_op(quantum.CX, main.add_op(quantum.H, r1), q1)
    phased_xx = qsystem_sol.phasedXX
    q0, q1 = main.add_op(
        phased_xx, q0, q1, main.load(FloatVal(theta)), main.load(FloatVal(phi))
    )

    qubit_types = [tys.Qubit] * 4
    array_type = Array(tys.Qubit, 4)
    args = [tys.BoundedNatArg(4), tys.TypeTypeArg(tys.Qubit)]
    pack = ARRAY_EXTENSION.get_op("new_array").instantiate(
        args, tys.FunctionType(qubit_types, [array_type])
    )
    unpack = ARRAY_EXTENSION.get_op("unpack").instantiate(
        args, tys.FunctionType([array_type], qubit_types)
    )
    state_result = debug.state_result("phased_xx", 4)
    state_result = state_result.op_def().instantiate(
        state_result.args, tys.FunctionType([array_type], [array_type])
    )
    qubits = main.add_op(pack, q0, q1, r0, r1)
    qubits = main.add_op(state_result, qubits)
    for qubit in main.add_op(unpack, qubits):
        main.add_op(quantum.qFree, qubit)
    main.set_outputs()

    lowered = QSystemRebasePass()(main.hugr)
    op_names = {data.op.name() for _, data in lowered.nodes()}
    assert phased_xx.name() not in op_names
    assert qsystem_helios.ZZPhase.name() in op_names

    bitcode = EmulatorState.from_python(lowered).compile_to_bitcode()
    with TemporaryDirectory() as build_dir:
        runner = selene_sim.build(bitcode, build_dir=Path(build_dir))
        results = runner.run(QuestPlugin(), n_qubits=4, random_seed=0, timeout=30.0)
        actual = QuestPlugin.extract_states_dict(results)[
            "phased_xx"
        ].get_single_state()

    # The Sol gate is exp(-i theta/2 * P ⊗ P), with P = cos(phi) X + sin(phi) Y.
    pauli = np.array([[0, np.exp(-1j * phi)], [np.exp(1j * phi), 0]])
    expected = np.cos(theta / 2) * np.eye(4) - 1j * np.sin(theta / 2) * np.kron(
        pauli, pauli
    )
    unitary = actual.reshape(4, 4) * 2
    phase = np.angle(np.vdot(expected, unitary))
    np.testing.assert_allclose(
        unitary * np.exp(-1j * phase), expected, atol=1e-8, rtol=0
    )
