import pytest

from hugr import tys
from hugr.build.function import Module

from tket._pattern import InvalidReplacementError, Rule, RuleMatcher
from tket._state import CompilationState
from tket._state.build import CX, OneQbGate, from_coms


S = OneQbGate("S")
T = OneQbGate("T")


def _single_gate_module(gate: OneQbGate) -> CompilationState:
    return from_coms(gate(0))


def test_rule_accepts_single_circuit_module() -> None:
    rule = Rule(_single_gate_module(T)._inner, _single_gate_module(S)._inner)

    assert rule.lhs().num_operations() == 1
    assert rule.rhs().num_operations() == 1


def test_rule_rejects_empty_module() -> None:
    empty = CompilationState.from_python(Module().hugr)
    valid = _single_gate_module(T)

    with pytest.raises(ValueError, match="contains no circuit"):
        Rule(empty._inner, valid._inner)


def test_rule_rejects_multiple_circuit_module() -> None:
    module = Module()
    for name in ("first", "second"):
        function = module.define_function(name, [tys.Qubit])
        [qubit] = function.inputs()
        function.set_outputs(qubit)

    multiple = CompilationState.from_python(module.hugr)
    valid = _single_gate_module(T)

    with pytest.raises(ValueError, match="exactly one circuit"):
        Rule(multiple._inner, valid._inner)


def test_find_methods_reject_non_circuit_entrypoint() -> None:
    matcher = RuleMatcher(
        [Rule(_single_gate_module(T)._inner, _single_gate_module(S)._inner)]
    )
    module = CompilationState.from_python(Module().hugr)

    with pytest.raises(ValueError, match="cannot be used as a circuit parent"):
        matcher.find_match(module._inner)

    with pytest.raises(ValueError, match="cannot be used as a circuit parent"):
        matcher.find_matches(module._inner)


def test_apply_exhaustive_restores_entrypoint_after_error() -> None:
    module = Module()
    function = module.define_function("contains_t", [tys.Qubit])
    [qubit] = function.inputs()
    t_gate = function.add(T(qubit))
    function.set_outputs(t_gate.out(0))

    state = CompilationState.from_python(module.hugr)
    original_entrypoint = state.to_python().modules[0].entrypoint
    invalid_matcher = RuleMatcher(
        [Rule(_single_gate_module(T)._inner, from_coms(CX(0, 1))._inner)]
    )

    with pytest.raises(
        InvalidReplacementError, match="Replacement graph type mismatch"
    ):
        invalid_matcher.apply_exhaustive(state._inner)

    assert state.to_python().modules[0].entrypoint == original_entrypoint
