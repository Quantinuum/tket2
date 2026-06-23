from hugr import Hugr
from hugr import tys
from hugr.build.function import Module
from hugr.passes.scope import LocalScope
from tket_exts import quantum

from tket._state.build import H, OneQbGate, from_coms
from tket.passes import Cliffordize


S = OneQbGate("S")
Sdg = OneQbGate("Sdg")
T = OneQbGate("T")
Tdg = OneQbGate("Tdg")


def _count_gate(hugr: Hugr, gate: str) -> int:
    return sum(
        data.op.name().rsplit(".", maxsplit=1)[-1] == gate for _, data in hugr.nodes()
    )


def test_cliffordize_replaces_supported_gates() -> None:
    input_hugr = from_coms(T(0), Tdg(1), H(0), S(0), Sdg(1)).to_python().modules[0]

    result = Cliffordize().run(input_hugr, inplace=False)

    assert result.results == [("Cliffordize", 2)]
    assert _count_gate(result.hugr, "T") == 0
    assert _count_gate(result.hugr, "Tdg") == 0
    assert _count_gate(result.hugr, "S") == 2
    assert _count_gate(result.hugr, "Sdg") == 2
    assert _count_gate(result.hugr, "H") == 1


def test_cliffordize_reaches_fixed_point() -> None:
    input_hugr = from_coms(T(0), Tdg(1)).to_python().modules[0]

    first_result = Cliffordize().run(input_hugr, inplace=False)
    second_result = Cliffordize().run(first_result.hugr, inplace=False)

    assert first_result.results == [("Cliffordize", 2)]
    assert second_result.results == [("Cliffordize", 0)]


def _nested_t_hugr() -> Hugr:
    module = Module()

    nested = module.define_function("nested", [tys.Qubit])
    [nested_q] = nested.inputs()
    nested_t = nested.add(T(nested_q))
    nested.set_outputs(nested_t.out(0))

    main = module.define_function("main", [tys.Qubit], visibility="Public")
    [main_q] = main.inputs()
    nested_call = main.call(nested.parent_node, main_q)
    main.set_outputs(nested_call.out(0))

    module.hugr.entrypoint = main.parent_node
    return module.hugr


def _cfg_t_hugr() -> Hugr:
    module = Module()
    main = module.define_function("main", [tys.Qubit], visibility="Public")
    [main_q] = main.inputs()

    cfg = main.add_cfg(main_q)
    entry = cfg.add_entry()
    [entry_q] = entry.inputs()
    entry_t = entry.add(T(entry_q))
    entry.set_single_succ_outputs(entry_t.out(0))
    cfg.branch_exit(entry[0])

    main.set_outputs(cfg.parent_node.out(0))
    module.hugr.entrypoint = main.parent_node
    return module.hugr


def test_cliffordize_respects_scope_and_restores_entrypoint() -> None:
    input_hugr = _nested_t_hugr()
    original_entrypoint = input_hugr.entrypoint

    local_result = (
        Cliffordize().with_scope(LocalScope.FLAT).run(input_hugr, inplace=False)
    )
    global_result = Cliffordize().run(input_hugr, inplace=False)

    assert local_result.results == [("Cliffordize", 0)]
    assert _count_gate(local_result.hugr, "T") == 1
    assert local_result.hugr.entrypoint == original_entrypoint

    assert global_result.results == [("Cliffordize", 1)]
    assert _count_gate(global_result.hugr, "T") == 0
    assert _count_gate(global_result.hugr, "S") == 1
    assert global_result.hugr.entrypoint == original_entrypoint


def test_cliffordize_skips_non_circuit_scope_regions() -> None:
    input_hugr = _cfg_t_hugr()

    result = Cliffordize().run(input_hugr, inplace=False)

    assert result.results == [("Cliffordize", 1)]
    assert _count_gate(result.hugr, "T") == 0
    assert _count_gate(result.hugr, "S") == 1


def test_cliffordize_leaves_unsupported_operations_unchanged() -> None:
    input_hugr = from_coms(quantum.toffoli(0, 1, 2), T(0)).to_python().modules[0]

    result = Cliffordize().run(input_hugr, inplace=False)

    assert result.results == [("Cliffordize", 1)]
    assert _count_gate(result.hugr, "Toffoli") == 1
    assert _count_gate(result.hugr, "T") == 0
    assert _count_gate(result.hugr, "S") == 1
