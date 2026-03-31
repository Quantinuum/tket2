import itertools

import pytest
from pytket._tket.circuit import Circuit

from hugr.ops import Custom
from hugr.hugr import Wire
from tket._state import (
    CompilationState,
    Node as Tk2Node,
)
from tket._state.build import (
    CircBuild,
    H,
    from_coms,
    CX,
    PauliX,
    PauliY,
    PauliZ,
    Measure,
    QAlloc,
    QFree,
    load_custom,
    id_circ,
)
from hugr.std.logic import Not
from tket._pattern import Rule, RuleMatcher  # type: ignore
from tket._rewrite import CircuitRewrite, Subcircuit  # type: ignore


@pytest.fixture
def merge_rules() -> list[Rule]:
    paulis = [PauliX(0), PauliY(0), PauliZ(0)]
    identities = [Rule(from_coms(p, p)._inner, id_circ(1)._inner) for p in paulis]

    off_diag = [
        Rule(from_coms(p0, p1)._inner, from_coms(p2)._inner)
        for p0, p1, p2 in itertools.permutations(paulis)
    ]
    return [*identities, *off_diag]


@pytest.fixture
def propagate_rules() -> list[Rule]:
    hadamard_rules = [
        Rule(from_coms(PauliX(0), H(0))._inner, from_coms(H(0), PauliZ(0))._inner),
        Rule(from_coms(PauliZ(0), H(0))._inner, from_coms(H(0), PauliX(0))._inner),
    ]

    cx_rules = [
        Rule(
            from_coms(PauliZ(0), CX(0, 1))._inner, from_coms(CX(0, 1), PauliZ(0))._inner
        ),
        Rule(
            from_coms(PauliX(1), CX(0, 1))._inner, from_coms(CX(0, 1), PauliX(1))._inner
        ),
        Rule(
            from_coms(PauliZ(1), CX(0, 1))._inner,
            from_coms(CX(0, 1), PauliZ(0), PauliZ(1))._inner,
        ),
        Rule(
            from_coms(PauliX(0), CX(0, 1))._inner,
            from_coms(CX(0, 1), PauliX(0), PauliX(1))._inner,
        ),
    ]

    return [*hadamard_rules, *cx_rules]


@pytest.fixture
def measure_rules() -> list[Rule]:
    # X flips the measured bit
    r_build = CircBuild.with_nqb(1)
    (_, measure) = r_build.extend(PauliX(0), Measure(0))
    r_build.set_indexed_outputs(0, measure[1])
    ltk = r_build.finish()

    r_build = CircBuild.with_nqb(1)
    b = r_build.add(Measure(0))[1]
    b = r_build.add(Not(b))[0]
    r_build.set_indexed_outputs(0, b)
    rtk = r_build.finish()

    rules = [Rule(ltk._inner, rtk._inner)]

    # Z does not affect measurement result
    r_build = CircBuild.with_nqb(1)
    (_, measure) = r_build.extend(PauliZ(0), Measure(0))
    r_build.set_indexed_outputs(0, measure[1])
    ltk = r_build.finish()

    r_build = CircBuild.with_nqb(1)
    b = r_build.add(Measure(0))[1]
    r_build.set_indexed_outputs(0, b)
    rtk = r_build.finish()

    rules.append(Rule(ltk._inner, rtk._inner))

    return rules


@pytest.fixture
def propagate_matcher(
    merge_rules: list[Rule], propagate_rules: list[Rule], measure_rules: list[Rule]
) -> RuleMatcher:
    # TODO: This broke with the update to hugr 0.8.0.
    # Custom ops in hugrs must be resolved into `ExtOp`s before they can be used.
    # In this case, we need to define an extension for PauliOps, and make sure the
    # `Rule`s use them, otherwise `RuleMatcher` will complain about finding opaque ops.
    return RuleMatcher([*merge_rules, *propagate_rules, *measure_rules])


def apply_exhaustive(circ: CompilationState, matcher: RuleMatcher) -> int:
    """Apply the first matching rule until no more matches are found. Return the
    number of rewrites applied."""
    match_count = 0
    while match := matcher.find_match(circ._inner):  # type: ignore[arg-type]
        match_count += 1
        circ._inner.apply_rewrite(match)

    return match_count


def add_error_after(circ: CompilationState, wire: Wire, error: Custom):
    """Use a rewrite to insert an operation on a qubit wire assuming the error is
    a one qubit operation, and the source gate of the wire only acts on qubits."""
    port = wire.out_port()
    node = Tk2Node(port.node.idx)
    port_offset = port.offset
    inner = circ._inner
    n_qb = len(inner.node_outputs(node)) - 1  # type: ignore[attr-defined]  # ignore Order port
    subc = Subcircuit([node], inner)  # type: ignore[arg-type]
    replace_build = CircBuild.with_nqb(n_qb)
    current = load_custom(inner.node_op(node))  # type: ignore[attr-defined]
    replace_build.add(current(*list(range(n_qb))))
    replace_build.add(error(port_offset))
    replace_build.set_tracked_outputs()
    replacement = replace_build.finish()

    rw = CircuitRewrite(subc, inner, replacement._inner)  # type: ignore[arg-type]

    circ._inner.apply_rewrite(rw)


def final_pauli_string(circ: CompilationState) -> str:
    """Assuming the circuit only has qubit outputs - check the final operations
    on each qubit, and if they are paulis concatenate them into a string."""
    inner = circ._inner

    def map_op(op: Custom) -> str:
        n = op.name()
        return n if n in ("X", "Y", "Z") else "I"

    # TODO ignore non-qubit outputs
    return "".join(
        map_op(load_custom(inner.node_op(w.node())))  # type: ignore[attr-defined]
        for w in inner.node_inputs(inner.output_node())  # type: ignore[attr-defined]
    )


@pytest.mark.skip(reason="Broken with hugr 0.8.0. See comment in `propagate_matcher`.")
def test_simple_z_prop(propagate_matcher: RuleMatcher):
    c = CircBuild.with_nqb(2)

    (h_node_e, *_) = c.extend(H(0), H(0), CX(0, 1))
    c.set_tracked_outputs()

    t2c = c.finish()

    add_error_after(t2c, h_node_e[0], PauliX)

    assert t2c.to_tket1() == Circuit(2).H(0).X(0).H(0).CX(0, 1)

    assert apply_exhaustive(t2c, propagate_matcher) == 2

    assert t2c.to_tket1() == Circuit(2).H(0).H(0).CX(0, 1).Z(0)

    assert final_pauli_string(t2c) == "ZI"


@pytest.mark.skip(reason="Broken with hugr 0.8.0. See comment in `propagate_matcher`.")
def test_cat(propagate_matcher: RuleMatcher):
    c = CircBuild.with_nqb(4)
    (h_node, *_) = c.extend(
        H(2),
        CX(2, 1),
        CX(2, 3),
        CX(1, 0),
    )
    c.set_tracked_outputs()
    t2c = c.finish()

    add_error_after(t2c, h_node[0], PauliX)
    assert t2c.to_tket1() == Circuit(4).H(2).X(2).CX(2, 1).CX(2, 3).CX(1, 0)

    assert apply_exhaustive(t2c, propagate_matcher) == 3

    assert t2c.to_tket1() == Circuit(4).H(2).CX(2, 1).CX(2, 3).CX(1, 0).X(0).X(1).X(
        2
    ).X(3)

    assert final_pauli_string(t2c) == "XXXX"


def test_alloc_free():
    c = CircBuild()
    alloc = c.add(QAlloc())
    c.add(QFree(alloc))
    c.set_outputs()
    c.finish()  # validates


@pytest.mark.skip(reason="Broken with hugr 0.8.0. See comment in `propagate_matcher`.")
def test_measure(propagate_matcher: RuleMatcher):
    c = CircBuild.with_nqb(2)
    (_, m0, m1) = c.extend(PauliX(0), Measure(0), Measure(1))

    c.set_indexed_outputs(0, m0[1], 1, m1[1])
    before = c.finish()
    """
    ──►X───►Measure─►
            │
            └────►
    ───────►Measure─►
            │
            └────►
    """
    assert apply_exhaustive(before, propagate_matcher) == 1
    c = CircBuild.with_nqb(2)
    (m0, m1) = c.extend(Measure(0), Measure(1))
    nt = c.add(Not(m0[1]))
    c.set_indexed_outputs(0, nt, 1, m1[1])
    after = c.finish()
    """
    ──►Measure──────►
            └─►Not─►
    ──────►Measure──►
                └────►
    """

    # can't compare using tket1 circuits because measure can't be converted.
    assert hash(before) == hash(after)
