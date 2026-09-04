//! Public semantic coverage for greedy synthesis.

#[cfg(feature = "simd")]
use crate::GreedySynthSimdPass;
use crate::{GreedySynthPass, ParallelMode};
use pg_canonical_form::CanonicalFormPass;
use pg_core::{
    BlackBoxData, ConditionalBoxData, GateData, GateType, MeasureData, Op, PGPass, Pauli,
    PauliGraph, ResetData, RotationData, TableauData,
};
use pg_ir_kernels::PGTableau;
use pg_optimise::GroupCommutingOpsPass;
use pg_qm_tableau::Tableau;
use pg_tk::compare_unitaries_via_tk;
use rstest::rstest;

fn graph(n_qubits: usize, ops: Vec<Op>) -> PauliGraph {
    PauliGraph::new(n_qubits).with_ops(ops)
}

fn gate(gate_type: GateType, args: Vec<usize>) -> Op {
    Op::Gate {
        data: GateData::new(gate_type, args),
    }
}

fn rotation_gate(gate_type: GateType, qubit: usize, angle: f64) -> Op {
    Op::Gate {
        data: GateData::new(gate_type, vec![qubit]).with_params(vec![angle]),
    }
}

fn pauli_rotation(string: Vec<Pauli>, angle: f64) -> Op {
    Op::Rotation {
        data: RotationData::new(string, angle),
    }
}

fn prepare_for_synthesis(input: &PauliGraph) -> PauliGraph {
    let canonical = CanonicalFormPass::new().transform(input);
    GroupCommutingOpsPass::new()
        .with_max_set_size(128)
        .transform(&canonical)
}

fn synthesise(input: &PauliGraph) -> PauliGraph {
    GreedySynthPass::new()
        .with_seed(17)
        .with_parallel_mode(ParallelMode::Off)
        .transform(&prepare_for_synthesis(input))
}

fn tableau_from_ops(n_qubits: usize, ops: &[Op]) -> Tableau {
    let mut tableau = Tableau::eye(n_qubits);
    for op in ops {
        tableau.postcompose_op(op);
    }
    tableau
}

fn tableau_op(n_qubits: usize, ops: &[Op]) -> Op {
    let data: TableauData = tableau_from_ops(n_qubits, ops).into();
    Op::Tableau { data }
}

fn one_qubit_graph() -> PauliGraph {
    graph(
        1,
        vec![
            gate(GateType::H, vec![0]),
            rotation_gate(GateType::RZ, 0, 0.37),
        ],
    )
}

fn overlapping_rotation_graph() -> PauliGraph {
    graph(
        3,
        vec![
            pauli_rotation(vec![Pauli::X, Pauli::Y, Pauli::I], 0.17),
            pauli_rotation(vec![Pauli::I, Pauli::Y, Pauli::Z], -0.29),
            pauli_rotation(vec![Pauli::Z, Pauli::I, Pauli::X], 0.41),
        ],
    )
}

fn multiple_set_graph() -> PauliGraph {
    graph(
        2,
        vec![
            pauli_rotation(vec![Pauli::X, Pauli::X], 0.13),
            pauli_rotation(vec![Pauli::Z, Pauli::I], 0.27),
            pauli_rotation(vec![Pauli::I, Pauli::Z], -0.31),
            pauli_rotation(vec![Pauli::Y, Pauli::Y], 0.19),
        ],
    )
}

fn two_tableaux_graph() -> PauliGraph {
    let leading = tableau_op(
        2,
        &[gate(GateType::H, vec![0]), gate(GateType::ZX, vec![0, 1])],
    );
    let trailing = tableau_op(2, &[gate(GateType::S, vec![1]), gate(GateType::H, vec![0])]);
    graph(
        2,
        vec![
            leading,
            pauli_rotation(vec![Pauli::X, Pauli::Z], 0.23),
            trailing,
        ],
    )
}

fn medium_graph() -> PauliGraph {
    graph(
        4,
        vec![
            gate(GateType::H, vec![0]),
            gate(GateType::ZX, vec![0, 1]),
            pauli_rotation(vec![Pauli::X, Pauli::Y, Pauli::Z, Pauli::I], 0.11),
            pauli_rotation(vec![Pauli::I, Pauli::Z, Pauli::X, Pauli::Y], -0.17),
            gate(GateType::S, vec![3]),
            pauli_rotation(vec![Pauli::Z, Pauli::Z, Pauli::I, Pauli::X], 0.29),
            gate(GateType::YY, vec![1, 2]),
            rotation_gate(GateType::RX, 3, 0.07),
        ],
    )
}

fn dense_graph() -> PauliGraph {
    let ops = (0usize..65)
        .map(|index| {
            let mask = index % 15 + 1;
            let string = (0..4)
                .map(|qubit| {
                    if mask & (1 << qubit) == 0 {
                        Pauli::I
                    } else {
                        Pauli::Z
                    }
                })
                .collect();
            pauli_rotation(string, 0.011 + 0.0001 * index as f64)
        })
        .collect();
    graph(4, ops)
}

fn two_chunks_graph() -> PauliGraph {
    let ops = (0usize..66)
        .map(|index| {
            let string = if index % 2 == 0 {
                vec![Pauli::X, Pauli::X]
            } else {
                vec![Pauli::Z, Pauli::X]
            };
            pauli_rotation(string, 0.013 + 0.0001 * index as f64)
        })
        .collect();
    graph(2, ops)
}

#[rstest]
#[case::one_qubit(one_qubit_graph())]
#[case::overlapping_rotations(overlapping_rotation_graph())]
#[case::multiple_sets(multiple_set_graph())]
#[case::tableaux(two_tableaux_graph())]
#[case::medium(medium_graph())]
#[case::dense(dense_graph())]
#[case::two_chunks(two_chunks_graph())]
fn test_synthesis(#[case] input: PauliGraph) {
    let output = synthesise(&input);
    assert!(compare_unitaries_via_tk(&input, &output));
}

#[test]
fn test_output() {
    let output = synthesise(&one_qubit_graph());
    assert!(
        output
            .get_ops()
            .iter()
            .all(|op| matches!(op, Op::Gate { .. }))
    );
}

#[rstest]
#[case::xy(Pauli::X, Pauli::Y)]
#[case::xz(Pauli::X, Pauli::Z)]
#[case::yx(Pauli::Y, Pauli::X)]
#[case::yz(Pauli::Y, Pauli::Z)]
#[case::zx(Pauli::Z, Pauli::X)]
#[case::zy(Pauli::Z, Pauli::Y)]
fn test_single_qubit_tableau(
    #[case] z_image: Pauli,
    #[case] x_image: Pauli,
    #[values(false, true)] z_sign_bit: bool,
    #[values(false, true)] x_sign_bit: bool,
) {
    let expected = Tableau::from(TableauData::new(
        vec![(vec![z_image], z_sign_bit)],
        vec![(vec![x_image], x_sign_bit)],
    ));
    let input = graph(
        1,
        vec![Op::Tableau {
            data: expected.clone().into(),
        }],
    );

    let output = synthesise(&input);

    assert_eq!(tableau_from_ops(1, output.get_ops()), expected);
}

#[test]
fn test_multiqubit_tableau() {
    let expected = tableau_from_ops(
        3,
        &[
            gate(GateType::H, vec![0]),
            gate(GateType::S, vec![1]),
            gate(GateType::ZX, vec![0, 1]),
            gate(GateType::YY, vec![1, 2]),
            gate(GateType::X, vec![0]),
            gate(GateType::Z, vec![2]),
            gate(GateType::SWAP, vec![0, 2]),
        ],
    );
    let input = graph(
        3,
        vec![Op::Tableau {
            data: expected.clone().into(),
        }],
    );

    let output = synthesise(&input);

    assert_eq!(tableau_from_ops(3, output.get_ops()), expected);
}

#[rstest]
#[case::zero(0)]
#[case::nonzero(3)]
fn test_empty_graph(#[case] n_qubits: usize) {
    let input = PauliGraph::new(n_qubits);
    let output = GreedySynthPass::new().transform(&input);

    assert_eq!(output, input);
}

#[rstest]
#[case::default(GreedySynthPass::new())]
#[case::small_window(GreedySynthPass::new().with_window_size(1))]
#[case::small_pool(GreedySynthPass::new().with_pool_size(4).with_top_up_size(2))]
#[case::large_pool(GreedySynthPass::new().with_pool_size(64).with_top_up_size(1))]
fn test_configuration(#[case] pass: GreedySynthPass) {
    let input = medium_graph();
    let prepared = prepare_for_synthesis(&input);
    let output = pass.transform(&prepared);

    assert!(compare_unitaries_via_tk(&input, &output));
}

#[rstest]
#[case::off(ParallelMode::Off)]
#[case::on(ParallelMode::On)]
fn test_parallel(#[case] mode: ParallelMode) {
    let input = medium_graph();
    let prepared = prepare_for_synthesis(&input);
    let output = GreedySynthPass::new()
        .with_seed(23)
        .with_parallel_mode(mode)
        .transform(&prepared);
    assert!(compare_unitaries_via_tk(&input, &output));
}

#[cfg(feature = "simd")]
#[rstest]
#[case::two_chunks(two_chunks_graph())]
#[case::dense(dense_graph())]
#[case::medium(medium_graph())]
fn test_simd(#[case] input: PauliGraph) {
    let prepared = prepare_for_synthesis(&input);
    let simd = GreedySynthSimdPass::new()
        .with_seed(31)
        .with_parallel_mode(ParallelMode::Off)
        .transform(&prepared);

    assert!(compare_unitaries_via_tk(&input, &simd));
}

fn assert_native_lowering(expected: Op, n_qubits: usize, native_gate: GateType) -> GateData {
    let input = graph(n_qubits, vec![expected.clone()]);
    let output = synthesise(&input);
    let native_index = output
        .get_ops()
        .iter()
        .position(|op| matches!(op, Op::Gate { data } if data.get_gate_type() == &native_gate))
        .expect("synthesis did not emit the native operation");
    assert_eq!(
        output
            .get_ops()
            .iter()
            .filter(|op| matches!(op, Op::Gate { data } if data.get_gate_type() == &native_gate))
            .count(),
        1
    );

    let prefix = tableau_from_ops(n_qubits, &output.get_ops()[..native_index]);
    let suffix = tableau_from_ops(n_qubits, &output.get_ops()[native_index + 1..]);
    let conjugated = suffix.conjugate(&output.get_ops()[native_index]);

    assert_eq!(
        conjugated,
        vec![expected],
        "emitted sequence: {:?}",
        output.get_ops()
    );
    assert_eq!(suffix, prefix.get_dagger());

    let Op::Gate { data } = &output.get_ops()[native_index] else {
        unreachable!()
    };
    data.clone()
}

#[rstest]
#[case::x(Pauli::X)]
#[case::y(Pauli::Y)]
#[case::z(Pauli::Z)]
fn test_measurement(#[case] basis: Pauli, #[values(false, true)] sign_bit: bool) {
    let expected = Op::Measure {
        data: MeasureData::new(vec![basis], sign_bit, 7),
    };

    let native = assert_native_lowering(expected, 1, GateType::Measure);
    assert_eq!(native.get_args(), &[0, 7]);
}

#[test]
fn test_nonlocal_measurement() {
    let expected = Op::Measure {
        data: MeasureData::new(vec![Pauli::X, Pauli::Y, Pauli::I, Pauli::Z], true, 7),
    };

    let native = assert_native_lowering(expected, 4, GateType::Measure);
    assert!([0, 1, 3].contains(&native.get_args()[0]));
    assert_eq!(native.get_args()[1], 7);
}

#[rstest]
#[case::xy(Pauli::X, Pauli::Y)]
#[case::yx(Pauli::Y, Pauli::X)]
#[case::xz(Pauli::X, Pauli::Z)]
#[case::zx(Pauli::Z, Pauli::X)]
#[case::yz(Pauli::Y, Pauli::Z)]
#[case::zy(Pauli::Z, Pauli::Y)]
fn test_reset(
    #[case] first: Pauli,
    #[case] second: Pauli,
    #[values(false, true)] first_sign_bit: bool,
    #[values(false, true)] second_sign_bit: bool,
) {
    let expected = Op::Reset {
        data: ResetData::new(vec![first], vec![second], first_sign_bit, second_sign_bit),
    };

    let native = assert_native_lowering(expected, 1, GateType::Reset);
    assert_eq!(native.get_args(), &[0]);
}

#[test]
fn test_nonlocal_reset() {
    let expected = Op::Reset {
        data: ResetData::new(
            vec![Pauli::Z, Pauli::Z, Pauli::I],
            vec![Pauli::X, Pauli::I, Pauli::I],
            true,
            false,
        ),
    };

    let native = assert_native_lowering(expected, 3, GateType::Reset);
    assert!([0, 1].contains(&native.get_args()[0]));
}

#[test]
fn test_black_box() {
    let before = vec![
        gate(GateType::H, vec![0]),
        rotation_gate(GateType::RZ, 1, 0.19),
        gate(GateType::ZX, vec![0, 1]),
    ];
    let after = vec![
        rotation_gate(GateType::RY, 2, -0.23),
        gate(GateType::ZZ, vec![1, 2]),
    ];
    let black_box = Op::BlackBox {
        data: BlackBoxData::new(vec![2, 0, 1], "opaque payload".to_string()),
    };
    let mut ops = before.clone();
    ops.push(black_box);
    ops.extend(after.clone());
    let input = graph(3, ops);
    let output = synthesise(&input);
    let black_box_index = output
        .get_ops()
        .iter()
        .position(
            |op| matches!(op, Op::Gate { data } if data.get_gate_type() == &GateType::BlackBox),
        )
        .expect("synthesis did not emit the black box");
    assert_eq!(
        output
            .get_ops()
            .iter()
            .filter(
                |op| matches!(op, Op::Gate { data } if data.get_gate_type() == &GateType::BlackBox)
            )
            .count(),
        1
    );
    let Op::Gate { data } = &output.get_ops()[black_box_index] else {
        unreachable!()
    };
    assert_eq!(data.get_args(), &[2, 0, 1]);
    assert_eq!(data.get_data().as_deref(), Some("opaque payload"));

    let expected_before = graph(3, before);
    let expected_after = graph(3, after);
    let actual_before = graph(3, output.get_ops()[..black_box_index].to_vec());
    let actual_after = graph(3, output.get_ops()[black_box_index + 1..].to_vec());
    assert!(compare_unitaries_via_tk(&expected_before, &actual_before));
    assert!(compare_unitaries_via_tk(&expected_after, &actual_after));
}

#[test]
fn test_conditional() {
    let inner = pauli_rotation(vec![Pauli::X, Pauli::Y, Pauli::Z], 0.37);
    let input = graph(
        3,
        vec![Op::ConditionalBox {
            data: ConditionalBoxData::new(vec![inner.clone()], vec![2, 5], vec![true, false]),
        }],
    );
    let output = synthesise(&input);
    assert!(output.get_ops().iter().all(|op| {
        matches!(
            op,
            Op::Gate { data }
                if data.get_conditional_bits() == &[2, 5]
                    && data.get_conditional_values() == &[true, false]
        )
    }));
    let active_input = graph(3, vec![inner]);
    let active_output = graph(
        3,
        output
            .get_ops()
            .iter()
            .map(|op| {
                let Op::Gate { data } = op else {
                    unreachable!()
                };
                Op::Gate {
                    data: data.clone().with_conditional(Vec::new(), Vec::new()),
                }
            })
            .collect(),
    );
    assert!(compare_unitaries_via_tk(&active_input, &active_output));
}

#[test]
#[should_panic(expected = "window size must be positive")]
fn test_window_size() {
    GreedySynthPass::new()
        .with_window_size(0)
        .transform(&PauliGraph::new(1));
}

#[test]
#[should_panic(expected = "pool size must be positive")]
fn test_pool_size() {
    GreedySynthPass::new()
        .with_pool_size(0)
        .transform(&PauliGraph::new(1));
}

#[test]
#[should_panic(expected = "top-up size must be positive")]
fn test_top_up_size() {
    GreedySynthPass::new()
        .with_top_up_size(0)
        .transform(&PauliGraph::new(1));
}

#[test]
#[should_panic(expected = "input PauliGraph must contain a SetBoundary")]
fn test_missing_boundary() {
    let input = graph(1, vec![pauli_rotation(vec![Pauli::Z], 0.2)]);
    GreedySynthPass::new().transform(&input);
}

#[test]
#[should_panic(expected = "unsupported operation in input PauliGraph")]
fn test_top_level_gate() {
    let input = graph(1, vec![Op::SetBoundary, gate(GateType::H, vec![0])]);
    GreedySynthPass::new().transform(&input);
}

#[test]
#[should_panic(expected = "conditional box is empty")]
fn test_empty_conditional() {
    let input = graph(
        1,
        vec![
            Op::SetBoundary,
            Op::ConditionalBox {
                data: ConditionalBoxData::new(Vec::new(), vec![0], vec![true]),
            },
        ],
    );
    GreedySynthPass::new().transform(&input);
}

#[test]
#[should_panic(expected = "conditional box has no packed operations")]
fn test_conditional_boundaries() {
    let input = graph(
        1,
        vec![
            Op::SetBoundary,
            Op::ConditionalBox {
                data: ConditionalBoxData::new(
                    vec![Op::SetBoundary, Op::SetBoundary],
                    vec![0],
                    vec![true],
                ),
            },
        ],
    );
    GreedySynthPass::new().transform(&input);
}

#[test]
#[should_panic(expected = "unsupported operation in conditional box")]
fn test_unsupported_conditional() {
    let input = graph(
        1,
        vec![
            Op::SetBoundary,
            Op::ConditionalBox {
                data: ConditionalBoxData::new(
                    vec![gate(GateType::H, vec![0])],
                    vec![0],
                    vec![true],
                ),
            },
        ],
    );
    GreedySynthPass::new().transform(&input);
}
