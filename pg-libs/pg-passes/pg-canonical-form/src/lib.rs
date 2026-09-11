//! Pass to convert a Pauli graph into canonical form.
use pg_core::{
    BlackBoxData, ConditionalBoxData, GateData, GateType, Op, PGPass, PauliGraph, TableauData,
};
use pg_ir_kernels::{PGTableau, get_dagger, is_clifford, is_clifford_gate_type};
use pg_qm_tableau::Tableau as QubitMajorTableau;
use pg_utils::cliff_angle;

/// Decompose a Clifford gate into a sequence of rotations. We do this to process conditional Clifford gates.
/// The current ConditionalBox only supports PauliRotation, Reset, and Measure, so we need to decompose Clifford gates into rotations to be able to process them.
/// Many of these decompositions are inefficient. The hope is that the synthesis method, when working inside each conditional box, will be able to compress the decomposed gates into fewer rotations and tableaux.
fn decompose_clifford(gate_type: &GateType, args: &[usize]) -> Vec<GateData> {
    match gate_type {
        GateType::H => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
        ],
        GateType::S => vec![GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5])],
        GateType::V => vec![GateData::new(GateType::RX, vec![args[0]]).with_params(vec![0.5])],
        GateType::Sdg => vec![GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![1.5])],
        GateType::Vdg => vec![GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.5])],
        GateType::X => vec![GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.0])],
        GateType::Y => vec![GateData::new(GateType::RY, vec![args[0]]).with_params(vec![1.0])],
        GateType::Z => vec![GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![1.0])],
        GateType::XX => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![2.5]),
            GateData::new(GateType::ZZPHASE, vec![args[0], args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![0.5]),
        ],
        GateType::XY => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.0]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![1.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::ZZPHASE, vec![args[0], args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![3.5]),
        ],
        GateType::XZ => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![2.5]),
            GateData::new(GateType::ZZPHASE, vec![args[1], args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
        ],
        GateType::YX => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.0]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![1.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::ZZPHASE, vec![args[1], args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![1.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![0.5]),
        ],
        GateType::YY => vec![
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.0]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![1.0]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![1.5]),
            GateData::new(GateType::ZZPHASE, vec![args[0], args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![3.5]),
        ],
        GateType::YZ => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.0]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.0]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![1.0]),
            GateData::new(GateType::ZZPHASE, vec![args[1], args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![1.0]),
        ],
        GateType::ZX => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![2.5]),
            GateData::new(GateType::ZZPHASE, vec![args[0], args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![0.5]),
        ],
        GateType::ZY => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.0]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.0]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.0]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![1.5]),
            GateData::new(GateType::ZZPHASE, vec![args[0], args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![3.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.0]),
            GateData::new(GateType::RX, vec![args[1]]).with_params(vec![3.5]),
        ],
        GateType::ZZ => vec![
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.5]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![1.5]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.0]),
            GateData::new(GateType::ZZPHASE, vec![args[0], args[1]]).with_params(vec![0.5]),
            GateData::new(GateType::RZ, vec![args[0]]).with_params(vec![3.0]),
            GateData::new(GateType::RZ, vec![args[1]]).with_params(vec![1.0]),
            GateData::new(GateType::RX, vec![args[0]]).with_params(vec![1.0]),
        ],
        GateType::SWAP => [[args[0], args[1]], [args[1], args[0]], [args[0], args[1]]]
            .into_iter()
            .flat_map(|args| decompose_clifford(&GateType::ZX, &args))
            .collect(),
        _ => panic!(
            "decompose_clifford called with non-Clifford gate type: {:?}",
            gate_type
        ),
    }
}

fn absorb_clifford(tab: &mut QubitMajorTableau, op: &Op, forward: bool) {
    // if moving forward, we maintain the dagger of the tableau.
    // so updating the dagger of the tableau with a gate on the right
    // is equivalent to pre-composing
    // the dagger tableau with the dagger of the gate.
    // (C;G)^\dagger = G^\dagger;C^\dagger
    if forward {
        tab.precompose_op(&get_dagger::<QubitMajorTableau>(op));
    } else {
        tab.precompose_op(op);
    }
}

/// Flush the accumulated tableau and emit either black box representation as an Op::BlackBox.
fn flush_black_box(
    pg: &mut PauliGraph,
    tab: &mut QubitMajorTableau,
    qubits: &[usize],
    content: &str,
    forward: bool,
) {
    let tab_moved = std::mem::replace(tab, QubitMajorTableau::eye(pg.get_n_qubits()));
    pg.add_op(Op::Tableau {
        data: TableauData::from(if forward {
            tab_moved.invert()
        } else {
            tab_moved
        }),
    });
    pg.add_op(Op::BlackBox {
        data: BlackBoxData::new(qubits.to_vec(), content.to_owned()),
    });
}

/// The main function for processing an Op in the input PauliGraph.
fn process_op(
    pg: &mut PauliGraph,
    op: &Op,
    tab: &mut QubitMajorTableau,
    forward: bool,
    cliff_eval: bool,
) {
    match op {
        Op::SetBoundary => return,
        Op::Tableau { data } => {
            let mut tableau_from_op = if forward {
                QubitMajorTableau::from(data.clone()).get_dagger()
            } else {
                QubitMajorTableau::from(data.clone())
            };
            tableau_from_op.compose(tab);
            *tab = tableau_from_op;
            return;
        }
        Op::BlackBox { data } => {
            flush_black_box(pg, tab, data.get_qubits(), data.get_content(), forward);
            return;
        }
        Op::Gate { data } => {
            let gate_type = data.get_gate_type();
            let conditional = !data.get_conditional_bits().is_empty();
            if gate_type == &GateType::BlackBox {
                assert!(
                    !conditional,
                    "Conditional black box gates are not supported at the moment"
                );
                flush_black_box(
                    pg,
                    tab,
                    data.get_args(),
                    data.get_data().as_ref().expect("BlackBox data is missing"),
                    forward,
                );
                return;
            }
            if is_clifford_gate_type(data) {
                if conditional {
                    let new_cond_ops = decompose_clifford(gate_type, data.get_args())
                        .into_iter()
                        .flat_map(|data| tab.conjugate(&Op::Gate { data }))
                        .collect();
                    pg.add_op(Op::ConditionalBox {
                        data: ConditionalBoxData::new(
                            new_cond_ops,
                            data.get_conditional_bits().clone(),
                            data.get_conditional_values().clone(),
                        ),
                    });
                } else {
                    absorb_clifford(tab, op, forward);
                }
                return;
            }
            if !conditional {
                if cliff_eval && is_clifford(op) {
                    absorb_clifford(tab, op, forward);
                    return;
                }
                if gate_type == &GateType::PHASEDX {
                    let alpha = data.get_params()[0];
                    let beta = data.get_params()[1];
                    let args = data.get_args();
                    let compose_alpha = cliff_eval && cliff_angle(alpha).is_some();
                    let compose_beta = cliff_eval && cliff_angle(beta).is_some();
                    // Forward traversal uses a backward facing tableau, so
                    // composition requires the inverse of the gate.
                    // Backward traversal doesn't require inverses, but we need to process
                    // the gates in reverse order, which means flipping the sign of beta.
                    let alpha = if forward && compose_alpha {
                        -alpha
                    } else {
                        alpha
                    };
                    let beta = if !forward || compose_beta {
                        -beta
                    } else {
                        beta
                    };
                    for (gate_type, angle, compose) in [
                        (GateType::RZ, -beta, compose_beta),
                        (GateType::RX, alpha, compose_alpha),
                        (GateType::RZ, beta, compose_beta),
                    ] {
                        let op = Op::Gate {
                            data: GateData::new(gate_type, args.clone()).with_params(vec![angle]),
                        };
                        if compose {
                            tab.precompose_op(&op);
                        } else {
                            for conjugated_op in tab.conjugate(&op) {
                                pg.add_op(conjugated_op);
                            }
                        }
                    }
                    return;
                }
            }
        }
        Op::Rotation { data } => {
            if cliff_eval && cliff_angle(data.get_angle()).is_some() {
                absorb_clifford(tab, op, forward);
                return;
            }
        }
        Op::Measure { .. } | Op::Reset { .. } | Op::ConditionalBox { .. } => {}
    }

    // All remaining operations share the same conjugation path.
    let mut conjugated_ops = tab.conjugate(op);
    if !forward {
        conjugated_ops.reverse();
    }
    for conjugated_op in conjugated_ops {
        pg.add_op(conjugated_op);
    }
}

fn to_canonical_form(pg: &PauliGraph, forward: bool, cliff_eval: bool) -> PauliGraph {
    let mut new_pg = PauliGraph::new(pg.get_n_qubits());
    // We maintain a tableau that represents the inverse of the Clifford unitary if we are moving forward. (i.e. backward facing tableau)
    let mut current_tableau = QubitMajorTableau::eye(pg.get_n_qubits());
    let ops: Box<dyn Iterator<Item = &Op>> = if forward {
        Box::new(pg.get_ops().iter())
    } else {
        Box::new(pg.get_ops().iter().rev())
    };
    for op in ops {
        process_op(&mut new_pg, op, &mut current_tableau, forward, cliff_eval);
    }
    if forward {
        // If we are moving forward, the tableau is facing backward, so we need to invert it to make sure the TableauData is forward facing.
        new_pg.add_op(Op::Tableau {
            data: TableauData::from(current_tableau.invert()),
        });
    } else {
        new_pg.add_op(Op::Tableau {
            data: TableauData::from(current_tableau),
        });
        // reverse the ops
        let mut ops = new_pg.get_ops().clone();
        ops.reverse();
        new_pg = PauliGraph::new(pg.get_n_qubits()).with_ops(ops);
    }
    // Merge adjacent conditional boxes once both outer and inner ops are in circuit order.
    let mut merged_pg = PauliGraph::new(pg.get_n_qubits());
    for op in new_pg.get_ops() {
        if let Op::ConditionalBox { data } = op {
            for inner_op in data.get_ops() {
                merged_pg.add_conditional_op(
                    inner_op.clone(),
                    data.get_conditional_bits().clone(),
                    data.get_conditional_values().clone(),
                );
            }
        } else {
            merged_pg.add_conditional_op(op.clone(), vec![], vec![]);
        }
    }
    merged_pg
}

/// Transform a Pauli graph into canonical form: non-Clifford gates are rewritten as Pauli
/// rotations, while Clifford gates are commuted towards the end of the circuit (or the start,
/// if the `forward` flag is false), merging into a tableau whenever they reach the boundary or
/// an obstruction such as a `BlackBox`. The resulting graph may therefore contain multiple
/// tableaux, interleaved with any obstructions and the non-Clifford ops (rotations, measures,
/// resets, conditional operations) that are conjugated through and left in place.
/// `cliff_eval` controls whether rotations with a Clifford angle are folded into the tableau
/// (`true`) or kept as explicit rotations (`false`).
///
/// # Panics
///
/// Panics if a conditional box contains an operation other than [`Op::Rotation`],
/// [`Op::Measure`], or [`Op::Reset`].
pub struct CanonicalFormPass {
    forward: bool,
    cliff_eval: bool,
}

impl Default for CanonicalFormPass {
    fn default() -> Self {
        Self::new()
    }
}

impl CanonicalFormPass {
    /// Create a new instance of `CanonicalFormPass` with default settings.
    pub fn new() -> Self {
        Self {
            forward: true,
            cliff_eval: true,
        }
    }
    /// Set the direction of the pass.
    pub fn with_forward(mut self, forward: bool) -> Self {
        self.forward = forward;
        self
    }
    /// Set whether to evaluate Clifford angle rotations as Clifford gates or keep them as rotations.
    pub fn with_cliff_eval(mut self, cliff_eval: bool) -> Self {
        self.cliff_eval = cliff_eval;
        self
    }
}

impl PGPass for CanonicalFormPass {
    fn transform(&self, pg: &PauliGraph) -> PauliGraph {
        to_canonical_form(pg, self.forward, self.cliff_eval)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pg_core::{
        GateData, GateType, MeasureData, Op, Pauli, PauliGraph, ResetData, RotationData,
    };
    use pg_tk::compare_unitaries_via_tk;
    use rstest::rstest;

    #[test]
    fn test_empty_circuit() {
        let pg = PauliGraph::new(3);
        let pass = CanonicalFormPass::new();
        let transformed = pass.transform(&pg);
        // empty tableau
        assert_eq!(transformed.get_ops().len(), 1);
    }

    #[rstest]
    #[case(true)]
    #[case(false)]
    fn test_tableau_compose_cliffords(#[case] forward: bool) {
        let mut pg = PauliGraph::new(3);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::X, vec![1]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::S, vec![2]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::ZX, vec![0, 1]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::SWAP, vec![2, 0]),
        });
        let pass = CanonicalFormPass::new().with_forward(forward);
        let transformed = pass.transform(&pg);
        assert_eq!(transformed.get_ops().len(), 1);
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[rstest]
    #[case(true)]
    #[case(false)]
    fn test_tableau_compose_rotations(#[case] forward: bool) {
        let mut pg = PauliGraph::new(3);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::RX, vec![0]).with_params(vec![1.0]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::RZ, vec![1]).with_params(vec![-0.5]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::RY, vec![2]).with_params(vec![1.5]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::PHASEDX, vec![0]).with_params(vec![0.5, 1.0]),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::Y, Pauli::Z], 0.5),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::I, Pauli::Z, Pauli::Z], 3.0),
        });
        let pass = CanonicalFormPass::new().with_forward(forward);
        let transformed = pass.transform(&pg);
        assert_eq!(transformed.get_ops().len(), 1);
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[rstest]
    #[case(true, true, vec![0.5, 1.0], 1)]
    #[case(false, true, vec![0.5, 1.0], 1)]
    #[case(true, false, vec![0.5, 1.0], 4)]
    #[case(false, false, vec![1.5, 0.0], 4)]
    #[case(true, true, vec![1.0, 1.0], 1)]
    #[case(false, true, vec![0.0, 1.5], 1)]
    fn test_tableau_compose_phasedx(
        #[case] forward: bool,
        #[case] cliff_eval: bool,
        #[case] params: Vec<f64>,
        #[case] expected_ops: usize,
    ) {
        let mut pg = PauliGraph::new(1);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::PHASEDX, vec![0]).with_params(params),
        });
        let pass = CanonicalFormPass::new()
            .with_forward(forward)
            .with_cliff_eval(cliff_eval);
        let transformed = pass.transform(&pg);
        assert_eq!(transformed.get_ops().len(), expected_ops);
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[rstest]
    #[case(true)]
    #[case(false)]
    fn test_tableau_compose_tableaux(#[case] forward: bool) {
        let mut pg = PauliGraph::new(3);
        pg.add_op(Op::Tableau {
            data: QubitMajorTableau::random(3, 0, 10, 10).into(),
        });
        pg.add_op(Op::Tableau {
            data: QubitMajorTableau::random(3, 1, 10, 10).into(),
        });
        let pass = CanonicalFormPass::new().with_forward(forward);
        let transformed = pass.transform(&pg);
        assert_eq!(transformed.get_ops().len(), 1);
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[rstest]
    #[case(true, true, 5)]
    #[case(false, true, 5)]
    #[case(true, false, 6)]
    #[case(false, false, 6)]
    fn test_unitary_circuits(
        #[case] forward: bool,
        #[case] cliff_eval: bool,
        #[case] expected_ops: usize,
    ) {
        let mut pg = PauliGraph::new(3);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::XX, vec![0, 1]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::RX, vec![0]).with_params(vec![1.3]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::XY, vec![1, 2]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::RZ, vec![1]).with_params(vec![-0.55]),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::Y, Pauli::Z], 0.5),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::I, Pauli::X, Pauli::X], 1.1),
        });
        pg.add_op(Op::Tableau {
            data: QubitMajorTableau::random(3, 0, 10, 10).into(),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::X, Pauli::Z], 2.1),
        });
        let pass = CanonicalFormPass::new()
            .with_forward(forward)
            .with_cliff_eval(cliff_eval);
        let transformed = pass.transform(&pg);
        assert_eq!(transformed.get_ops().len(), expected_ops);
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[test]
    fn test_measurement() {
        let mut pg1 = PauliGraph::new(1);
        pg1.add_op(Op::Gate {
            data: GateData::new(GateType::V, vec![0]),
        });
        pg1.add_op(Op::Measure {
            data: MeasureData::new(vec![Pauli::Z], false, 0),
        });
        let canonical_pass = CanonicalFormPass::new().with_forward(true);
        let transformed1 = canonical_pass.transform(&pg1);

        let mut pg2 = PauliGraph::new(1);
        pg2.add_op(Op::Gate {
            data: GateData::new(GateType::V, vec![0]),
        });
        pg2.add_op(Op::Gate {
            data: GateData::new(GateType::Measure, vec![0, 0]),
        });
        let transformed2 = canonical_pass.transform(&pg2);

        assert_eq!(transformed1.get_ops().len(), 2);
        if let Op::Measure { data } = &transformed1.get_ops()[0] {
            assert_eq!(*data.get_string(), vec![Pauli::Y]);
            assert!(!data.get_sign_bit());
            assert_eq!(data.get_cbit(), 0);
        } else {
            panic!("Expected a Measure op in the transformed graph");
        }
        assert_eq!(transformed1, transformed2);
    }

    #[test]
    fn test_reset_gate() {
        let mut pg = PauliGraph::new(1);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::Reset, vec![0]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::V, vec![0]),
        });
        let canonical_pass = CanonicalFormPass::new().with_forward(false);
        let transformed = canonical_pass.transform(&pg);
        assert_eq!(transformed.get_ops().len(), 2);
        if let Op::Reset { data } = &transformed.get_ops()[1] {
            assert_eq!(*data.get_first_string(), vec![Pauli::Y]);
            assert_eq!(*data.get_second_string(), vec![Pauli::X]);
            assert!(data.get_first_sign_bit());
            assert!(!data.get_second_sign_bit());
        } else {
            panic!("Expected a Reset op in the transformed graph");
        }
    }

    #[test]
    fn test_reset() {
        let mut pg = PauliGraph::new(1);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::S, vec![0]),
        });
        pg.add_op(Op::Reset {
            data: ResetData::new(vec![Pauli::X], vec![Pauli::Y], false, true),
        });
        let canonical_pass = CanonicalFormPass::new().with_forward(true);
        let transformed = canonical_pass.transform(&pg);
        assert_eq!(transformed.get_ops().len(), 2);
        if let Op::Reset { data } = &transformed.get_ops()[0] {
            assert_eq!(*data.get_first_string(), vec![Pauli::Y]);
            assert_eq!(*data.get_second_string(), vec![Pauli::X]);
            assert!(data.get_first_sign_bit());
            assert!(data.get_second_sign_bit());
        } else {
            panic!("Expected a Reset op in the transformed graph");
        }
    }

    #[test]
    fn test_conditional_cliffords() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::Y, vec![0]).with_conditional(vec![0], vec![true]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::XZ, vec![0, 1]).with_conditional(vec![0], vec![true]),
        });
        let forward_canonical = CanonicalFormPass::new().with_forward(true).transform(&pg);
        let backward_canonical = CanonicalFormPass::new().with_forward(false).transform(&pg);
        let expectd_cond_ops = vec![
            // Y
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Y, Pauli::I], 1.0),
            },
            // XZ decomposed
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z, Pauli::I], 3.5),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::I, Pauli::Z], 0.5),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X, Pauli::I], 2.5),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z, Pauli::Z], 0.5),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z, Pauli::I], 3.5),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X, Pauli::I], 3.5),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.5),
            },
        ];
        assert_eq!(forward_canonical.get_ops().len(), 2);
        let forward_cond_box = if let Op::ConditionalBox { data } = &forward_canonical.get_ops()[0]
        {
            data
        } else {
            panic!("Expected a ConditionalBox op in the transformed graph");
        };
        let backward_cond_box =
            if let Op::ConditionalBox { data } = &backward_canonical.get_ops()[1] {
                data
            } else {
                panic!("Expected a ConditionalBox op in the transformed graph");
            };
        assert_eq!(forward_cond_box.get_ops(), &expectd_cond_ops);
        assert_eq!(forward_cond_box, backward_cond_box);
    }

    #[test]
    fn test_conditional_box() {
        let mut pg = PauliGraph::new(2);
        let cond_ops = vec![
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X, Pauli::I], 1.0),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z, Pauli::Z], 0.5),
            },
        ];
        pg.add_op(Op::ConditionalBox {
            data: ConditionalBoxData::new(cond_ops.clone(), vec![0], vec![true]),
        });
        let forward_canonical = CanonicalFormPass::new().with_forward(true).transform(&pg);
        let backward_canonical = CanonicalFormPass::new().with_forward(false).transform(&pg);
        assert_eq!(forward_canonical.get_ops().len(), 2);
        let forward_cond_box = if let Op::ConditionalBox { data } = &forward_canonical.get_ops()[0]
        {
            data
        } else {
            panic!("Expected a ConditionalBox op in the transformed graph");
        };
        let backward_cond_box =
            if let Op::ConditionalBox { data } = &backward_canonical.get_ops()[1] {
                data
            } else {
                panic!("Expected a ConditionalBox op in the transformed graph");
            };
        // unchanged by the identity tableau
        assert_eq!(forward_cond_box.get_ops(), &cond_ops);
        assert_eq!(forward_cond_box, backward_cond_box);
    }

    #[rstest]
    #[case(true)]
    #[case(false)]
    fn test_black_box(#[case] forward: bool) {
        let mut pg = PauliGraph::new(2);
        let black_box_data = BlackBoxData::new(vec![0, 1], "bb".into());
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        });
        pg.add_op(Op::BlackBox {
            data: black_box_data.clone(),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::S, vec![0]),
        });
        let transformed = CanonicalFormPass::new()
            .with_forward(forward)
            .transform(&pg);
        assert_eq!(transformed.get_ops().len(), 3);
        // circuit is the same for forward and backward passes
        // H tableau, black box, S tableau
        if let Op::Tableau { .. } = &transformed.get_ops()[0] {
            assert!(
                compare_unitaries_via_tk(
                    &PauliGraph::new(2).with_ops(vec![transformed.get_ops()[0].clone()]),
                    &PauliGraph::new(2).with_ops(vec![Op::Gate {
                        data: GateData::new(GateType::H, vec![0]),
                    }]),
                ),
                "Expected first tableau to be equivalent to an H gate on qubit 0"
            );
        } else {
            panic!("Expected a Tableau op in the transformed graph");
        }
        if let Op::BlackBox { data } = &transformed.get_ops()[1] {
            assert_eq!(data, &black_box_data);
        } else {
            panic!("Expected a BlackBox op in the transformed graph");
        }
        if let Op::Tableau { .. } = &transformed.get_ops()[2] {
            assert!(
                compare_unitaries_via_tk(
                    &PauliGraph::new(2).with_ops(vec![transformed.get_ops()[2].clone()]),
                    &PauliGraph::new(2).with_ops(vec![Op::Gate {
                        data: GateData::new(GateType::S, vec![0]),
                    }]),
                ),
                "Expected last tableau to be equivalent to an S gate on qubit 0"
            );
        } else {
            panic!("Expected a Tableau op in the transformed graph");
        }
    }

    #[rstest]
    #[case(GateType::H, vec![0])]
    #[case(GateType::S, vec![0])]
    #[case(GateType::V, vec![0])]
    #[case(GateType::Vdg, vec![0])]
    #[case(GateType::Sdg, vec![0])]
    #[case(GateType::X, vec![0])]
    #[case(GateType::Y, vec![0])]
    #[case(GateType::Z, vec![0])]
    #[case(GateType::XX, vec![0,1])]
    #[case(GateType::XY, vec![0,1])]
    #[case(GateType::XZ, vec![0,1])]
    #[case(GateType::YX, vec![0,1])]
    #[case(GateType::YY, vec![0,1])]
    #[case(GateType::YZ, vec![0,1])]
    #[case(GateType::ZX, vec![0,1])]
    #[case(GateType::ZY, vec![0,1])]
    #[case(GateType::ZZ, vec![0,1])]
    fn test_clifford_decomposition(#[case] gate_type: GateType, #[case] args: Vec<usize>) {
        let mut og_pg = PauliGraph::new(2);
        og_pg.add_op(Op::Gate {
            data: GateData::new(gate_type.clone(), args.clone()),
        });
        let decomposed_gates = decompose_clifford(&gate_type, &args);
        let mut decomposed_pg = PauliGraph::new(2);
        for sub_gate in decomposed_gates {
            decomposed_pg.add_op(Op::Gate { data: sub_gate });
        }
        assert!(compare_unitaries_via_tk(&og_pg, &decomposed_pg));
    }

    #[rstest]
    #[case(true, 2.0, 1.6)]
    #[case(false, 2.0, 1.6)]
    #[case(false, 3.0, 1.25)]
    #[case(true, 3.0, 1.25)]
    fn test_clifford_phased_x(#[case] forward: bool, #[case] alpha: f64, #[case] beta: f64) {
        let mut pg = PauliGraph::new(1);
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::PHASEDX, vec![0]).with_params(vec![alpha, beta]),
        });
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.7]),
        });
        let transformed = CanonicalFormPass::new()
            .with_forward(forward)
            .transform(&pg);
        assert_eq!(transformed.get_ops().len(), 2);
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }
}
