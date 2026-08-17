//! Pass to convert a Pauli graph into canonical form.
use pg_core::{
    BlackBoxData, ConditionalBoxData, GateData, GateType, MeasureData, Op, PGPass, Pauli,
    PauliGraph, ResetData, RotationData, TableauData,
};
use pg_ir_kernels::{PGTableau, get_dagger};
use pg_qm_tableau::Tableau as QubitMajorTableau;
use pg_utils::{cliff_angle, equiv_0};

/// Add a conjugated Pauli rotation to the Pauli graph.
/// If `cliff_eval` is enabled, the rotation has a Clifford angle, and it is unconditional,
/// we fold it into the current tableau; otherwise we emit it as a (possibly conditional) `Op::Rotation`.
fn add_rotation(
    pg: &mut PauliGraph,
    s: Vec<Pauli>,
    theta: f64,
    tab: &mut QubitMajorTableau,
    data: &GateData,
    forward: bool,
    cliff_eval: bool,
) {
    let cliff_theta = cliff_angle(theta);
    if cliff_theta.is_some() && data.get_conditional_bits().is_empty() && cliff_eval {
        // Since the op is already conjugated, we need to post-compose the rotation.
        // If the pass is forward, the tableau is backward facing, so we need to post-compose
        // the tableau with the inverse of the rotation, and this means negating theta.
        tab.postcompose_op(&Op::Rotation {
            data: RotationData::new(s, if forward { -theta } else { theta }),
        });
    } else {
        pg.add_conditional_op(
            Op::Rotation {
                data: RotationData::new(s, theta),
            },
            data.get_conditional_bits().clone(),
            data.get_conditional_values().clone(),
        );
    }
}

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
        GateType::SWAP => vec![
            GateData::new(GateType::ZX, vec![args[0], args[1]]),
            GateData::new(GateType::ZX, vec![args[1], args[0]]),
            GateData::new(GateType::ZX, vec![args[0], args[1]]),
        ],
        _ => panic!(
            "decompose_clifford called with non-Clifford gate type: {:?}",
            gate_type
        ),
    }
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
        Op::Gate { data } => {
            match data.get_gate_type() {
                GateType::H
                | GateType::S
                | GateType::V
                | GateType::Sdg
                | GateType::Vdg
                | GateType::X
                | GateType::Y
                | GateType::Z
                | GateType::XX
                | GateType::XY
                | GateType::XZ
                | GateType::YX
                | GateType::YY
                | GateType::YZ
                | GateType::ZX
                | GateType::ZY
                | GateType::ZZ => {
                    if data.get_conditional_bits().is_empty() {
                        if forward {
                            // if moving forward, we maintain the dagger of the tableau.
                            // so updating the dagger of the tableau with a gate on the right
                            // is equivalent to pre-composing
                            // the dagger tableau with the dagger of the gate.
                            // (C;G)^\dagger = G^\dagger;C^\dagger
                            tab.precompose_op(&get_dagger::<QubitMajorTableau>(op));
                        } else {
                            tab.precompose_op(op);
                        }
                    } else {
                        // If the gate is conditional, we treat the gate as a sequence of rotations
                        // and conjugate them.
                        let mut decomposed_gates =
                            decompose_clifford(data.get_gate_type(), data.get_args());
                        // If the pass is moving backward, we need to add the gates in reverse order.
                        if !forward {
                            decomposed_gates.reverse();
                        }
                        for sub_gate in decomposed_gates {
                            let sub_gate = sub_gate.with_conditional(
                                data.get_conditional_bits().clone(),
                                data.get_conditional_values().clone(),
                            );
                            // We set cliff_eval to false so the gates are not composed into the tableau.
                            process_op(pg, &Op::Gate { data: sub_gate }, tab, forward, false);
                        }
                    }
                }
                GateType::RX => {
                    let (s, sign_bit) = tab.x_image(data.get_args()[0]);
                    let theta = if sign_bit {
                        -data.get_params()[0]
                    } else {
                        data.get_params()[0]
                    };
                    add_rotation(pg, s, theta, tab, data, forward, cliff_eval);
                }
                GateType::RY => {
                    let mut s = vec![Pauli::I; pg.get_n_qubits()];
                    s[data.get_args()[0]] = Pauli::Y;
                    let (s, sign_bit) = tab.conjugate_string(&s);
                    let theta = if sign_bit {
                        -data.get_params()[0]
                    } else {
                        data.get_params()[0]
                    };
                    add_rotation(pg, s, theta, tab, data, forward, cliff_eval);
                }
                GateType::RZ => {
                    let (s, sign_bit) = tab.z_image(data.get_args()[0]);
                    let theta = if sign_bit {
                        -data.get_params()[0]
                    } else {
                        data.get_params()[0]
                    };
                    add_rotation(pg, s, theta, tab, data, forward, cliff_eval);
                }
                GateType::ZZPHASE => {
                    let mut s = vec![Pauli::I; pg.get_n_qubits()];
                    s[data.get_args()[0]] = Pauli::Z;
                    s[data.get_args()[1]] = Pauli::Z;
                    let (s, sign_bit) = tab.conjugate_string(&s);
                    let theta = if sign_bit {
                        -data.get_params()[0]
                    } else {
                        data.get_params()[0]
                    };
                    add_rotation(pg, s, theta, tab, data, forward, cliff_eval);
                }
                GateType::PHASEDX => {
                    let alpha = data.get_params()[0];
                    let mut beta = data.get_params()[1];
                    let args = data.get_args();
                    // Special cases
                    // If alpha is a multiple of 2 pi, the gate is identity and we can skip it.
                    if equiv_0(alpha, 2.0) {
                        return;
                    }
                    // If the pass is moving backward, we need to add the gates in reverse order.
                    // This is equivalent to flipping the sign of beta.
                    if !forward {
                        beta = -beta;
                    }
                    // If alpha is a multiple of pi and beta is a multiple of pi/4
                    if equiv_0(alpha, 1.0) && equiv_0(beta, 0.25) {
                        process_op(
                            pg,
                            &Op::Gate {
                                data: GateData::new(GateType::RX, args.clone())
                                    .with_params(vec![alpha])
                                    .with_conditional(
                                        data.get_conditional_bits().clone(),
                                        data.get_conditional_values().clone(),
                                    ),
                            },
                            tab,
                            forward,
                            cliff_eval,
                        );
                        process_op(
                            pg,
                            &Op::Gate {
                                data: GateData::new(GateType::RZ, args.clone())
                                    .with_params(vec![2.0 * beta])
                                    .with_conditional(
                                        data.get_conditional_bits().clone(),
                                        data.get_conditional_values().clone(),
                                    ),
                            },
                            tab,
                            forward,
                            cliff_eval,
                        );
                        return;
                    }
                    process_op(
                        pg,
                        &Op::Gate {
                            data: GateData::new(GateType::RZ, args.clone())
                                .with_params(vec![-beta])
                                .with_conditional(
                                    data.get_conditional_bits().clone(),
                                    data.get_conditional_values().clone(),
                                ),
                        },
                        tab,
                        forward,
                        cliff_eval,
                    );
                    process_op(
                        pg,
                        &Op::Gate {
                            data: GateData::new(GateType::RX, args.clone())
                                .with_params(vec![alpha])
                                .with_conditional(
                                    data.get_conditional_bits().clone(),
                                    data.get_conditional_values().clone(),
                                ),
                        },
                        tab,
                        forward,
                        cliff_eval,
                    );
                    process_op(
                        pg,
                        &Op::Gate {
                            data: GateData::new(GateType::RZ, args.clone())
                                .with_params(vec![beta])
                                .with_conditional(
                                    data.get_conditional_bits().clone(),
                                    data.get_conditional_values().clone(),
                                ),
                        },
                        tab,
                        forward,
                        cliff_eval,
                    );
                }
                GateType::Measure => {
                    let (z_string, z_sign_bit) = tab.z_image(data.get_args()[0]);
                    pg.add_conditional_op(
                        Op::Measure {
                            data: MeasureData::new(z_string, z_sign_bit, data.get_args()[1]),
                        },
                        data.get_conditional_bits().clone(),
                        data.get_conditional_values().clone(),
                    );
                }
                GateType::Reset => {
                    let (z_string, z_sign_bit) = tab.z_image(data.get_args()[0]);
                    let (x_string, x_sign_bit) = tab.x_image(data.get_args()[0]);
                    pg.add_conditional_op(
                        Op::Reset {
                            data: ResetData::new(z_string, x_string, z_sign_bit, x_sign_bit),
                        },
                        data.get_conditional_bits().clone(),
                        data.get_conditional_values().clone(),
                    );
                }
                GateType::BlackBox => {
                    assert!(
                        data.get_conditional_bits().is_empty(),
                        "Conditional black box gates are not supported at the moment"
                    );
                    // If the pass is forward, the tableau is backward facing, so we need to invert it to make sure the TableauDataOp
                    // is forward facing.
                    let tab_moved =
                        std::mem::replace(tab, QubitMajorTableau::eye(pg.get_n_qubits()));
                    if forward {
                        pg.add_op(Op::Tableau {
                            data: TableauData::from(tab_moved.invert()),
                        });
                    } else {
                        pg.add_op(Op::Tableau {
                            data: TableauData::from(tab_moved),
                        });
                    }
                    pg.add_op(Op::BlackBox {
                        data: BlackBoxData::new(
                            data.get_args().clone(),
                            data.get_data()
                                .as_ref()
                                .expect("BlackBox data is missing")
                                .clone(),
                        ),
                    });
                    // TODO: we can optimise this by pushing through part of the tableau.
                }
                GateType::SWAP => {
                    if data.get_conditional_bits().is_empty() {
                        let q0 = data.get_args()[0];
                        let q1 = data.get_args()[1];
                        tab.precompose_swap(q0, q1);
                    } else {
                        // No need to check forward or backward here since the decomposition is symmetric
                        for sub_gate in decompose_clifford(&GateType::SWAP, data.get_args()) {
                            let sub_gate = sub_gate.with_conditional(
                                data.get_conditional_bits().clone(),
                                data.get_conditional_values().clone(),
                            );
                            process_op(pg, &Op::Gate { data: sub_gate }, tab, forward, cliff_eval);
                        }
                    }
                }
            }
        }
        Op::Rotation { data } => {
            let (s, sign_bit) = tab.conjugate_string(data.get_string());
            let theta = if sign_bit {
                -data.get_angle()
            } else {
                data.get_angle()
            };
            let cliff_theta = cliff_angle(theta);
            if cliff_theta.is_some() && cliff_eval {
                // Since the op is already conjugated, we need to post-compose the rotation
                // If the pass is forward, the tableau is backward facing, so we need to post-compose
                // the tableau with the inverse of the rotation, which means negating theta.
                tab.postcompose_op(&Op::Rotation {
                    data: RotationData::new(s, if forward { -theta } else { theta }),
                });
            } else {
                pg.add_op(Op::Rotation {
                    data: RotationData::new(s, theta),
                });
            }
        }
        Op::Measure { data } => {
            let (s, mut sign_bit) = tab.conjugate_string(data.get_string());
            sign_bit ^= data.get_sign_bit();
            pg.add_op(Op::Measure {
                data: MeasureData::new(s, sign_bit, data.get_cbit()),
            });
        }
        Op::Reset { data } => {
            let (z_string, mut z_sign_bit) = tab.conjugate_string(data.get_first_string());
            let (x_string, mut x_sign_bit) = tab.conjugate_string(data.get_second_string());
            z_sign_bit ^= data.get_first_sign_bit();
            x_sign_bit ^= data.get_second_sign_bit();
            pg.add_op(Op::Reset {
                data: ResetData::new(z_string, x_string, z_sign_bit, x_sign_bit),
            });
        }
        Op::ConditionalBox { data } => {
            let mut new_cond_ops = Vec::with_capacity(data.get_ops().len());
            for cond_op in data.get_ops() {
                match cond_op {
                    Op::Rotation { data } => {
                        let (s, sign_bit) = tab.conjugate_string(data.get_string());
                        let theta = if sign_bit {
                            -data.get_angle()
                        } else {
                            data.get_angle()
                        };
                        new_cond_ops.push(Op::Rotation {
                            data: RotationData::new(s, theta),
                        });
                    }
                    Op::Reset { data } => {
                        let (z_string, mut z_sign_bit) =
                            tab.conjugate_string(data.get_first_string());
                        let (x_string, mut x_sign_bit) =
                            tab.conjugate_string(data.get_second_string());
                        z_sign_bit ^= data.get_first_sign_bit();
                        x_sign_bit ^= data.get_second_sign_bit();
                        new_cond_ops.push(Op::Reset {
                            data: ResetData::new(z_string, x_string, z_sign_bit, x_sign_bit),
                        });
                    }
                    Op::Measure { data } => {
                        let (s, mut sign_bit) = tab.conjugate_string(data.get_string());
                        sign_bit ^= data.get_sign_bit();
                        new_cond_ops.push(Op::Measure {
                            data: MeasureData::new(s, sign_bit, data.get_cbit()),
                        });
                    }
                    _ => unimplemented!(
                        "Only non-blocking Pauli ops (i.e. PauliRotation, Reset, Measure) are supported inside conditional boxes for now"
                    ),
                }
            }
            if !forward {
                new_cond_ops.reverse();
            }
            pg.add_op(Op::ConditionalBox {
                data: ConditionalBoxData::new(
                    new_cond_ops,
                    data.get_conditional_bits().clone(),
                    data.get_conditional_values().clone(),
                ),
            });
        }
        Op::BlackBox { data } => {
            let tab_moved = std::mem::replace(tab, QubitMajorTableau::eye(pg.get_n_qubits()));
            if forward {
                pg.add_op(Op::Tableau {
                    data: TableauData::from(tab_moved.invert()),
                });
            } else {
                pg.add_op(Op::Tableau {
                    data: TableauData::from(tab_moved),
                });
            }
            pg.add_op(Op::BlackBox {
                data: BlackBoxData::new(data.get_qubits().clone(), data.get_content().clone()),
            });
        }
        Op::Tableau { data } => {
            let mut tableau_from_op: QubitMajorTableau = if forward {
                QubitMajorTableau::from(data.clone()).get_dagger()
            } else {
                QubitMajorTableau::from(data.clone())
            };
            tableau_from_op.compose(tab);
            *tab = tableau_from_op;
        }
        Op::SetBoundary => {
            // We treat commuting boundaries as identities
        }
    }
}

fn to_canonical_form(pg: &PauliGraph, forward: bool, cliff_eval: bool) -> PauliGraph {
    let mut new_pg = PauliGraph::new(pg.get_n_qubits());
    // We maintain a tableau that represents the inverse of the Clifford unitary if we are moving forward. (i.e. backward facing tableau)
    let mut current_tableau: QubitMajorTableau = QubitMajorTableau::eye(pg.get_n_qubits());
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
        // also reverse the inner ops of conditional boxes
        for op in ops.iter_mut() {
            if let Op::ConditionalBox { data } = op {
                let mut inner_ops = data.get_ops().clone();
                inner_ops.reverse();
                *data = ConditionalBoxData::new(
                    inner_ops,
                    data.get_conditional_bits().clone(),
                    data.get_conditional_values().clone(),
                );
            }
        }
        new_pg = PauliGraph::new(pg.get_n_qubits()).with_ops(ops);
    }
    new_pg
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
        assert_eq!(transformed1.get_ops(), transformed2.get_ops());
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
