use crate::{BitPackedOp, bitpack_paulis};
use pg_core::{
    BlackBoxData, ConditionalBoxData, MeasureData, Op, PGPass, Pauli, PauliGraph, ResetData,
    RotationData,
};
use pg_ir_kernels::PGTableau;
use pg_qm_tableau::Tableau;
use pg_utils::cliff_angle;
use std::collections::HashMap;

// Use bit-packed Pauli strings as lookup keys when finding rotations with the
// same Pauli string.
fn get_dense_key(paulis: &[Pauli]) -> Vec<u64> {
    bitpack_paulis(paulis, false)
}

fn rotation_merging(pg: &PauliGraph) -> PauliGraph {
    let n_qubits = pg.get_n_qubits();
    let n_ops = pg.get_ops().len();
    let mut output_pg = PauliGraph::new(n_qubits);
    let mut output_ops: Vec<BitPackedOp> = Vec::with_capacity(n_ops);
    // Use `removed` to flag operations removed from `output_ops`.
    let mut removed: Vec<bool> = Vec::with_capacity(n_ops);
    let mut tab: Tableau = Tableau::eye(n_qubits);
    let mut tab_touched = false;
    // Group rotations by their Pauli strings while preserving their order in
    // the original graph. For each rotation, find the most recent rotation with
    // the same Pauli string. If it commutes with every intervening operation,
    // merge its angle into the current rotation and mark the earlier rotation
    // for removal. Otherwise, append the current rotation without merging it.
    //
    // Lookup keys use XZ bit-packing, whereas output operations use ZX
    // bit-packing. Commutation checks require these opposite encodings.
    let mut lookup: HashMap<Vec<u64>, Vec<usize>> = HashMap::new();
    for op in pg.get_ops().iter() {
        match op {
            Op::Rotation { data } => {
                let (s, sign_bit) = tab.conjugate_string(data.get_string());
                let mut angle = if sign_bit {
                    -data.get_angle()
                } else {
                    data.get_angle()
                };
                let key = get_dense_key(&s);
                // Check whether a previous rotation with the same Pauli string
                // can be merged with the current one.
                if let Some(indices) = lookup.get_mut(&key)
                    && let Some(&last_index) = indices.last()
                {
                    let last_op = &output_ops[last_index];
                    // Check whether `last_op` commutes with every intervening
                    // op, ignoring operations already marked for removal.
                    let commute =
                        output_ops[last_index + 1..]
                            .iter()
                            .enumerate()
                            .all(|(i, inter_op)| {
                                removed[last_index + 1 + i] || inter_op.commute_with_string(&key)
                            });
                    if commute {
                        // Merge the angles.
                        if let Op::Rotation { data: last_data } = &last_op.op {
                            angle += last_data.get_angle();
                        } else {
                            // The lookup contains only rotations.
                            panic!("expected a rotation op");
                        }
                        // Mark the previous rotation for removal.
                        removed[last_index] = true;
                        // Remove the previous rotation from the lookup.
                        indices.pop();
                    }
                }
                // If the angle is Clifford, fold the rotation into the tableau.
                // Because the tableau faces backwards and the rotation is to
                // its left, post-compose the inverse rotation (with the
                // negative angle).
                let cliff = cliff_angle(angle);
                if cliff.is_some() {
                    tab.postcompose_op(&Op::Rotation {
                        data: RotationData::new(s, -angle),
                    });
                    tab_touched = true;
                } else {
                    output_ops.push(BitPackedOp::new(
                        Op::Rotation {
                            data: RotationData::new(s.clone(), angle),
                        },
                        true,
                    ));
                    removed.push(false);
                    lookup.entry(key).or_default().push(output_ops.len() - 1);
                }
            }
            Op::Measure { data } => {
                let (s, sign_bit) = tab.conjugate_string(data.get_string());
                output_ops.push(BitPackedOp::new(
                    Op::Measure {
                        data: MeasureData::new(s, sign_bit ^ data.get_sign_bit(), data.get_cbit()),
                    },
                    true,
                ));
                removed.push(false);
            }
            Op::Reset { data } => {
                let (z_string, mut z_sign_bit) = tab.conjugate_string(data.get_first_string());
                let (x_string, mut x_sign_bit) = tab.conjugate_string(data.get_second_string());
                z_sign_bit ^= data.get_first_sign_bit();
                x_sign_bit ^= data.get_second_sign_bit();
                output_ops.push(BitPackedOp::new(
                    Op::Reset {
                        data: ResetData::new(z_string, x_string, z_sign_bit, x_sign_bit),
                    },
                    true,
                ));
                removed.push(false);
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
                            "only non-blocking Pauli operations (`Rotation`, `Reset`, and `Measure`) are currently supported inside conditional boxes"
                        ),
                    }
                }
                output_ops.push(BitPackedOp::new(
                    Op::ConditionalBox {
                        data: ConditionalBoxData::new(
                            new_cond_ops,
                            data.get_conditional_bits().clone(),
                            data.get_conditional_values().clone(),
                        ),
                    },
                    true,
                ));
                removed.push(false);
            }
            Op::BlackBox { data } => {
                if tab_touched {
                    output_ops.push(BitPackedOp::new(
                        Op::Tableau {
                            data: tab.invert().into(),
                        },
                        true,
                    ));
                    removed.push(false);
                }
                output_ops.push(BitPackedOp::new(
                    Op::BlackBox {
                        data: BlackBoxData::new(
                            data.get_qubits().clone(),
                            data.get_content().clone(),
                        ),
                    },
                    true,
                ));
                removed.push(false);
                tab = Tableau::eye(n_qubits);
                tab_touched = false;
            }
            Op::Tableau { data } => {
                let mut tableau_from_op = Tableau::from(data.clone()).invert();
                tableau_from_op.compose(&tab);
                tab = tableau_from_op;
                tab_touched = true;
            }
            Op::SetBoundary => {
                // Remove set boundaries because rotation merging can produce
                // empty sets.
                continue;
            }
            _ => {
                panic!(
                    "Unsupported op type in `RotationMergingPass`. Consider running the `CanonicalFormPass` first."
                );
            }
        }
    }
    assert_eq!(output_ops.len(), removed.len());
    for (op, is_removed) in output_ops.into_iter().zip(removed.into_iter()) {
        if !is_removed {
            output_pg.add_op(op.op);
        }
    }
    if tab_touched {
        output_pg.add_op(Op::Tableau {
            data: tab.invert().into(),
        });
    }
    output_pg
}

/// Greedily merges Pauli rotations with the same Pauli string when possible.
///
/// Consider running the `CanonicalFormPass` first.
///
/// # Panics
///
/// The supported operations are [`Op::Rotation`], [`Op::Measure`],
/// [`Op::Reset`], [`Op::ConditionalBox`], [`Op::BlackBox`], [`Op::Tableau`],
/// and [`Op::SetBoundary`]. The pass panics if the graph contains any other
/// operation, or if a conditional box contains an operation other than
/// [`Op::Rotation`], [`Op::Reset`], or [`Op::Measure`].
///
pub struct RotationMergingPass {}

impl Default for RotationMergingPass {
    fn default() -> Self {
        Self::new()
    }
}

impl RotationMergingPass {
    /// Create a new instance of `RotationMergingPass`.
    pub fn new() -> Self {
        Self {}
    }
}
impl PGPass for RotationMergingPass {
    fn transform(&self, pg: &PauliGraph) -> PauliGraph {
        rotation_merging(pg)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use pg_core::{GateData, GateType, TableauData};
    use pg_tk::compare_unitaries_via_tk;
    use rand::{RngExt, SeedableRng, rngs::StdRng};

    fn s_tableau_op() -> Op {
        let mut tableau = Tableau::eye(1);
        tableau.postcompose_op(&Op::Gate {
            data: GateData::new(GateType::S, vec![0]),
        });
        Op::Tableau {
            data: tableau.into(),
        }
    }

    #[test]
    fn test_rotation_merging() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.2),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.2),
        });
        let merged_pg = rotation_merging(&pg);
        assert_eq!(merged_pg.get_ops().len(), 2);
        assert!(compare_unitaries_via_tk(&pg, &merged_pg));
    }

    #[test]
    fn test_rotation_merging_v() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.4),
        });
        let merged_pg = rotation_merging(&pg);
        assert_eq!(merged_pg.get_ops().len(), 1);
        assert!(compare_unitaries_via_tk(&pg, &merged_pg));
    }

    #[test]
    fn test_x_rotations_only() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.4),
        });

        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.4),
        });

        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.4),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.4),
        });
        let merged_pg = rotation_merging(&pg);
        assert_eq!(merged_pg.get_ops().len(), 2);
        assert!(compare_unitaries_via_tk(&pg, &merged_pg));
    }

    #[test]
    fn test_rotation_merging_across_commuting_ops() {
        let commuting_ops = [
            (
                "measurement",
                Op::Measure {
                    data: MeasureData::new(vec![Pauli::I, Pauli::Z], true, 3),
                },
            ),
            (
                "reset",
                Op::Reset {
                    data: ResetData::new(
                        vec![Pauli::I, Pauli::Z],
                        vec![Pauli::I, Pauli::X],
                        true,
                        false,
                    ),
                },
            ),
            (
                "conditional box",
                Op::ConditionalBox {
                    data: ConditionalBoxData::new(
                        vec![Op::Rotation {
                            data: RotationData::new(vec![Pauli::I, Pauli::Z], 0.125),
                        }],
                        vec![2],
                        vec![false],
                    ),
                },
            ),
        ];

        for (case, commuting_op) in commuting_ops {
            let pg = PauliGraph::new(2).with_ops(vec![
                Op::Rotation {
                    data: RotationData::new(vec![Pauli::X, Pauli::I], 0.125),
                },
                commuting_op.clone(),
                Op::Rotation {
                    data: RotationData::new(vec![Pauli::X, Pauli::I], 0.125),
                },
            ]);

            let merged_pg = rotation_merging(&pg);

            assert_eq!(
                merged_pg,
                PauliGraph::new(2).with_ops(vec![
                    commuting_op,
                    Op::Rotation {
                        data: RotationData::new(vec![Pauli::X, Pauli::I], 0.25),
                    },
                ]),
                "failed case: {case}"
            );
        }
    }

    #[test]
    fn test_rotation_not_merged_across_anticommuting_op() {
        let pg = PauliGraph::new(1).with_ops(vec![
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X], 0.125),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z], 0.25),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X], 0.125),
            },
        ]);

        let merged_pg = rotation_merging(&pg);

        assert_eq!(merged_pg, pg);
    }

    #[test]
    fn test_tableau_propagation_through_supported_ops() {
        let conditional_box = Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![
                    Op::Rotation {
                        data: RotationData::new(vec![Pauli::X], 0.25),
                    },
                    Op::Measure {
                        data: MeasureData::new(vec![Pauli::X], false, 4),
                    },
                    Op::Reset {
                        data: ResetData::new(vec![Pauli::Z], vec![Pauli::X], false, true),
                    },
                ],
                vec![4, 5],
                vec![true, false],
            ),
        };
        let pg = PauliGraph::new(1).with_ops(vec![
            s_tableau_op(),
            Op::Measure {
                data: MeasureData::new(vec![Pauli::X], true, 3),
            },
            Op::Reset {
                data: ResetData::new(vec![Pauli::Z], vec![Pauli::X], true, false),
            },
            conditional_box,
        ]);

        let transformed = rotation_merging(&pg);

        assert_eq!(
            transformed,
            PauliGraph::new(1).with_ops(vec![
                Op::Measure {
                    data: MeasureData::new(vec![Pauli::Y], false, 3),
                },
                Op::Reset {
                    data: ResetData::new(vec![Pauli::Z], vec![Pauli::Y], true, true),
                },
                Op::ConditionalBox {
                    data: ConditionalBoxData::new(
                        vec![
                            Op::Rotation {
                                data: RotationData::new(vec![Pauli::Y], -0.25),
                            },
                            Op::Measure {
                                data: MeasureData::new(vec![Pauli::Y], true, 4),
                            },
                            Op::Reset {
                                data: ResetData::new(vec![Pauli::Z], vec![Pauli::Y], false, false,),
                            },
                        ],
                        vec![4, 5],
                        vec![true, false],
                    ),
                },
                s_tableau_op(),
            ])
        );
    }

    #[test]
    fn test_preserving_tableau() {
        let mut pg = PauliGraph::new(2);
        let z_images = vec![
            (vec![Pauli::X, Pauli::X], true),
            (vec![Pauli::Z, Pauli::Z], false),
        ];
        let x_images = vec![
            (vec![Pauli::Z, Pauli::I], true),
            (vec![Pauli::I, Pauli::X], true),
        ];
        pg.add_op(Op::Tableau {
            data: TableauData::new(z_images, x_images),
        });
        let pass = RotationMergingPass::new();
        let transformed = pass.transform(&pg);
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[test]
    fn test_preserving_black_box() {
        let mut pg = PauliGraph::new(1);
        pg.add_op(Op::BlackBox {
            data: BlackBoxData::new(vec![0], "payload".into()),
        });
        let transformed = RotationMergingPass::new().transform(&pg);
        assert_eq!(pg, transformed);
    }

    #[test]
    fn test_pending_tableau_emitted_before_black_box() {
        let black_box = Op::BlackBox {
            data: BlackBoxData::new(vec![0], "payload".into()),
        };
        let pg = PauliGraph::new(1).with_ops(vec![
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z], 0.5),
            },
            black_box.clone(),
        ]);

        let transformed = rotation_merging(&pg);

        assert_eq!(
            transformed,
            PauliGraph::new(1).with_ops(vec![s_tableau_op(), black_box])
        );
    }

    #[test]
    fn test_set_boundaries_removed() {
        let rotation = Op::Rotation {
            data: RotationData::new(vec![Pauli::X], 0.25),
        };
        let pg =
            PauliGraph::new(1).with_ops(vec![Op::SetBoundary, rotation.clone(), Op::SetBoundary]);

        let transformed = rotation_merging(&pg);

        assert_eq!(transformed, PauliGraph::new(1).with_ops(vec![rotation]));
    }

    #[test]
    fn test_negative_angle_rotation_merging() {
        let pg = PauliGraph::new(1).with_ops(vec![
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X], -0.125),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X], 0.375),
            },
        ]);

        let transformed = rotation_merging(&pg);

        assert_eq!(
            transformed,
            PauliGraph::new(1).with_ops(vec![Op::Rotation {
                data: RotationData::new(vec![Pauli::X], 0.25),
            }])
        );
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[test]
    fn test_rotation_cancellation() {
        let pg = PauliGraph::new(1).with_ops(vec![
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X], 0.125),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X], -0.125),
            },
        ]);

        let transformed = rotation_merging(&pg);

        assert_eq!(
            transformed,
            PauliGraph::new(1).with_ops(vec![Op::Tableau {
                data: Tableau::eye(1).into(),
            }])
        );
        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[test]
    #[should_panic(expected = "Unsupported op type in `RotationMergingPass`")]
    fn test_gate_panics() {
        let pg = PauliGraph::new(1).with_ops(vec![Op::Gate {
            data: GateData::new(GateType::RX, vec![0]).with_params(vec![0.25]),
        }]);
        RotationMergingPass::new().transform(&pg);
    }

    #[test]
    #[should_panic(expected = "only non-blocking Pauli operations")]
    fn test_unsupported_conditional_op_panics() {
        let pg = PauliGraph::new(1).with_ops(vec![Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::BlackBox {
                    data: BlackBoxData::new(vec![0], "payload".into()),
                }],
                vec![0],
                vec![true],
            ),
        }]);
        RotationMergingPass::new().transform(&pg);
    }

    #[test]
    #[should_panic(expected = "No non-identity Pauli found in the string")]
    fn test_all_identity_clifford_rotation_panics() {
        let pg = PauliGraph::new(1).with_ops(vec![Op::Rotation {
            data: RotationData::new(vec![Pauli::I], 0.5),
        }]);
        RotationMergingPass::new().transform(&pg);
    }

    #[test]
    fn test_random_rotation_merging() {
        let strings = [
            vec![Pauli::X, Pauli::I, Pauli::I],
            vec![Pauli::I, Pauli::Y, Pauli::I],
            vec![Pauli::I, Pauli::I, Pauli::Z],
            vec![Pauli::X, Pauli::X, Pauli::I],
            vec![Pauli::Z, Pauli::Z, Pauli::Z],
        ];
        let angles = [-0.125, 0.125, 0.25, 0.5, 0.75, 1.0];
        let mut rng = StdRng::seed_from_u64(7);
        let ops = (0..48)
            .map(|_| Op::Rotation {
                data: RotationData::new(
                    strings[rng.random_range(0..strings.len())].clone(),
                    angles[rng.random_range(0..angles.len())],
                ),
            })
            .collect();
        let pg = PauliGraph::new(3).with_ops(ops);

        let transformed = rotation_merging(&pg);

        assert!(compare_unitaries_via_tk(&pg, &transformed));
    }

    #[test]
    fn test_empty_graph() {
        let pg = PauliGraph::new(2);
        let transformed = RotationMergingPass::new().transform(&pg);
        assert_eq!(pg, transformed);
    }
}
