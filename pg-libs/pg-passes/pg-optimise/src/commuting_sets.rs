use crate::BitPackedOp;
use pg_core::{Op, PGPass, PauliGraph};

fn group_ops(pg: &PauliGraph, max_set_size: usize) -> PauliGraph {
    let mut output_pg = PauliGraph::new(pg.get_n_qubits());
    let mut commuting_set: Vec<BitPackedOp> = Vec::new();
    if pg.get_ops().is_empty() {
        return output_pg;
    }
    output_pg.add_op(Op::SetBoundary);
    for op in pg.get_ops() {
        if matches!(op, Op::SetBoundary) {
            continue;
        }
        // The new op uses XZ encoding (`z_first = false`), while the
        // commuting set uses ZX encoding (`z_first = true`). The commutation
        // check requires these opposite encodings.
        let mut rich_op = BitPackedOp::new(op.clone(), false);
        if commuting_set.iter().all(|s| s.commute_with(&rich_op))
            && commuting_set.len() < max_set_size
        {
            rich_op.flip_to_zx();
            commuting_set.push(rich_op);
        } else if !commuting_set.is_empty() {
            commuting_set
                .drain(..)
                .for_each(|set_op| output_pg.add_op(set_op.into()));
            output_pg.add_op(Op::SetBoundary);
            rich_op.flip_to_zx();
            commuting_set.push(rich_op);
        }
    }
    if !commuting_set.is_empty() {
        commuting_set
            .drain(..)
            .for_each(|set_op| output_pg.add_op(set_op.into()));
        output_pg.add_op(Op::SetBoundary);
    }
    if output_pg.get_ops().len() == 1 {
        output_pg.remove_op(0);
    }
    output_pg
}

/// Groups commuting [`PauliGraph`] operations by inserting [`Op::SetBoundary`]
/// operations.
///
/// This pass checks only the commutation of [`Op::Rotation`], [`Op::Measure`],
/// [`Op::Reset`], and [`Op::ConditionalBox`]. All other operations are treated
/// as non-commuting.
///
/// Consider running the `CanonicalFormPass` first.
///
/// # Warning
///
/// This pass does not insert group boundaries inside an [`Op::ConditionalBox`].
///
/// # Fields
///
/// - `max_set_size` (`usize`) - Maximum size of a commuting set. The default is
///   50.
///
/// # Examples
///
/// ```
/// use pg_optimise::GroupCommutingOpsPass;
///
/// let s = GroupCommutingOpsPass::new().with_max_set_size(100);
/// ```
pub struct GroupCommutingOpsPass {
    max_set_size: usize,
}

impl Default for GroupCommutingOpsPass {
    fn default() -> Self {
        Self::new()
    }
}

impl GroupCommutingOpsPass {
    /// Create a new instance of `GroupCommutingOpsPass` with default parameters.
    pub fn new() -> Self {
        Self { max_set_size: 50 }
    }

    /// Set the maximum size of a commuting set.
    ///
    /// # Arguments
    ///
    /// - `mut self` (`GroupCommutingOpsPass`) - The current instance of the pass.
    /// - `max_set_size` (`usize`) - Maximum size of a commuting set.
    ///
    /// # Returns
    ///
    /// - `Self` - The updated instance of the pass.
    ///
    /// # Panics
    ///
    /// Panics if `max_set_size` is zero.
    ///
    pub fn with_max_set_size(mut self, max_set_size: usize) -> Self {
        if max_set_size == 0 {
            panic!("max_set_size must be greater than 0");
        }
        self.max_set_size = max_set_size;
        self
    }
}

impl PGPass for GroupCommutingOpsPass {
    fn transform(&self, pg: &PauliGraph) -> PauliGraph {
        group_ops(pg, self.max_set_size)
    }
}

#[cfg(test)]
mod tests {
    use pg_core::{
        BlackBoxData, ConditionalBoxData, GateData, GateType, MeasureData, Pauli, ResetData,
        RotationData,
    };

    use super::*;

    #[test]
    fn test_empty_graph() {
        let pg = PauliGraph::new(2);
        let grouped_pg = GroupCommutingOpsPass::new().transform(&pg);
        assert_eq!(grouped_pg, pg);
    }

    #[test]
    fn test_group_ops() {
        let mut pg = PauliGraph::new(2);
        // boundary
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.1),
        });
        // boundary
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::X], 0.2),
        });
        pg.add_op(Op::Measure {
            data: MeasureData::new(vec![Pauli::Z, Pauli::X], true, 0),
        });
        // boundary
        pg.add_op(Op::Reset {
            data: ResetData::new(
                vec![Pauli::Z, Pauli::I],
                vec![Pauli::X, Pauli::I],
                true,
                true,
            ),
        });
        // boundary
        let group_pass = GroupCommutingOpsPass::new();
        let grouped_pg = group_pass.transform(&pg);
        // 3 groups
        assert_eq!(grouped_pg.get_ops().len(), 10);
    }

    #[test]
    fn test_measurements_writing_same_cbit_are_in_separate_sets() {
        let first_measurement = Op::Measure {
            data: MeasureData::new(vec![Pauli::X, Pauli::I], false, 0),
        };
        let second_measurement = Op::Measure {
            data: MeasureData::new(vec![Pauli::I, Pauli::Z], false, 0),
        };
        let pg = PauliGraph::new(2)
            .with_ops(vec![first_measurement.clone(), second_measurement.clone()]);

        let grouped_pg = GroupCommutingOpsPass::new().transform(&pg);

        assert_eq!(
            grouped_pg,
            PauliGraph::new(2).with_ops(vec![
                Op::SetBoundary,
                first_measurement,
                Op::SetBoundary,
                second_measurement,
                Op::SetBoundary,
            ])
        );
    }

    #[test]
    fn test_black_boxes() {
        let mut pg = PauliGraph::new(2);
        // boundary
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.5),
        });
        // boundary
        pg.add_op(Op::BlackBox {
            data: BlackBoxData::new(vec![0], "bb".into()),
        });
        // boundary
        pg.add_op(Op::BlackBox {
            data: BlackBoxData::new(vec![1], "bb2".into()),
        });
        // boundary
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::Z], 0.5),
        });
        // boundary
        let group_pass = GroupCommutingOpsPass::new();
        let grouped_pg = group_pass.transform(&pg);
        // Each black box should occupy its own group.
        assert_eq!(grouped_pg.get_ops().len(), 10);
    }

    #[test]
    fn test_non_pauli_ops() {
        let mut pg = PauliGraph::new(2);
        // boundary
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::RX, vec![0]).with_params(vec![1.5]),
        });
        // boundary
        pg.add_op(Op::Gate {
            data: GateData::new(GateType::RX, vec![0]).with_params(vec![1.2]),
        });
        // boundary
        let group_pass = GroupCommutingOpsPass::new();
        let grouped_pg = group_pass.transform(&pg);
        assert_eq!(grouped_pg.get_ops().len(), 5);
    }

    #[test]
    fn test_conditional_boxes() {
        let mut pg = PauliGraph::new(2);
        // boundary
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.5),
        });
        pg.add_op(Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![
                    Op::Rotation {
                        data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
                    },
                    Op::Rotation {
                        data: RotationData::new(vec![Pauli::X, Pauli::X], 0.1),
                    },
                ],
                vec![0],
                vec![true],
            ),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::I, Pauli::X], 0.5),
        });
        // boundary
        let group_pass = GroupCommutingOpsPass::new();
        let grouped_pg = group_pass.transform(&pg);
        // One set
        assert_eq!(grouped_pg.get_ops().len(), 6);
    }

    #[test]
    fn test_measurement_writing_condition_bit_is_separate_from_conditional_box() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Measure {
            data: MeasureData::new(vec![Pauli::X, Pauli::I], false, 0),
        });
        pg.add_op(Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::Rotation {
                    data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
                }],
                vec![0],
                vec![true],
            ),
        });

        let grouped_pg = GroupCommutingOpsPass::new().transform(&pg);

        assert_eq!(grouped_pg.get_ops().len(), 5);
        assert!(matches!(grouped_pg.get_ops()[2], Op::SetBoundary));
    }

    #[test]
    fn test_measurement_and_conditional_measurement_writing_same_cbit_are_separate() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Measure {
            data: MeasureData::new(vec![Pauli::X, Pauli::I], false, 0),
        });
        pg.add_op(Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::Measure {
                    data: MeasureData::new(vec![Pauli::I, Pauli::Z], false, 0),
                }],
                vec![1],
                vec![true],
            ),
        });

        let grouped_pg = GroupCommutingOpsPass::new().transform(&pg);

        assert_eq!(grouped_pg.get_ops().len(), 5);
        assert!(matches!(grouped_pg.get_ops()[2], Op::SetBoundary));
    }

    #[test]
    fn test_conditional_boxes_with_classical_dependencies_are_separate() {
        let writer = Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::Measure {
                    data: MeasureData::new(vec![Pauli::X, Pauli::I], false, 0),
                }],
                vec![1],
                vec![true],
            ),
        };
        let reader = Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::Rotation {
                    data: RotationData::new(vec![Pauli::I, Pauli::Z], 0.1),
                }],
                vec![0],
                vec![true],
            ),
        };
        let second_writer = Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::Measure {
                    data: MeasureData::new(vec![Pauli::I, Pauli::Z], false, 0),
                }],
                vec![2],
                vec![true],
            ),
        };
        let dependent_pairs = [
            ("write followed by read", writer.clone(), reader.clone()),
            ("read followed by write", reader, writer.clone()),
            ("write followed by write", writer, second_writer),
        ];

        for (case, first_box, second_box) in dependent_pairs {
            let grouped_pg = GroupCommutingOpsPass::new().transform(
                &PauliGraph::new(2).with_ops(vec![first_box.clone(), second_box.clone()]),
            );

            assert_eq!(
                grouped_pg,
                PauliGraph::new(2).with_ops(vec![
                    Op::SetBoundary,
                    first_box,
                    Op::SetBoundary,
                    second_box,
                    Op::SetBoundary,
                ]),
                "failed case: {case}"
            );
        }
    }

    #[test]
    fn test_conditional_boxes_reading_same_cbit_commute() {
        let first_box = Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::Rotation {
                    data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
                }],
                vec![0],
                vec![true],
            ),
        };
        let second_box = Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::Rotation {
                    data: RotationData::new(vec![Pauli::I, Pauli::Z], 0.1),
                }],
                vec![0],
                vec![true],
            ),
        };

        let grouped_pg = GroupCommutingOpsPass::new()
            .transform(&PauliGraph::new(2).with_ops(vec![first_box.clone(), second_box.clone()]));

        assert_eq!(
            grouped_pg,
            PauliGraph::new(2).with_ops(vec![
                Op::SetBoundary,
                first_box,
                second_box,
                Op::SetBoundary,
            ])
        );
    }

    #[test]
    fn test_measurement_commutes_with_classically_independent_conditional_box() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Measure {
            data: MeasureData::new(vec![Pauli::X, Pauli::I], false, 0),
        });
        pg.add_op(Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![Op::Rotation {
                    data: RotationData::new(vec![Pauli::I, Pauli::Z], 0.1),
                }],
                vec![1],
                vec![true],
            ),
        });

        let grouped_pg = GroupCommutingOpsPass::new().transform(&pg);

        assert_eq!(grouped_pg.get_ops().len(), 4);
    }

    #[test]
    fn test_max_set_size() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::X], 0.2),
        });
        let group_pass = GroupCommutingOpsPass::new().with_max_set_size(1);
        let grouped_pg = group_pass.transform(&pg);
        assert_eq!(grouped_pg.get_ops().len(), 9);
    }

    #[test]
    fn test_pg_contains_set_boundary() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::SetBoundary);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.1),
        });
        let group_pass = GroupCommutingOpsPass::new();
        let grouped_pg = group_pass.transform(&pg);
        assert_eq!(grouped_pg.get_ops().len(), 4);
    }

    #[test]
    fn test_pg_contains_only_set_boundaries() {
        let pg = PauliGraph::new(2).with_ops(vec![Op::SetBoundary, Op::SetBoundary]);
        let grouped_pg = GroupCommutingOpsPass::new().transform(&pg);
        assert_eq!(grouped_pg, PauliGraph::new(2));
    }

    #[test]
    fn test_idempotent() {
        let mut pg = PauliGraph::new(2);
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.1),
        });
        pg.add_op(Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::X], 0.2),
        });
        let group_pass = GroupCommutingOpsPass::new();
        let grouped_pg = group_pass.transform(&pg);
        let regrouped_pg = group_pass.transform(&grouped_pg);
        assert_eq!(grouped_pg, regrouped_pg);
    }
}
