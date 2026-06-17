use crate::ops::string_to_paulis;
use crate::*;
use std::collections::HashSet;

#[test]
fn string_to_paulis_parses_all_symbols() {
    let paulis = string_to_paulis("IXYZ").unwrap();
    assert_eq!(paulis, vec![Pauli::I, Pauli::X, Pauli::Y, Pauli::Z]);
}

#[test]
fn string_to_paulis_panics_on_invalid_symbol() {
    assert!(string_to_paulis("+IXYZ").is_err());
}

#[test]
fn add_op_merges_same_conditionals() {
    let mut pg = PauliGraph::new(2);
    let conditional_bits = vec![0];
    let conditional_values = vec![true];

    let op1 = Op::Gate {
        data: GateData::new(GateType::X, vec![0]),
    };
    let op2 = Op::Gate {
        data: GateData::new(GateType::Y, vec![1]),
    };

    pg.add_conditional_op(op1, conditional_bits.clone(), conditional_values.clone());
    pg.add_conditional_op(op2, conditional_bits, conditional_values);

    let ops = pg.get_ops();
    assert_eq!(ops.len(), 1);

    match &ops[0] {
        Op::ConditionalBox { data } => {
            assert_eq!(data.get_ops().len(), 2);
        }
        _ => panic!("expected ConditionalBox"),
    }
}

#[test]
fn add_op_does_not_merge_when_conditionals_differ() {
    let mut pg = PauliGraph::new(2);

    let op1 = Op::Gate {
        data: GateData::new(GateType::Z, vec![0]),
    };
    let op2 = Op::Gate {
        data: GateData::new(GateType::H, vec![1]),
    };

    pg.add_conditional_op(op1, vec![0], vec![true]);
    pg.add_conditional_op(op2, vec![1], vec![true]);

    let ops = pg.get_ops();
    assert_eq!(ops.len(), 2);
}

#[test]
fn insert_op_places_operation_at_requested_index() {
    let mut pg = PauliGraph::new(2);
    let op_x = Op::Gate {
        data: GateData::new(GateType::X, vec![0]),
    };
    let op_y = Op::Gate {
        data: GateData::new(GateType::Y, vec![1]),
    };
    let op_h = Op::Gate {
        data: GateData::new(GateType::H, vec![0]),
    };

    pg.add_op(op_x.clone());
    pg.add_op(op_y.clone());
    pg.insert_op(1, op_h.clone());

    assert_eq!(pg.get_ops().len(), 3);
    assert!(
        matches!(&pg.get_ops()[0], Op::Gate { data } if data == &GateData::new(GateType::X, vec![0]))
    );
    assert!(
        matches!(&pg.get_ops()[1], Op::Gate { data } if data == &GateData::new(GateType::H, vec![0]))
    );
    assert!(
        matches!(&pg.get_ops()[2], Op::Gate { data } if data == &GateData::new(GateType::Y, vec![1]))
    );
}

#[test]
fn insert_op_supports_front_and_end_indices() {
    let mut pg = PauliGraph::new(1);
    let op_z = Op::Gate {
        data: GateData::new(GateType::Z, vec![0]),
    };
    let op_x = Op::Gate {
        data: GateData::new(GateType::X, vec![0]),
    };
    let op_h = Op::Gate {
        data: GateData::new(GateType::H, vec![0]),
    };

    pg.add_op(op_x.clone());
    pg.insert_op(0, op_z.clone());
    pg.insert_op(pg.get_ops().len(), op_h.clone());

    assert!(
        matches!(&pg.get_ops()[0], Op::Gate { data } if data == &GateData::new(GateType::Z, vec![0]))
    );
    assert!(
        matches!(&pg.get_ops()[1], Op::Gate { data } if data == &GateData::new(GateType::X, vec![0]))
    );
    assert!(
        matches!(&pg.get_ops()[2], Op::Gate { data } if data == &GateData::new(GateType::H, vec![0]))
    );
}

#[test]
#[should_panic]
fn insert_op_panics_when_index_is_out_of_range() {
    let mut pg = PauliGraph::new(1);
    pg.insert_op(
        1,
        Op::Gate {
            data: GateData::new(GateType::X, vec![0]),
        },
    );
}

#[test]
fn insert_conditional_op_inserts_plain_op_for_empty_conditions() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::X, vec![0]),
    });

    pg.insert_conditional_op(
        0,
        Op::Gate {
            data: GateData::new(GateType::Z, vec![0]),
        },
        vec![],
        vec![],
    );

    assert_eq!(pg.get_ops().len(), 2);
    assert!(
        matches!(&pg.get_ops()[0], Op::Gate { data } if data == &GateData::new(GateType::Z, vec![0]))
    );
}

#[test]
fn insert_conditional_op_merges_with_previous_matching_box() {
    let mut pg = PauliGraph::new(2);
    pg.add_conditional_op(
        Op::Gate {
            data: GateData::new(GateType::X, vec![0]),
        },
        vec![0],
        vec![true],
    );
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::H, vec![1]),
    });

    pg.insert_conditional_op(
        1,
        Op::Gate {
            data: GateData::new(GateType::Y, vec![1]),
        },
        vec![0],
        vec![true],
    );

    assert_eq!(pg.get_ops().len(), 2);
    match &pg.get_ops()[0] {
        Op::ConditionalBox { data } => {
            assert_eq!(data.get_ops().len(), 2);
            assert!(
                matches!(&data.get_ops()[1], Op::Gate { data } if data == &GateData::new(GateType::Y, vec![1]))
            );
        }
        _ => panic!("expected ConditionalBox"),
    }
}

#[test]
fn insert_conditional_op_merges_with_next_matching_box() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::H, vec![0]),
    });
    pg.add_conditional_op(
        Op::Gate {
            data: GateData::new(GateType::X, vec![0]),
        },
        vec![1],
        vec![false],
    );

    pg.insert_conditional_op(
        1,
        Op::Gate {
            data: GateData::new(GateType::Z, vec![1]),
        },
        vec![1],
        vec![false],
    );

    assert_eq!(pg.get_ops().len(), 2);
    match &pg.get_ops()[1] {
        Op::ConditionalBox { data } => {
            assert_eq!(data.get_ops().len(), 2);
            assert!(
                matches!(&data.get_ops()[0], Op::Gate { data } if data == &GateData::new(GateType::Z, vec![1]))
            );
            assert!(
                matches!(&data.get_ops()[1], Op::Gate { data } if data == &GateData::new(GateType::X, vec![0]))
            );
        }
        _ => panic!("expected ConditionalBox"),
    }
}

#[test]
fn insert_conditional_op_does_not_merge_when_conditions_differ() {
    let mut pg = PauliGraph::new(1);
    pg.add_conditional_op(
        Op::Gate {
            data: GateData::new(GateType::X, vec![0]),
        },
        vec![0],
        vec![true],
    );

    pg.insert_conditional_op(
        1,
        Op::Gate {
            data: GateData::new(GateType::Z, vec![0]),
        },
        vec![0],
        vec![false],
    );

    assert_eq!(pg.get_ops().len(), 2);
}

#[test]
fn add_qubit_extends_rotation_measure_and_reset_strings() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Rotation {
        data: RotationData::new(vec![Pauli::X, Pauli::Z], 0.25),
    });
    pg.add_op(Op::Measure {
        data: MeasureData::new(vec![Pauli::Y, Pauli::I], false, 2),
    });
    pg.add_op(Op::Reset {
        data: ResetData::new(
            vec![Pauli::Z, Pauli::I],
            vec![Pauli::X, Pauli::Y],
            true,
            false,
        ),
    });

    pg.add_qubit();

    assert_eq!(pg.get_n_qubits(), 3);
    match &pg.get_ops()[0] {
        Op::Rotation { data } => {
            assert_eq!(data.get_string(), &vec![Pauli::X, Pauli::Z, Pauli::I])
        }
        _ => panic!("expected Rotation"),
    }
    match &pg.get_ops()[1] {
        Op::Measure { data } => {
            assert_eq!(data.get_string(), &vec![Pauli::Y, Pauli::I, Pauli::I])
        }
        _ => panic!("expected Measure"),
    }
    match &pg.get_ops()[2] {
        Op::Reset { data } => {
            assert_eq!(data.get_first_string(), &vec![Pauli::Z, Pauli::I, Pauli::I]);
            assert_eq!(
                data.get_second_string(),
                &vec![Pauli::X, Pauli::Y, Pauli::I]
            );
        }
        _ => panic!("expected Reset"),
    }
}

#[test]
fn add_qubit_extends_tableau_and_appends_basis_outputs() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Tableau {
        data: TableauData::new(
            vec![(vec![Pauli::Z, Pauli::I], true)],
            vec![(vec![Pauli::X, Pauli::Y], false)],
        ),
    });

    pg.add_qubit();

    match &pg.get_ops()[0] {
        Op::Tableau { data } => {
            assert_eq!(data.get_z_outputs().len(), 2);
            assert_eq!(data.get_x_outputs().len(), 2);
            assert_eq!(
                data.get_z_outputs()[0],
                (vec![Pauli::Z, Pauli::I, Pauli::I], true)
            );
            assert_eq!(
                data.get_x_outputs()[0],
                (vec![Pauli::X, Pauli::Y, Pauli::I], false)
            );
            assert_eq!(
                data.get_z_outputs()[1],
                (vec![Pauli::I, Pauli::I, Pauli::Z], true)
            );
            assert_eq!(
                data.get_x_outputs()[1],
                (vec![Pauli::I, Pauli::I, Pauli::X], true)
            );
        }
        _ => panic!("expected Tableau"),
    }
}

#[test]
fn add_qubit_recurses_into_conditional_boxes() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::ConditionalBox {
        data: ConditionalBoxData::new(
            vec![Op::Rotation {
                data: RotationData::new(vec![Pauli::X], 0.5),
            }],
            vec![0],
            vec![true],
        ),
    });

    pg.add_qubit();

    match &pg.get_ops()[0] {
        Op::ConditionalBox { data } => match &data.get_ops()[0] {
            Op::Rotation { data } => {
                assert_eq!(data.get_string(), &vec![Pauli::X, Pauli::I]);
            }
            _ => panic!("expected Rotation"),
        },
        _ => panic!("expected ConditionalBox"),
    }
}

#[test]
fn add_qubit_preserves_graph_validity_for_mixed_ops() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Rotation {
        data: RotationData::new(vec![Pauli::X, Pauli::Z], 0.25),
    });
    pg.add_op(Op::Tableau {
        data: TableauData::new(
            vec![(vec![Pauli::Z, Pauli::I], true)],
            vec![(vec![Pauli::X, Pauli::I], true)],
        ),
    });
    pg.add_op(Op::ConditionalBox {
        data: ConditionalBoxData::new(
            vec![Op::Measure {
                data: MeasureData::new(vec![Pauli::I, Pauli::Z], true, 1),
            }],
            vec![0],
            vec![false],
        ),
    });
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::ZX, vec![0, 1]),
    });

    pg.add_qubit();
    assert!(pg.try_validate().is_ok());
}

#[test]
fn data_getters_return_expected_values() {
    let rotation = RotationData::new(vec![Pauli::X, Pauli::Z], 0.5);
    assert_eq!(rotation.get_string(), &vec![Pauli::X, Pauli::Z]);
    assert_eq!(rotation.get_angle(), 0.5);

    let z_outputs = vec![(vec![Pauli::Z], true)];
    let x_outputs = vec![(vec![Pauli::X], false)];
    let tableau = TableauData::new(z_outputs.clone(), x_outputs.clone());
    assert_eq!(tableau.get_z_outputs(), &z_outputs);
    assert_eq!(tableau.get_x_outputs(), &x_outputs);
}

#[test]
fn gate_type_is_hashable() {
    let mut seen = HashSet::new();
    seen.insert(GateType::X);
    seen.insert(GateType::H);
    assert!(seen.contains(&GateType::X));
    assert!(seen.contains(&GateType::H));
    assert!(!seen.contains(&GateType::Z));
}

#[test]
fn op_equality_compares_same_payloads() {
    let lhs = Op::ConditionalBox {
        data: ConditionalBoxData::new(
            vec![Op::Gate {
                data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.25]),
            }],
            vec![1],
            vec![true],
        ),
    };
    let rhs = Op::ConditionalBox {
        data: ConditionalBoxData::new(
            vec![Op::Gate {
                data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.25]),
            }],
            vec![1],
            vec![true],
        ),
    };

    assert_eq!(lhs, rhs);
}

#[test]
fn op_equality_detects_different_payloads() {
    let lhs = Op::Gate {
        data: GateData::new(GateType::X, vec![0]),
    };
    let rhs = Op::Gate {
        data: GateData::new(GateType::Z, vec![0]),
    };

    assert_ne!(lhs, rhs);
}

#[test]
fn validate_valid_graph_does_not_panic() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Rotation {
        data: RotationData::new(vec![Pauli::X, Pauli::Z], 0.25),
    });
    pg.add_op(Op::Measure {
        data: MeasureData::new(vec![Pauli::Z, Pauli::I], true, 0),
    });
    pg.add_op(Op::Reset {
        data: ResetData::new(
            vec![Pauli::Z, Pauli::I],
            vec![Pauli::X, Pauli::I],
            true,
            false,
        ),
    });
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::ZX, vec![0, 1]),
    });
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.5]),
    });
    assert!(pg.try_validate().is_ok());
}

#[test]
#[should_panic]
fn validate_rotation_wrong_string_length_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Rotation {
        data: RotationData::new(vec![Pauli::X], 0.25),
    });
}

#[test]
#[should_panic]
fn validate_measure_wrong_string_length_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Measure {
        data: MeasureData::new(vec![Pauli::Z], true, 0),
    });
}

#[test]
#[should_panic]
fn validate_reset_first_string_wrong_length_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Reset {
        data: ResetData::new(vec![Pauli::Z], vec![Pauli::X, Pauli::I], true, false),
    });
}

#[test]
#[should_panic]
fn validate_reset_second_string_wrong_length_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Reset {
        data: ResetData::new(vec![Pauli::Z, Pauli::I], vec![Pauli::X], true, false),
    });
}

#[test]
#[should_panic]
fn validate_tableau_wrong_string_length_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Tableau {
        data: TableauData::new(
            vec![(vec![Pauli::Z], true)],
            vec![(vec![Pauli::X, Pauli::I], false)],
        ),
    });
}

#[test]
#[should_panic]
fn validate_gate_wrong_arg_count_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::ZX, vec![0]),
    });
}

#[test]
#[should_panic]
fn validate_gate_wrong_param_count_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::RZ, vec![0]),
    });
}

#[test]
#[should_panic]
fn validate_gate_arg_out_of_range_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::H, vec![2]),
    });
}

#[test]
fn validate_measure_gate_cbit_can_exceed_n_qubits() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::Measure, vec![0, 999]),
    });
}

#[test]
#[should_panic]
fn validate_measure_gate_qubit_out_of_range_panics() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::Measure, vec![1, 0]),
    });
}

#[test]
#[should_panic]
fn validate_blackbox_with_conditions_panics() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::BlackBox, vec![]).with_conditional(vec![0], vec![true]),
    });
}

#[test]
#[should_panic]
fn validate_conditional_box_with_invalid_inner_op_panics() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::ConditionalBox {
        data: ConditionalBoxData::new(
            vec![Op::Gate {
                data: GateData::new(GateType::H, vec![2]),
            }],
            vec![0],
            vec![true],
        ),
    });
}

#[test]
#[should_panic]
fn validate_conditional_box_with_mismatched_condition_lengths_panics() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::ConditionalBox {
        data: ConditionalBoxData::new(
            vec![Op::Gate {
                data: GateData::new(GateType::H, vec![0]),
            }],
            vec![0],
            vec![],
        ),
    });
}

#[test]
#[should_panic]
fn validate_blackbox_missing_payload_panics() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::BlackBox, vec![]),
    });
}

#[test]
#[should_panic]
fn validate_blackbox_arg_out_of_range_panics() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::BlackBox {
        data: BlackBoxData::new(vec![1], "payload".to_string()),
    });
}

#[test]
fn serde_roundtrip_pauli_graph() {
    let mut pg = PauliGraph::new(2);
    pg.add_op(Op::Rotation {
        data: RotationData::new(vec![Pauli::X, Pauli::Z], 0.25),
    });
    pg.add_op(Op::Measure {
        data: MeasureData::new(vec![Pauli::Y, Pauli::I], false, 3),
    });
    pg.add_op(Op::Reset {
        data: ResetData::new(
            vec![Pauli::Z, Pauli::I],
            vec![Pauli::X, Pauli::Y],
            true,
            false,
        ),
    });
    pg.add_op(Op::Tableau {
        data: TableauData::new(
            vec![(vec![Pauli::Z, Pauli::I], true)],
            vec![(vec![Pauli::X, Pauli::Y], false)],
        ),
    });
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.5]),
    });
    pg.add_conditional_op(
        Op::Gate {
            data: GateData::new(GateType::H, vec![1]),
        },
        vec![0],
        vec![true],
    );

    let json = serde_json::to_string(&pg).unwrap();
    let restored: PauliGraph = serde_json::from_str(&json).unwrap();

    assert_eq!(restored.get_n_qubits(), pg.get_n_qubits());
    assert_eq!(restored.get_ops(), pg.get_ops());
}

#[test]
fn serde_rotation_uses_compact_pauli_string() {
    let op = Op::Rotation {
        data: RotationData::new(vec![Pauli::X, Pauli::Z, Pauli::I], 0.25),
    };
    let json = serde_json::to_string(&op).unwrap();
    assert!(
        json.contains("\"XZI\""),
        "expected compact Pauli string \"XZI\" in JSON, got: {json}"
    );
}

#[test]
fn serde_deserialize_mismatched_rotation_string_fails_validation() {
    // n_qubits = 2 but the Rotation Pauli string is length 1 ("X").
    // Deserialization succeeds; validate() should panic.
    let json = r#"{"n_qubits":2,"ops":[{"type":"Rotation","data":{"string":"X","angle":0.25}}]}"#;
    assert!(serde_json::from_str::<PauliGraph>(json).is_err());
}
