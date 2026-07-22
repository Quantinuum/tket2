//! Test crate for `pg-ir-kernels`.

use pg_core::{
    BlackBoxData, ConditionalBoxData, GateData, GateType, Op, Pauli, PauliGraph, RotationData,
};
use pg_ir_kernels::PGRewrite;
use pg_qm_tableau::Tableau;
use pg_tk::compare_unitaries_via_tk;

#[test]
fn test_non_identities() {
    let mut pg = PauliGraph::new(1);
    // Non-identity rotation
    pg.add_op(Op::Rotation {
        data: RotationData::new(vec![Pauli::Z], 0.3),
    });
    // Clifford gates
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::H, vec![0]),
    });
    assert!(!pg.is_identity::<Tableau>(1));
    // set boundary
    pg.add_op(Op::SetBoundary);
    assert!(!pg.is_identity::<Tableau>(2));
    // black box
    pg.add_op(Op::BlackBox {
        data: BlackBoxData::new(vec![0], "opaque".to_string()),
    });
    assert!(!pg.is_identity::<Tableau>(3));
    // conditional box
    pg.add_op(Op::ConditionalBox {
        data: ConditionalBoxData::new(
            vec![Op::Gate {
                data: GateData::new(GateType::H, vec![0]),
            }],
            vec![0],
            vec![true],
        ),
    });
    assert!(!pg.is_identity::<Tableau>(4));
}

#[test]
fn test_identities() {
    let mut pg = PauliGraph::new(1);
    // identity rotation
    pg.add_op(Op::Rotation {
        data: RotationData::new(vec![Pauli::Z], 0.0),
    });
    assert!(pg.is_identity::<Tableau>(0));
    // identity rotation gates
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.0]),
    });
    assert!(pg.is_identity::<Tableau>(1));
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::PHASEDX, vec![0]).with_params(vec![0.0, 0.0]),
    });
    assert!(pg.is_identity::<Tableau>(2));
    // PhasedX is identity when the first parameter is a multiple of 2pi
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::PHASEDX, vec![0]).with_params(vec![2.0, 1.6]),
    });
    assert!(pg.is_identity::<Tableau>(3));
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::PHASEDX, vec![0]).with_params(vec![-4.0, 1.6]),
    });
    assert!(pg.is_identity::<Tableau>(4));
}

#[test]
fn test_tableau_conversion() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::Gate {
        data: GateData::new(GateType::H, vec![0]),
    });
    let mut pg_new = pg.clone();
    pg_new.op_to_tableau::<Tableau>(0);
    assert!(compare_unitaries_via_tk(&pg, &pg_new));
}

#[test]
fn test_tableau_conversion2() {
    let mut pg = PauliGraph::new(1);
    pg.add_op(Op::Rotation {
        data: RotationData::new(vec![Pauli::X], 0.5),
    });
    let mut pg_new = pg.clone();
    pg_new.op_to_tableau::<Tableau>(0);
    assert!(compare_unitaries_via_tk(&pg, &pg_new));
}

#[test]
fn test_conj_left() {
    let before = PauliGraph::new(1).with_ops(vec![
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 1.3),
        },
        Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        },
    ]);
    let mut after = before.clone();
    after.conj_left_op::<Tableau>(1);
    assert!(before.get_ops() != after.get_ops());
    assert!(compare_unitaries_via_tk(&before, &after));
}

#[test]
fn test_conj_right() {
    let before = PauliGraph::new(1).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        },
        Op::Gate {
            data: GateData::new(GateType::PHASEDX, vec![0]).with_params(vec![0.3, 0.4]),
        },
    ]);
    let mut after = before.clone();
    after.conj_right_op::<Tableau>(0);
    assert!(before.get_ops() != after.get_ops());
    assert!(compare_unitaries_via_tk(&before, &after));
}

#[test]
#[should_panic]
fn test_conj_left_op_at_left_boundary_panics() {
    let mut pg = PauliGraph::new(1).with_ops(vec![Op::Gate {
        data: GateData::new(GateType::H, vec![0]),
    }]);
    pg.conj_left_op::<Tableau>(0);
}

#[test]
fn test_merge_cliffords() {
    // S·S = Z (a tableau)
    let before = PauliGraph::new(3).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::S, vec![0]),
        },
        Op::Gate {
            data: GateData::new(GateType::H, vec![1]),
        },
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::X, Pauli::Y], 1.5),
        },
    ]);
    let mut after = before.clone();
    after.merge_ops::<Tableau>(0, 1);
    after.merge_ops::<Tableau>(0, 1);
    assert_eq!(after.get_ops().len(), 1);
    assert!(compare_unitaries_via_tk(&before, &after));
}

#[test]
fn test_merge_rotations() {
    let before = PauliGraph::new(1).with_ops(vec![
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 0.1),
        },
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 0.2),
        },
    ]);
    let mut after = before.clone();
    after.merge_ops::<Tableau>(0, 1);
    assert_eq!(after.get_ops().len(), 1);
    assert!(compare_unitaries_via_tk(&before, &after));
}

#[test]
fn test_merge_gates() {
    let before = PauliGraph::new(1).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.1]),
        },
        Op::Gate {
            data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.3]),
        },
    ]);
    let mut after = before.clone();
    after.merge_ops::<Tableau>(0, 1);
    assert_eq!(after.get_ops().len(), 1);
    assert!(compare_unitaries_via_tk(&before, &after));
}

#[test]
fn test_do_commute1() {
    let pg = PauliGraph::new(1).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::S, vec![0]),
        },
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 0.3),
        },
    ]);
    assert!(pg.do_commute::<Tableau>(0, 1));
}

#[test]
fn test_do_commute2() {
    let pg = PauliGraph::new(2).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::XX, vec![0, 1]),
        },
        Op::Gate {
            data: GateData::new(GateType::X, vec![0]),
        },
    ]);
    assert!(pg.do_commute::<Tableau>(0, 1));
}

#[test]
fn test_do_commute3() {
    let pg = PauliGraph::new(2).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::Measure, vec![0, 0]),
        },
        Op::Gate {
            data: GateData::new(GateType::ZZ, vec![0, 1]),
        },
    ]);
    assert!(pg.do_commute::<Tableau>(0, 1));
}

#[test]
fn test_do_not_commute1() {
    let pg = PauliGraph::new(1).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        },
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 0.3),
        },
    ]);
    assert!(!pg.do_commute::<Tableau>(0, 1));
}

#[test]
fn test_do_not_commute2() {
    let pg = PauliGraph::new(1).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        },
        Op::Gate {
            data: GateData::new(GateType::S, vec![0]),
        },
    ]);
    assert!(!pg.do_commute::<Tableau>(0, 1));
}

#[test]
fn test_do_commute_h_h_cliffords_commute() {
    // H·H = I, so H commutes with H
    let pg = PauliGraph::new(1).with_ops(vec![
        Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        },
        Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        },
    ]);
    assert!(pg.do_commute::<Tableau>(0, 1));
}

#[test]
fn test_commute_ops() {
    let before = PauliGraph::new(2).with_ops(vec![
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.3),
        },
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::Z], 0.2),
        },
    ]);
    let mut after = before.clone();
    after.commute_ops::<Tableau>(0, 1);
    assert!(before.get_ops() != after.get_ops());
    assert!(compare_unitaries_via_tk(&before, &after));
}

#[test]
#[should_panic]
fn commute_ops_anticommuting_panics() {
    let mut pg = PauliGraph::new(1).with_ops(vec![
        Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 0.3),
        },
        Op::Rotation {
            data: RotationData::new(vec![Pauli::X], 0.2),
        },
    ]);
    pg.commute_ops::<Tableau>(0, 1);
}
