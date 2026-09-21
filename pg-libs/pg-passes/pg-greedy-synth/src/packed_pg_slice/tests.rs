use super::*;
use crate::backend::ScalarBackend;
use pg_bitpacked::XX_U8;
use pg_core::{BlackBoxData, ResetData, RotationData, TableauData};
use rstest::rstest;

fn rotation(paulis: Vec<Pauli>) -> Op {
    Op::Rotation {
        data: RotationData::new(paulis, 0.25),
    }
}

#[rstest]
#[case::end_of_chunk(62, (0, 62), (1, 0))]
#[case::next_chunk(63, (1, 0), (1, 2))]
fn test_reset_alignment(
    #[case] prefix_len: usize,
    #[case] start: (usize, usize),
    #[case] end: (usize, usize),
) {
    let backend = ScalarBackend;
    let position = |chunk, bit| SliceBitPosition { chunk, bit };

    let mut slice = PackedPGSlice::new(1);
    for _ in 0..prefix_len {
        slice.push_op(&rotation(vec![Pauli::X]), &backend);
    }
    slice.push_op(
        &Op::Reset {
            data: ResetData::new(vec![Pauli::X], vec![Pauli::Z], false, false),
        },
        &backend,
    );
    assert_eq!(
        slice.op_sets().next().unwrap().last().unwrap().1.bit_range,
        (position(start.0, start.1), position(end.0, end.1))
    );
}

#[test]
fn test_progress_set() {
    let backend = ScalarBackend;

    // Check that views are rebased after progressing the slice.
    let mut slice = PackedPGSlice::new(1);
    for _ in 0..64 {
        slice.push_op(&rotation(vec![Pauli::X]), &backend);
    }
    slice.start_new_set();
    slice.push_op(&rotation(vec![Pauli::Z]), &backend);
    assert!(slice.progress_set());
    assert_eq!(
        slice.op_sets().next().unwrap().next().unwrap().1.bit_range,
        (
            SliceBitPosition { chunk: 0, bit: 0 },
            SliceBitPosition { chunk: 0, bit: 1 }
        )
    );
}

#[test]
fn test_split_ref_stop_mask() {
    let backend = ScalarBackend;

    // A stop immediately before a black box includes the preceding column and
    // excludes the black box column.
    let mut slice = PackedPGSlice::new(2);
    slice.push_op(&rotation(vec![Pauli::X, Pauli::I]), &backend);
    slice.push_op(
        &Op::BlackBox {
            data: BlackBoxData::new(vec![0], "box".into()),
        },
        &backend,
    );
    let stop = slice.stop_point(TQE {
        gate_type: XX_U8,
        q0: 0,
        q1: 1,
    });
    let view = slice.split_ref(0, 1, stop);
    assert_eq!(view.last_chunk_mask, 1);
    assert_eq!(view.single_mask[0] & view.last_chunk_mask, 1);
}

#[test]
fn test_black_box_support() {
    let backend = ScalarBackend;
    let mut slice = PackedPGSlice::new(3);
    slice.push_op(
        &Op::BlackBox {
            data: BlackBoxData::new(vec![0], "box".into()),
        },
        &backend,
    );

    let disjoint = TQE {
        gate_type: XX_U8,
        q0: 1,
        q1: 2,
    };
    assert!(slice.stop_point(disjoint).is_end());

    let overlapping = TQE {
        gate_type: XX_U8,
        q0: 0,
        q1: 1,
    };
    let stop = slice.stop_point(overlapping);
    assert!(!stop.is_end());
    slice.apply_tqe(overlapping, stop, &backend);

    assert!(!slice.stop_point(disjoint).is_end());
}

#[test]
fn test_attached_tableau_support() {
    let backend = ScalarBackend;
    let mut tableau = Tableau::eye(4);
    backend.apply_tqe_to_tableau(
        &mut tableau,
        TQE {
            gate_type: XX_U8,
            q0: 0,
            q1: 1,
        },
    );

    let mut slice = PackedPGSlice::new(4);
    slice.push_op(
        &Op::Tableau {
            data: TableauData::from(tableau),
        },
        &backend,
    );
    slice.push_op(
        &Op::BlackBox {
            data: BlackBoxData::new(vec![0], "box".into()),
        },
        &backend,
    );

    let disjoint = TQE {
        gate_type: XX_U8,
        q0: 2,
        q1: 3,
    };
    assert!(!slice.stop_point(disjoint).is_end());
}
