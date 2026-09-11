use super::*;
use pg_bitpacked::{
    I_U8 as I, X_U8 as X, XX_U8, XY_U8, XZ_U8, Y_U8 as Y, YX_U8, YY_U8, YZ_U8, Z_U8 as Z, ZX_U8,
    ZY_U8, ZZ_U8, apply_u8_tqe_slice, bits_to_u8_pauli,
};

const NONIDENTITY_PAULIS: [PauliU8; 3] = [X, Y, Z];
const PAULIS: [PauliU8; 4] = [X, Y, Z, I];
const TQES: [TQEType; 9] = [
    XX_U8, XY_U8, XZ_U8, YX_U8, YY_U8, YZ_U8, ZX_U8, ZY_U8, ZZ_U8,
];

fn pauli_bits(pauli: PauliU8) -> (u64, u64) {
    match pauli {
        X => (0, 1),
        Y => (1, 1),
        Z => (1, 0),
        I => (0, 0),
        _ => unreachable!(),
    }
}

fn pair_class(left: PauliU8, right: PauliU8) -> u8 {
    if left == I && right == I {
        TYPE_I
    } else if left == I {
        TYPE_R
    } else if right == I {
        TYPE_L
    } else if left == right {
        TYPE_E
    } else {
        TYPE_A
    }
}

fn apply_single(p0: PauliU8, p1: PauliU8, gate: TQEType) -> (PauliU8, PauliU8, bool) {
    let (z0, x0) = pauli_bits(p0);
    let (z1, x1) = pauli_bits(p1);
    let (mut z0, mut x0, mut z1, mut x1, mut sign_bits) = ([z0], [x0], [z1], [x1], [0]);
    apply_u8_tqe_slice(&mut z0, &mut x0, &mut z1, &mut x1, &mut sign_bits, gate);
    (
        bits_to_u8_pauli(z0[0] as u8, x0[0] as u8),
        bits_to_u8_pauli(z1[0] as u8, x1[0] as u8),
        sign_bits[0] == 1,
    )
}

fn apply_pair(
    left0: PauliU8,
    right0: PauliU8,
    left1: PauliU8,
    right1: PauliU8,
    gate: TQEType,
) -> ([[PauliU8; 2]; 2], [bool; 2]) {
    let (left_z0, left_x0) = pauli_bits(left0);
    let (right_z0, right_x0) = pauli_bits(right0);
    let (left_z1, left_x1) = pauli_bits(left1);
    let (right_z1, right_x1) = pauli_bits(right1);
    let mut z0 = [left_z0 | (right_z0 << 1)];
    let mut x0 = [left_x0 | (right_x0 << 1)];
    let mut z1 = [left_z1 | (right_z1 << 1)];
    let mut x1 = [left_x1 | (right_x1 << 1)];
    let mut sign_bits = [0];
    apply_u8_tqe_slice(&mut z0, &mut x0, &mut z1, &mut x1, &mut sign_bits, gate);
    (
        [
            [
                bits_to_u8_pauli((z0[0] & 1) as u8, (x0[0] & 1) as u8),
                bits_to_u8_pauli(((z0[0] >> 1) & 1) as u8, ((x0[0] >> 1) & 1) as u8),
            ],
            [
                bits_to_u8_pauli((z1[0] & 1) as u8, (x1[0] & 1) as u8),
                bits_to_u8_pauli(((z1[0] >> 1) & 1) as u8, ((x1[0] >> 1) & 1) as u8),
            ],
        ],
        [sign_bits[0] & 1 != 0, sign_bits[0] & 2 != 0],
    )
}

fn support_delta(before: PauliU8, after: PauliU8) -> i8 {
    i8::from(after != I) - i8::from(before != I)
}

fn class_delta(before: u8, after: u8, class: u8) -> i8 {
    i8::from(after == class) - i8::from(before == class)
}

#[test]
fn reducing_candidate_tables_are_complete() {
    for p0 in NONIDENTITY_PAULIS {
        for p1 in NONIDENTITY_PAULIS {
            let actual = (0..4)
                .map(|offset| single_tqes(p0, p1, offset))
                .collect::<std::collections::HashSet<_>>();
            let expected = TQES
                .into_iter()
                .filter(|&gate| {
                    let (next0, next1, _) = apply_single(p0, p1, gate);
                    usize::from(next0 != I) + usize::from(next1 != I) == 1
                })
                .collect();
            assert_eq!(actual, expected, "local Paulis ({p0}, {p1})");
            assert_eq!(actual.len(), 4);
        }
    }

    for left0 in NONIDENTITY_PAULIS {
        for right0 in NONIDENTITY_PAULIS {
            for left1 in NONIDENTITY_PAULIS {
                for right1 in NONIDENTITY_PAULIS {
                    if pair_class(left0, right0) != TYPE_A || pair_class(left1, right1) != TYPE_A {
                        continue;
                    }
                    let actual = (0..6)
                        .map(|offset| aa_tqes(left0, right0, left1, right1, offset))
                        .collect::<std::collections::HashSet<_>>();
                    let expected = TQES
                        .into_iter()
                        .filter(|&gate| {
                            let (next, _) = apply_pair(left0, right0, left1, right1, gate);
                            pair_class(next[0][0], next[0][1]) != TYPE_A
                                && pair_class(next[1][0], next[1][1]) != TYPE_A
                        })
                        .collect();
                    assert_eq!(actual, expected);
                    assert_eq!(actual.len(), 6);
                }
            }
        }
    }
}

#[test]
fn reduction_transition_tables_match_conjugation() {
    for p0 in PAULIS {
        for p1 in PAULIS {
            for gate in TQES {
                let (next0, next1, sign_flip) = apply_single(p0, p1, gate);
                assert_eq!(
                    single_reduction_with_all_stats(p0, p1, gate),
                    (
                        next0,
                        next1,
                        sign_flip,
                        support_delta(p0, next0),
                        support_delta(p1, next1),
                    )
                );
            }
        }
    }

    for left0 in PAULIS {
        for right0 in PAULIS {
            for left1 in PAULIS {
                for right1 in PAULIS {
                    for gate in TQES {
                        let (next, sign_flips) = apply_pair(left0, right0, left1, right1, gate);
                        let old = [pair_class(left0, right0), pair_class(left1, right1)];
                        let new = [
                            pair_class(next[0][0], next[0][1]),
                            pair_class(next[1][0], next[1][1]),
                        ];
                        let actual = pair_reduction(left0, right0, left1, right1, gate);
                        assert_eq!(actual.q0, next[0]);
                        assert_eq!(actual.q1, next[1]);
                        assert_eq!(actual.sign_flips, sign_flips);
                        assert_eq!(
                            actual.class_deltas,
                            [
                                [
                                    class_delta(old[0], new[0], TYPE_A),
                                    class_delta(old[0], new[0], TYPE_L),
                                    class_delta(old[0], new[0], TYPE_R),
                                    class_delta(old[0], new[0], TYPE_E),
                                ],
                                [
                                    class_delta(old[1], new[1], TYPE_A),
                                    class_delta(old[1], new[1], TYPE_L),
                                    class_delta(old[1], new[1], TYPE_R),
                                    class_delta(old[1], new[1], TYPE_E),
                                ],
                            ]
                        );
                    }
                }
            }
        }
    }
}
