//! Optimisation passes for Pauli graphs.

mod commuting_sets;
mod rotation_merging;
pub use commuting_sets::GroupCommutingOpsPass;
pub use rotation_merging::RotationMergingPass;

use ::pg_core::{Op, Pauli};
use std::collections::HashSet;
// Lookup arrays for two different Pauli encodings: [X, Y, Z, I].
// Z_FIRST_LUT: low bit = Z, high bit = X.
const Z_FIRST_LUT: [u64; 4] = [0b10, 0b11, 0b01, 0b00];
// X_FIRST_LUT: low bit = X, high bit = Z.
const X_FIRST_LUT: [u64; 4] = [0b01, 0b11, 0b10, 0b00];

/// Packs a Pauli string into two bits per Pauli.
///
/// Each Pauli is represented by its X and Z components. When `z_first` is
/// `true`, the Z component is the low bit of each pair and the X component is
/// the high bit; when it is `false`, the X component is the low bit and the Z
/// component is the high bit. The first Pauli occupies the least-significant
/// pair, followed by later Paulis in increasingly significant pairs.
///
fn bitpack_paulis(paulis: &[Pauli], z_first: bool) -> Vec<u64> {
    let lut = if z_first { &Z_FIRST_LUT } else { &X_FIRST_LUT };
    let n_u64s = paulis.len().div_ceil(32);
    let mut bits = Vec::with_capacity(n_u64s);
    for chunk in paulis.chunks(32) {
        let mut word: u64 = 0;
        for (i, pauli) in chunk.iter().enumerate() {
            word |= lut[*pauli as usize] << (i * 2);
        }
        bits.push(word);
    }
    bits
}

/// Checks whether two bit-packed Pauli strings commute.
///
/// The strings must use opposite encodings: one must have been packed with
/// `z_first` set to `true`, and the other with it set to `false`.
///
/// # Panics
///
/// Panics if the packed slices have different lengths.
///
fn commute_bitpacked(p0: &[u64], p1: &[u64]) -> bool {
    if p0.len() != p1.len() {
        panic!("bitpacked Pauli strings must have the same length");
    }
    p0.iter()
        .zip(p1.iter())
        .map(|(&b0, &b1)| (b0 & b1).count_ones())
        .sum::<u32>()
        % 2
        == 0
}

/// Wraps an [`Op`] together with its bit-packed Pauli strings for commutation checks.
///
/// The bits initially alternate between X and Z components. Calling
/// `flip_to_zx` repacks them so that they alternate between Z and X components.
/// A commutation check requires the two wrapped operations to use opposite
/// encodings.
#[derive(Clone)]
struct BitPackedOp {
    op: Op,
    strings: Option<Vec<Vec<u64>>>,
    classical_access: Option<Box<ClassicalAccess>>,
    z_first: bool,
}

/// Classical bits read and written by an operation and its nested operations.
#[derive(Clone, Default)]
struct ClassicalAccess {
    read_bits: HashSet<usize>,
    write_bits: HashSet<usize>,
}

fn classical_access(op: &Op) -> Option<ClassicalAccess> {
    match op {
        Op::Measure { data } => Some(ClassicalAccess {
            write_bits: HashSet::from([data.get_cbit()]),
            ..Default::default()
        }),
        Op::ConditionalBox { data } => {
            let mut access = ClassicalAccess {
                read_bits: data.get_conditional_bits().iter().copied().collect(),
                ..Default::default()
            };
            for inner_access in data.get_ops().iter().filter_map(classical_access) {
                access.read_bits.extend(inner_access.read_bits);
                access.write_bits.extend(inner_access.write_bits);
            }
            Some(access)
        }
        _ => None,
    }
}

/// Returns whether reordering two operations is invalid due to a classical conflict.
/// Two reads are safe, but any shared bit involving a write is a conflict.
fn classical_conflict(first: &ClassicalAccess, second: &ClassicalAccess) -> bool {
    !first.read_bits.is_disjoint(&second.write_bits)
        || !first.write_bits.is_disjoint(&second.read_bits)
        || !first.write_bits.is_disjoint(&second.write_bits)
}

impl BitPackedOp {
    fn new(op: Op, z_first: bool) -> Self {
        let classical_access = classical_access(&op).map(Box::new);
        let strings = op
            .get_paulis()
            .map(|ps| ps.iter().map(|p| bitpack_paulis(p, z_first)).collect());
        Self {
            op,
            strings,
            classical_access,
            z_first,
        }
    }
    fn flip_to_zx(&mut self) {
        self.strings = self
            .op
            .get_paulis()
            .map(|ps| ps.iter().map(|p| bitpack_paulis(p, true)).collect());
        self.z_first = true;
    }
    fn commute_with(&self, other: &Self) -> bool {
        // Set boundaries commute with every operation.
        if matches!(self.op, Op::SetBoundary) || matches!(other.op, Op::SetBoundary) {
            return true;
        }
        // Operations cannot be reordered across a classical write dependency.
        if let (Some(self_access), Some(other_access)) =
            (&self.classical_access, &other.classical_access)
            && classical_conflict(self_access, other_access)
        {
            return false;
        }
        assert!(
            self.z_first != other.z_first,
            "both Ops use the same bit-packing; the commutation check may be incorrect"
        );
        match (&self.strings, &other.strings) {
            (Some(p0), Some(p1)) => p0
                .iter()
                .all(|p0| p1.iter().all(|p1| commute_bitpacked(p0, p1))),
            _ => false,
        }
    }
    fn commute_with_string(&self, other: &[u64]) -> bool {
        if matches!(self.op, Op::SetBoundary) {
            return true;
        }
        match &self.strings {
            Some(strings) => strings.iter().all(|s| commute_bitpacked(s, other)),
            None => false,
        }
    }
}

impl From<BitPackedOp> for Op {
    fn from(rich: BitPackedOp) -> Op {
        rich.op
    }
}

trait PauliOp {
    fn get_paulis(&self) -> Option<Vec<&Vec<Pauli>>>;
}
impl PauliOp for Op {
    fn get_paulis(&self) -> Option<Vec<&Vec<Pauli>>> {
        match self {
            Op::Rotation { data } => Some(vec![data.get_string()]),
            Op::Measure { data } => Some(vec![data.get_string()]),
            Op::Reset { data } => Some(vec![data.get_first_string(), data.get_second_string()]),
            // A conditional box should contain only the Op above, so
            // recursively collect their Pauli strings.
            Op::ConditionalBox { data } => {
                let mut paulis = Vec::new();
                for op in data.get_ops() {
                    if let Some(op_paulis) = op.get_paulis() {
                        paulis.extend(op_paulis);
                    } else {
                        return None;
                    }
                }
                Some(paulis)
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use pg_core::{MeasureData, ResetData, RotationData};

    use super::*;

    #[test]
    fn test_commute_bitpacked() {
        let p0 = vec![Pauli::X, Pauli::I, Pauli::Z];
        let p1 = vec![Pauli::X, Pauli::I, Pauli::Z];
        let p2 = vec![Pauli::X, Pauli::Z, Pauli::I];
        let p3 = vec![Pauli::Y, Pauli::I, Pauli::Z];
        let p4 = vec![Pauli::I, Pauli::I, Pauli::I];
        assert!(commute_bitpacked(
            &bitpack_paulis(&p0, true),
            &bitpack_paulis(&p1, false)
        ));
        assert!(commute_bitpacked(
            &bitpack_paulis(&p0, true),
            &bitpack_paulis(&p2, false)
        ));
        assert!(!commute_bitpacked(
            &bitpack_paulis(&p0, true),
            &bitpack_paulis(&p3, false)
        ));
        assert!(commute_bitpacked(
            &bitpack_paulis(&p0, true),
            &bitpack_paulis(&p0, false)
        ));
        assert!(!commute_bitpacked(
            &bitpack_paulis(&p1, true),
            &bitpack_paulis(&p3, false)
        ));
        assert!(!commute_bitpacked(
            &bitpack_paulis(&p2, true),
            &bitpack_paulis(&p3, false)
        ));
        assert!(commute_bitpacked(
            &bitpack_paulis(&p1, true),
            &bitpack_paulis(&p2, false)
        ));
        assert!(commute_bitpacked(
            &bitpack_paulis(&p3, true),
            &bitpack_paulis(&p3, false)
        ));
        assert!(commute_bitpacked(
            &bitpack_paulis(&p3, true),
            &bitpack_paulis(&p4, false)
        ));
        assert!(commute_bitpacked(
            &bitpack_paulis(&p4, true),
            &bitpack_paulis(&p4, false)
        ));
    }

    #[test]
    #[should_panic(expected = "bitpacked Pauli strings must have the same length")]
    fn test_commute_bitpacked_rejects_unequal_lengths() {
        commute_bitpacked(&[0], &[]);
    }

    #[test]
    fn test_op_commute() {
        let op0 = Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::I], 0.1),
        };
        let op1 = Op::Rotation {
            data: RotationData::new(vec![Pauli::X, Pauli::X], 0.1),
        };
        let op2 = Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.1),
        };
        let op3 = Op::Rotation {
            data: RotationData::new(vec![Pauli::Z, Pauli::X], 0.2),
        };
        let op4 = Op::Measure {
            data: MeasureData::new(vec![Pauli::Z, Pauli::X], true, 0),
        };
        let op5 = Op::Reset {
            data: ResetData::new(
                vec![Pauli::I, Pauli::Z],
                vec![Pauli::I, Pauli::X],
                true,
                true,
            ),
        };
        let op6 = Op::Reset {
            data: ResetData::new(
                vec![Pauli::Z, Pauli::I],
                vec![Pauli::X, Pauli::I],
                true,
                true,
            ),
        };

        assert!(
            BitPackedOp::new(op0.clone(), false).commute_with(&BitPackedOp::new(op1.clone(), true))
        );
        assert!(
            !BitPackedOp::new(op0.clone(), false)
                .commute_with(&BitPackedOp::new(op2.clone(), true))
        );
        assert!(
            !BitPackedOp::new(op1.clone(), false)
                .commute_with(&BitPackedOp::new(op3.clone(), true))
        );
        assert!(
            BitPackedOp::new(op3.clone(), false).commute_with(&BitPackedOp::new(op4.clone(), true))
        );
        assert!(
            BitPackedOp::new(op0.clone(), false).commute_with(&BitPackedOp::new(op5.clone(), true))
        );
        assert!(
            BitPackedOp::new(op2.clone(), false).commute_with(&BitPackedOp::new(op4.clone(), true))
        );
        assert!(
            !BitPackedOp::new(op6.clone(), false)
                .commute_with(&BitPackedOp::new(op4.clone(), true))
        );
    }
}
