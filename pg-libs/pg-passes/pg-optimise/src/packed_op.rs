use ::pg_core::{Op, Pauli};
use std::collections::HashSet;
// Lookup arrays for two different Pauli encodings: [X, Y, Z, I].
// ZX_LUT: low bit = Z, high bit = X.
const ZX_LUT: [u64; 4] = [0b10, 0b11, 0b01, 0b00];
// XZ_LUT: low bit = X, high bit = Z.
const XZ_LUT: [u64; 4] = [0b01, 0b11, 0b10, 0b00];

const LOW_BITS: u64 = 0x5555_5555_5555_5555;
const HIGH_BITS: u64 = 0xAAAA_AAAA_AAAA_AAAA;

/// Encoding used for packed Pauli strings.
///
/// Each Pauli occupies two consecutive bits for its X and Z components. From
/// low to high, `ZX` stores Z then X, while `XZ` stores X then Z. The first
/// Pauli occupies the least significant pair.
enum PauliStringEncoding {
    ZX,
    XZ,
}

#[derive(Clone, Eq, PartialEq, Hash)]
pub(crate) struct XZPackedPaulis(Vec<u64>);

#[derive(Clone, Eq, PartialEq, Hash)]
pub(crate) struct ZXPackedPaulis(Vec<u64>);

impl From<&[Pauli]> for XZPackedPaulis {
    fn from(paulis: &[Pauli]) -> Self {
        Self(bitpack_paulis(paulis, PauliStringEncoding::XZ))
    }
}

impl From<&[Pauli]> for ZXPackedPaulis {
    fn from(paulis: &[Pauli]) -> Self {
        Self(bitpack_paulis(paulis, PauliStringEncoding::ZX))
    }
}

#[inline(always)]
fn swap_xz_bits(word: u64) -> u64 {
    ((word & LOW_BITS) << 1) | ((word & HIGH_BITS) >> 1)
}

impl From<XZPackedPaulis> for ZXPackedPaulis {
    fn from(mut packed: XZPackedPaulis) -> Self {
        for word in &mut packed.0 {
            *word = swap_xz_bits(*word);
        }
        Self(packed.0)
    }
}

/// Packs a Pauli string into two bits per Pauli.
fn bitpack_paulis(paulis: &[Pauli], encoding: PauliStringEncoding) -> Vec<u64> {
    let lut = if let PauliStringEncoding::ZX = encoding {
        &ZX_LUT
    } else {
        &XZ_LUT
    };
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
fn commute_bitpacked(zx_paulis: &ZXPackedPaulis, xz_paulis: &XZPackedPaulis) -> bool {
    if zx_paulis.0.len() != xz_paulis.0.len() {
        panic!("bitpacked Pauli strings must have the same length");
    }
    zx_paulis
        .0
        .iter()
        .zip(xz_paulis.0.iter())
        .map(|(&b0, &b1)| (b0 & b1).count_ones())
        .sum::<u32>()
        % 2
        == 0
}

/// Wraps an [`Op`] together with its bit-packed Pauli strings for commutation checks.
///
/// `P` determines the XZ or ZX encoding at compile time.
#[derive(Clone)]
pub(crate) struct BasePackedOp<P> {
    pub op: Op,
    strings: Option<Vec<P>>,
    classical_access: Option<Box<ClassicalAccess>>,
}

pub(crate) type PendingPackedOp = BasePackedOp<XZPackedPaulis>;
pub(crate) type PackedOp = BasePackedOp<ZXPackedPaulis>;

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

impl From<PendingPackedOp> for PackedOp {
    fn from(pending_op: PendingPackedOp) -> Self {
        Self {
            op: pending_op.op,
            strings: pending_op
                .strings
                .map(|s| s.into_iter().map(ZXPackedPaulis::from).collect()),
            classical_access: pending_op.classical_access,
        }
    }
}

impl<P> BasePackedOp<P>
where
    P: for<'a> From<&'a [Pauli]>,
{
    pub fn new(op: Op) -> Self {
        let classical_access = classical_access(&op).map(Box::new);
        let strings = op
            .get_paulis()
            .map(|ps| ps.iter().map(|p| P::from(p)).collect());
        Self {
            op,
            strings,
            classical_access,
        }
    }
}

impl PackedOp {
    pub fn commute_with(&self, other: &PendingPackedOp) -> bool {
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
        match (&self.strings, &other.strings) {
            (Some(p0), Some(p1)) => p0
                .iter()
                .all(|p0| p1.iter().all(|p1| commute_bitpacked(p0, p1))),
            _ => false,
        }
    }
    pub fn commute_with_string(&self, other: &XZPackedPaulis) -> bool {
        if matches!(self.op, Op::SetBoundary) {
            return true;
        }
        match &self.strings {
            Some(strings) => strings.iter().all(|s| commute_bitpacked(s, other)),
            None => false,
        }
    }
}

impl<P> From<BasePackedOp<P>> for Op {
    fn from(packed_op: BasePackedOp<P>) -> Op {
        packed_op.op
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
        let p1 = vec![Pauli::Z, Pauli::Z, Pauli::Y];
        let p2 = vec![Pauli::X, Pauli::Z, Pauli::I];
        let p3 = vec![Pauli::Y, Pauli::I, Pauli::Z];
        let p4 = vec![Pauli::I, Pauli::I, Pauli::I];
        assert!(commute_bitpacked(
            &p0.as_slice().into(),
            &p1.as_slice().into()
        ));
        assert!(commute_bitpacked(
            &p0.as_slice().into(),
            &p2.as_slice().into()
        ));
        assert!(!commute_bitpacked(
            &p0.as_slice().into(),
            &p3.as_slice().into()
        ));
        assert!(commute_bitpacked(
            &p0.as_slice().into(),
            &p0.as_slice().into()
        ));
        assert!(commute_bitpacked(
            &p1.as_slice().into(),
            &p3.as_slice().into()
        ));
        assert!(!commute_bitpacked(
            &p2.as_slice().into(),
            &p3.as_slice().into()
        ));
        assert!(!commute_bitpacked(
            &p1.as_slice().into(),
            &p2.as_slice().into()
        ));
        assert!(commute_bitpacked(
            &p3.as_slice().into(),
            &p3.as_slice().into()
        ));
        assert!(commute_bitpacked(
            &p3.as_slice().into(),
            &p4.as_slice().into()
        ));
        assert!(commute_bitpacked(
            &p4.as_slice().into(),
            &p4.as_slice().into()
        ));
    }

    #[test]
    #[should_panic(expected = "bitpacked Pauli strings must have the same length")]
    fn test_commute_bitpacked_rejects_unequal_lengths() {
        commute_bitpacked(&ZXPackedPaulis(vec![0]), &XZPackedPaulis(vec![]));
    }

    #[test]
    fn test_xz_to_zx_conversion_matches_direct_packing() {
        let paulis = (0..65)
            .map(|i| match i % 4 {
                0 => Pauli::I,
                1 => Pauli::X,
                2 => Pauli::Y,
                _ => Pauli::Z,
            })
            .collect::<Vec<_>>();

        let converted = ZXPackedPaulis::from(XZPackedPaulis::from(paulis.as_slice()));
        let directly_packed = ZXPackedPaulis::from(paulis.as_slice());

        assert_eq!(converted.0, directly_packed.0);
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

        assert!(PackedOp::new(op0.clone()).commute_with(&PendingPackedOp::new(op1.clone())));
        assert!(!PackedOp::new(op0.clone()).commute_with(&PendingPackedOp::new(op2.clone())));
        assert!(!PackedOp::new(op1.clone()).commute_with(&PendingPackedOp::new(op3.clone())));
        assert!(PackedOp::new(op3.clone()).commute_with(&PendingPackedOp::new(op4.clone())));
        assert!(PackedOp::new(op0.clone()).commute_with(&PendingPackedOp::new(op5.clone())));
        assert!(PackedOp::new(op2.clone()).commute_with(&PendingPackedOp::new(op4.clone())));
        assert!(!PackedOp::new(op6.clone()).commute_with(&PendingPackedOp::new(op4.clone())));
    }
}
