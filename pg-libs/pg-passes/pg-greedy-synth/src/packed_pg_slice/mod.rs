//! Storage for Pauli graph operations in packed bit form.

use crate::tqe::{PauliU8, TQE, TQEType};
use crate::utils::get_two_mut;
use pg_bitpacked::{apply_u8_tqe, bits_to_u8_pauli, paulis_to_u64s, u64s_to_paulis};
use pg_core::{BlackBoxData, ConditionalBoxData, MeasureData, Op, Pauli, ResetData, RotationData};
use pg_qm_tableau::Tableau;
use std::collections::HashSet;

pub(crate) trait PackedBackend {
    fn apply_tableau_to_pauli(
        &self,
        tableau: &Tableau,
        z_bits: &[u64],
        x_bits: &[u64],
    ) -> (Vec<u64>, Vec<u64>, bool);
    fn invert_tableau(&self, tableau: &Tableau) -> Tableau;
    fn compose_tableau(&self, lhs: &mut Tableau, rhs: &Tableau);
    fn apply_tqe_all(
        &self,
        zb_q0: &mut [u64],
        xb_q0: &mut [u64],
        zb_q1: &mut [u64],
        xb_q1: &mut [u64],
        sign_bits: &mut [u64],
        gate_type: TQEType,
    );
    fn apply_tqe_to_tableau(&self, tableau: &mut Tableau, tqe: TQE) {
        let (zb_q0, zb_q1, xb_q0, xb_q1, sign_bits) =
            tableau.split_qubit_slices_mut(tqe.q0, tqe.q1);
        self.apply_tqe_all(zb_q0, xb_q0, zb_q1, xb_q1, sign_bits, tqe.gate_type);
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
enum SlicePosition {
    Operation { set: usize, offset: usize },
    End,
}

/// Opaque index issued and consumed by `PackedPGSlice`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct SliceIndex(SlicePosition);

impl SliceIndex {
    const fn operation(set: usize, offset: usize) -> Self {
        Self(SlicePosition::Operation { set, offset })
    }

    const fn end() -> Self {
        Self(SlicePosition::End)
    }

    /// Returns true if this index points past the final operation.
    pub(crate) const fn is_end(&self) -> bool {
        matches!(self.0, SlicePosition::End)
    }
}

/// Bit position relative to the first visible chunk.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct SliceBitPosition {
    pub(crate) chunk: usize,
    pub(crate) bit: usize,
}

pub(crate) type SliceBitRange = (SliceBitPosition, SliceBitPosition);

/// Internal absolute bit position.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct PackedStoragePosition {
    chunk: usize,
    bit: usize,
}

impl PackedStoragePosition {
    fn after(self) -> Self {
        if self.bit == 63 {
            Self {
                chunk: self.chunk + 1,
                bit: 0,
            }
        } else {
            Self {
                chunk: self.chunk,
                bit: self.bit + 1,
            }
        }
    }

    fn relative_to(self, first_visible_chunk: usize) -> SliceBitPosition {
        SliceBitPosition {
            chunk: self.chunk - first_visible_chunk,
            bit: self.bit,
        }
    }
}

type StorageBitRange = (PackedStoragePosition, PackedStoragePosition);

/// Slice for two qubits containing their Z and X bit rows, operation masks and
/// the mask for the final chunk.
pub(crate) struct Packed2QSlice<'a> {
    pub(crate) zb_q0: &'a [u64],
    pub(crate) xb_q0: &'a [u64],
    pub(crate) zb_q1: &'a [u64],
    pub(crate) xb_q1: &'a [u64],
    pub(crate) single_mask: &'a [u64],
    pub(crate) pair_mask: &'a [u64],
    pub(crate) last_chunk_mask: u64,
}

pub(crate) enum PackedOpMeta {
    Rotation(PackedRotation),
    Measure(PackedMeasure),
    Reset(PackedReset),
    ConditionalBox(PackedConditionalBox),
    BlackBox(PackedBlackBox),
}

impl PackedOpMeta {
    fn bit_range(&self) -> StorageBitRange {
        match self {
            Self::Rotation(op) => (op.position, op.position.after()),
            Self::Measure(op) => (op.position, op.position.after()),
            Self::Reset(op) => (op.positions[0], op.positions[1].after()),
            Self::ConditionalBox(op) => op.bit_range,
            Self::BlackBox(op) => (op.position, op.position.after()),
        }
    }
}

/// Borrowed packed operation metadata and its visible bit range.
pub(crate) struct PackedOpView<'a> {
    pub(crate) meta: &'a PackedOpMeta,
    pub(crate) bit_range: SliceBitRange,
}

/// Metadata for a packed Pauli rotation.
pub(crate) struct PackedRotation {
    position: PackedStoragePosition,
    angle: f64,
}

impl PackedRotation {
    /// Returns the rotation angle.
    pub(crate) fn angle(&self) -> f64 {
        self.angle
    }
}

/// Metadata for a packed Pauli measurement.
pub(crate) struct PackedMeasure {
    position: PackedStoragePosition,
    cbit: usize,
}

impl PackedMeasure {
    /// Returns the target classical bit.
    pub(crate) fn cbit(&self) -> usize {
        self.cbit
    }
}

/// Storage positions for a packed reset pair.
pub(crate) struct PackedReset {
    positions: [PackedStoragePosition; 2],
}

/// Metadata and inner operations for a packed conditional box.
pub(crate) struct PackedConditionalBox {
    bit_range: StorageBitRange,
    conditional_bits: Vec<usize>,
    conditional_values: Vec<bool>,
    inner_ops: Vec<PackedOpMeta>,
}

/// Packed black box with its captured tableau and barrier support.
pub(crate) struct PackedBlackBox {
    position: PackedStoragePosition,
    data: BlackBoxData,
    tableau: Tableau,
    /// Qubits that prevent a TQE from commuting past this black box.
    ///
    /// The set starts with the qubits declared by the black box. It grows to the
    /// full register when a nonidentity tableau is attached, and absorbed TQEs
    /// add their endpoints. Entries are never removed.
    support: HashSet<usize>,
}

impl PackedBlackBox {
    /// Returns the black box data.
    pub(crate) fn data(&self) -> &BlackBoxData {
        &self.data
    }

    /// Returns the tableau captured before the black box.
    pub(crate) fn tableau(&self) -> &Tableau {
        &self.tableau
    }
    /// Reports whether both TQE endpoints lie outside the conservative support.
    ///
    /// The support starts with the declared black box qubits and only grows as
    /// tableaux and TQEs are absorbed.
    fn commutes_with(&self, tqe: TQE) -> bool {
        !(self.support.contains(&tqe.q0) || self.support.contains(&tqe.q1))
    }
}

/// Bit-packed Pauli graph window.
pub(crate) struct PackedPGSlice {
    zb_rows: Vec<Vec<u64>>,
    xb_rows: Vec<Vec<u64>>,
    sign_bits: Vec<u64>,
    single_mask: Vec<u64>,
    pair_mask: Vec<u64>,
    sets: Vec<Vec<PackedOpMeta>>,
    barriers: Vec<SliceIndex>,
    trailing_tableau: Tableau,
    next_column: usize,
    front_set: usize,
    front_barrier: usize,
    len: usize,
    next_push_starts_new_set: bool,
}

impl PackedPGSlice {
    /// Creates an empty packed slice for the given number of qubits.
    pub(crate) fn new(n_qubits: usize) -> Self {
        Self {
            zb_rows: vec![Vec::new(); n_qubits],
            xb_rows: vec![Vec::new(); n_qubits],
            sign_bits: Vec::new(),
            single_mask: Vec::new(),
            pair_mask: Vec::new(),
            sets: Vec::new(),
            barriers: Vec::new(),
            trailing_tableau: Tableau::eye(n_qubits),
            next_column: 0,
            front_set: 0,
            front_barrier: 0,
            len: 0,
            next_push_starts_new_set: false,
        }
    }

    /// Returns the number of visible operations.
    pub(crate) fn len(&self) -> usize {
        self.len
    }

    /// Returns true if the slice has no visible operations.
    pub(crate) fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Makes the next operation start a new set.
    pub(crate) fn start_new_set(&mut self) {
        self.next_push_starts_new_set = true;
    }

    /// Iterates over visible operation sets using positions relative to the
    /// first visible storage chunk.
    pub(crate) fn op_sets(
        &self,
    ) -> impl Iterator<Item = impl Iterator<Item = (SliceIndex, PackedOpView<'_>)>> {
        let first_visible_chunk = self
            .sets
            .get(self.front_set)
            .and_then(|set| set.first())
            .map(|op| op.bit_range().0.chunk)
            .unwrap_or(0);
        self.sets[self.front_set..]
            .iter()
            .enumerate()
            .map(move |(relative_set, set)| {
                let set_index = self.front_set + relative_set;
                set.iter().enumerate().map(move |(offset, meta)| {
                    let (start, end) = meta.bit_range();
                    (
                        SliceIndex::operation(set_index, offset),
                        PackedOpView {
                            meta,
                            bit_range: (
                                start.relative_to(first_visible_chunk),
                                end.relative_to(first_visible_chunk),
                            ),
                        },
                    )
                })
            })
    }

    /// Returns the visible operation at `index`.
    pub(crate) fn op(&self, index: SliceIndex) -> &PackedOpMeta {
        let SlicePosition::Operation { set, offset } = index.0 else {
            panic!("the trailing end does not identify an operation")
        };
        assert!(
            set >= self.front_set,
            "operation index is no longer visible"
        );
        &self.sets[set][offset]
    }

    /// Returns the trailing Clifford tableau.
    pub(crate) fn trailing_tableau(&self) -> &Tableau {
        &self.trailing_tableau
    }

    /// Consumes an empty slice and returns its trailing tableau.
    pub(crate) fn into_trailing_tableau(self) -> Tableau {
        assert!(self.is_empty(), "visible packed operations remain");
        self.trailing_tableau
    }

    /// Returns one local Pauli value from a packed operation string.
    pub(crate) fn pauli(&self, index: SliceIndex, string_index: usize, qubit: usize) -> PauliU8 {
        let position = Self::operation_position(self.op(index), string_index);
        bits_to_u8_pauli(
            ((self.zb_rows[qubit][position.chunk] >> position.bit) & 1) as u8,
            ((self.xb_rows[qubit][position.chunk] >> position.bit) & 1) as u8,
        )
    }

    /// Returns the sign bit of a packed operation string.
    pub(crate) fn sign_bit(&self, index: SliceIndex, string_index: usize) -> bool {
        let position = Self::operation_position(self.op(index), string_index);
        (self.sign_bits[position.chunk] >> position.bit) & 1 == 1
    }

    /// Decodes a packed conditional box.
    pub(crate) fn decode_conditional_box(&self, index: SliceIndex) -> ConditionalBoxData {
        let PackedOpMeta::ConditionalBox(boxed) = self.op(index) else {
            panic!("operation index does not identify a conditional box")
        };
        let ops = boxed
            .inner_ops
            .iter()
            .map(|op| self.decode_ordinary_op(op))
            .collect();
        ConditionalBoxData::new(
            ops,
            boxed.conditional_bits.clone(),
            boxed.conditional_values.clone(),
        )
    }

    /// Borrows the bit planes for two qubits and the operation masks from the
    /// first visible operation up to but not including `stop`.
    pub(crate) fn split_ref(&self, q0: usize, q1: usize, stop: SliceIndex) -> Packed2QSlice<'_> {
        self.assert_not_empty();
        let first_chunk = self.first_visible_op().bit_range().0.chunk;
        let (last_chunk, last_chunk_mask) = match stop.0 {
            SlicePosition::End => {
                let end = self.last_visible_op().bit_range().1;
                let last_chunk = if end.bit == 0 {
                    end.chunk - 1
                } else {
                    end.chunk
                };
                (last_chunk, u64::MAX)
            }
            SlicePosition::Operation { .. } => {
                let (start, _) = self.op(stop).bit_range();
                let mask = if start.bit == 0 {
                    0
                } else {
                    (1u64 << start.bit) - 1
                };
                (start.chunk, mask)
            }
        };
        let range = first_chunk..last_chunk + 1;
        Packed2QSlice {
            zb_q0: &self.zb_rows[q0][range.clone()],
            xb_q0: &self.xb_rows[q0][range.clone()],
            zb_q1: &self.zb_rows[q1][range.clone()],
            xb_q1: &self.xb_rows[q1][range.clone()],
            single_mask: &self.single_mask[range.clone()],
            pair_mask: &self.pair_mask[range],
            last_chunk_mask,
        }
    }

    /// Finds the first black box that the TQE cannot commute through.
    ///
    /// A black box blocks the TQE when either endpoint is in its support set.
    pub(crate) fn stop_point(&self, tqe: TQE) -> SliceIndex {
        self.barriers[self.front_barrier..]
            .iter()
            .copied()
            .find(|index| {
                let PackedOpMeta::BlackBox(black_box) = self.op(*index) else {
                    unreachable!()
                };
                !black_box.commutes_with(tqe)
            })
            .unwrap_or_else(SliceIndex::end)
    }

    /// Removes an operation from the masks used for candidate costing.
    pub(crate) fn clear_op_mask(&mut self, index: SliceIndex) {
        let (first, second) = match self.op(index) {
            PackedOpMeta::Rotation(op) => (op.position, None),
            PackedOpMeta::Measure(op) => (op.position, None),
            PackedOpMeta::Reset(op) => (op.positions[0], Some(op.positions[1])),
            PackedOpMeta::ConditionalBox(_) | PackedOpMeta::BlackBox(_) => {
                panic!("operation does not have a clearable mask")
            }
        };
        if let Some(second) = second {
            self.pair_mask[first.chunk] &= !(1 << first.bit);
            self.pair_mask[second.chunk] &= !(1 << second.bit);
        } else {
            self.single_mask[first.chunk] &= !(1 << first.bit);
        }
    }

    /// Logically retires the first visible set.
    pub(crate) fn progress_set(&mut self) -> bool {
        assert!(!self.is_empty(), "cannot progress an empty packed slice");
        self.len -= self.sets[self.front_set].len();
        self.front_set += 1;
        while self.front_barrier < self.barriers.len() {
            let SlicePosition::Operation { set, .. } = self.barriers[self.front_barrier].0 else {
                unreachable!()
            };
            if set >= self.front_set {
                break;
            }
            self.front_barrier += 1;
        }
        !self.is_empty()
    }

    /// Adds an operation to the packed slice.
    pub(crate) fn push_op<P: PackedBackend>(&mut self, op: &Op, backend: &P) {
        if let Op::Tableau { data } = op {
            let mut incoming = backend.invert_tableau(&Tableau::from(data.clone()));
            backend.compose_tableau(&mut incoming, &self.trailing_tableau);
            self.trailing_tableau = incoming;
            return;
        }

        let packed = match op {
            Op::Rotation { data } => self.pack_rotation(data, backend),
            Op::Measure { data } => self.pack_measure(data, backend),
            Op::Reset { data } => self.pack_reset(data, backend),
            Op::ConditionalBox { data } => self.pack_conditional(data, backend),
            Op::BlackBox { data } => self.pack_black_box(data),
            _ => panic!("unsupported operation in packed slice"),
        };
        // TODO: Some downstream code assumes that boxes are stored in standalone
        // sets. Remove this restriction once it can handle boxes mixed with
        // other operations.
        let standalone = matches!(
            packed,
            PackedOpMeta::ConditionalBox(_) | PackedOpMeta::BlackBox(_)
        );
        self.insert_packed(packed, standalone);
    }

    /// Conjugates visible packed operations by a TQE up to its barrier.
    ///
    /// At a black box barrier, the TQE is absorbed into the box's captured
    /// tableau and both endpoints are added permanently to its support. Without
    /// a barrier, the TQE is absorbed into the trailing tableau.
    pub(crate) fn apply_tqe<P: PackedBackend>(&mut self, tqe: TQE, stop: SliceIndex, backend: &P) {
        assert_eq!(stop, self.stop_point(tqe));
        let start = self.first_visible_op().bit_range().0;
        match stop.0 {
            SlicePosition::Operation { set, offset } => {
                let end = self.sets[set][offset].bit_range().0;
                self.apply_tqe_range(tqe, start, end, backend);
                let PackedOpMeta::BlackBox(black_box) = &mut self.sets[set][offset] else {
                    unreachable!()
                };
                backend.apply_tqe_to_tableau(&mut black_box.tableau, tqe);
                black_box.support.extend([tqe.q0, tqe.q1]);
            }
            SlicePosition::End => {
                let end = self.last_visible_op().bit_range().1;
                self.apply_tqe_range(tqe, start, end, backend);
                backend.apply_tqe_to_tableau(&mut self.trailing_tableau, tqe);
            }
        }
    }

    fn n_qubits(&self) -> usize {
        self.zb_rows.len()
    }

    fn assert_not_empty(&self) {
        assert!(!self.is_empty(), "packed slice is empty");
    }

    fn first_visible_op(&self) -> &PackedOpMeta {
        self.assert_not_empty();
        self.sets[self.front_set].first().unwrap()
    }

    fn last_visible_op(&self) -> &PackedOpMeta {
        self.assert_not_empty();
        self.sets.last().unwrap().last().unwrap()
    }

    fn operation_position(op: &PackedOpMeta, string_index: usize) -> PackedStoragePosition {
        match (op, string_index) {
            (PackedOpMeta::Rotation(op), 0) => op.position,
            (PackedOpMeta::Measure(op), 0) => op.position,
            (PackedOpMeta::Reset(op), 0) => op.positions[0],
            (PackedOpMeta::Reset(op), 1) => op.positions[1],
            _ => panic!("operation does not contain the requested Pauli string"),
        }
    }

    /// Allocates the next absolute packed position, optionally aligned to an
    /// even bit for the first string of a pair operation.
    fn next_position(&mut self, even: bool) -> PackedStoragePosition {
        // Pair operations are read with alternating-bit masks. Padding an odd
        // next column keeps a reset's two strings in the expected even/odd
        // positions, including when the pair crosses no chunk boundary.
        if even && !(self.next_column % 64).is_multiple_of(2) {
            self.next_column += 1;
        }
        let position = PackedStoragePosition {
            chunk: self.next_column / 64,
            bit: self.next_column % 64,
        };
        self.next_column += 1;
        position
    }

    /// Packs one Pauli string into one shared bit position.
    ///
    /// The Z and X rows for each qubit, the sign bit and the operation kind mask
    /// all use the returned position.
    fn pack_string(
        &mut self,
        paulis: &[Pauli],
        sign_bit: bool,
        single: bool,
        even: bool,
    ) -> PackedStoragePosition {
        let position = self.next_position(even);
        if position.chunk == self.sign_bits.len() {
            for row in &mut self.zb_rows {
                row.push(0);
            }
            for row in &mut self.xb_rows {
                row.push(0);
            }
            self.sign_bits.push(0);
            self.single_mask.push(0);
            self.pair_mask.push(0);
        }
        for (qubit, pauli) in paulis.iter().enumerate() {
            if matches!(pauli, Pauli::X | Pauli::Y) {
                self.xb_rows[qubit][position.chunk] |= 1 << position.bit;
            }
            if matches!(pauli, Pauli::Z | Pauli::Y) {
                self.zb_rows[qubit][position.chunk] |= 1 << position.bit;
            }
        }
        if sign_bit {
            self.sign_bits[position.chunk] |= 1 << position.bit;
        }
        if single {
            self.single_mask[position.chunk] |= 1 << position.bit;
        } else {
            self.pair_mask[position.chunk] |= 1 << position.bit;
        }
        position
    }

    fn conjugate<P: PackedBackend>(&self, paulis: &[Pauli], backend: &P) -> (Vec<Pauli>, bool) {
        let (z_bits, x_bits) = paulis_to_u64s(paulis);
        let (z_bits, x_bits, sign_bit) =
            backend.apply_tableau_to_pauli(&self.trailing_tableau, &z_bits, &x_bits);
        (u64s_to_paulis(&z_bits, &x_bits, self.n_qubits()), sign_bit)
    }

    fn pack_rotation<P: PackedBackend>(
        &mut self,
        data: &RotationData,
        backend: &P,
    ) -> PackedOpMeta {
        let (paulis, sign_bit) = self.conjugate(data.get_string(), backend);
        let position = self.pack_string(&paulis, sign_bit, true, false);
        PackedOpMeta::Rotation(PackedRotation {
            position,
            angle: data.get_angle(),
        })
    }

    fn pack_measure<P: PackedBackend>(&mut self, data: &MeasureData, backend: &P) -> PackedOpMeta {
        let (paulis, sign_bit) = self.conjugate(data.get_string(), backend);
        let position = self.pack_string(&paulis, sign_bit ^ data.get_sign_bit(), true, false);
        PackedOpMeta::Measure(PackedMeasure {
            position,
            cbit: data.get_cbit(),
        })
    }

    /// Packs a reset as two adjacent positions whose first bit is even, keeping
    /// both strings in the same chunk.
    fn pack_reset<P: PackedBackend>(&mut self, data: &ResetData, backend: &P) -> PackedOpMeta {
        let (first, first_sign_bit) = self.conjugate(data.get_first_string(), backend);
        let (second, second_sign_bit) = self.conjugate(data.get_second_string(), backend);
        let first_position = self.pack_string(
            &first,
            first_sign_bit ^ data.get_first_sign_bit(),
            false,
            true,
        );
        let second_position = self.pack_string(
            &second,
            second_sign_bit ^ data.get_second_sign_bit(),
            false,
            false,
        );
        PackedOpMeta::Reset(PackedReset {
            positions: [first_position, second_position],
        })
    }

    fn pack_conditional<P: PackedBackend>(
        &mut self,
        data: &ConditionalBoxData,
        backend: &P,
    ) -> PackedOpMeta {
        assert!(!data.get_ops().is_empty(), "conditional box is empty");
        let inner_ops: Vec<_> = data
            .get_ops()
            .iter()
            .filter_map(|op| match op {
                Op::SetBoundary => None,
                Op::Rotation { data } => Some(self.pack_rotation(data, backend)),
                Op::Measure { data } => Some(self.pack_measure(data, backend)),
                Op::Reset { data } => Some(self.pack_reset(data, backend)),
                _ => panic!("unsupported operation in conditional box"),
            })
            .collect();
        assert!(
            !inner_ops.is_empty(),
            "conditional box has no packed operations"
        );
        let first = inner_ops.first().unwrap();
        let last = inner_ops.last().unwrap();
        let bit_range = (first.bit_range().0, last.bit_range().1);
        PackedOpMeta::ConditionalBox(PackedConditionalBox {
            bit_range,
            conditional_bits: data.get_conditional_bits().clone(),
            conditional_values: data.get_conditional_values().clone(),
            inner_ops,
        })
    }

    /// Captures the current trailing tableau in a black box and starts a new
    /// trailing identity tableau.
    ///
    /// Its conservative support starts with the declared qubits.
    /// A captured tableau expands the support to the full register.
    fn pack_black_box(&mut self, data: &BlackBoxData) -> PackedOpMeta {
        let n_qubits = self.n_qubits();
        let tableau = std::mem::replace(&mut self.trailing_tableau, Tableau::eye(n_qubits));
        let position = self.pack_string(&vec![Pauli::I; n_qubits], false, true, false);
        let mut support: HashSet<_> = data.get_qubits().iter().copied().collect();
        // Treat an attached tableau as acting on the full register. This is a
        // cheap conservative barrier check; absorbed TQEs extend the same set.
        if &tableau != self.trailing_tableau() {
            support.extend(0..n_qubits);
        }
        PackedOpMeta::BlackBox(PackedBlackBox {
            position,
            data: data.clone(),
            tableau,
            support,
        })
    }

    /// Inserts a packed operation, placing boxes in standalone sets and
    /// recording black boxes as TQE barriers.
    fn insert_packed(&mut self, packed: PackedOpMeta, standalone: bool) {
        if standalone {
            self.sets.push(vec![packed]);
            let set = self.sets.len() - 1;
            if matches!(self.sets[set][0], PackedOpMeta::BlackBox(_)) {
                self.barriers.push(SliceIndex::operation(set, 0));
            }
            self.next_push_starts_new_set = true;
        } else {
            if self.sets.is_empty() || self.next_push_starts_new_set {
                self.sets.push(Vec::new());
                self.next_push_starts_new_set = false;
            }
            self.sets.last_mut().unwrap().push(packed);
        }
        self.len += 1;
    }

    fn decode_ordinary_op(&self, op: &PackedOpMeta) -> Op {
        let decode = |op: &PackedOpMeta, string_index| {
            let position = Self::operation_position(op, string_index);
            let paulis = (0..self.n_qubits())
                .map(|qubit| {
                    match (
                        (self.zb_rows[qubit][position.chunk] >> position.bit) & 1,
                        (self.xb_rows[qubit][position.chunk] >> position.bit) & 1,
                    ) {
                        (0, 0) => Pauli::I,
                        (0, 1) => Pauli::X,
                        (1, 0) => Pauli::Z,
                        (1, 1) => Pauli::Y,
                        _ => unreachable!(),
                    }
                })
                .collect();
            (
                paulis,
                (self.sign_bits[position.chunk] >> position.bit) & 1 == 1,
            )
        };
        match op {
            PackedOpMeta::Rotation(rotation) => {
                let (paulis, sign_bit) = decode(op, 0);
                Op::Rotation {
                    data: RotationData::new(
                        paulis,
                        if sign_bit {
                            -rotation.angle
                        } else {
                            rotation.angle
                        },
                    ),
                }
            }
            PackedOpMeta::Measure(measure) => {
                let (paulis, sign_bit) = decode(op, 0);
                Op::Measure {
                    data: MeasureData::new(paulis, sign_bit, measure.cbit),
                }
            }
            PackedOpMeta::Reset(_) => {
                let (first, first_sign_bit) = decode(op, 0);
                let (second, second_sign_bit) = decode(op, 1);
                Op::Reset {
                    data: ResetData::new(first, second, first_sign_bit, second_sign_bit),
                }
            }
            _ => panic!("conditional box contains an unsupported operation"),
        }
    }

    /// Applies a TQE to the absolute storage range from `start` up to but not
    /// including `end`.
    fn apply_tqe_range<P: PackedBackend>(
        &mut self,
        tqe: TQE,
        start: PackedStoragePosition,
        end: PackedStoragePosition,
        backend: &P,
    ) {
        if start == end {
            return;
        }
        let (zb_q0, zb_q1) = get_two_mut(&mut self.zb_rows, tqe.q0, tqe.q1);
        let (xb_q0, xb_q1) = get_two_mut(&mut self.xb_rows, tqe.q0, tqe.q1);
        if start.chunk == end.chunk {
            let mask = (u64::MAX << start.bit) & ((1u64 << end.bit) - 1);
            apply_u8_tqe(
                &mut zb_q0[start.chunk],
                &mut xb_q0[start.chunk],
                &mut zb_q1[start.chunk],
                &mut xb_q1[start.chunk],
                &mut self.sign_bits[start.chunk],
                mask,
                tqe.gate_type,
            );
            return;
        }
        let full_start = if start.bit == 0 {
            start.chunk
        } else {
            apply_u8_tqe(
                &mut zb_q0[start.chunk],
                &mut xb_q0[start.chunk],
                &mut zb_q1[start.chunk],
                &mut xb_q1[start.chunk],
                &mut self.sign_bits[start.chunk],
                u64::MAX << start.bit,
                tqe.gate_type,
            );
            start.chunk + 1
        };
        if full_start < end.chunk {
            backend.apply_tqe_all(
                &mut zb_q0[full_start..end.chunk],
                &mut xb_q0[full_start..end.chunk],
                &mut zb_q1[full_start..end.chunk],
                &mut xb_q1[full_start..end.chunk],
                &mut self.sign_bits[full_start..end.chunk],
                tqe.gate_type,
            );
        }
        if end.bit != 0 {
            apply_u8_tqe(
                &mut zb_q0[end.chunk],
                &mut xb_q0[end.chunk],
                &mut zb_q1[end.chunk],
                &mut xb_q1[end.chunk],
                &mut self.sign_bits[end.chunk],
                (1u64 << end.bit) - 1,
                tqe.gate_type,
            );
        }
    }
}

#[cfg(test)]
mod tests;
