//! Mutable reduction frontier for packed operations and tableau generators.

use crate::packed_pg_slice::{PackedOpMeta, PackedPGSlice, SliceIndex};
use crate::tqe::{PauliU8, TQE, TQEType};
use crate::utils::get_two_mut;
use indexmap::IndexSet;
use pg_bitpacked::{I_U8, X_U8, Y_U8, Z_U8, bits_to_u8_pauli};
use pg_core::{GateData, GateType, Op};
use pg_qm_tableau::Tableau;
use rand::Rng;
use rand::seq::index::sample;

mod tables;

use tables::{
    TYPE_A, TYPE_E, TYPE_I, TYPE_L, TYPE_R, aa_tqes, pair_reduction, pair_type,
    single_reduction_with_all_stats, single_tqes,
};

/// A sampled TQE and the frontier operation it is intended to reduce.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct FrontierReductionGate {
    pub(crate) tqe: TQE,
    pub(crate) frontier_index: FrontierIndex,
}

/// An opaque index of an operation in the frontier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub(crate) struct FrontierIndex(usize);

/// Operations produced by lowering one [`Op`].
pub(crate) struct FrontierEmission {
    // Index of the source operation in the packed slice.
    pub(crate) slice_index: SliceIndex,
    pub(crate) ops: Vec<Op>,
}

/// A tableau generator pair reduced to one qubit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct ReducedTableauPair {
    pub(crate) pair_index: usize,
    pub(crate) qubit: usize,
    pub(crate) left: PauliU8,
    pub(crate) right: PauliU8,
    pub(crate) left_sign_bit: bool,
    pub(crate) right_sign_bit: bool,
}

#[derive(Debug, Clone, Copy)]
enum FrontierMetadata {
    Rotation { angle: f64 },
    Measure { cbit: usize },
}

#[derive(Debug)]
struct FrontierSingleOp {
    slice_index: SliceIndex,
    metadata: FrontierMetadata,
    array_index: usize,
    supports: IndexSet<usize>,
    tqe_cost: usize,
    support_pairs: usize,
    reduction_gates: usize,
}

impl FrontierSingleOp {
    fn new(
        slice_index: SliceIndex,
        metadata: FrontierMetadata,
        array_index: usize,
        n_qubits: usize,
    ) -> Self {
        Self {
            slice_index,
            metadata,
            array_index,
            supports: IndexSet::with_capacity(n_qubits),
            tqe_cost: 0,
            support_pairs: 0,
            reduction_gates: 0,
        }
    }

    /// Recomputes the cost of a single string as one less than its support size
    /// and updates the number of TQE candidates that reduce it.
    fn recompute(&mut self) {
        assert!(!self.supports.is_empty());
        self.tqe_cost = self.supports.len() - 1;
        self.support_pairs = self.supports.len() * self.tqe_cost / 2;
        self.reduction_gates = self.support_pairs * 4;
    }

    fn gate(&self, index: usize, paulis: &[Vec<PauliU8>]) -> TQE {
        let gate_offset = index / self.support_pairs;
        let pair_index = index % self.support_pairs;
        let (first, second) = index_to_pair(pair_index, self.supports.len());
        let q0 = self.supports[first];
        let q1 = self.supports[second];
        TQE {
            gate_type: single_tqes(
                paulis[q0][self.array_index],
                paulis[q1][self.array_index],
                gate_offset,
            ),
            q0,
            q1,
        }
    }

    fn update(&mut self, q0: usize, q1: usize, change0: i8, change1: i8) {
        update_support(&mut self.supports, q0, change0);
        update_support(&mut self.supports, q1, change1);
        self.recompute();
    }
}

/// Reduction state for two anticommuting Pauli strings.
#[derive(Debug, Clone)]
struct PairState {
    array_index: usize,
    anticommuting_supports: IndexSet<usize>,
    left_supports: IndexSet<usize>,
    right_supports: IndexSet<usize>,
    equal_supports: IndexSet<usize>,
    tqe_cost: usize,
    aa_pair_count: usize,
    al_pair_count: usize,
    ar_pair_count: usize,
    ae_pair_count: usize,
    ll_pair_count: usize,
    rr_pair_count: usize,
    ee_pair_count: usize,
    aa_range_end: usize,
    al_range_end: usize,
    ar_range_end: usize,
    ae_range_end: usize,
    ll_range_end: usize,
    rr_range_end: usize,
    reduction_gates: usize,
}

impl PairState {
    fn new(array_index: usize, n_qubits: usize) -> Self {
        Self {
            array_index,
            anticommuting_supports: IndexSet::with_capacity(n_qubits),
            left_supports: IndexSet::with_capacity(n_qubits),
            right_supports: IndexSet::with_capacity(n_qubits),
            equal_supports: IndexSet::with_capacity(n_qubits),
            tqe_cost: 0,
            aa_pair_count: 0,
            al_pair_count: 0,
            ar_pair_count: 0,
            ae_pair_count: 0,
            ll_pair_count: 0,
            rr_pair_count: 0,
            ee_pair_count: 0,
            aa_range_end: 0,
            al_range_end: 0,
            ar_range_end: 0,
            ae_range_end: 0,
            ll_range_end: 0,
            rr_range_end: 0,
            reduction_gates: 0,
        }
    }

    /// Records how the two Pauli strings act on one qubit.
    ///
    /// `A` means the two Paulis anticommute. `L` means only the left Pauli is
    /// nonidentity, while `R` means only the right Pauli is nonidentity. `E`
    /// means both are the same nonidentity Pauli, and `I` means both are
    /// identity. Identity pairs are not stored.
    fn add_support(&mut self, pair_type: u8, qubit: usize) {
        match pair_type {
            TYPE_I => {}
            TYPE_A => {
                self.anticommuting_supports.insert(qubit);
            }
            TYPE_L => {
                self.left_supports.insert(qubit);
            }
            TYPE_R => {
                self.right_supports.insert(qubit);
            }
            TYPE_E => {
                self.equal_supports.insert(qubit);
            }
            _ => unreachable!("invalid pair support type"),
        }
    }

    /// Returns the reducing TQE identified by a flat candidate index.
    ///
    /// Cached range ends partition the index without allocating the complete
    /// candidate list. Each support set keeps its insertion order, so the
    /// mapping remains stable until the state is updated.
    fn gate(&self, index: usize, paulis: &[Vec<PauliU8>]) -> TQE {
        // TQE values use `3 * first_port + second_port`. When the two local
        // Paulis anticommute, `3 - left - right` gives the third nonidentity
        // Pauli because X, Y and Z have compact values 0, 1 and 2.
        let (gate_type, q0, q1) = if index < self.aa_range_end {
            let gate_offset = index / self.aa_pair_count;
            let pair_index = index % self.aa_pair_count;
            let (first, second) = index_to_pair(pair_index, self.anticommuting_supports.len());
            let q0 = self.anticommuting_supports[first];
            let q1 = self.anticommuting_supports[second];
            let left0 = paulis[q0][2 * self.array_index];
            let right0 = paulis[q0][2 * self.array_index + 1];
            let left1 = paulis[q1][2 * self.array_index];
            let right1 = paulis[q1][2 * self.array_index + 1];
            (aa_tqes(left0, right0, left1, right1, gate_offset), q0, q1)
        } else if index < self.al_range_end {
            let index = index - self.aa_range_end;
            let q0 = self.anticommuting_supports[index / self.left_supports.len()];
            let q1 = self.left_supports[index % self.left_supports.len()];
            let right0 = paulis[q0][2 * self.array_index + 1];
            let left1 = paulis[q1][2 * self.array_index];
            (3 * right0 + left1, q0, q1)
        } else if index < self.ar_range_end {
            let index = index - self.al_range_end;
            let q0 = self.anticommuting_supports[index / self.right_supports.len()];
            let q1 = self.right_supports[index % self.right_supports.len()];
            let left0 = paulis[q0][2 * self.array_index];
            let right1 = paulis[q1][2 * self.array_index + 1];
            (3 * left0 + right1, q0, q1)
        } else if index < self.ae_range_end {
            let index = index - self.ar_range_end;
            let q0 = self.anticommuting_supports[index / self.equal_supports.len()];
            let q1 = self.equal_supports[index % self.equal_supports.len()];
            let left0 = paulis[q0][2 * self.array_index];
            let right0 = paulis[q0][2 * self.array_index + 1];
            let right1 = paulis[q1][2 * self.array_index + 1];
            (3 * (3 - left0 - right0) + right1, q0, q1)
        } else if index < self.ll_range_end {
            self.single_set_gate(
                index - self.ae_range_end,
                self.ll_pair_count,
                &self.left_supports,
                0,
                paulis,
            )
        } else if index < self.rr_range_end {
            self.single_set_gate(
                index - self.ll_range_end,
                self.rr_pair_count,
                &self.right_supports,
                1,
                paulis,
            )
        } else {
            self.single_set_gate(
                index - self.rr_range_end,
                self.ee_pair_count,
                &self.equal_supports,
                0,
                paulis,
            )
        };
        TQE { gate_type, q0, q1 }
    }

    fn single_set_gate(
        &self,
        index: usize,
        pair_count: usize,
        supports: &IndexSet<usize>,
        column: usize,
        paulis: &[Vec<PauliU8>],
    ) -> (TQEType, usize, usize) {
        let gate_offset = index / pair_count;
        let (first, second) = index_to_pair(index % pair_count, supports.len());
        let q0 = supports[first];
        let q1 = supports[second];
        let pauli0 = paulis[q0][2 * self.array_index + column];
        let pauli1 = paulis[q1][2 * self.array_index + column];
        (single_tqes(pauli0, pauli1, gate_offset), q0, q1)
    }

    /// Recomputes the candidate ranges and the number of TQEs needed to reduce
    /// the two strings to one qubit.
    ///
    /// An anticommuting pair has an odd number of qubits where its local Paulis
    /// anticommute.
    fn recompute(&mut self) {
        let anticommuting_count = self.anticommuting_supports.len();
        let left_count = self.left_supports.len();
        let right_count = self.right_supports.len();
        let equal_count = self.equal_supports.len();
        let commuting_count = left_count + right_count + equal_count;
        assert!(anticommuting_count % 2 == 1);

        // Candidates from the same support set use unordered qubit pairs.
        // Candidates from different sets use their Cartesian product. AA
        // pairs have six reducing TQEs, LL, RR and EE pairs have four, and
        // mixed pairs have one.
        self.aa_pair_count = (anticommuting_count * anticommuting_count - anticommuting_count) / 2;
        self.al_pair_count = anticommuting_count * left_count;
        self.ar_pair_count = anticommuting_count * right_count;
        self.ae_pair_count = anticommuting_count * equal_count;
        self.ll_pair_count = (left_count * left_count - left_count) / 2;
        self.rr_pair_count = (right_count * right_count - right_count) / 2;
        self.ee_pair_count = (equal_count * equal_count - equal_count) / 2;
        self.aa_range_end = self.aa_pair_count * 6;
        self.al_range_end = self.aa_range_end + self.al_pair_count;
        self.ar_range_end = self.al_range_end + self.ar_pair_count;
        self.ae_range_end = self.ar_range_end + self.ae_pair_count;
        self.ll_range_end = self.ae_range_end + 4 * self.ll_pair_count;
        self.rr_range_end = self.ll_range_end + 4 * self.rr_pair_count;
        self.reduction_gates = self.aa_pair_count * 6
            + self.al_pair_count
            + self.ar_pair_count
            + self.ae_pair_count
            + 4 * (self.ll_pair_count + self.rr_pair_count + self.ee_pair_count);
        // Pairing anticommuting supports leaves one as the pivot. Reducing
        // every other nonidentity support against it takes
        // `anticommuting_count + commuting_count - 1` more TQEs.
        self.tqe_cost = (anticommuting_count - 1) / 2 + anticommuting_count + commuting_count - 1;
    }

    fn update(&mut self, q0: usize, q1: usize, class_deltas: [[i8; 4]; 2]) {
        for (qubit, [anticommuting, left, right, equal]) in
            [(q0, class_deltas[0]), (q1, class_deltas[1])]
        {
            update_support(&mut self.anticommuting_supports, qubit, anticommuting);
            update_support(&mut self.left_supports, qubit, left);
            update_support(&mut self.right_supports, qubit, right);
            update_support(&mut self.equal_supports, qubit, equal);
        }
        self.recompute();
    }
}

#[derive(Debug)]
struct FrontierPairOp {
    slice_index: SliceIndex,
    state: PairState,
}

#[derive(Debug)]
struct FrontierTableauPair {
    pair_index: usize,
    state: PairState,
}

#[derive(Debug)]
enum FrontierOp {
    Single(FrontierSingleOp),
    Pair(FrontierPairOp),
    TableauPair(FrontierTableauPair),
}

impl FrontierOp {
    fn array_index(&self) -> usize {
        match self {
            Self::Single(op) => op.array_index,
            Self::Pair(op) => op.state.array_index,
            Self::TableauPair(op) => op.state.array_index,
        }
    }

    fn cost(&self) -> usize {
        match self {
            Self::Single(op) => op.tqe_cost,
            Self::Pair(op) => op.state.tqe_cost,
            Self::TableauPair(op) => op.state.tqe_cost,
        }
    }

    fn reduction_gates(&self) -> usize {
        match self {
            Self::Single(op) => op.reduction_gates,
            Self::Pair(op) => op.state.reduction_gates,
            Self::TableauPair(op) => op.state.reduction_gates,
        }
    }

    fn gate(
        &self,
        index: usize,
        single_paulis: &[Vec<PauliU8>],
        pair_paulis: &[Vec<PauliU8>],
    ) -> TQE {
        match self {
            Self::Single(op) => op.gate(index, single_paulis),
            Self::Pair(op) => op.state.gate(index, pair_paulis),
            Self::TableauPair(op) => op.state.gate(index, pair_paulis),
        }
    }

    fn recompute(&mut self) {
        match self {
            Self::Single(op) => op.recompute(),
            Self::Pair(op) => op.state.recompute(),
            Self::TableauPair(op) => op.state.recompute(),
        }
    }

    fn pair_state_mut(&mut self) -> &mut PairState {
        match self {
            Self::Pair(op) => &mut op.state,
            Self::TableauPair(op) => &mut op.state,
            Self::Single(_) => unreachable!("single operation in pair frontier set"),
        }
    }
}

/// Mutable reduction state for the current operation set.
pub(crate) struct Frontier {
    single_paulis: Vec<Vec<PauliU8>>,
    single_sign_bits: Vec<bool>,
    pair_paulis: Vec<Vec<PauliU8>>,
    pair_sign_bits: Vec<bool>,
    ops: Vec<FrontierOp>,
    remaining_single_ops: IndexSet<FrontierIndex>,
    remaining_pair_ops: IndexSet<FrontierIndex>,
    cost_buckets: Vec<IndexSet<FrontierIndex>>,
    minimum_cost: Option<usize>,
}

impl Frontier {
    /// Creates an empty frontier for the given number of qubits.
    pub(crate) fn new(n_qubits: usize) -> Self {
        Self {
            single_paulis: vec![Vec::new(); n_qubits],
            single_sign_bits: Vec::new(),
            pair_paulis: vec![Vec::new(); n_qubits],
            pair_sign_bits: Vec::new(),
            ops: Vec::new(),
            remaining_single_ops: IndexSet::new(),
            remaining_pair_ops: IndexSet::new(),
            cost_buckets: vec![IndexSet::new(); (3 * n_qubits).div_ceil(2)],
            minimum_cost: None,
        }
    }

    /// Builds a frontier from aligned pairs of tableau generators.
    pub(crate) fn from_tableau(tableau: &Tableau) -> Self {
        let n_qubits = tableau.get_n_qubits();
        assert!(n_qubits > 0);
        let mut frontier = Self::new(n_qubits);
        frontier.ops.reserve(n_qubits);
        frontier.pair_sign_bits.reserve(2 * n_qubits);
        frontier.remaining_pair_ops.reserve(n_qubits);
        for bucket in &mut frontier.cost_buckets {
            bucket.reserve(n_qubits);
        }
        for row in &mut frontier.pair_paulis {
            row.reserve(2 * n_qubits);
        }

        for (qubit, (z_words, x_words)) in tableau
            .qubit_slices_z_bits()
            .iter()
            .zip(tableau.qubit_slices_x_bits())
            .enumerate()
        {
            let mut pair_index = 0;
            'word_loop: for (word_index, (z_word, x_word)) in
                z_words.iter().zip(x_words).enumerate()
            {
                for bit in 0..32 {
                    if pair_index == n_qubits {
                        break 'word_loop;
                    }
                    let left_bit = 2 * bit;
                    let right_bit = left_bit + 1;
                    let left = bits_to_u8_pauli(
                        ((z_word >> left_bit) & 1) as u8,
                        ((x_word >> left_bit) & 1) as u8,
                    );
                    let right = bits_to_u8_pauli(
                        ((z_word >> right_bit) & 1) as u8,
                        ((x_word >> right_bit) & 1) as u8,
                    );
                    frontier.pair_paulis[qubit].extend([left, right]);
                    if qubit == 0 {
                        let sign_bits = tableau.get_sign_bits()[word_index];
                        frontier.pair_sign_bits.extend([
                            (sign_bits >> left_bit) & 1 == 1,
                            (sign_bits >> right_bit) & 1 == 1,
                        ]);
                        frontier
                            .ops
                            .push(FrontierOp::TableauPair(FrontierTableauPair {
                                pair_index,
                                state: PairState::new(pair_index, n_qubits),
                            }));
                    }
                    frontier.ops[pair_index]
                        .pair_state_mut()
                        .add_support(pair_type(left, right), qubit);
                    pair_index += 1;
                }
            }
        }
        frontier.finish_build();
        frontier
    }

    /// Rebuilds the frontier for the first visible packed operation set.
    ///
    /// Each operation is inserted into a bucket indexed by its current integer
    /// reduction cost.
    pub(crate) fn rebuild(&mut self, slice: &PackedPGSlice) {
        assert!(self.is_empty(), "frontier still contains active operations");
        assert_eq!(self.single_paulis.len(), self.pair_paulis.len());
        self.single_sign_bits.clear();
        self.pair_sign_bits.clear();
        self.ops.clear();
        self.remaining_single_ops.clear();
        self.remaining_pair_ops.clear();
        for row in &mut self.single_paulis {
            row.clear();
        }
        for row in &mut self.pair_paulis {
            row.clear();
        }
        for bucket in &mut self.cost_buckets {
            bucket.clear();
        }

        let front_len = slice
            .op_sets()
            .next()
            .expect("cannot rebuild from an empty slice")
            .count();
        self.ops.reserve(front_len);
        self.single_sign_bits.reserve(front_len);
        self.pair_sign_bits.reserve(2 * front_len);
        self.remaining_single_ops.reserve(front_len);
        self.remaining_pair_ops.reserve(front_len);
        for row in &mut self.single_paulis {
            row.reserve(front_len);
        }
        for row in &mut self.pair_paulis {
            row.reserve(2 * front_len);
        }
        for bucket in &mut self.cost_buckets {
            bucket.reserve(front_len);
        }

        let n_qubits = self.single_paulis.len();
        for qubit in 0..n_qubits {
            let mut single_index = 0;
            let mut pair_index = 0;
            for (frontier_position, (slice_index, packed_op)) in slice
                .op_sets()
                .next()
                .expect("cannot rebuild from an empty slice")
                .enumerate()
            {
                match packed_op.meta {
                    PackedOpMeta::Rotation(rotation) => {
                        let pauli = slice.pauli(slice_index, 0, qubit);
                        self.single_paulis[qubit].push(pauli);
                        if qubit == 0 {
                            self.single_sign_bits.push(slice.sign_bit(slice_index, 0));
                            self.ops.push(FrontierOp::Single(FrontierSingleOp::new(
                                slice_index,
                                FrontierMetadata::Rotation {
                                    angle: rotation.angle(),
                                },
                                single_index,
                                n_qubits,
                            )));
                        }
                        if pauli != I_U8 {
                            let FrontierOp::Single(op) = &mut self.ops[frontier_position] else {
                                unreachable!()
                            };
                            op.supports.insert(qubit);
                        }
                        single_index += 1;
                    }
                    PackedOpMeta::Measure(measure) => {
                        let pauli = slice.pauli(slice_index, 0, qubit);
                        self.single_paulis[qubit].push(pauli);
                        if qubit == 0 {
                            self.single_sign_bits.push(slice.sign_bit(slice_index, 0));
                            self.ops.push(FrontierOp::Single(FrontierSingleOp::new(
                                slice_index,
                                FrontierMetadata::Measure {
                                    cbit: measure.cbit(),
                                },
                                single_index,
                                n_qubits,
                            )));
                        }
                        if pauli != I_U8 {
                            let FrontierOp::Single(op) = &mut self.ops[frontier_position] else {
                                unreachable!()
                            };
                            op.supports.insert(qubit);
                        }
                        single_index += 1;
                    }
                    PackedOpMeta::Reset(_) => {
                        let left = slice.pauli(slice_index, 0, qubit);
                        let right = slice.pauli(slice_index, 1, qubit);
                        self.pair_paulis[qubit].extend([left, right]);
                        if qubit == 0 {
                            self.pair_sign_bits.extend([
                                slice.sign_bit(slice_index, 0),
                                slice.sign_bit(slice_index, 1),
                            ]);
                            self.ops.push(FrontierOp::Pair(FrontierPairOp {
                                slice_index,
                                state: PairState::new(pair_index, n_qubits),
                            }));
                        }
                        self.ops[frontier_position]
                            .pair_state_mut()
                            .add_support(pair_type(left, right), qubit);
                        pair_index += 1;
                    }
                    PackedOpMeta::ConditionalBox(_) | PackedOpMeta::BlackBox(_) => {
                        panic!("frontier accepts only rotation, measurement, and reset operations")
                    }
                }
            }
        }
        self.finish_build();
    }

    /// Returns true if no active operations remain.
    pub(crate) fn is_empty(&self) -> bool {
        self.remaining_single_ops.is_empty() && self.remaining_pair_ops.is_empty()
    }

    /// Samples reducing gates from the lowest nonzero cost bucket.
    pub(crate) fn sample_gates<R: Rng>(&self, k: usize, rng: &mut R) -> Vec<FrontierReductionGate> {
        let minimum = self
            .minimum_cost
            .expect("frontier has no positive cost operation");
        let bucket = &self.cost_buckets[minimum];
        assert!(!bucket.is_empty());

        if self.remaining_pair_ops.is_empty() {
            let gates_per_op = self.ops[bucket[0].0].reduction_gates();
            let total = bucket.len() * gates_per_op;
            return sample_indices(rng, total, k)
                .into_iter()
                .map(|global_index| {
                    let bucket_index = global_index / gates_per_op;
                    let frontier_index = bucket[bucket_index];
                    let local_index = global_index % gates_per_op;
                    FrontierReductionGate {
                        tqe: self.ops[frontier_index.0].gate(
                            local_index,
                            &self.single_paulis,
                            &self.pair_paulis,
                        ),
                        frontier_index,
                    }
                })
                .collect();
        }

        let mut prefix = Vec::with_capacity(bucket.len());
        let mut total = 0;
        for frontier_index in bucket {
            total += self.ops[frontier_index.0].reduction_gates();
            prefix.push(total);
        }
        sample_indices(rng, total, k)
            .into_iter()
            .map(|global_index| {
                let bucket_index = prefix
                    .binary_search(&(global_index + 1))
                    .unwrap_or_else(|position| position);
                let frontier_index = bucket[bucket_index];
                let local_index = if bucket_index == 0 {
                    global_index
                } else {
                    global_index - prefix[bucket_index - 1]
                };
                FrontierReductionGate {
                    tqe: self.ops[frontier_index.0].gate(
                        local_index,
                        &self.single_paulis,
                        &self.pair_paulis,
                    ),
                    frontier_index,
                }
            })
            .collect()
    }

    /// Returns true if an operation is in the lowest nonzero cost bucket.
    pub(crate) fn is_least_cost(&self, index: FrontierIndex) -> bool {
        self.minimum_cost
            .is_some_and(|cost| self.cost_buckets[cost].contains(&index))
    }

    /// Applies a TQE to every active frontier operation.
    ///
    /// Precomputed transition tables update local Paulis, sign bits and pair
    /// support sets. Affected operations are then moved between cost buckets.
    pub(crate) fn apply_tqe(&mut self, tqe: TQE) {
        let (single0, single1) = get_two_mut(&mut self.single_paulis, tqe.q0, tqe.q1);
        for frontier_index in &self.remaining_single_ops {
            let FrontierOp::Single(op) = &mut self.ops[frontier_index.0] else {
                unreachable!()
            };
            let array_index = op.array_index;
            let (next0, next1, sign_flip, change0, change1) = single_reduction_with_all_stats(
                single0[array_index],
                single1[array_index],
                tqe.gate_type,
            );
            single0[array_index] = next0;
            single1[array_index] = next1;
            self.single_sign_bits[array_index] ^= sign_flip;
            let old_cost = op.tqe_cost;
            op.update(tqe.q0, tqe.q1, change0, change1);
            Self::move_bucket(
                &mut self.cost_buckets,
                *frontier_index,
                old_cost,
                op.tqe_cost,
            );
        }

        let (pair0, pair1) = get_two_mut(&mut self.pair_paulis, tqe.q0, tqe.q1);
        for frontier_index in &self.remaining_pair_ops {
            let state = self.ops[frontier_index.0].pair_state_mut();
            let array_index = state.array_index;
            let left0 = pair0[2 * array_index];
            let right0 = pair0[2 * array_index + 1];
            let left1 = pair1[2 * array_index];
            let right1 = pair1[2 * array_index + 1];
            let reduction = pair_reduction(left0, right0, left1, right1, tqe.gate_type);
            pair0[2 * array_index] = reduction.q0[0];
            pair0[2 * array_index + 1] = reduction.q0[1];
            pair1[2 * array_index] = reduction.q1[0];
            pair1[2 * array_index + 1] = reduction.q1[1];
            self.pair_sign_bits[2 * array_index] ^= reduction.sign_flips[0];
            self.pair_sign_bits[2 * array_index + 1] ^= reduction.sign_flips[1];
            let old_cost = state.tqe_cost;
            state.update(tqe.q0, tqe.q1, reduction.class_deltas);
            Self::move_bucket(
                &mut self.cost_buckets,
                *frontier_index,
                old_cost,
                state.tqe_cost,
            );
        }
        self.minimum_cost = self.next_minimum_cost();
    }

    /// Removes and lowers all ordinary operations with zero cost.
    pub(crate) fn pop_ops(&mut self) -> Vec<FrontierEmission> {
        let zero_indices: Vec<_> = self.cost_buckets[0].iter().copied().collect();
        let mut emissions = Vec::with_capacity(zero_indices.len());
        for frontier_index in zero_indices {
            let op = &self.ops[frontier_index.0];
            let array_index = op.array_index();
            let (slice_index, ops) = match op {
                FrontierOp::Single(single) => {
                    let qubit = single.supports[0];
                    let sign_bit = self.single_sign_bits[array_index];
                    let pauli = self.single_paulis[qubit][array_index];
                    self.remaining_single_ops.swap_remove(&frontier_index);
                    let ops = match single.metadata {
                        FrontierMetadata::Rotation { angle } => {
                            let angle = if sign_bit { -angle } else { angle };
                            let gate_type = match pauli {
                                X_U8 => GateType::RX,
                                Y_U8 => GateType::RY,
                                Z_U8 => GateType::RZ,
                                _ => unreachable!("zero cost rotation must act on one qubit"),
                            };
                            vec![Op::Gate {
                                data: GateData::new(gate_type, vec![qubit])
                                    .with_params(vec![angle]),
                            }]
                        }
                        FrontierMetadata::Measure { cbit } => {
                            lower_measurement(qubit, cbit, sign_bit, pauli)
                        }
                    };
                    (single.slice_index, ops)
                }
                FrontierOp::Pair(pair) => {
                    let qubit = pair.state.anticommuting_supports[0];
                    let left = self.pair_paulis[qubit][2 * array_index];
                    let right = self.pair_paulis[qubit][2 * array_index + 1];
                    let left_sign_bit = self.pair_sign_bits[2 * array_index];
                    let right_sign_bit = self.pair_sign_bits[2 * array_index + 1];
                    self.remaining_pair_ops.swap_remove(&frontier_index);
                    (
                        pair.slice_index,
                        lower_reset(qubit, left, right, left_sign_bit, right_sign_bit),
                    )
                }
                FrontierOp::TableauPair(_) => {
                    panic!("tableau pairs must be popped with pop_tableau_pairs")
                }
            };
            emissions.push(FrontierEmission { slice_index, ops });
        }
        self.cost_buckets[0].clear();
        emissions
    }

    /// Removes and returns all tableau generator pairs with zero cost.
    pub(crate) fn pop_tableau_pairs(&mut self) -> Vec<ReducedTableauPair> {
        let zero_indices: Vec<_> = self.cost_buckets[0].iter().copied().collect();
        let mut reduced = Vec::with_capacity(zero_indices.len());
        for frontier_index in zero_indices {
            let FrontierOp::TableauPair(op) = &self.ops[frontier_index.0] else {
                panic!("ordinary operations must be popped with pop_ops")
            };
            assert_eq!(op.state.anticommuting_supports.len(), 1);
            assert!(op.state.left_supports.is_empty());
            assert!(op.state.right_supports.is_empty());
            assert!(op.state.equal_supports.is_empty());
            let qubit = op.state.anticommuting_supports[0];
            let column = 2 * op.state.array_index;
            reduced.push(ReducedTableauPair {
                pair_index: op.pair_index,
                qubit,
                left: self.pair_paulis[qubit][column],
                right: self.pair_paulis[qubit][column + 1],
                left_sign_bit: self.pair_sign_bits[column],
                right_sign_bit: self.pair_sign_bits[column + 1],
            });
            self.remaining_pair_ops.swap_remove(&frontier_index);
        }
        self.cost_buckets[0].clear();
        reduced
    }

    /// Computes the initial costs, fills the cost buckets and records the
    /// lowest nonzero cost after building the frontier.
    fn finish_build(&mut self) {
        self.minimum_cost = None;
        for (index, op) in self.ops.iter_mut().enumerate() {
            op.recompute();
            let frontier_index = FrontierIndex(index);
            let cost = op.cost();
            self.cost_buckets[cost].insert(frontier_index);
            if cost > 0 && cost < self.minimum_cost.unwrap_or(usize::MAX) {
                self.minimum_cost = Some(cost);
            }
            match op {
                FrontierOp::Single(_) => {
                    self.remaining_single_ops.insert(frontier_index);
                }
                FrontierOp::Pair(_) | FrontierOp::TableauPair(_) => {
                    self.remaining_pair_ops.insert(frontier_index);
                }
            }
        }
    }

    fn move_bucket(
        buckets: &mut [IndexSet<FrontierIndex>],
        index: FrontierIndex,
        old_cost: usize,
        new_cost: usize,
    ) {
        if old_cost != new_cost {
            buckets[old_cost].swap_remove(&index);
            buckets[new_cost].insert(index);
        }
    }

    fn next_minimum_cost(&self) -> Option<usize> {
        self.cost_buckets
            .iter()
            .enumerate()
            .skip(1)
            .find_map(|(cost, bucket)| (!bucket.is_empty()).then_some(cost))
    }
}

fn sample_indices<R: Rng>(rng: &mut R, total: usize, k: usize) -> Vec<usize> {
    if k < total {
        sample(rng, total, k).into_iter().collect()
    } else {
        (0..total).collect()
    }
}

fn gate(gate_type: GateType, qubit: usize) -> Op {
    Op::Gate {
        data: GateData::new(gate_type, vec![qubit]),
    }
}

fn lower_measurement(qubit: usize, cbit: usize, sign_bit: bool, pauli: PauliU8) -> Vec<Op> {
    let measurement = Op::Gate {
        data: GateData::new(GateType::Measure, vec![qubit, cbit]),
    };
    match (sign_bit, pauli) {
        (false, Z_U8) => vec![measurement],
        (true, Z_U8) => vec![
            gate(GateType::X, qubit),
            measurement,
            gate(GateType::X, qubit),
        ],
        (false, X_U8) => vec![
            gate(GateType::H, qubit),
            measurement,
            gate(GateType::H, qubit),
        ],
        (true, X_U8) => vec![
            gate(GateType::Z, qubit),
            gate(GateType::H, qubit),
            measurement,
            gate(GateType::H, qubit),
            gate(GateType::Z, qubit),
        ],
        (false, Y_U8) => vec![
            gate(GateType::V, qubit),
            measurement,
            gate(GateType::Vdg, qubit),
        ],
        (true, Y_U8) => vec![
            gate(GateType::Z, qubit),
            gate(GateType::V, qubit),
            measurement,
            gate(GateType::Vdg, qubit),
            gate(GateType::Z, qubit),
        ],
        _ => unreachable!("zero cost measurement must act on one qubit"),
    }
}

fn lower_reset(
    qubit: usize,
    left: PauliU8,
    right: PauliU8,
    left_sign_bit: bool,
    right_sign_bit: bool,
) -> Vec<Op> {
    let mut ops = Vec::new();
    append_reset_sign_correction(&mut ops, qubit, left, right, left_sign_bit, right_sign_bit);
    match (left, right) {
        (X_U8, Y_U8) => ops.extend([
            gate(GateType::V, qubit),
            gate(GateType::H, qubit),
            gate(GateType::Reset, qubit),
            gate(GateType::H, qubit),
            gate(GateType::Vdg, qubit),
        ]),
        (X_U8, Z_U8) => ops.extend([
            gate(GateType::H, qubit),
            gate(GateType::Reset, qubit),
            gate(GateType::H, qubit),
        ]),
        (Y_U8, X_U8) => ops.extend([
            gate(GateType::V, qubit),
            gate(GateType::Reset, qubit),
            gate(GateType::Vdg, qubit),
        ]),
        (Y_U8, Z_U8) => ops.extend([
            gate(GateType::Sdg, qubit),
            gate(GateType::H, qubit),
            gate(GateType::Reset, qubit),
            gate(GateType::H, qubit),
            gate(GateType::S, qubit),
        ]),
        (Z_U8, X_U8) => ops.push(gate(GateType::Reset, qubit)),
        (Z_U8, Y_U8) => ops.extend([
            gate(GateType::Sdg, qubit),
            gate(GateType::Reset, qubit),
            gate(GateType::S, qubit),
        ]),
        _ => unreachable!("zero cost reset must act on one qubit"),
    }
    append_reset_sign_correction(&mut ops, qubit, left, right, left_sign_bit, right_sign_bit);
    ops
}

fn pauli_to_gate(pauli: PauliU8) -> GateType {
    match pauli {
        X_U8 => GateType::X,
        Y_U8 => GateType::Y,
        Z_U8 => GateType::Z,
        _ => unreachable!("unsupported Pauli"),
    }
}

fn append_reset_sign_correction(
    ops: &mut Vec<Op>,
    qubit: usize,
    left: PauliU8,
    right: PauliU8,
    left_sign_bit: bool,
    right_sign_bit: bool,
) {
    let gate_type = match (left_sign_bit, right_sign_bit) {
        (false, false) => return,
        (true, true) => match (left, right) {
            (X_U8, Z_U8) => GateType::Y,
            (Z_U8, X_U8) => GateType::Y,
            (X_U8, Y_U8) => GateType::Z,
            (Y_U8, X_U8) => GateType::Z,
            (Y_U8, Z_U8) => GateType::X,
            (Z_U8, Y_U8) => GateType::X,
            _ => unreachable!("unsupported sign correction"),
        },
        (true, false) => pauli_to_gate(right),
        (false, true) => pauli_to_gate(left),
    };
    ops.push(gate(gate_type, qubit));
}

fn update_support(supports: &mut IndexSet<usize>, qubit: usize, change: i8) {
    match change {
        1 => {
            supports.insert(qubit);
        }
        -1 => {
            supports.swap_remove(&qubit);
        }
        0 => {}
        _ => unreachable!("invalid support delta"),
    }
}

fn index_to_pair(index: usize, n: usize) -> (usize, usize) {
    // Invert the triangular prefix counts for lexicographically enumerated
    // pairs (0, 1), (0, 2), ... (n - 2, n - 1). This avoids allocating all
    // `n choose 2` pairs merely to sample one flat index.
    let n = n as f64;
    let index_float = index as f64;
    let first = (((2.0 * n - 1.0) - ((2.0 * n - 1.0).powi(2) - 8.0 * index_float).sqrt()) / 2.0)
        .floor() as usize;
    let base = first * n as usize - first * (first + 1) / 2;
    (first, first + 1 + index - base)
}

#[cfg(test)]
mod tests;
