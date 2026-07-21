use std::vec;

use crate::tableau_trait::PGTableau;
use pg_core::{ConditionalBoxData, GateData, GateType, Op, Pauli, PauliGraph, RotationData};
use pg_utils::cliff_angle;

// =============================================================================
//                                 Op Queries
// =============================================================================

/// Check if an operation can be conjugated by a tableau
/// i.e. if it is a rotation, measure, reset, conditional box
/// or a rotation/measure/reset gate (with any conditionality).
///
/// # Arguments
///
/// - `op` (`&Op`) - The Op to check.
///
/// # Returns
///
/// - `bool` - `true` if the operation can be conjugated by a tableau, `false` otherwise.
///
pub fn can_be_conjugated(op: &Op) -> bool {
    match op {
        Op::Rotation { .. } | Op::Measure { .. } | Op::Reset { .. } | Op::ConditionalBox { .. } => {
            true
        }
        Op::Gate { data } => {
            is_rotation_gate_type(data)
                || (*data.get_gate_type() == GateType::Measure)
                || (*data.get_gate_type() == GateType::Reset)
        }
        _ => false,
    }
}

/// Return the dagger of a Clifford gate.
fn clifford_dagger(gate: &GateData) -> GateData {
    let gate_type = match gate.get_gate_type() {
        GateType::H => GateType::H,
        GateType::S => GateType::Sdg,
        GateType::V => GateType::Vdg,
        GateType::Sdg => GateType::S,
        GateType::Vdg => GateType::V,
        GateType::X
        | GateType::Y
        | GateType::Z
        | GateType::XX
        | GateType::XY
        | GateType::XZ
        | GateType::YX
        | GateType::YY
        | GateType::YZ
        | GateType::ZX
        | GateType::ZY
        | GateType::ZZ
        | GateType::SWAP => gate.get_gate_type().clone(),
        gate_type => panic!("Unsupported gate type for Clifford dagger: {gate_type:?}"),
    };
    GateData::new(gate_type, gate.get_args().clone()).with_conditional(
        gate.get_conditional_bits().clone(),
        gate.get_conditional_values().clone(),
    )
}

/// Get the dagger (inverse) of an operation.
///
/// # Arguments
///
/// - `op` (`&Op`) - The operation to get the dagger of.
///
/// # Returns
///
/// - `Op` - The dagger (inverse) of the given operation.
///
/// # Panics
///
/// - Panics if the operation is not a (conditional) unitary.
pub fn get_dagger<T: PGTableau>(op: &Op) -> Op {
    match op {
        Op::Rotation { data } => Op::Rotation {
            data: RotationData::new(data.get_string().clone(), -data.get_angle()),
        },
        Op::Gate { data } => {
            match data.get_gate_type() {
                GateType::RX | GateType::RY | GateType::RZ | GateType::ZZPHASE => Op::Gate {
                    data: data.clone().with_params(vec![-data.get_params()[0]]),
                },
                GateType::PHASEDX => {
                    // PhasedX(a,b).dagger() == PhasedX(-a,b)
                    Op::Gate {
                        data: data
                            .clone()
                            .with_params(vec![-data.get_params()[0], data.get_params()[1]]),
                    }
                }
                GateType::S
                | GateType::V
                | GateType::H
                | GateType::Sdg
                | GateType::Vdg
                | GateType::X
                | GateType::Y
                | GateType::Z
                | GateType::XX
                | GateType::XY
                | GateType::XZ
                | GateType::YX
                | GateType::YY
                | GateType::YZ
                | GateType::ZX
                | GateType::ZY
                | GateType::ZZ
                | GateType::SWAP => Op::Gate {
                    data: clifford_dagger(data),
                },
                _ => panic!("Unsupported gate type for getting dagger"),
            }
        }
        Op::ConditionalBox { data } => {
            let new_ops = data
                .get_ops()
                .iter()
                .rev()
                .map(|op| get_dagger::<T>(op))
                .collect();
            Op::ConditionalBox {
                data: ConditionalBoxData::new(
                    new_ops,
                    data.get_conditional_bits().clone(),
                    data.get_conditional_values().clone(),
                ),
            }
        }
        Op::Tableau { data } => Op::Tableau {
            data: T::from(data.clone()).get_dagger().into(),
        },
        _ => panic!("Unsupported op type for getting dagger"),
    }
}

/// Check if a gate is a Clifford gate type
fn is_clifford_gate_type(gate: &GateData) -> bool {
    matches!(
        gate.get_gate_type(),
        GateType::H
            | GateType::S
            | GateType::V
            | GateType::Sdg
            | GateType::Vdg
            | GateType::X
            | GateType::Y
            | GateType::Z
            | GateType::XX
            | GateType::XY
            | GateType::XZ
            | GateType::YX
            | GateType::YY
            | GateType::YZ
            | GateType::ZX
            | GateType::ZY
            | GateType::ZZ
            | GateType::SWAP,
    )
}

/// Check if an operation is a Clifford operation
/// i.e. if it is a Clifford unitary.
///
/// # Arguments
///
/// - `op` (`&Op`) - The operation to check.
///
/// # Returns
///
/// - `bool` - `true` if the operation is a Clifford operation, `false` otherwise.
pub fn is_clifford(op: &Op) -> bool {
    match op {
        Op::Gate { data } => {
            data.get_conditional_bits().is_empty()
                && (is_clifford_gate_type(data)
                    || (is_rotation_gate_type(data)
                        && data.get_params().iter().all(|&p| cliff_angle(p).is_some())))
        }
        Op::Rotation { data } => cliff_angle(data.get_angle()).is_some(),
        Op::Tableau { .. } => true,
        _ => false,
    }
}

/// Check if a gate is a rotation gate type
fn is_rotation_gate_type(gate: &GateData) -> bool {
    matches!(
        gate.get_gate_type(),
        GateType::RX | GateType::RY | GateType::RZ | GateType::ZZPHASE | GateType::PHASEDX
    )
}

/// Check if two Pauli strings commute
fn strings_commute(s0: &[Pauli], s1: &[Pauli]) -> bool {
    let diff_count = s0
        .iter()
        .zip(s1.iter())
        .filter(|(p0, p1)| **p0 != Pauli::I && **p1 != Pauli::I && **p0 != **p1)
        .count();
    diff_count % 2 == 0
}

/// Create a Pauli string of weight 1 with the given Pauli at the given index
fn get_weight_1_string(n_qubits: usize, i: usize, pauli: Pauli) -> Vec<Pauli> {
    let mut s = vec![Pauli::I; n_qubits];
    s[i] = pauli;
    s
}

/// Get the Pauli strings for an operation, panics if the operation does not have a
/// associated Pauli string.
fn get_pauli_strings(op: &Op, n_qubits: usize) -> Vec<Vec<Pauli>> {
    match op {
        Op::Rotation { data } => vec![data.get_string().clone()],
        Op::Measure { data } => vec![data.get_string().clone()],
        Op::Reset { data } => vec![
            data.get_first_string().clone(),
            data.get_second_string().clone(),
        ],
        Op::Gate { data } => match data.get_gate_type() {
            GateType::RX => {
                vec![get_weight_1_string(n_qubits, data.get_args()[0], Pauli::X)]
            }
            GateType::RY => {
                vec![get_weight_1_string(n_qubits, data.get_args()[0], Pauli::Y)]
            }
            GateType::RZ => {
                vec![get_weight_1_string(n_qubits, data.get_args()[0], Pauli::Z)]
            }
            GateType::ZZPHASE => {
                let mut s = vec![Pauli::I; n_qubits];
                s[data.get_args()[0]] = Pauli::Z;
                s[data.get_args()[1]] = Pauli::Z;
                vec![s]
            }
            GateType::PHASEDX => {
                vec![
                    get_weight_1_string(n_qubits, data.get_args()[0], Pauli::Z),
                    get_weight_1_string(n_qubits, data.get_args()[0], Pauli::X),
                    get_weight_1_string(n_qubits, data.get_args()[0], Pauli::Z),
                ]
            }
            GateType::Measure => {
                vec![get_weight_1_string(n_qubits, data.get_args()[0], Pauli::Z)]
            }
            GateType::Reset => {
                vec![
                    get_weight_1_string(n_qubits, data.get_args()[0], Pauli::Z),
                    get_weight_1_string(n_qubits, data.get_args()[0], Pauli::X),
                ]
            }
            _ => panic!("Unsupported gate type for getting Pauli strings"),
        },
        Op::ConditionalBox { data } => {
            let mut strings = Vec::new();
            for op in data.get_ops() {
                strings.extend(get_pauli_strings(op, n_qubits));
            }
            strings
        }
        _ => panic!("Unsupported gate type for getting Pauli strings"),
    }
}

/// Check if a Pauli-like operation is unchanged by conjugating it with a Clifford operation
/// (optionally using the Clifford's dagger instead).
fn commutes_via_conjugation<T: PGTableau>(
    n_qubits: usize,
    cliff: &Op,
    op: &Op,
    use_dagger: bool,
) -> bool {
    let mut tab = T::eye(n_qubits);
    if use_dagger {
        tab.postcompose_op(&get_dagger::<T>(cliff));
    } else {
        tab.postcompose_op(cliff);
    }
    // we check C(P) == I(P)
    tab.conjugate(op) == T::eye(n_qubits).conjugate(op)
}

// =============================================================================
//                             PauliGraph Rewrites
// =============================================================================

/// Trait for basic PauliGraph rewrites.
pub trait PGRewrite {
    /// Check if a given operation is at the left boundary of the PauliGraph.
    fn is_left_boundary(&self, i: usize) -> bool;
    /// Check if a given operation is at the right boundary of the PauliGraph.
    fn is_right_boundary(&self, i: usize) -> bool;
    /// Check if a given operation is the identity operation.
    fn is_identity<T: PGTableau>(&self, i: usize) -> bool;
    /// Check if two operations commute.
    fn do_commute<T: PGTableau>(&self, i: usize, j: usize) -> bool;
    /// Commute two operations.
    fn commute_ops<T: PGTableau>(&mut self, i: usize, j: usize);
    /// Check if a given operation is a Clifford operation.
    fn is_clifford(&self, i: usize) -> bool;
    /// Check if a given operation can be conjugated by a Clifford operation.
    fn can_be_conjugated(&self, i: usize) -> bool;
    /// Given the index of a Clifford operation, conjugate the operation to its left by it.
    fn conj_left_op<T: PGTableau>(&mut self, i: usize);
    /// Given the index of a Clifford operation, conjugate the operation to its right by it.
    fn conj_right_op<T: PGTableau>(&mut self, i: usize);
    /// Check if two operations can be merged.
    fn can_merge(&self, i: usize, j: usize) -> bool;
    /// Merge two operations.
    fn merge_ops<T: PGTableau>(&mut self, i: usize, j: usize);
    /// Convert an operation to a tableau representation.
    fn op_to_tableau<T: PGTableau>(&mut self, i: usize);
}

impl PGRewrite for PauliGraph {
    fn is_left_boundary(&self, i: usize) -> bool {
        i == 0
    }

    fn is_right_boundary(&self, i: usize) -> bool {
        i == self.get_ops().len() - 1
    }

    fn is_clifford(&self, i: usize) -> bool {
        is_clifford(&self.get_ops()[i])
    }

    /// determine if the op can be conjugated
    fn can_be_conjugated(&self, i: usize) -> bool {
        can_be_conjugated(&self.get_ops()[i])
    }

    fn is_identity<T: PGTableau>(&self, i: usize) -> bool {
        if !self.is_clifford(i) {
            return false;
        }
        // We convert the op to a tableau and check if it is the identity tableau
        let mut tab = T::eye(self.get_n_qubits());
        tab.postcompose_op(&self.get_ops()[i]);
        tab == T::eye(self.get_n_qubits())
    }

    fn op_to_tableau<T: PGTableau>(&mut self, i: usize) {
        if !self.is_clifford(i) {
            panic!("Only Clifford unitary can be converted to tableau");
        }
        let mut tab = T::eye(self.get_n_qubits());
        tab.postcompose_op(&self.get_ops()[i]);
        self.remove_op(i);
        self.insert_op(i, Op::Tableau { data: tab.into() });
    }

    fn conj_left_op<T: PGTableau>(&mut self, i: usize) {
        if !self.is_clifford(i) {
            panic!("Only Clifford unitary can be used for conjugation");
        }
        if self.is_left_boundary(i) {
            panic!("Cannot conjugate left by a Clifford unitary at the left boundary");
        }
        if !self.can_be_conjugated(i - 1) {
            panic!("The operation to be conjugated cannot be conjugated");
        }
        let cliff = &self.get_ops()[i];
        let left_op = &self.get_ops()[i - 1];
        let mut tab = T::eye(self.get_n_qubits());
        tab.postcompose_op(cliff);
        let mut replacement = vec![cliff.clone()];
        replacement.extend(tab.conjugate(left_op));
        self.replace_slice(i - 1..i + 1, replacement);
    }

    fn conj_right_op<T: PGTableau>(&mut self, i: usize) {
        if !self.is_clifford(i) {
            panic!("Only Clifford unitary can be used for conjugation");
        }
        if self.is_right_boundary(i) {
            panic!("Cannot conjugate right by a Clifford unitary at the right boundary");
        }
        if !self.can_be_conjugated(i + 1) {
            panic!("The operation to be conjugated cannot be conjugated");
        }
        let cliff = &self.get_ops()[i];
        let right_op = &self.get_ops()[i + 1];
        let mut tab = T::eye(self.get_n_qubits());
        tab.postcompose_op(&get_dagger::<T>(cliff));
        let mut replacement = tab.conjugate(right_op);
        replacement.push(cliff.clone());
        self.replace_slice(i..i + 2, replacement);
    }

    fn do_commute<T: PGTableau>(&self, i: usize, j: usize) -> bool {
        // we first consider they are both pauli ops
        let op_i = &self.get_ops()[i];
        let op_j = &self.get_ops()[j];
        // if either one is SetBoundary, then they commute
        if matches!(op_i, Op::SetBoundary) || matches!(op_j, Op::SetBoundary) {
            return true;
        }
        if self.can_be_conjugated(i) && self.can_be_conjugated(j) {
            let strings_i = get_pauli_strings(op_i, self.get_n_qubits());
            let strings_j = get_pauli_strings(op_j, self.get_n_qubits());
            return strings_i
                .iter()
                .all(|s0| strings_j.iter().all(|s1| strings_commute(s0, s1)));
        }
        // if one is Clifford and the other is Pauli, check conjugation
        if self.is_clifford(i) && self.can_be_conjugated(j) {
            return commutes_via_conjugation::<T>(self.get_n_qubits(), op_i, op_j, true);
        }
        if self.is_clifford(j) && self.can_be_conjugated(i) {
            return commutes_via_conjugation::<T>(self.get_n_qubits(), op_j, op_i, false);
        }

        // If both are Clifford, we check if two t0*t1 == t1*t0 where t0 and t1 are the tableaux of the two ops
        if self.is_clifford(i) && self.is_clifford(j) {
            let mut t0 = T::eye(self.get_n_qubits());
            t0.postcompose_op(op_i);
            t0.postcompose_op(op_j);
            let mut t1 = T::eye(self.get_n_qubits());
            t1.postcompose_op(op_j);
            t1.postcompose_op(op_i);
            return t0 == t1;
        }
        false
    }

    fn commute_ops<T: PGTableau>(&mut self, i: usize, j: usize) {
        if !self.do_commute::<T>(i, j) {
            panic!("Cannot commute ops that do not commute");
        }
        self.swap_ops(i, j);
    }

    fn can_merge(&self, i: usize, j: usize) -> bool {
        let op_i = &self.get_ops()[i];
        let op_j = &self.get_ops()[j];
        // True if both clifford
        if self.is_clifford(i) && self.is_clifford(j) {
            return true;
        }
        match (&op_i, &op_j) {
            (Op::Rotation { data: data_i }, Op::Rotation { data: data_j }) => {
                data_i.get_string() == data_j.get_string()
            }
            (Op::Gate { data: data_i }, Op::Gate { data: data_j }) => {
                match (data_i.get_gate_type(), data_j.get_gate_type()) {
                    (GateType::RX, GateType::RX)
                    | (GateType::RY, GateType::RY)
                    | (GateType::RZ, GateType::RZ)
                    | (GateType::ZZPHASE, GateType::ZZPHASE) => {
                        data_i.get_args() == data_j.get_args()
                            && data_i.get_conditional_bits() == data_j.get_conditional_bits()
                            && data_i.get_conditional_values() == data_j.get_conditional_values()
                    }
                    _ => false,
                }
            }
            // we allow merging of conditional boxes if they have the same conditionality, regardless of the ops inside
            (Op::ConditionalBox { data: data_i }, Op::ConditionalBox { data: data_j }) => {
                data_i.get_conditional_bits() == data_j.get_conditional_bits()
                    && data_i.get_conditional_values() == data_j.get_conditional_values()
            }
            _ => false,
        }
    }

    fn merge_ops<T: PGTableau>(&mut self, i: usize, j: usize) {
        if !self.can_merge(i, j) {
            panic!("Ops that cannot be merged");
        }
        let op_i = &self.get_ops()[i];
        let op_j = &self.get_ops()[j];
        if self.is_clifford(i) && self.is_clifford(j) {
            let mut tab_i = T::eye(self.get_n_qubits());
            tab_i.postcompose_op(op_i);
            tab_i.postcompose_op(op_j);
            self.replace_slice(i..j + 1, vec![Op::Tableau { data: tab_i.into() }]);
            return;
        }
        match (&op_i, &op_j) {
            (Op::Rotation { data: data_i }, Op::Rotation { data: data_j }) => self.replace_slice(
                i..j + 1,
                vec![Op::Rotation {
                    data: RotationData::new(
                        data_i.get_string().clone(),
                        data_i.get_angle() + data_j.get_angle(),
                    ),
                }],
            ),
            (Op::Gate { data: data_i }, Op::Gate { data: data_j }) => {
                match (data_i.get_gate_type(), data_j.get_gate_type()) {
                    (GateType::RX, GateType::RX)
                    | (GateType::RY, GateType::RY)
                    | (GateType::RZ, GateType::RZ)
                    | (GateType::ZZPHASE, GateType::ZZPHASE) => self.replace_slice(
                        i..j + 1,
                        vec![Op::Gate {
                            data: data_i
                                .clone()
                                .with_params(vec![data_i.get_params()[0] + data_j.get_params()[0]]),
                        }],
                    ),
                    _ => panic!("Unsupported gate type for merging"),
                }
            }
            // we allow merging of conditional boxes if they have the same conditionality, regardless of the ops inside
            (Op::ConditionalBox { data: data_i }, Op::ConditionalBox { data: data_j }) => {
                let mut new_ops = data_i.get_ops().clone();
                new_ops.extend(data_j.get_ops().clone());
                self.replace_slice(
                    i..j + 1,
                    vec![Op::ConditionalBox {
                        data: ConditionalBoxData::new(
                            new_ops,
                            data_i.get_conditional_bits().clone(),
                            data_i.get_conditional_values().clone(),
                        ),
                    }],
                )
            }
            _ => panic!("Unsupported op type for merging"),
        }
    }
}

// Here we only test methods that do not require tableau. Remaining methods will be tested in a separate test crate.
#[cfg(test)]
mod tests {
    use super::*;
    use pg_core::{
        BlackBoxData, ConditionalBoxData, GateData, GateType, MeasureData, Op, Pauli, PauliGraph,
        ResetData, RotationData, TableauData,
    };

    #[test]
    fn test_can_be_conjugated() {
        // rotation
        assert!(can_be_conjugated(&Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 0.3)
        }));
        // measure
        assert!(can_be_conjugated(&Op::Measure {
            data: MeasureData::new(vec![Pauli::Z], true, 0)
        }));
        // reset
        assert!(can_be_conjugated(&Op::Reset {
            data: ResetData::new(vec![Pauli::Z], vec![Pauli::X], true, true)
        }));
        // gate
        assert!(can_be_conjugated(&Op::Gate {
            data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.3])
        }));
        // conditional box
        let inner = Op::Gate {
            data: GateData::new(GateType::H, vec![0]),
        };
        assert!(can_be_conjugated(&Op::ConditionalBox {
            data: ConditionalBoxData::new(vec![inner], vec![0], vec![true])
        }));
        // conditional rotation
        assert!(can_be_conjugated(&Op::Gate {
            data: GateData::new(GateType::RZ, vec![0])
                .with_params(vec![0.3])
                .with_conditional(vec![0], vec![true])
        }));
    }

    #[test]
    fn test_cannot_be_conjugated() {
        // Clifford gate
        assert!(!can_be_conjugated(&Op::Gate {
            data: GateData::new(GateType::H, vec![0])
        }));
        // tableau
        assert!(!can_be_conjugated(&Op::Tableau {
            data: TableauData::new(vec![(vec![Pauli::Z], true)], vec![(vec![Pauli::X], false)])
        }));
        // set boundary
        assert!(!can_be_conjugated(&Op::SetBoundary));
        // black box
        assert!(!can_be_conjugated(&Op::BlackBox {
            data: BlackBoxData::new(vec![0], "test".to_string())
        }));
    }

    #[test]
    fn test_is_clifford() {
        // Clifford gate type
        assert!(is_clifford(&Op::Gate {
            data: GateData::new(GateType::H, vec![0])
        }));
        // Rotation gate with Clifford angle
        assert!(is_clifford(&Op::Gate {
            data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.5])
        }));
        // Clifford rotation
        assert!(is_clifford(&Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 0.5)
        }));
        // Tableau
        assert!(is_clifford(&Op::Tableau {
            data: TableauData::new(vec![(vec![Pauli::Z], true)], vec![(vec![Pauli::X], false)])
        }));
    }

    #[test]
    fn test_is_not_clifford() {
        // Non-Clifford gate type
        assert!(!is_clifford(&Op::Gate {
            data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.3])
        }));
        // Non-Clifford rotation
        assert!(!is_clifford(&Op::Rotation {
            data: RotationData::new(vec![Pauli::Z], 0.3)
        }));
        // Set boundary
        assert!(!is_clifford(&Op::SetBoundary));
        // Black box
        assert!(!is_clifford(&Op::BlackBox {
            data: BlackBoxData::new(vec![0], "test".to_string())
        }));
    }

    #[test]
    fn test_boundary_ops() {
        let pg = PauliGraph::new(1).with_ops(vec![
            Op::Gate {
                data: GateData::new(GateType::H, vec![0]),
            },
            Op::Gate {
                data: GateData::new(GateType::S, vec![0]),
            },
            Op::Gate {
                data: GateData::new(GateType::H, vec![0]),
            },
        ]);
        assert!(pg.is_left_boundary(0));
        assert!(!pg.is_left_boundary(1));
        assert!(!pg.is_right_boundary(0));
        assert!(!pg.is_right_boundary(1));
        assert!(pg.is_right_boundary(2));
    }

    #[test]
    fn can_merge_same_rotation_string() {
        let pg = PauliGraph::new(2).with_ops(vec![
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.3),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.5),
            },
        ]);
        assert!(pg.can_merge(0, 1));
    }

    #[test]
    fn can_merge_diff_rotation_string() {
        let pg = PauliGraph::new(2).with_ops(vec![
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z, Pauli::I], 0.3),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::X, Pauli::I], 0.5),
            },
        ]);
        assert!(!pg.can_merge(0, 1));
    }

    #[test]
    fn can_merge_rz_same_qubit() {
        let pg = PauliGraph::new(2).with_ops(vec![
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.3]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.5]),
            },
        ]);
        assert!(pg.can_merge(0, 1));
    }

    #[test]
    fn can_merge_rz_diff_qubit() {
        let pg = PauliGraph::new(2).with_ops(vec![
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![0]).with_params(vec![0.3]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![1]).with_params(vec![0.5]),
            },
        ]);
        assert!(!pg.can_merge(0, 1));
    }

    #[test]
    fn can_merge_two_cliffords() {
        let pg = PauliGraph::new(1).with_ops(vec![
            Op::Gate {
                data: GateData::new(GateType::H, vec![0]),
            },
            Op::Gate {
                data: GateData::new(GateType::S, vec![0]),
            },
        ]);
        assert!(pg.can_merge(0, 1));
    }

    #[test]
    fn can_merge_clifford_and_rotation_returns_false() {
        let pg = PauliGraph::new(1).with_ops(vec![
            Op::Gate {
                data: GateData::new(GateType::H, vec![0]),
            },
            Op::Rotation {
                data: RotationData::new(vec![Pauli::Z], 0.3),
            },
        ]);
        assert!(!pg.can_merge(0, 1));
    }
}
