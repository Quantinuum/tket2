use serde::{Deserialize, Serialize};

use crate::errors::PauliGraphError;
use crate::{ConditionalBoxData, GateType, Op, gate_type_n_args, gate_type_n_params};

/// A single-qubit Pauli operator.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Pauli {
    /// The $X$ Pauli operator.
    X = 0,
    /// The $Y$ Pauli operator.
    Y = 1,
    /// The $Z$ Pauli operator.
    Z = 2,
    /// The identity operator.
    I = 3,
}

/// A pg-core program consisting of a qubit count and an ordered list of operations.
///
/// The following structural validity checks are performed at `Op` insertion time:
/// - All `Op::Gate` operations have valid parameter counts for their gate type.
/// - All `Op::Gate` operations have valid arguments for their gate type, within the valid range.
/// - All Pauli graph native operations have Pauli strings with length matching the graph's qubit count.
/// - Conditional bits/values lengths match.
/// - `Op::BlackBox` operations have all argument qubits within the valid range.
/// - `GateType::BlackBox` gates have no conditions, and they must have data.
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(try_from = "PauliGraphRaw")]
pub struct PauliGraph {
    n_qubits: usize,
    ops: Vec<Op>,
}

/// Internal struct used for deserialization before validation.
#[derive(Deserialize)]
struct PauliGraphRaw {
    n_qubits: usize,
    ops: Vec<Op>,
}

impl TryFrom<PauliGraphRaw> for PauliGraph {
    type Error = String;
    fn try_from(raw: PauliGraphRaw) -> Result<Self, Self::Error> {
        let pg = PauliGraph {
            n_qubits: raw.n_qubits,
            ops: raw.ops,
        };
        pg.try_validate().map_err(|e| e.to_string())?;
        Ok(pg)
    }
}

impl PauliGraph {
    /// Creates a new `PauliGraph` with the specified number of qubits and an empty `Op` list.
    ///
    /// # Arguments
    ///
    /// - `n_qubits` (`usize`) - The number of qubits in the graph.
    ///
    /// # Returns
    ///
    /// - `Self` - A new `PauliGraph` instance with the specified number of qubits and an empty `Op` list.
    ///
    pub fn new(n_qubits: usize) -> Self {
        Self {
            n_qubits,
            ops: vec![],
        }
    }

    /// Returns the graph with its operation list replaced by `ops`.
    ///
    /// # Arguments
    ///
    /// - `mut self` (`Self`) - The current graph instance.
    /// - `ops` (`Vec<Op>`) - The new list of `Op`s to replace the current list.
    ///
    /// # Returns
    ///
    /// - `Self` - The updated graph instance with the new `Op` list.
    ///
    /// # Panics
    ///
    /// Panics if any operation in `ops` fails the structural validity checks.
    pub fn with_ops(mut self, ops: Vec<Op>) -> Self {
        for op in &ops {
            validate_op(op, self.get_n_qubits());
        }
        self.ops = ops;
        self
    }

    /// Appends a single `op` to the graph.
    ///
    /// # Arguments
    ///
    /// - `&mut self` (`Self`) - The current graph instance.
    /// - `op` (`Op`) - The `Op` to append.
    ///
    /// # Panics
    ///
    /// Panics if `op` fails the structural validity checks.
    pub fn add_op(&mut self, op: Op) {
        validate_op(&op, self.get_n_qubits());
        self.ops.push(op);
    }

    /// Inserts `op` at position `index` in the `Op` list.
    ///
    /// # Arguments
    ///
    /// - `&mut self` (`Self`) - The current graph instance.
    /// - `index` (`usize`) - The position at which to insert the `Op`.
    /// - `op` (`Op`) - The `Op` to insert.
    ///
    /// # Panics
    ///
    /// Panics if `op` fails the structural validity checks.
    pub fn insert_op(&mut self, index: usize, op: Op) {
        validate_op(&op, self.get_n_qubits());
        self.ops.insert(index, op);
    }

    /// Appends all operations from `other` onto this graph.
    ///
    /// # Arguments
    ///
    /// - `&mut self` (`Self`) - The current graph instance.
    /// - `mut other` (`PauliGraph`) - The graph whose operations are to be appended.
    ///
    /// # Panics
    ///
    /// Panics if the number of qubits in `self` and `other` do not match.
    pub fn extend(&mut self, mut other: PauliGraph) {
        if self.n_qubits != other.n_qubits {
            panic!(
                "PauliGraph n_qubits must match: self has {}, other has {}",
                self.n_qubits, other.n_qubits
            );
        }
        self.ops.append(&mut other.ops);
    }

    /// Appends conditional `op`.
    ///
    /// If the last operation is a
    /// [`Op::ConditionalBox`] with matching conditions, `op` is appended into it instead of
    /// creating a new box. If `conditional_bits` is empty, `op` is added unconditionally
    /// via [`Self::add_op`].
    ///
    /// # Panics
    ///
    /// Panics if `op` fails the structural validity checks, or if `conditional_bits` and
    /// `conditional_values` have different lengths.
    pub fn add_conditional_op(
        &mut self,
        op: Op,
        conditional_bits: Vec<usize>,
        conditional_values: Vec<bool>,
    ) {
        if conditional_bits.len() != conditional_values.len() {
            panic!("Length of conditional bits and values must match");
        }
        if conditional_bits.is_empty() {
            self.add_op(op);
        } else {
            let n_qubits = self.get_n_qubits();
            if let Some(last_op) = self.ops.last_mut()
                && let Op::ConditionalBox { data } = last_op
                && conditional_box_matches(data, &conditional_bits, &conditional_values)
            {
                validate_op(&op, n_qubits);
                data.ops.push(op);
                return;
            }
            self.add_op(Op::ConditionalBox {
                data: ConditionalBoxData::new(vec![op], conditional_bits, conditional_values),
            });
        }
    }

    /// Inserts conditional `op` at position `index`.
    ///
    /// If the adjacent operation at `index - 1` or `index` is a [`Op::ConditionalBox`] with matching conditions, `op` is merged into
    /// it instead of creating a new box. If `conditional_bits` is empty, `op` is inserted
    /// unconditionally via [`Self::insert_op`].
    ///
    /// # Panics
    ///
    /// Panics if `op` fails the structural validity checks, or if `conditional_bits` and
    /// `conditional_values` have different lengths.
    pub fn insert_conditional_op(
        &mut self,
        index: usize,
        op: Op,
        conditional_bits: Vec<usize>,
        conditional_values: Vec<bool>,
    ) {
        if conditional_bits.len() != conditional_values.len() {
            panic!("Length of conditional bits and values must match");
        }
        if conditional_bits.is_empty() {
            self.insert_op(index, op);
            return;
        }

        let n_qubits = self.get_n_qubits();
        if index > 0
            && let Some(Op::ConditionalBox { data }) = self.ops.get_mut(index - 1)
            && conditional_box_matches(data, &conditional_bits, &conditional_values)
        {
            validate_op(&op, n_qubits);
            data.ops.push(op);
            return;
        }

        if let Some(Op::ConditionalBox { data }) = self.ops.get_mut(index)
            && conditional_box_matches(data, &conditional_bits, &conditional_values)
        {
            validate_op(&op, n_qubits);
            data.ops.insert(0, op);
            return;
        }

        self.insert_op(
            index,
            Op::ConditionalBox {
                data: ConditionalBoxData::new(vec![op], conditional_bits, conditional_values),
            },
        );
    }

    /// Removes the operation at position `index`.
    ///
    /// # Panics
    ///
    /// Panics if `index` is out of bounds.
    pub fn remove_op(&mut self, index: usize) {
        self.ops.remove(index);
    }

    /// Swaps the operations at positions `index1` and `index2`.
    ///
    /// # Panics
    ///
    /// Panics if `index1` or `index2` is out of bounds.
    pub fn swap_ops(&mut self, index1: usize, index2: usize) {
        self.ops.swap(index1, index2);
    }

    /// Replaces the operations in the index range `range` with `new_ops`.
    ///
    /// # Panics
    ///
    /// Panics if any operation in `new_ops` fails the structural validity checks or if `range` is out of bounds.
    pub fn replace_slice(&mut self, range: std::ops::Range<usize>, new_ops: Vec<Op>) {
        for op in &new_ops {
            validate_op(op, self.get_n_qubits());
        }
        self.ops.splice(range, new_ops);
    }

    /// Increases the qubit count by one, extending all Pauli strings with an identity on the new qubit.
    pub fn add_qubit(&mut self) {
        let old_n_qubits = self.n_qubits;
        self.n_qubits += 1;
        for op in &mut self.ops {
            add_qubit_to_op(op, old_n_qubits);
        }
    }

    /// Returns the ordered operations in the graph.
    pub fn get_ops(&self) -> &Vec<Op> {
        &self.ops
    }

    /// Returns the number of qubits.
    pub fn get_n_qubits(&self) -> usize {
        self.n_qubits
    }

    /// Validates all operations against the structural validity checks.
    ///
    /// # Panics
    ///
    /// Panics if any operation fails the structural validity checks.
    pub fn try_validate(&self) -> Result<(), PauliGraphError> {
        for op in self.get_ops() {
            try_validate_op(op, self.get_n_qubits())?;
        }
        Ok(())
    }
}

fn try_validate_op(op: &Op, pg_nqubits: usize) -> Result<(), PauliGraphError> {
    match op {
        Op::Rotation { data } => validate_pauli_string_len(data.get_string().len(), pg_nqubits, op),
        Op::Measure { data } => validate_pauli_string_len(data.get_string().len(), pg_nqubits, op),
        Op::Reset { data } => {
            validate_pauli_string_len(data.get_first_string().len(), pg_nqubits, op)?;
            validate_pauli_string_len(data.get_second_string().len(), pg_nqubits, op)
        }
        Op::Tableau { data } => {
            for (string, _) in data.get_z_outputs().iter().chain(data.get_x_outputs()) {
                validate_pauli_string_len(string.len(), pg_nqubits, op)?;
            }
            Ok(())
        }
        Op::Gate { data } => {
            if let Some(expected_arg_counts) = gate_type_n_args(data.get_gate_type())
                && data.get_args().len() != expected_arg_counts
            {
                return Err(PauliGraphError::InvalidOp {
                    op: op.clone(),
                    message: format!(
                        "Gate has wrong number of arguments.\nExpected: {}\nGot: {}",
                        expected_arg_counts,
                        data.get_args().len()
                    ),
                });
            }
            let expected_param_counts = gate_type_n_params(data.get_gate_type());
            if data.get_params().len() != expected_param_counts {
                return Err(PauliGraphError::InvalidOp {
                    op: op.clone(),
                    message: format!(
                        "Gate has wrong number of parameters.\nExpected: {}\nGot: {}",
                        expected_param_counts,
                        data.get_params().len()
                    ),
                });
            }
            if data.get_gate_type() == &GateType::Measure {
                validate_op_arg_range(
                    *data
                        .get_args()
                        .first()
                        .expect("Measure gate missing qubit argument"),
                    pg_nqubits,
                    op,
                )?;
            } else {
                validate_op_arg_range(
                    *data
                        .get_args()
                        .iter()
                        .max()
                        .expect("Gate missing qubit arguments"),
                    pg_nqubits,
                    op,
                )?;
            }
            let conditional_bits_len = data.get_conditional_bits().len();
            let conditional_values_len = data.get_conditional_values().len();
            if conditional_bits_len != conditional_values_len {
                return Err(PauliGraphError::InvalidOp {
                    op: op.clone(),
                    message: format!(
                        "Gate has mismatched conditional bits and values lengths.\nBits: {}\nValues: {}",
                        conditional_bits_len, conditional_values_len
                    ),
                });
            }
            if data.get_gate_type() == &GateType::BlackBox {
                if data.get_args().is_empty() {
                    return Err(PauliGraphError::InvalidOp {
                        op: op.clone(),
                        message: "BlackBox gate cannot have empty arguments.".into(),
                    });
                }
                if !data.get_conditional_bits().is_empty()
                    || !data.get_conditional_values().is_empty()
                {
                    return Err(PauliGraphError::InvalidOp {
                        op: op.clone(),
                        message: "BlackBox gates cannot have conditions.".into(),
                    });
                }
                if data.get_data().is_none() {
                    return Err(PauliGraphError::InvalidOp {
                        op: op.clone(),
                        message: "BlackBox gate is missing payload data.".into(),
                    });
                }
            }
            Ok(())
        }
        Op::BlackBox { data } => validate_op_arg_range(
            *data
                .get_qubits()
                .iter()
                .max()
                .expect("Gate missing qubit argument"),
            pg_nqubits,
            op,
        ),
        Op::ConditionalBox { data } => {
            let conditional_bits_len = data.get_conditional_bits().len();
            let conditional_values_len = data.get_conditional_values().len();
            if conditional_bits_len != conditional_values_len {
                return Err(PauliGraphError::InvalidOp {
                    op: op.clone(),
                    message: format!(
                        "ConditionalBox has mismatched conditional bits and values lengths.\nBits: {}\nValues: {}",
                        conditional_bits_len, conditional_values_len
                    ),
                });
            }
            if conditional_bits_len == 0 {
                return Err(PauliGraphError::InvalidOp {
                    op: op.clone(),
                    message: "ConditionalBox must have non-empty conditional bits and values."
                        .into(),
                });
            }
            for inner_op in data.get_ops() {
                try_validate_op(inner_op, pg_nqubits)?;
            }
            Ok(())
        }
        Op::SetBoundary => Ok(()),
    }
}

fn validate_op(op: &Op, pg_nqubits: usize) {
    try_validate_op(op, pg_nqubits).unwrap_or_else(|e| panic!("{e}"))
}

fn validate_op_arg_range(maxq: usize, n_qubits: usize, op: &Op) -> Result<(), PauliGraphError> {
    if maxq >= n_qubits {
        Err(PauliGraphError::InvalidOp {
            op: op.clone(),
            message: format!(
                "Operation has argument qubit index out of range.\nExpected: < {}\nGot: {}",
                n_qubits, maxq
            ),
        })?
    }
    Ok(())
}
fn validate_pauli_string_len(len: usize, n_qubits: usize, op: &Op) -> Result<(), PauliGraphError> {
    if len != n_qubits {
        Err(PauliGraphError::InvalidOp {
            op: op.clone(),
            message: format!(
                "String length does not match graph qubit count.\nExpected: {}\nGot: {}",
                n_qubits, len
            ),
        })?
    }
    Ok(())
}

fn conditional_box_matches(
    data: &ConditionalBoxData,
    conditional_bits: &[usize],
    conditional_values: &[bool],
) -> bool {
    data.get_conditional_bits() == conditional_bits
        && data.get_conditional_values() == conditional_values
}

fn append_identity(paulis: &mut Vec<Pauli>) {
    paulis.push(Pauli::I);
}

fn single_qubit_tableau_output(n_qubits: usize, pauli: Pauli) -> Vec<Pauli> {
    let mut output = vec![Pauli::I; n_qubits];
    output.push(pauli);
    output
}

fn add_qubit_to_op(op: &mut Op, old_n_qubits: usize) {
    match op {
        Op::Rotation { data } => {
            append_identity(&mut data.string);
        }
        Op::Measure { data } => {
            append_identity(&mut data.string);
        }
        Op::Reset { data } => {
            append_identity(&mut data.first_string);
            append_identity(&mut data.second_string);
        }
        Op::Tableau { data } => {
            for (string, _) in &mut data.z_outputs {
                append_identity(string);
            }
            for (string, _) in &mut data.x_outputs {
                append_identity(string);
            }
            data.z_outputs
                .push((single_qubit_tableau_output(old_n_qubits, Pauli::Z), true));
            data.x_outputs
                .push((single_qubit_tableau_output(old_n_qubits, Pauli::X), true));
        }
        Op::ConditionalBox { data } => {
            for inner_op in &mut data.ops {
                add_qubit_to_op(inner_op, old_n_qubits);
            }
        }
        Op::Gate { .. } | Op::BlackBox { .. } | Op::SetBoundary => {}
    }
}
