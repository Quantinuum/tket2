use serde::{Deserialize, Serialize};

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
/// - All Op::Gate operations have valid parameter counts for their gate type.
/// - All Op::Gate operations have valid arguments for their gate type, and the arguments are within the valid range.
/// - All Pauli graph native operations have Pauli strings with length matching the graph's qubit count.
/// - Conditional bits/values lengths match
/// - Op::BlackBox operations have all argument qubits within the valid range
/// - GateType::BlackBox gates have no conditions and have an associated payload
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(from = "PauliGraphRaw")]
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

impl From<PauliGraphRaw> for PauliGraph {
    fn from(raw: PauliGraphRaw) -> Self {
        let pg = PauliGraph {
            n_qubits: raw.n_qubits,
            ops: raw.ops,
        };
        pg.validate();
        pg
    }
}

impl PauliGraph {
    /// Creates an empty Pauli graph for `n_qubits` qubits.
    pub fn new(n_qubits: usize) -> Self {
        Self {
            n_qubits,
            ops: vec![],
        }
    }

    /// Replaces the operation list and returns the updated graph.
    pub fn with_ops(mut self, ops: Vec<Op>) -> Self {
        for op in &ops {
            validate_op(op, self.get_n_qubits());
        }
        self.ops = ops;
        self
    }

    /// Appends a single operation to the graph.
    pub fn add_op(&mut self, op: Op) {
        validate_op(&op, self.get_n_qubits());
        self.ops.push(op);
    }

    /// Inserts a single operation at the provided index.
    pub fn insert_op(&mut self, index: usize, op: Op) {
        validate_op(&op, self.get_n_qubits());
        self.ops.insert(index, op);
    }

    /// Appends all operations from another graph with the same qubit count.
    pub fn extend(&mut self, mut other: PauliGraph) {
        debug_assert_eq!(
            self.n_qubits, other.n_qubits,
            "PauliGraph n_qubits must match"
        );
        self.ops.append(&mut other.ops);
    }

    /// Adds an operation with classical conditions, merging with the last matching box when possible.
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

    /// Inserts an operation with classical conditions, merging with an adjacent matching box when possible.
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

    /// Removes the operation at the provided index.
    pub fn remove_op(&mut self, index: usize) {
        self.ops.remove(index);
    }

    /// Swaps the operations at the provided indices.
    pub fn swap_ops(&mut self, index1: usize, index2: usize) {
        self.ops.swap(index1, index2);
    }

    pub fn replace_slice(&mut self, range: std::ops::Range<usize>, new_ops: Vec<Op>) {
        for op in &new_ops {
            validate_op(op, self.get_n_qubits());
        }
        self.ops.splice(range, new_ops);
    }

    /// Extends the graph by one qubit and pads Pauli-string-based operations accordingly.
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

    pub fn validate(&self) {
        for op in self.get_ops() {
            validate_op(op, self.get_n_qubits());
        }
    }
}

fn validate_op(op: &Op, pg_nqubits: usize) {
    match op {
        Op::Rotation { data } => {
            validate_pauli_string_len(data.get_string().len(), pg_nqubits, op);
        }
        Op::Measure { data } => {
            validate_pauli_string_len(data.get_string().len(), pg_nqubits, op);
        }
        Op::Reset { data } => {
            validate_pauli_string_len(data.get_first_string().len(), pg_nqubits, op);
            validate_pauli_string_len(data.get_second_string().len(), pg_nqubits, op);
        }
        Op::Tableau { data } => {
            for (string, _) in data.get_z_outputs().iter().chain(data.get_x_outputs()) {
                validate_pauli_string_len(string.len(), pg_nqubits, op);
            }
        }
        Op::Gate { data } => {
            if let Some(expected_arg_counts) = gate_type_n_args(data.get_gate_type())
                && data.get_args().len() != expected_arg_counts
            {
                panic!(
                    "Gate has wrong number of arguments.\nExpected: {}\nGot: {}\nOp: {:?}",
                    expected_arg_counts,
                    data.get_args().len(),
                    op
                );
            }
            if let Some(expected_param_counts) = gate_type_n_params(data.get_gate_type())
                && data.get_params().len() != expected_param_counts
            {
                panic!(
                    "Gate has wrong number of parameters.\nExpected: {}\nGot: {}\nOp: {:?}",
                    expected_param_counts,
                    data.get_params().len(),
                    op
                );
            }
            if data.get_gate_type() == &GateType::Measure {
                validate_op_arg_range(
                    *data
                        .get_args()
                        .first()
                        .expect("Measure gate missing qubit argument"),
                    pg_nqubits,
                    op,
                );
            } else {
                validate_op_arg_range(
                    *data
                        .get_args()
                        .iter()
                        .max()
                        .expect("Gate missing qubit argument"),
                    pg_nqubits,
                    op,
                );
            }
            let conditional_bits_len = data.get_conditional_bits().len();
            let conditional_values_len = data.get_conditional_values().len();
            if conditional_bits_len != conditional_values_len {
                panic!(
                    "Gate has mismatched conditional bits and values lengths.\nBits: {}\nValues: {}\nOp: {:?}",
                    conditional_bits_len, conditional_values_len, op
                );
            }
            if data.get_gate_type() == &GateType::BlackBox {
                if !data.get_conditional_bits().is_empty()
                    || !data.get_conditional_values().is_empty()
                {
                    panic!("BlackBox gates cannot have conditions.\nOp: {:?}", op);
                }
                if data.get_data().is_none() {
                    panic!("BlackBox gate is missing payload data.\nOp: {:?}", op);
                }
            }
        }
        Op::BlackBox { data } => {
            validate_op_arg_range(
                *data
                    .get_qubits()
                    .iter()
                    .max()
                    .expect("Gate missing qubit argument"),
                pg_nqubits,
                op,
            );
        }
        Op::ConditionalBox { data } => {
            let conditional_bits_len = data.get_conditional_bits().len();
            let conditional_values_len = data.get_conditional_values().len();
            if conditional_bits_len != conditional_values_len {
                panic!(
                    "ConditionalBox has mismatched conditional bits and values lengths.\nBits: {}\nValues: {}\nOp: {:?}",
                    conditional_bits_len, conditional_values_len, op
                );
            }
            if conditional_bits_len == 0 {
                panic!(
                    "ConditionalBox must have non-empty conditional bits and values.\nOp: {:?}",
                    op
                );
            }
            for inner_op in data.get_ops() {
                validate_op(inner_op, pg_nqubits);
            }
        }
        Op::SetBoundary => {}
    }
}

fn validate_op_arg_range(maxq: usize, n_qubits: usize, op: &Op) {
    assert!(
        maxq < n_qubits,
        "Operation has argument qubit index out of range.\nExpected: < {}\nGot: {}\nOp: {:?}",
        n_qubits,
        maxq,
        op
    );
}
fn validate_pauli_string_len(len: usize, n_qubits: usize, op: &Op) {
    assert!(
        len == n_qubits,
        "String length does not match graph qubit count.\nExpected: {}\nGot: {}\nOp: {:?}",
        n_qubits,
        len,
        op
    );
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
