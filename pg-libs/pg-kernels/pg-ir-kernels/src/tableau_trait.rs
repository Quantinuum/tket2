use pg_core::{
    ConditionalBoxData, GateData, GateType, MeasureData, Op, Pauli, ResetData, RotationData,
    TableauData,
};

use crate::ops::*;

fn build_op_from_gate(op: Op, data: &GateData) -> Op {
    if data.get_conditional_bits().is_empty() {
        op
    } else {
        Op::ConditionalBox {
            data: ConditionalBoxData::new(
                vec![op],
                data.get_conditional_bits().clone(),
                data.get_conditional_values().clone(),
            ),
        }
    }
}

/// A tableau implements the necessary functionality to interact with PG Ops and Pauli strings.
/// This includes methods for getting the image of X and Z operators under the tableau,
/// pre- and post-composing operations and other tableaux, getting the dagger (inverse) of the tableau,
/// and conjugating PG Ops and Pauli strings by the tableau.
///
/// Note that pre- and post-compositions are defined as function compositions.
/// Not to be confused with composing gates on the left or right of a tableau, which is not necessarily the same thing.
pub trait PGTableau: From<TableauData> + Into<TableauData> + Eq {
    /// Construct an identity tableau
    ///
    /// # Arguments
    ///
    /// - `n_qubits` (`usize`) - The number of qubits for the identity tableau.
    ///
    /// # Returns
    ///
    /// - `Self` - The identity tableau.
    fn eye(n_qubits: usize) -> Self;

    /// Return the number of qubits that the tableau acts on.
    ///
    /// # Returns
    ///
    /// - `usize` - The number of qubits that the tableau acts on.
    fn get_n_qubits(&self) -> usize;

    /// Get the image of an X operator under the tableau. The returned boolean indicates whether the image has a positive sign.
    ///
    /// # Arguments
    ///
    /// - `qubit` (`usize`) - The qubit for which to get the image of the X operator.
    ///
    /// # Returns
    ///
    /// - `(Vec<Pauli>, bool)` - A tuple where the first element is a vector of Pauli operators representing the image of the X operator,
    ///   and the second element is a boolean indicating whether the image has a positive sign.
    fn x(&self, qubit: usize) -> (Vec<Pauli>, bool);

    /// Get the image of a Z operator under the tableau. The returned boolean indicates whether the image has a positive sign.
    ///
    /// # Arguments
    ///
    /// - `qubit` (`usize`) - The qubit for which to get the image of the Z operator.
    ///
    /// # Returns
    ///
    /// - `(Vec<Pauli>, bool)` - A tuple where the first element is a vector of Pauli operators representing the image of the Z operator,
    ///   and the second element is a boolean indicating whether the image has a positive sign.
    fn z(&self, qubit: usize) -> (Vec<Pauli>, bool);

    /// Pre-compose an operation to the tableau.
    ///
    /// # Arguments
    ///
    /// - `op` (`&Op`) - The Clifford operation to pre-compose.
    fn precompose_op(&mut self, op: &Op);

    /// Post-compose an operation to the tableau.
    ///
    /// # Arguments
    ///
    /// - `op` (`&Op`) - The Clifford operation to post-compose.
    fn postcompose_op(&mut self, op: &Op);

    /// Pre-compose a tableau to the current tableau.
    ///
    /// # Arguments
    ///
    /// - `other` (`&Self`) - The tableau to pre-compose.
    fn precompose_tableau(&mut self, other: &Self);

    /// Post-compose a tableau to the current tableau.
    ///
    /// # Arguments
    ///
    /// - `other` (`&Self`) - The tableau to post-compose.
    fn postcompose_tableau(&mut self, other: &Self);

    /// Get the dagger (inverse) of the tableau.
    ///
    /// # Returns
    ///
    /// - `Self` - The dagger (inverse) of the tableau.
    fn get_dagger(&self) -> Self;

    /// Get the image of a Pauli string under the tableau. The returned boolean indicates whether the image has a positive sign.
    ///
    /// # Arguments
    ///
    /// - `paulis` (`&[Pauli]`) - The Pauli string for which to get the image under the tableau.
    ///
    /// # Returns
    ///
    /// - `(Vec<Pauli>, bool)` - A tuple where the first element is a vector of Pauli operators representing the image of the Pauli string,
    ///   and the second element is a boolean indicating whether the image has a positive sign.
    fn conjugate_string(&self, paulis: &[Pauli]) -> (Vec<Pauli>, bool);

    /// Get the image of a PG Op under the tableau. The returned boolean indicates whether the image has a positive sign.
    ///
    /// # Arguments
    ///
    /// - `op` (`&Op`) - The PG Op for which to get the image under the tableau.
    ///
    /// # Returns
    ///
    /// - `Vec<Op>` - A vector of PG Ops representing the image of the input PG Op under the tableau.
    ///
    /// # Panics
    ///
    /// - If the input operation cannot be conjugated by a tableau (i.e. if `can_be_conjugated(op)` returns `false`).
    fn conjugate(&self, op: &Op) -> Vec<Op> {
        if !can_be_conjugated(op) {
            panic!("Operation {:?} cannot be conjugated by a tableau", op);
        }
        match op {
            Op::Gate { data } => match data.get_gate_type() {
                GateType::RX => {
                    let (s, sign) = self.x(data.get_args()[0]);
                    let theta = if sign {
                        data.get_params()[0]
                    } else {
                        -data.get_params()[0]
                    };
                    vec![build_op_from_gate(
                        Op::Rotation {
                            data: RotationData::new(s, theta),
                        },
                        data,
                    )]
                }
                GateType::RY => {
                    let mut s = vec![Pauli::I; self.get_n_qubits()];
                    s[data.get_args()[0]] = Pauli::Y;
                    let (s, sign) = self.conjugate_string(&s);
                    let theta = if sign {
                        data.get_params()[0]
                    } else {
                        -data.get_params()[0]
                    };
                    vec![build_op_from_gate(
                        Op::Rotation {
                            data: RotationData::new(s, theta),
                        },
                        data,
                    )]
                }
                GateType::RZ => {
                    let (s, sign) = self.z(data.get_args()[0]);
                    let theta = if sign {
                        data.get_params()[0]
                    } else {
                        -data.get_params()[0]
                    };
                    vec![build_op_from_gate(
                        Op::Rotation {
                            data: RotationData::new(s, theta),
                        },
                        data,
                    )]
                }
                GateType::ZZPHASE => {
                    let mut s = vec![Pauli::I; self.get_n_qubits()];
                    s[data.get_args()[0]] = Pauli::Z;
                    s[data.get_args()[1]] = Pauli::Z;
                    let (s, sign) = self.conjugate_string(&s);
                    let theta = if sign {
                        data.get_params()[0]
                    } else {
                        -data.get_params()[0]
                    };
                    vec![build_op_from_gate(
                        Op::Rotation {
                            data: RotationData::new(s, theta),
                        },
                        data,
                    )]
                }
                GateType::PHASEDX => {
                    let alpha = data.get_params()[0];
                    let beta = data.get_params()[1];
                    let args = data.get_args();
                    let op1 = Op::Gate {
                        data: GateData::new(GateType::RZ, args.clone())
                            .with_params(vec![-beta])
                            .with_conditional(
                                data.get_conditional_bits().clone(),
                                data.get_conditional_values().clone(),
                            ),
                    };
                    let op2 = Op::Gate {
                        data: GateData::new(GateType::RX, args.clone())
                            .with_params(vec![alpha])
                            .with_conditional(
                                data.get_conditional_bits().clone(),
                                data.get_conditional_values().clone(),
                            ),
                    };
                    let op3 = Op::Gate {
                        data: GateData::new(GateType::RZ, args.clone())
                            .with_params(vec![beta])
                            .with_conditional(
                                data.get_conditional_bits().clone(),
                                data.get_conditional_values().clone(),
                            ),
                    };
                    let mut new_ops = self.conjugate(&op1);
                    new_ops.extend(self.conjugate(&op2));
                    new_ops.extend(self.conjugate(&op3));
                    new_ops
                }
                GateType::Measure => {
                    let (z_string, z_sign) = self.z(data.get_args()[0]);
                    vec![build_op_from_gate(
                        Op::Measure {
                            data: MeasureData::new(z_string, z_sign, data.get_args()[1]),
                        },
                        data,
                    )]
                }
                GateType::Reset => {
                    let (z_string, z_sign) = self.z(data.get_args()[0]);
                    let (x_string, x_sign) = self.x(data.get_args()[0]);
                    vec![build_op_from_gate(
                        Op::Reset {
                            data: ResetData::new(z_string, x_string, z_sign, x_sign),
                        },
                        data,
                    )]
                }
                _ => panic!(
                    "Unsupported gate type in conjugate: {:?}. Should have been caught by can_be_conjugated()",
                    data.get_gate_type()
                ),
            },
            Op::Rotation { data } => {
                let (s, sign) = self.conjugate_string(data.get_string());
                let theta = if sign {
                    data.get_angle()
                } else {
                    -data.get_angle()
                };
                vec![Op::Rotation {
                    data: RotationData::new(s, theta),
                }]
            }
            Op::Measure { data } => {
                let (s, mut sign) = self.conjugate_string(data.get_string());
                sign = !sign ^ data.get_sign();
                vec![Op::Measure {
                    data: MeasureData::new(s, sign, data.get_cbit()),
                }]
            }
            Op::Reset { data } => {
                let (z_string, mut z_sign) = self.conjugate_string(data.get_first_string());
                let (x_string, mut x_sign) = self.conjugate_string(data.get_second_string());
                z_sign = !z_sign ^ data.get_first_sign();
                x_sign = !x_sign ^ data.get_second_sign();
                vec![Op::Reset {
                    data: ResetData::new(z_string, x_string, z_sign, x_sign),
                }]
            }
            Op::ConditionalBox { data } => {
                let mut new_ops = Vec::with_capacity(data.get_ops().len());
                for cond_op in data.get_ops() {
                    new_ops.extend(self.conjugate(cond_op));
                }
                vec![Op::ConditionalBox {
                    data: ConditionalBoxData::new(
                        new_ops,
                        data.get_conditional_bits().clone(),
                        data.get_conditional_values().clone(),
                    ),
                }]
            }
            _ => unreachable!(
                "Unsupported operation type in conjugate: {:?}. Should have been caught by can_be_conjugated()",
                op
            ),
        }
    }
}
