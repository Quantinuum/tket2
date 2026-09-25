//! Conversion utilities between serial circuits and Pauli graphs.
//!
//! Provides [`RegisterMap`], [`serial_circuit_to_pauli_graph`] and
//! [`pauli_graph_to_cmds`] utilities required for passes that resynthesize
//! a circuit using a [`PauliGraph`]

use pg_core::{BlackBoxData, GateData, GateType, Op, PauliGraph};
use tket_json_rs::circuit_json::{Command, Operation};
use tket_json_rs::register::{Bit, ElementId, Qubit};
use tket_json_rs::{OpType, SerialCircuit};

use serde::{Deserialize, Serialize};
use serde_json;

use indexmap::IndexSet;

/// Maps a [`SerialCircuit`]'s [`ElementId`]s to unique indices
pub struct RegisterMap {
    qubit_map: IndexSet<ElementId>,
    bit_map: IndexSet<ElementId>,
}

impl RegisterMap {
    /// Creates a new [`RegisterMap`] from the given [`Qubit`]s and [`Bit`]s
    pub fn new(qubits: &[Qubit], bits: &[Bit]) -> Self {
        Self {
            qubit_map: qubits.iter().map(|q| q.id.clone()).collect(),
            bit_map: bits.iter().map(|b| b.id.clone()).collect(),
        }
    }

    fn get_indices(&self, args: &[ElementId]) -> Result<(Vec<usize>, Vec<usize>), ConversionError> {
        let mut qubits = Vec::new();
        let mut bits = Vec::new();

        for arg in args {
            if let Some(i) = self.qubit_map.get_index_of(arg) {
                qubits.push(i);
            } else if let Some(i) = self.bit_map.get_index_of(arg) {
                bits.push(i);
            } else {
                return Err(ConversionError::UnknownRegister(arg.to_string()));
            }
        }

        Ok((qubits, bits))
    }

    fn get_qubit_id(&self, index: usize) -> Result<ElementId, ConversionError> {
        self.qubit_map
            .get_index(index)
            .cloned()
            .ok_or_else(|| ConversionError::UnknownRegister(index.to_string()))
    }

    fn get_bit_id(&self, index: usize) -> Result<ElementId, ConversionError> {
        self.bit_map
            .get_index(index)
            .cloned()
            .ok_or_else(|| ConversionError::UnknownRegister(index.to_string()))
    }
}

/// Holds the data and registers of an [`OpType::Barrier`]
#[derive(Serialize, Deserialize)]
struct BarrierContent {
    #[serde(default)]
    op_data: Option<String>,
    #[serde(default)]
    args: Vec<ElementId>,
}

/// Converts a serial circuit to an equivalent pauli graph
pub fn serial_circuit_to_pauli_graph(
    serial_circuit: &mut SerialCircuit,
    register_map: &RegisterMap,
) -> Result<PauliGraph, ConversionError> {
    let num_qubits = serial_circuit.qubits.len();

    let mut ops = Vec::new();
    for cmd in &serial_circuit.commands {
        ops.extend(cmd_to_op(cmd, register_map)?);
    }

    Ok(PauliGraph::new(num_qubits).with_ops(ops))
}

/// Converts a pauli graph to an equivalent vector of hugr commands
pub fn pauli_graph_to_cmds(
    pauli_graph: PauliGraph,
    register_map: &RegisterMap,
) -> Result<Vec<Command<String>>, ConversionError> {
    let mut cmds = Vec::new();

    for op in pauli_graph.get_ops() {
        cmds.extend(op_to_cmd(op, register_map)?);
    }

    Ok(cmds)
}

/// Converts a hugr command to an equivalent vector of pauli graph ops
fn cmd_to_op(
    cmd: &Command<String>,
    register_map: &RegisterMap,
) -> Result<Vec<Op>, ConversionError> {
    let (qubits, bits) = register_map.get_indices(&cmd.args)?;
    let params = cmd.op.params.clone();

    match cmd.op.op_type {
        OpType::H => Ok(vec![Op::Gate {
            data: GateData::new(GateType::H, qubits),
        }]),
        OpType::CX => Ok(vec![Op::Gate {
            data: GateData::new(GateType::ZX, qubits),
        }]),
        OpType::CY => Ok(vec![Op::Gate {
            data: GateData::new(GateType::ZY, qubits),
        }]),
        OpType::CZ => Ok(vec![Op::Gate {
            data: GateData::new(GateType::ZZ, qubits),
        }]),
        OpType::CRz => {
            let angle_string =
                params.ok_or(ConversionError::RotationAngleRequired(cmd.op.op_type))?;
            let angle = angle_string[0]
                .parse::<f64>()
                .map_err(|_| ConversionError::SymbolicParameter(angle_string[0].clone()))?;

            Ok(vec![
                Op::Gate {
                    data: GateData::new(GateType::RZ, vec![qubits[1]])
                        .with_params(vec![angle / 2.0]),
                },
                Op::Gate {
                    data: GateData::new(GateType::ZX, qubits.clone()),
                },
                Op::Gate {
                    data: GateData::new(GateType::RZ, vec![qubits[1]])
                        .with_params(vec![-angle / 2.0]),
                },
                Op::Gate {
                    data: GateData::new(GateType::ZX, qubits),
                },
            ])
        }
        OpType::T => Ok(vec![Op::Gate {
            data: GateData::new(GateType::RZ, qubits).with_params(vec![0.25]),
        }]),
        OpType::Tdg => Ok(vec![Op::Gate {
            data: GateData::new(GateType::RZ, qubits).with_params(vec![-0.25]),
        }]),
        OpType::S => Ok(vec![Op::Gate {
            data: GateData::new(GateType::S, qubits),
        }]),
        OpType::Sdg => Ok(vec![Op::Gate {
            data: GateData::new(GateType::Sdg, qubits),
        }]),
        OpType::V => Ok(vec![Op::Gate {
            data: GateData::new(GateType::V, qubits),
        }]),
        OpType::Vdg => Ok(vec![Op::Gate {
            data: GateData::new(GateType::Vdg, qubits),
        }]),
        OpType::X => Ok(vec![Op::Gate {
            data: GateData::new(GateType::X, qubits),
        }]),
        OpType::Y => Ok(vec![Op::Gate {
            data: GateData::new(GateType::Y, qubits),
        }]),
        OpType::Z => Ok(vec![Op::Gate {
            data: GateData::new(GateType::Z, qubits),
        }]),
        OpType::Rx => {
            let angle_string =
                params.ok_or(ConversionError::RotationAngleRequired(cmd.op.op_type))?;
            let angle = angle_string[0]
                .parse::<f64>()
                .map_err(|_| ConversionError::SymbolicParameter(angle_string[0].clone()))?;

            Ok(vec![Op::Gate {
                data: GateData::new(GateType::RX, qubits).with_params(vec![angle]),
            }])
        }
        OpType::Ry => {
            let angle_string =
                params.ok_or(ConversionError::RotationAngleRequired(cmd.op.op_type))?;
            let angle = angle_string[0]
                .parse::<f64>()
                .map_err(|_| ConversionError::SymbolicParameter(angle_string[0].clone()))?;

            Ok(vec![Op::Gate {
                data: GateData::new(GateType::RY, qubits).with_params(vec![angle]),
            }])
        }
        OpType::Rz => {
            let angle_string =
                params.ok_or(ConversionError::RotationAngleRequired(cmd.op.op_type))?;
            let angle = angle_string[0]
                .parse::<f64>()
                .map_err(|_| ConversionError::SymbolicParameter(angle_string[0].clone()))?;

            Ok(vec![Op::Gate {
                data: GateData::new(GateType::RZ, qubits).with_params(vec![angle]),
            }])
        }
        OpType::CCX => Ok(vec![
            Op::Gate {
                data: GateData::new(GateType::H, vec![qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[1], qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![qubits[2]]).with_params(vec![-0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[0], qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![qubits[2]]).with_params(vec![0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[1], qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![qubits[2]]).with_params(vec![-0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[0], qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![qubits[1]]).with_params(vec![0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![qubits[2]]).with_params(vec![0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::H, vec![qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[0], qubits[1]]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![qubits[0]]).with_params(vec![0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::RZ, vec![qubits[1]]).with_params(vec![-0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[0], qubits[1]]),
            },
        ]),
        OpType::SWAP => Ok(vec![Op::Gate {
            data: GateData::new(GateType::SWAP, qubits),
        }]),
        OpType::Measure => Ok(vec![Op::Gate {
            data: GateData::new(GateType::Measure, vec![qubits[0], bits[0]]),
        }]),
        OpType::Reset => Ok(vec![Op::Gate {
            data: GateData::new(GateType::Reset, qubits),
        }]),
        // Currently, pg-lib blackboxes support specifying the qubits they act on, but not the bits.
        // To prevent a blackbox which acts on bits from being incorrectly reordered during optimisation,
        // we say it acts on all qubits, to prevent it from being reordered.
        OpType::Barrier => {
            let blackbox_qubits = if !bits.is_empty() {
                (0..register_map.qubit_map.len()).collect()
            } else {
                qubits.clone()
            };

            let content = serde_json::to_string(&BarrierContent {
                op_data: cmd.op.data.clone(),
                args: cmd.args.clone(),
            })
            .map_err(|_| {
                ConversionError::UnsupportedBlackBox("Failed barrier conversion".to_string())
            })?;

            Ok(vec![Op::BlackBox {
                data: BlackBoxData::new(blackbox_qubits, content),
            }])
        }
        _ => Err(ConversionError::UnsupportedOpType(cmd.op.op_type)),
    }
}

/// Converts a pauli graph op to an equivalent vector of hugr commands
fn op_to_cmd(op: &Op, register_map: &RegisterMap) -> Result<Vec<Command<String>>, ConversionError> {
    match op {
        Op::Gate { data } => {
            let args = data.get_args();
            match data.get_gate_type() {
                GateType::H => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::H),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::S => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::S),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::Sdg => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::Sdg),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::Z => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::Z),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::V => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::V),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::Vdg => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::Vdg),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::X => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::X),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::Y => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::Y),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::ZX => {
                    let control = register_map.get_qubit_id(args[0])?;
                    let target = register_map.get_qubit_id(args[1])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::CX),
                        args: vec![control, target],
                        opgroup: None,
                    }])
                }
                GateType::XZ => {
                    let control = register_map.get_qubit_id(args[0])?;
                    let target = register_map.get_qubit_id(args[1])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::CX),
                        args: vec![target, control],
                        opgroup: None,
                    }])
                }
                GateType::RX => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    let params = data.get_params();

                    if params.len() != 1 {
                        let msg = format!("RX must have 1 parameter, found {}", params.len());
                        return Err(ConversionError::ImpossibleParams(msg));
                    }

                    match params[0] {
                        0.25 => Ok(vec![
                            Command {
                                op: Operation::from_optype(OpType::H),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::T),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::H),
                                args: vec![qubit],
                                opgroup: None,
                            },
                        ]),
                        -0.25 => Ok(vec![
                            Command {
                                op: Operation::from_optype(OpType::H),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::Tdg),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::H),
                                args: vec![qubit],
                                opgroup: None,
                            },
                        ]),
                        0.5 => Ok(vec![Command {
                            op: Operation::from_optype(OpType::V),
                            args: vec![qubit],
                            opgroup: None,
                        }]),
                        -0.5 => Ok(vec![Command {
                            op: Operation::from_optype(OpType::Vdg),
                            args: vec![qubit],
                            opgroup: None,
                        }]),
                        angle => Err(ConversionError::UnsupportedRotation {
                            gate_type: GateType::RX,
                            angle,
                        }),
                    }
                }
                GateType::RY => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    let params = data.get_params();

                    if params.len() != 1 {
                        let msg = format!("RY must have 1 parameter, found {}", params.len());
                        return Err(ConversionError::ImpossibleParams(msg));
                    }

                    match params[0] {
                        0.25 => Ok(vec![
                            Command {
                                op: Operation::from_optype(OpType::Sdg),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::H),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::T),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::H),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::S),
                                args: vec![qubit],
                                opgroup: None,
                            },
                        ]),
                        -0.25 => Ok(vec![
                            Command {
                                op: Operation::from_optype(OpType::Sdg),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::H),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::Tdg),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::H),
                                args: vec![qubit.clone()],
                                opgroup: None,
                            },
                            Command {
                                op: Operation::from_optype(OpType::S),
                                args: vec![qubit],
                                opgroup: None,
                            },
                        ]),
                        angle => Err(ConversionError::UnsupportedRotation {
                            gate_type: GateType::RY,
                            angle,
                        }),
                    }
                }
                GateType::RZ => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    let params = data.get_params();

                    if params.len() != 1 {
                        let msg = format!("RZ must have 1 parameter, found {}", params.len());
                        return Err(ConversionError::ImpossibleParams(msg));
                    }

                    match params[0] {
                        0.25 => Ok(vec![Command {
                            op: Operation::from_optype(OpType::T),
                            args: vec![qubit],
                            opgroup: None,
                        }]),
                        -0.25 => Ok(vec![Command {
                            op: Operation::from_optype(OpType::Tdg),
                            args: vec![qubit],
                            opgroup: None,
                        }]),
                        0.5 => Ok(vec![Command {
                            op: Operation::from_optype(OpType::S),
                            args: vec![qubit],
                            opgroup: None,
                        }]),
                        -0.5 => Ok(vec![Command {
                            op: Operation::from_optype(OpType::Sdg),
                            args: vec![qubit],
                            opgroup: None,
                        }]),
                        angle => Err(ConversionError::UnsupportedRotation {
                            gate_type: GateType::RZ,
                            angle,
                        }),
                    }
                }
                GateType::SWAP => {
                    let qubit_0 = register_map.get_qubit_id(args[0])?;
                    let qubit_1 = register_map.get_qubit_id(args[1])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::SWAP),
                        args: vec![qubit_0, qubit_1],
                        opgroup: None,
                    }])
                }
                GateType::Measure => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    let bit = register_map.get_bit_id(args[1])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::Measure),
                        args: vec![qubit, bit],
                        opgroup: None,
                    }])
                }
                GateType::Reset => {
                    let qubit = register_map.get_qubit_id(args[0])?;
                    Ok(vec![Command {
                        op: Operation::from_optype(OpType::Reset),
                        args: vec![qubit],
                        opgroup: None,
                    }])
                }
                GateType::BlackBox => {
                    let content = data.get_data().as_deref().ok_or_else(|| {
                        ConversionError::UnsupportedBlackBox("Missing black-box payload".to_owned())
                    })?;

                    let barrier_content: BarrierContent = serde_json::from_str(content)
                        .map_err(|_| ConversionError::UnsupportedBlackBox(content.to_owned()))?;

                    let mut op = Operation::from_optype(OpType::Barrier);
                    op.data = barrier_content.op_data;

                    Ok(vec![Command {
                        op,
                        args: barrier_content.args,
                        opgroup: None,
                    }])
                }
                _ => Err(ConversionError::UnsupportedGate(
                    data.get_gate_type().clone(),
                )),
            }
        }
        Op::BlackBox { data } => {
            let content = data.get_content();
            let barrier_content: BarrierContent = serde_json::from_str(content)
                .map_err(|_| ConversionError::UnsupportedBlackBox(content.to_string()))?;

            let mut op = Operation::from_optype(OpType::Barrier);
            op.data = barrier_content.op_data;

            Ok(vec![Command {
                op,
                args: barrier_content.args,
                opgroup: None,
            }])
        }
        _ => Err(ConversionError::UnsupportedOp(op.clone())),
    }
}

/// Errors that can occur when converting between serial circuit and pauli graph
#[derive(derive_more::Error, Debug, derive_more::Display)]
pub enum ConversionError {
    /// Circuit contains symbolic parameter
    #[display("Error converting to pauli graph: Circuit contains symbolic parameter: {_0}")]
    #[error(ignore)]
    SymbolicParameter(String),
    /// Rotation angle is not specified
    #[display("Error converting to pauli graph: {_0} gate requires a rotation angle")]
    #[error(ignore)]
    RotationAngleRequired(OpType),
    /// Unsupported OpType
    #[display("Error converting to pauli graph: Unsupported OpType: {_0}")]
    #[error(ignore)]
    UnsupportedOpType(OpType),
    /// Unsupported Op
    #[display("Error converting to serial circuit: Unsupported Op: {:?}", _0)]
    #[error(ignore)]
    UnsupportedOp(Op),
    /// Unsupported Gate
    #[display("Error converting to serial circuit: Unsupported Gate: {:?}", _0)]
    #[error(ignore)]
    UnsupportedGate(GateType),
    /// Rotation angle is not supported by the Clifford + T conversion.
    #[display(
        "Error converting to serial circuit: Unsupported {gate_type:?} rotation angle: {angle}"
    )]
    #[error(ignore)]
    UnsupportedRotation {
        /// The rotation gate being converted.
        gate_type: GateType,
        /// The unsupported angle, in half turns.
        angle: f64,
    },
    /// Impossible Params
    #[display("Error converting to serial circuit: {_0}")]
    #[error(ignore)]
    ImpossibleParams(String),
    /// Unsupported BlackBox
    #[display("Error converting to serial circuit: Unsupported BlackBox content: {_0}")]
    #[error(ignore)]
    UnsupportedBlackBox(String),
    /// A qubit or bit is not present in the [`RegisterMap`].
    #[display("Error converting to pauli graph: Unknown register: {_0}")]
    #[error(ignore)]
    UnknownRegister(String),
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::{fixture, rstest};

    fn element(register: &str, index: i64) -> ElementId {
        ElementId(register.to_owned(), vec![index])
    }

    fn command(op_type: OpType, args: Vec<ElementId>) -> Command<String> {
        Command {
            op: Operation::from_optype(op_type),
            args,
            opgroup: None,
        }
    }

    fn rotation(op_type: OpType, angle: &str, args: Vec<ElementId>) -> Command<String> {
        let mut command = command(op_type, args);
        command.op.params = Some(vec![angle.to_owned()]);
        command
    }

    fn gate(gate_type: GateType, args: Vec<usize>) -> Op {
        Op::Gate {
            data: GateData::new(gate_type, args),
        }
    }

    fn rotation_gate(gate_type: GateType, angle: f64, qubit: usize) -> Op {
        Op::Gate {
            data: GateData::new(gate_type, vec![qubit]).with_params(vec![angle]),
        }
    }

    #[fixture]
    fn circuit() -> SerialCircuit {
        let mut circuit = SerialCircuit::new(None, "0".to_owned());
        circuit.qubits = vec![
            Qubit::from(element("left", 2)),
            Qubit::from(element("right", 4)),
            Qubit::from(element("ancilla", 0)),
        ];
        circuit.bits = vec![
            Bit::from(element("result", 3)),
            Bit::from(element("result", 8)),
        ];
        circuit
    }

    #[fixture]
    fn registers(circuit: SerialCircuit) -> RegisterMap {
        RegisterMap::new(&circuit.qubits, &circuit.bits)
    }

    #[rstest]
    #[case::h(OpType::H, GateType::H)]
    #[case::s(OpType::S, GateType::S)]
    #[case::sdg(OpType::Sdg, GateType::Sdg)]
    #[case::v(OpType::V, GateType::V)]
    #[case::vdg(OpType::Vdg, GateType::Vdg)]
    #[case::x(OpType::X, GateType::X)]
    #[case::y(OpType::Y, GateType::Y)]
    #[case::z(OpType::Z, GateType::Z)]
    fn converts_single_qubit_gates(
        registers: RegisterMap,
        #[case] op_type: OpType,
        #[case] gate_type: GateType,
    ) {
        let command = command(op_type, vec![element("right", 4)]);
        let op = gate(gate_type, vec![1]);

        assert_eq!(cmd_to_op(&command, &registers).unwrap(), vec![op.clone()]);
        assert_eq!(op_to_cmd(&op, &registers).unwrap(), vec![command]);
    }

    #[rstest]
    #[case::cx(OpType::CX, GateType::ZX)]
    #[case::cy(OpType::CY, GateType::ZY)]
    #[case::cz(OpType::CZ, GateType::ZZ)]
    #[case::swap(OpType::SWAP, GateType::SWAP)]
    fn converts_two_qubit_commands(
        registers: RegisterMap,
        #[case] op_type: OpType,
        #[case] gate_type: GateType,
    ) {
        let command = command(op_type, vec![element("left", 2), element("right", 4)]);
        let ops = cmd_to_op(&command, &registers).unwrap();

        assert_eq!(ops, vec![gate(gate_type, vec![0, 1])]);
    }

    #[rstest]
    #[case::cx(GateType::ZX, OpType::CX, vec![0, 1])]
    #[case::reversed_cx(GateType::XZ, OpType::CX, vec![1, 0])]
    #[case::swap(GateType::SWAP, OpType::SWAP, vec![0, 1])]
    fn converts_two_qubit_pauli_ops(
        registers: RegisterMap,
        #[case] gate_type: GateType,
        #[case] op_type: OpType,
        #[case] args: Vec<usize>,
    ) {
        let op = gate(gate_type, args);
        let commands = op_to_cmd(&op, &registers).unwrap();
        let expected = command(op_type, vec![element("left", 2), element("right", 4)]);

        assert_eq!(commands, vec![expected]);
    }

    #[rstest]
    #[case::t(OpType::T, 0.25)]
    #[case::tdg(OpType::Tdg, -0.25)]
    fn converts_t_commands_to_z_rotations(
        registers: RegisterMap,
        #[case] op_type: OpType,
        #[case] angle: f64,
    ) {
        let command = command(op_type, vec![element("left", 2)]);
        let ops = cmd_to_op(&command, &registers).unwrap();

        assert_eq!(ops, vec![rotation_gate(GateType::RZ, angle, 0)]);
    }

    #[rstest]
    #[case::rx(OpType::Rx, GateType::RX, "0.1", 0.1)]
    #[case::ry(OpType::Ry, GateType::RY, "0.2", 0.2)]
    #[case::rz(OpType::Rz, GateType::RZ, "0.3", 0.3)]
    fn converts_rotation_commands(
        registers: RegisterMap,
        #[case] op_type: OpType,
        #[case] gate_type: GateType,
        #[case] angle_string: &str,
        #[case] angle: f64,
    ) {
        let command = rotation(op_type, angle_string, vec![element("ancilla", 0)]);
        let ops = cmd_to_op(&command, &registers).unwrap();

        assert_eq!(ops, vec![rotation_gate(gate_type, angle, 2)]);
    }

    #[rstest]
    fn decomposes_controlled_rz(registers: RegisterMap) {
        let command = rotation(
            OpType::CRz,
            "1.0",
            vec![element("left", 2), element("right", 4)],
        );
        let ops = cmd_to_op(&command, &registers).unwrap();

        let expected = vec![
            rotation_gate(GateType::RZ, 0.5, 1),
            gate(GateType::ZX, vec![0, 1]),
            rotation_gate(GateType::RZ, -0.5, 1),
            gate(GateType::ZX, vec![0, 1]),
        ];
        assert_eq!(ops, expected);
    }

    #[rstest]
    fn decomposes_ccx(registers: RegisterMap) {
        let command = command(
            OpType::CCX,
            vec![
                element("left", 2),
                element("right", 4),
                element("ancilla", 0),
            ],
        );
        let ops = cmd_to_op(&command, &registers).unwrap();

        let expected = vec![
            gate(GateType::H, vec![2]),
            gate(GateType::ZX, vec![1, 2]),
            rotation_gate(GateType::RZ, -0.25, 2),
            gate(GateType::ZX, vec![0, 2]),
            rotation_gate(GateType::RZ, 0.25, 2),
            gate(GateType::ZX, vec![1, 2]),
            rotation_gate(GateType::RZ, -0.25, 2),
            gate(GateType::ZX, vec![0, 2]),
            rotation_gate(GateType::RZ, 0.25, 1),
            rotation_gate(GateType::RZ, 0.25, 2),
            gate(GateType::H, vec![2]),
            gate(GateType::ZX, vec![0, 1]),
            rotation_gate(GateType::RZ, 0.25, 0),
            rotation_gate(GateType::RZ, -0.25, 1),
            gate(GateType::ZX, vec![0, 1]),
        ];
        assert_eq!(ops, expected);
    }

    #[rstest]
    #[case::rx_t(GateType::RX, 0.25, vec![OpType::H, OpType::T, OpType::H])]
    #[case::rx_tdg(GateType::RX, -0.25, vec![OpType::H, OpType::Tdg, OpType::H])]
    #[case::rx_v(GateType::RX, 0.5, vec![OpType::V])]
    #[case::rx_vdg(GateType::RX, -0.5, vec![OpType::Vdg])]
    #[case::ry_t(GateType::RY, 0.25, vec![OpType::Sdg, OpType::H, OpType::T, OpType::H, OpType::S])]
    #[case::ry_tdg(GateType::RY, -0.25, vec![OpType::Sdg, OpType::H, OpType::Tdg, OpType::H, OpType::S])]
    #[case::rz_t(GateType::RZ, 0.25, vec![OpType::T])]
    #[case::rz_tdg(GateType::RZ, -0.25, vec![OpType::Tdg])]
    #[case::rz_s(GateType::RZ, 0.5, vec![OpType::S])]
    #[case::rz_sdg(GateType::RZ, -0.5, vec![OpType::Sdg])]
    fn decomposes_pauli_rotations(
        registers: RegisterMap,
        #[case] gate_type: GateType,
        #[case] angle: f64,
        #[case] expected_types: Vec<OpType>,
    ) {
        let op = rotation_gate(gate_type, angle, 2);
        let commands = op_to_cmd(&op, &registers).unwrap();
        let expected: Vec<_> = expected_types
            .into_iter()
            .map(|op_type| command(op_type, vec![element("ancilla", 0)]))
            .collect();

        assert_eq!(commands, expected);
    }

    #[rstest]
    fn converts_measurement_registers(registers: RegisterMap) {
        let command = command(
            OpType::Measure,
            vec![element("ancilla", 0), element("result", 8)],
        );
        let op = gate(GateType::Measure, vec![2, 1]);

        assert_eq!(cmd_to_op(&command, &registers).unwrap(), vec![op.clone()]);
        assert_eq!(op_to_cmd(&op, &registers).unwrap(), vec![command]);
    }

    #[rstest]
    #[case::qubit_only(vec![element("right", 4)], vec![1], None)]
    #[case::with_bit(
        vec![element("left", 2), element("result", 3)],
        vec![0, 1, 2],
        Some("keep together"),
    )]
    fn preserves_barriers(
        registers: RegisterMap,
        #[case] args: Vec<ElementId>,
        #[case] affected_qubits: Vec<usize>,
        #[case] op_data: Option<&str>,
    ) {
        let mut command = command(OpType::Barrier, args.clone());
        command.op.data = op_data.map(str::to_owned);
        let content = serde_json::to_string(&BarrierContent {
            op_data: command.op.data.clone(),
            args,
        })
        .unwrap();
        let op = Op::BlackBox {
            data: BlackBoxData::new(affected_qubits, content),
        };

        assert_eq!(cmd_to_op(&command, &registers).unwrap(), vec![op.clone()]);
        assert_eq!(op_to_cmd(&op, &registers).unwrap(), vec![command]);
    }

    #[rstest]
    fn converts_complete_circuit(mut circuit: SerialCircuit, registers: RegisterMap) {
        circuit.commands = vec![
            command(OpType::H, vec![element("left", 2)]),
            command(OpType::CX, vec![element("left", 2), element("right", 4)]),
            command(
                OpType::Measure,
                vec![element("right", 4), element("result", 8)],
            ),
        ];

        let graph = serial_circuit_to_pauli_graph(&mut circuit, &registers).unwrap();

        assert_eq!(graph.get_n_qubits(), 3);
        assert_eq!(
            graph.get_ops(),
            &vec![
                gate(GateType::H, vec![0]),
                gate(GateType::ZX, vec![0, 1]),
                gate(GateType::Measure, vec![1, 1]),
            ]
        );
        assert_eq!(
            pauli_graph_to_cmds(graph, &registers).unwrap(),
            circuit.commands
        );
    }

    #[rstest]
    fn error_on_unknown_register_name(registers: RegisterMap) {
        let unknown = element("unknown", 0);
        let result = registers.get_indices(std::slice::from_ref(&unknown));

        assert!(matches!(
            result,
            Err(ConversionError::UnknownRegister(register)) if register == unknown.to_string()
        ));
    }

    #[rstest]
    fn error_on_unknown_qubit_index(registers: RegisterMap) {
        assert!(matches!(
            registers.get_qubit_id(3),
            Err(ConversionError::UnknownRegister(register)) if register == "3"
        ));
    }

    #[rstest]
    fn error_on_unknown_bit_index(registers: RegisterMap) {
        assert!(matches!(
            registers.get_bit_id(2),
            Err(ConversionError::UnknownRegister(register)) if register == "2"
        ));
    }

    #[rstest]
    fn error_on_commands_without_rotation_angles(
        registers: RegisterMap,
        #[values(OpType::Rx, OpType::Ry, OpType::Rz, OpType::CRz)] op_type: OpType,
    ) {
        let command = command(op_type, vec![element("left", 2)]);
        let result = cmd_to_op(&command, &registers);

        assert!(matches!(
            result,
            Err(ConversionError::RotationAngleRequired(error_type)) if error_type == op_type
        ));
    }

    #[rstest]
    fn error_on_symbolic_rotation_angles(
        registers: RegisterMap,
        #[values(OpType::Rx, OpType::Ry, OpType::Rz, OpType::CRz)] op_type: OpType,
    ) {
        let command = rotation(op_type, "theta", vec![element("left", 2)]);
        let result = cmd_to_op(&command, &registers);

        assert!(matches!(
            result,
            Err(ConversionError::SymbolicParameter(parameter)) if parameter == "theta"
        ));
    }

    #[rstest]
    fn error_on_unsupported_commands(registers: RegisterMap) {
        let command = command(OpType::Create, vec![element("left", 2)]);
        let result = cmd_to_op(&command, &registers);

        assert!(matches!(
            result,
            Err(ConversionError::UnsupportedOpType(OpType::Create))
        ));
    }

    #[rstest]
    fn error_on_unsupported_pauli_ops(registers: RegisterMap) {
        let result = op_to_cmd(&Op::SetBoundary, &registers);

        assert!(matches!(
            result,
            Err(ConversionError::UnsupportedOp(Op::SetBoundary))
        ));
    }

    #[rstest]
    fn error_on_unsupported_pauli_gates(registers: RegisterMap) {
        let result = op_to_cmd(&gate(GateType::ZZ, vec![0, 1]), &registers);

        assert!(matches!(
            result,
            Err(ConversionError::UnsupportedGate(GateType::ZZ))
        ));
    }

    #[rstest]
    fn error_on_pauli_rotations_without_angles(
        registers: RegisterMap,
        #[values(GateType::RX, GateType::RY, GateType::RZ)] gate_type: GateType,
    ) {
        let op = gate(gate_type.clone(), vec![0]);
        let result = op_to_cmd(&op, &registers);
        let expected_message = format!("{gate_type:?} must have 1 parameter, found 0");

        assert!(matches!(
            result,
            Err(ConversionError::ImpossibleParams(message)) if message == expected_message
        ));
    }

    #[rstest]
    fn error_on_invalid_black_box_content(registers: RegisterMap) {
        let op = Op::BlackBox {
            data: BlackBoxData::new(vec![], "not json".to_owned()),
        };
        let result = op_to_cmd(&op, &registers);

        assert!(matches!(
            result,
            Err(ConversionError::UnsupportedBlackBox(content)) if content == "not json"
        ));
    }

    #[rstest]
    #[case::rx(GateType::RX, 0.1)]
    #[case::ry(GateType::RY, 0.1)]
    #[case::rz(GateType::RZ, 0.1)]
    #[case::unsupported_clifford_angle(GateType::RY, 0.5)]
    fn error_on_unsupported_rotation(
        registers: RegisterMap,
        #[case] gate_type: GateType,
        #[case] angle: f64,
    ) {
        let graph = PauliGraph::new(3).with_ops(vec![rotation_gate(gate_type.clone(), angle, 0)]);
        let result = pauli_graph_to_cmds(graph, &registers);

        assert!(matches!(
            result,
            Err(ConversionError::UnsupportedRotation {
                gate_type: error_gate,
                angle: error_angle,
            }) if error_gate == gate_type && error_angle == angle
        ));
    }
}
