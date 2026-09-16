use pg_core::{BlackBoxData, GateData, GateType, Op, PauliGraph};
use tket_json_rs::circuit_json::{Command, Operation};
use tket_json_rs::register::{Bit, ElementId, Qubit};
use tket_json_rs::{OpType, SerialCircuit};

use serde::{Deserialize, Serialize};
use serde_json;

use indexmap::IndexMap;
use indexmap::IndexSet;

pub struct RegisterMap {
    qubit_map: IndexSet<ElementId>,
    bit_map: IndexSet<ElementId>,
}

impl RegisterMap {
    pub fn new(qubits: &[Qubit], bits: &[Bit]) -> Self {
        Self {
            qubit_map: qubits.iter().map(|q| q.id.clone()).collect(),
            bit_map: bits.iter().map(|b| b.id.clone()).collect()
        }
    }

    pub fn get_indices(&self, args: &[ElementId]) -> Result<(Vec<usize>, Vec<usize>), ConversionError> {
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

    pub fn get_qubit_id(&self, index: usize) -> Result<ElementId, ConversionError> {
        self.qubit_map.get_index(index).cloned()
            .ok_or_else(|| ConversionError::UnknownRegister(index.to_string()))
    }

    pub fn get_bit_id(&self, index: usize) -> Result<ElementId, ConversionError> {
        self.bit_map.get_index(index).cloned()
            .ok_or_else(|| ConversionError::UnknownRegister(index.to_string()))
    }
}

#[derive(Serialize, Deserialize)]
struct BarrierContent {
    #[serde(default)]
    op_data: Option<String>,
    #[serde(default)]
    args: Vec<ElementId>,
}

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
        // TODO: Use a native CCX or toffoli when available
        OpType::CCX => Ok(vec![
            Op::Gate {
                data: GateData::new(GateType::H, vec![qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[1], qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::Z, vec![qubits[2]]).with_params(vec![-0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[0], qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::Z, vec![qubits[2]]).with_params(vec![0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[1], qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::Z, vec![qubits[2]]).with_params(vec![-0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[0], qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::Z, vec![qubits[1]]).with_params(vec![0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::Z, vec![qubits[2]]).with_params(vec![0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::H, vec![qubits[2]]),
            },
            Op::Gate {
                data: GateData::new(GateType::ZX, vec![qubits[0], qubits[1]]),
            },
            Op::Gate {
                data: GateData::new(GateType::Z, vec![qubits[0]]).with_params(vec![0.25]),
            },
            Op::Gate {
                data: GateData::new(GateType::Z, vec![qubits[1]]).with_params(vec![-0.25]),
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
        // Currently, blackboxes support specifying the qubits they act on, but not the bits. 
        // To prevent gates acting on bits from being incorrectly reordered, we say they act 
        // on all qubits, which prevents them from being reordered at all.
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
            .map_err(|_| ConversionError::UnsupportedBlackBox("Failed barrier conversion".to_string()))?;

            Ok(vec![Op::BlackBox {
                data: BlackBoxData::new(blackbox_qubits, content),
            }])
        }
        _ => Err(ConversionError::UnsupportedOpType(cmd.op.op_type)),
    }
}

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
                return Ok(vec![Command {
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
                    _ => {
                        panic!("RX {} not in Clifford + T", params[0]);
                    }
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
                    _ => {
                        panic!("RY {} not in Clifford + T", params[0]);
                    }
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
                    _ => panic!("arbitrary RZ gate not in Clifford + T"),
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
            _ => Err(ConversionError::UnsupportedGate(
                data.get_gate_type().clone(),
            )),
            }
        },
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
// TODO: usage of derive_more::Error and error(ignore)
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
    /// Impossible Params
    #[display("Error converting to serial circuit: {_0}")]
    #[error(ignore)]
    ImpossibleParams(String),
    /// Unsupported BlackBox
    #[display("Error converting to serial circuit: Unsupported BlackBox content: {_0}")]
    #[error(ignore)]
    UnsupportedBlackBox(String),
    #[display("Error converting to pauli graph: Unknown register: {_0}")]
    #[error(ignore)]
    UnknownRegister(String),
}
