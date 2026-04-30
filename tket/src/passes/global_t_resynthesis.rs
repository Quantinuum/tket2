use crate::metadata::QubitRegisters;
use crate::passes::NormalizeGuppy;
use crate::passes::PassScope;
use crate::passes::WithScope;
use crate::passes::composable::Preserve;
use crate::passes::guppy::NormalizeGuppyErrors;
use crate::passes::inline_funcs::InlineFuncsError;
use crate::serialize::pytket::{
    EncodeOptions, EncodedCircuit, PytketDecodeError, PytketEncodeError, default_decoder_config,
    default_encoder_config,
};
use crate::{Circuit, CircuitError};
use hugr::hugr::ValidationError;
use hugr::hugr::hugrmut::HugrMut;

use basic_passes::CanonicalFormPass;
use fast_todd::FastTODDPass;
use greedy_synth::{GreedySynthPass, RebaseTQEToZXPass};
use hugr::types::EdgeKind;
use hugr_core::hugr::internal::{HugrInternals, PortgraphNodeMap};
use pauli_graph::{BlackBoxData, GateData, GateType, Op, PauliGraph, PauliGraphPass};
use petgraph::visit as pv;
use pg_optimise::{GroupCommutingOpsPass, RotationMergingPass};

use crate::passes::composable::ComposablePass;
use crate::passes::inline_funcs::inline_acyclic_scoped;
use hugr::HugrView;
use hugr::hugr::OpType as TketOp;
use hugr::{Hugr, Node};

use tket_json_rs::circuit_json::{Command, Operation};
use tket_json_rs::register::{Bit, ElementId, Qubit};
use tket_json_rs::{OpType, SerialCircuit};

use std::cell::Cell;
use std::collections::HashSet;
use std::sync::Arc;

#[derive(Clone, Debug)]
pub struct GlobalTResynthesis {
    /// The scope within which the pass will operate.
    scope: PassScope,
    ancilla_budget: usize,
}

impl Default for GlobalTResynthesis {
    fn default() -> Self {
        Self {
            scope: PassScope::Global(Preserve::All),
            ancilla_budget: 0,
        }
    }
}

impl WithScope for GlobalTResynthesis {
    fn with_scope(mut self, scope: impl Into<crate::passes::PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl GlobalTResynthesis {
    pub fn with_ancilla_budget(&mut self, ancilla_budget: usize) -> &mut Self {
        self.ancilla_budget = ancilla_budget;
        self
    }
}

impl ComposablePass<Hugr> for GlobalTResynthesis {
    type Error = GlobalTResynthesisErrors;
    type Result = ();
    fn run(&self, hugr: &mut Hugr) -> Result<Self::Result, Self::Error> {
        // TODO: think about what order edges we can/should remove
        // (currently removing all call order edges)
        for n in hugr.nodes().collect::<Vec<_>>() {
            if hugr.get_optype(n).is_call() {
                let outport = hugr.get_optype(n).other_output_port().unwrap();
                let inport = hugr.get_optype(n).other_input_port().unwrap();

                let func_node = hugr.static_source(n).unwrap();
                let func_defn = hugr.get_optype(func_node).as_func_defn().unwrap();

                hugr.disconnect(n, outport);
                hugr.disconnect(n, inport);
            }
        }

        inline_acyclic_scoped(hugr, self.scope.clone(), |_, _| true).unwrap();
        NormalizeGuppy::default()
            .constant_folding(false)
            .run(hugr)?;

        let encode_options = EncodeOptions::new()
            .with_subcircuits(true)
            .with_config(default_encoder_config());

        let mut encoded_circs = EncodedCircuit::new(&hugr, encode_options)?;

        for (_, serial_circ) in encoded_circs.iter_mut() {
            let pauli_graph = serial_circuit_to_pauli_graph(serial_circ)?;

            let canonical_pass = CanonicalFormPass::new().with_forward(true);
            let grouping_pass = GroupCommutingOpsPass::new();
            let rotation_merging_pass = RotationMergingPass::new();
            let fast_todd_pass = FastTODDPass::new();
            let synth_pass = GreedySynthPass::new()
                .with_window_size(100)
                .with_pool_size(100)
                .with_top_up_size(100);
            let rebase_pass = RebaseTQEToZXPass::new().with_allowed_tqes(vec![GateType::ZX]);

            let pauli_graph = canonical_pass.transform(&pauli_graph);
            let pauli_graph = rotation_merging_pass.transform(&pauli_graph);
            let pauli_graph = fast_todd_pass.transform(&pauli_graph);
            let pauli_graph = grouping_pass.transform(&pauli_graph);
            let pauli_graph = synth_pass.transform(&pauli_graph);
            let pauli_graph = rebase_pass.transform(&pauli_graph);

            serial_circ.commands = pauli_graph_to_cmds(pauli_graph, serial_circ)?;
        }

        encoded_circs.reassemble_inplace(hugr, Some(Arc::new(default_decoder_config())))?;

        hugr.validate()?;

        let mermaid_string = hugr.mermaid_string();

        Ok(())
    }
}

/// Errors that can occur during the global-t resynthesis
#[derive(derive_more::Error, Debug, derive_more::Display, derive_more::From)]
pub enum GlobalTResynthesisErrors {
    /// Error inlining functions
    #[from]
    InlineError(InlineFuncsError),
    /// Error normalizing the hugr
    #[from]
    NormalizeError(NormalizeGuppyErrors),
    /// Error loading the circuit.
    #[display("Error loading the circuit: {_0}")]
    #[from]
    CircuitLoadError(CircuitError),
    /// Error encoding the circuit.
    #[display("Error encoding the circuit: {_0}")]
    #[from]
    CircuitEncodeError(PytketEncodeError<Node>),
    /// Error converting between pauli graph and serial circuit
    #[from]
    ConversionError(ConversionError),
    /// Error reassembling the circuit
    #[display("Error reassembling the circuit: {_0}")]
    #[from]
    ReassemblyError(PytketDecodeError),
    /// Error validating the reassembled circuit.
    #[from]
    ValidationError(ValidationError<Node>),
}

fn serial_circuit_to_pauli_graph(
    serial_circuit: &mut SerialCircuit,
) -> Result<PauliGraph, ConversionError> {
    let num_qubits = serial_circuit.qubits.len();
    let qubits = &serial_circuit.qubits;
    let bits = &serial_circuit.bits;

    let mut ops = Vec::new();
    for cmd in &serial_circuit.commands {
        ops.extend(cmd_to_op(cmd, qubits, bits)?);
    }

    Ok(PauliGraph::new(num_qubits).with_ops(ops))
}

fn pauli_graph_to_cmds(
    pauli_graph: PauliGraph,
    serial_circuit: &mut SerialCircuit,
) -> Result<Vec<Command<String>>, ConversionError> {
    let qubits = &serial_circuit.qubits;
    let bits = &serial_circuit.bits;

    let (start_barriers, end_barriers) = sort_barriers(&serial_circuit.commands);
    let mut cmds = start_barriers;

    for op in pauli_graph.get_ops() {
        cmds.extend(op_to_cmd(op, qubits, bits)?);
    }

    cmds.extend(end_barriers);

    Ok(cmds)
}

// TODO: think about refactoring match to separate functions
// (some of the cases are pretty long)
fn cmd_to_op(
    cmd: &Command<String>,
    qubits: &[Qubit],
    bits: &[Bit],
) -> Result<Vec<Op>, ConversionError> {
    let qubits = cmd
        .args
        .iter()
        .filter_map(|id| qubits.iter().position(|q| q.id == *id))
        .collect();
    let bits: Vec<usize> = cmd
        .args
        .iter()
        .filter_map(|id| bits.iter().position(|b| b.id == *id))
        .collect();
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

            // TODO: this is a bit cumbersome, refactor to parse_float
            // and then putting the float into a vec
            let half_angle: Vec<f64> = parse_floats(angle_string)?
                .iter()
                .map(|a| a / 2.0)
                .collect();

            let negative_half_angle: Vec<f64> = half_angle.iter().map(|a| -a).collect();

            Ok(vec![
                Op::Gate {
                    data: GateData::new(GateType::RZ, vec![qubits[1]]).with_params(half_angle),
                },
                Op::Gate {
                    data: GateData::new(GateType::ZX, qubits.clone()),
                },
                Op::Gate {
                    data: GateData::new(GateType::RZ, vec![qubits[1]])
                        .with_params(negative_half_angle),
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
            let angle = parse_floats(angle_string)?;

            Ok(vec![Op::Gate {
                data: GateData::new(GateType::RX, qubits).with_params(angle),
            }])
        }
        OpType::Ry => {
            let angle_string =
                params.ok_or(ConversionError::RotationAngleRequired(cmd.op.op_type))?;
            let angle = parse_floats(angle_string)?;

            Ok(vec![Op::Gate {
                data: GateData::new(GateType::RY, qubits).with_params(angle),
            }])
        }
        OpType::Rz => {
            let angle_string =
                params.ok_or(ConversionError::RotationAngleRequired(cmd.op.op_type))?;
            let angle = parse_floats(angle_string)?;

            Ok(vec![Op::Gate {
                data: GateData::new(GateType::RZ, qubits).with_params(angle),
            }])
        }
        // TODO: it may be possible to improve T-count with given ancilla budget by
        // pushing CCX decomposition into the FastTODD pass
        // this requires CCX as a gate in pauli graph interface
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
        OpType::Measure => Ok(vec![Op::Gate {
            data: GateData::new(GateType::Measure, vec![qubits[0], bits[0]]),
        }]),
        OpType::Barrier => Ok(vec![]),
        _ => Err(ConversionError::UnsupportedOpType(cmd.op.op_type)),
    }
}

// TODO: think about refactoring match to separate functions
// (some of the cases are pretty long)
fn op_to_cmd(
    op: &Op,
    qubit_map: &[Qubit],
    bit_map: &[Bit],
) -> Result<Vec<Command<String>>, ConversionError> {
    match op {
        Op::Gate { data } => match data.get_gate_type() {
            GateType::H => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::H),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::S => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::S),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::Sdg => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::Sdg),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::Z => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::Z),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::V => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::V),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::Vdg => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::Vdg),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::X => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::X),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::Y => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::Y),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::Z => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::Z),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::ZX => {
                let qubits = apply_map(qubit_map, data.get_args());
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::CX),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::XZ => {
                let mut qubits = apply_map(qubit_map, data.get_args());
                qubits.reverse();
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::CX),
                    args: qubits,
                    opgroup: None,
                }])
            }
            GateType::RX => {
                let qubits = apply_map(qubit_map, data.get_args());
                let params = data.get_params();

                if params.len() != 1 {
                    let msg = format!("RX must have 1 parameter, found {}", params.len());
                    return Err(ConversionError::ImpossibleParams(msg));
                }

                match params[0] {
                    0.25 => Ok(vec![
                        Command {
                            op: Operation::from_optype(OpType::H),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::T),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::H),
                            args: qubits,
                            opgroup: None,
                        },
                    ]),
                    -0.25 => Ok(vec![
                        Command {
                            op: Operation::from_optype(OpType::H),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::T),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::H),
                            args: qubits,
                            opgroup: None,
                        },
                    ]),
                    0.5 => Ok(vec![Command {
                        op: Operation::from_optype(OpType::V),
                        args: qubits,
                        opgroup: None,
                    }]),
                    -0.5 => Ok(vec![Command {
                        op: Operation::from_optype(OpType::Vdg),
                        args: qubits,
                        opgroup: None,
                    }]),
                    _ => {
                        panic!("RX {} not in Clifford + T", params[0]);
                    }
                }
            }
            GateType::RY => {
                let qubits = apply_map(qubit_map, data.get_args());
                let params = data.get_params();

                if params.len() != 1 {
                    let msg = format!("RX must have 1 parameter, found {}", params.len());
                    return Err(ConversionError::ImpossibleParams(msg));
                }

                match params[0] {
                    0.25 => Ok(vec![
                        Command {
                            op: Operation::from_optype(OpType::H),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::Sdg),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::T),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::S),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::H),
                            args: qubits,
                            opgroup: None,
                        },
                    ]),
                    -0.25 => Ok(vec![
                        Command {
                            op: Operation::from_optype(OpType::H),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::Sdg),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::Tdg),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::S),
                            args: qubits.clone(),
                            opgroup: None,
                        },
                        Command {
                            op: Operation::from_optype(OpType::H),
                            args: qubits,
                            opgroup: None,
                        },
                    ]),
                    _ => {
                        panic!("RY {} not in Clifford + T", params[0]);
                    }
                }
            }
            GateType::RZ => {
                let qubits = apply_map(qubit_map, data.get_args());
                let params = data.get_params();

                if params.len() != 1 {
                    let msg = format!("RZ must have 1 parameter, found {}", params.len());
                    return Err(ConversionError::ImpossibleParams(msg));
                }

                match params[0] {
                    0.25 => Ok(vec![Command {
                        op: Operation::from_optype(OpType::T),
                        args: qubits,
                        opgroup: None,
                    }]),
                    -0.25 => Ok(vec![Command {
                        op: Operation::from_optype(OpType::Tdg),
                        args: qubits,
                        opgroup: None,
                    }]),
                    0.5 => Ok(vec![Command {
                        op: Operation::from_optype(OpType::S),
                        args: qubits,
                        opgroup: None,
                    }]),
                    -0.5 => Ok(vec![Command {
                        op: Operation::from_optype(OpType::Sdg),
                        args: qubits,
                        opgroup: None,
                    }]),
                    _ => panic!("arbitrary RZ gate not in Clifford + T"),
                }
            }
            // TODO: implement this with relabelling
            GateType::SWAP => {
                println!("SWAP");
                let qubits = apply_map(qubit_map, data.get_args());
                let reversed_qubits = qubits.iter().rev().cloned().collect();
                Ok(vec![
                    Command {
                        op: Operation::from_optype(OpType::CX),
                        args: qubits.clone(),
                        opgroup: None,
                    },
                    Command {
                        op: Operation::from_optype(OpType::CX),
                        args: reversed_qubits,
                        opgroup: None,
                    },
                    Command {
                        op: Operation::from_optype(OpType::CX),
                        args: qubits,
                        opgroup: None,
                    },
                ])
            }
            GateType::Measure => {
                let args = data.get_args();
                let qubit = qubit_map[args[0]].clone();
                let bit = bit_map[args[1]].clone();
                Ok(vec![Command {
                    op: Operation::from_optype(OpType::Measure),
                    args: vec![qubit.into(), bit.into()],
                    opgroup: None,
                }])
            }
            // TODO: check the difference between BlackBox gate and Op
            GateType::BlackBox => {
                let qubits_and_bits = apply_map(qubit_map, data.get_args());

                Ok(vec![Command {
                    op: Operation::from_optype(OpType::Barrier),
                    args: qubits_and_bits,
                    opgroup: None,
                }])
            }
            gate => Err(ConversionError::UnsupportedGate(gate.clone())),
        },
        Op::BlackBox { data } => {
            let qubits = apply_map(qubit_map, data.get_qubits());
            let content = data.get_content().to_string();

            if content != "" {
                return Err(ConversionError::UnsupportedBlackBox(content));
            }

            Ok(vec![Command {
                op: Operation::from_optype(OpType::Barrier),
                args: qubits,
                opgroup: None,
            }])
        }
        _ => Err(ConversionError::UnsupportedOp(op.clone())),
    }
}

// This function will panic if there are any barriers in the middle of the circuit
fn sort_barriers(commands: &[Command<String>]) -> (Vec<Command<String>>, Vec<Command<String>>) {
    let mut start_barriers = Vec::new();
    let mut end_barriers = Vec::new();
    let mut qubits_with_gates: HashSet<ElementId> = HashSet::new();
    let mut end_barrier_qubits: HashSet<ElementId> = HashSet::new();

    for cmd in commands {
        if cmd.op.op_type == OpType::Barrier {
            if cmd.args.iter().any(|q| qubits_with_gates.contains(q)) {
                end_barriers.push(cmd.clone());
                end_barrier_qubits.extend(cmd.args.iter().cloned());
            } else {
                start_barriers.push(cmd.clone());
            }
        } else {
            if cmd.args.iter().any(|q| end_barrier_qubits.contains(q)) {
                panic!("Barrier is in the middle of a serial circuit: {:?}", cmd);
            }
            qubits_with_gates.extend(cmd.args.iter().cloned());
        }
    }

    (start_barriers, end_barriers)
}

fn apply_map<T: Clone + Into<ElementId>>(map: &[T], indices: &[usize]) -> Vec<ElementId> {
    indices.iter().map(|i| map[*i].clone().into()).collect()
}

fn parse_floats(values: Vec<String>) -> Result<Vec<f64>, ConversionError> {
    values
        .into_iter()
        .map(|s| {
            s.parse::<f64>()
                .map_err(|_| ConversionError::SymbolicParameter(s))
        })
        .collect()
}

/// Errors that can occur when converting between serial circuit and pauli graph
// TODO: q4 usage of derive_more::Error and error(ignore)
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use rstest::*;

    use crate::TketOp;
    use crate::utils::build_simple_circuit;

    fn count_t_gates_in_mermaid_string(input: &str) -> usize {
        input.matches("tket.quantum.T").count()
    }

    #[fixture]
    fn simple_circ() -> Circuit {
        build_simple_circuit(2, |circ| {
            circ.append(TketOp::Z, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            Ok(())
        })
        .unwrap()
    }

    #[fixture]
    fn hhl_circ() -> Circuit {
        build_simple_circuit(5, |circ| {
            circ.append(TketOp::H, [0])?; // h(q0)
            circ.append(TketOp::H, [1])?; // h(q1)
            circ.append(TketOp::S, [0])?; // s(q0)
            circ.append(TketOp::Z, [1])?; // z(q1)
            // mem_swap(q0, q1)
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::H, [1])?; // h(q1)
            // csdg(q1, q0)
            circ.append(TketOp::Tdg, [1])?;
            circ.append(TketOp::Tdg, [0])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::H, [0])?; // h(q0)

            circ.append(TketOp::CX, [0, 1])?; // cx(q0, q1)
            circ.append(TketOp::H, [2])?; // h(q2)
            circ.append(TketOp::H, [3])?; // h(q3)
            // cs(q3, q0)
            circ.append(TketOp::T, [3])?;
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::CX, [3, 0])?;
            circ.append(TketOp::Tdg, [0])?;
            circ.append(TketOp::CX, [3, 0])?;
            // cs(q3, q1)
            circ.append(TketOp::T, [3])?;
            circ.append(TketOp::T, [1])?;
            circ.append(TketOp::CX, [3, 1])?;
            circ.append(TketOp::Tdg, [1])?;
            circ.append(TketOp::CX, [3, 1])?;
            circ.append(TketOp::CX, [0, 1])?; // cx(q0, q1)
            circ.append(TketOp::CZ, [1, 2])?; // cz(q1, q2)

            circ.append(TketOp::H, [0])?; // h(q0)
            // cs(q1, q0)
            circ.append(TketOp::T, [1])?;
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::Tdg, [0])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::H, [1])?; // h(q1)
            // mem_swap(q0, q1)
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::CX, [0, 1])?;

            // mem_swap(q2, q3)
            circ.append(TketOp::CX, [2, 3])?;
            circ.append(TketOp::CX, [3, 2])?;
            circ.append(TketOp::CX, [2, 3])?;
            circ.append(TketOp::H, [3])?; // h(q3)
            // csdg(q3, q2)
            circ.append(TketOp::Tdg, [3])?;
            circ.append(TketOp::Tdg, [2])?;
            circ.append(TketOp::CX, [3, 2])?;
            circ.append(TketOp::T, [2])?;
            circ.append(TketOp::CX, [3, 2])?;
            circ.append(TketOp::H, [2])?; // h(q2)
            circ.append(TketOp::CX, [3, 4])?; // cx(q3, q4)
            circ.append(TketOp::X, [2])?; // x(q2)
            circ.append(TketOp::CX, [2, 4])?; // cx(q2, q4)
            circ.append(TketOp::X, [2])?; // x(q2)
            circ.append(TketOp::V, [4])?; // v(q4)
            circ.append(TketOp::T, [4])?; // t(q4)
            circ.append(TketOp::CX, [2, 4])?; // cx(q2, q4)
            circ.append(TketOp::Tdg, [4])?; // tdg(q4)
            circ.append(TketOp::CX, [2, 4])?; // cx(q2, q4)
            circ.append(TketOp::Vdg, [4])?; // vdg(q4)
            // mem_swap(q0, q1)
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::H, [1])?; // h(q1)
            // csdg(q1, q0)
            circ.append(TketOp::Tdg, [1])?;
            circ.append(TketOp::Tdg, [0])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::H, [0])?; // h(q0)

            circ.append(TketOp::H, [2])?; // h(q2)
            // cs(q3, q2)
            circ.append(TketOp::T, [3])?;
            circ.append(TketOp::T, [2])?;
            circ.append(TketOp::CX, [3, 2])?;
            circ.append(TketOp::Tdg, [2])?;
            circ.append(TketOp::CX, [3, 2])?;
            circ.append(TketOp::H, [3])?; // h(q3)
            // mem_swap(q2, q3)
            circ.append(TketOp::CX, [2, 3])?;
            circ.append(TketOp::CX, [3, 2])?;
            circ.append(TketOp::CX, [2, 3])?;

            circ.append(TketOp::CZ, [1, 2])?; // cz(q1, q2)
            circ.append(TketOp::CX, [0, 1])?; // cx(q0, q1)
            // csdg(q3, q1)
            circ.append(TketOp::Tdg, [3])?;
            circ.append(TketOp::Tdg, [1])?;
            circ.append(TketOp::CX, [3, 1])?;
            circ.append(TketOp::T, [1])?;
            circ.append(TketOp::CX, [3, 1])?;
            // csdg(q3, q0)
            circ.append(TketOp::Tdg, [3])?;
            circ.append(TketOp::Tdg, [0])?;
            circ.append(TketOp::CX, [3, 0])?;
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::CX, [3, 0])?;
            circ.append(TketOp::H, [2])?; // h(q2)
            circ.append(TketOp::H, [3])?; // h(q3)
            circ.append(TketOp::CX, [0, 1])?; // cx(q0, q1)

            circ.append(TketOp::H, [0])?; // h(q0)
            // cs(q1, q0)
            circ.append(TketOp::T, [1])?;
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::Tdg, [0])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::H, [1])?; // h(q1)
            // mem_swap(q0, q1)
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::CX, [1, 0])?;
            circ.append(TketOp::CX, [0, 1])?;
            // We can't put the measures in the simple_circuit because
            // they're not pureley quantum

            Ok(())
        })
        .unwrap()
    }

    #[rstest]
    fn hhl_test(mut hhl_circ: Circuit) {
        GlobalTResynthesis::default()
            .with_ancilla_budget(0)
            .run(hhl_circ.hugr_mut())
            .unwrap();

        let t_count = count_t_gates_in_mermaid_string(&hhl_circ.mermaid_string());

        assert_eq!(t_count, 14);
    }
}
