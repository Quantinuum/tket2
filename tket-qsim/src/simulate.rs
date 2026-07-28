use std::collections::HashMap;

use hugr::extension::prelude::qb_t;
use hugr::ops::{OpType, Value};
use hugr::std_extensions::arithmetic::float_types::ConstF64;
use hugr::{HugrView, IncomingPort, Node, OutgoingPort, PortIndex};
use petgraph::visit::{Topo, Walker as _};
use tket::TketOp;
use tket::extension::rotation::ConstRotation;
use tket_qsystem::extension::qsystem::helios::HeliosOp;
use tket_qsystem::extension::qsystem::sol::SolOp;

use crate::{Simulatable, UnitaryMatrix};

/// An error that occurred during simulation.
#[derive(Debug, Clone)]
pub enum SimulationError {
    /// An unsupported operation was encountered.
    UnsupportedOp(String),
    /// Could not resolve a parameter value for a gate.
    UnresolvedParam(String),
    /// The circuit has no qubit wires.
    NoQubits,
}

impl std::fmt::Display for SimulationError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SimulationError::UnsupportedOp(s) => write!(f, "Unsupported operation: {s}"),
            SimulationError::UnresolvedParam(s) => write!(f, "Unresolved parameter: {s}"),
            SimulationError::NoQubits => write!(f, "Circuit has no qubit wires"),
        }
    }
}

impl std::error::Error for SimulationError {}

/// Classification of a node in the circuit for simulation purposes.
enum GateInfo {
    /// A unitary gate that we can simulate.
    Unitary {
        /// The gate op (for getting the unitary).
        gate: Box<dyn Simulatable>,
        /// Input qubit ports (in order).
        qubit_ports: Vec<IncomingPort>,
        /// Input parameter ports (in order, for parametric gates).
        param_ports: Vec<IncomingPort>,
    },
    /// A non-unitary op that we skip (alloc, free, measure, etc.)
    Skip,
}

/// Simulate a quantum circuit (HUGR) and return its unitary matrix.
///
/// The circuit must be a pure quantum circuit with concrete float parameters.
/// Non-unitary operations (alloc, free, measure) are skipped.
///
/// `input_params` maps each input port index of the function to a concrete
/// f64 value. This is needed for parametric circuits where the rotation/float
/// values come from function inputs.
///
/// # Errors
///
/// Returns an error if the circuit contains unsupported operations or if
/// parameter values cannot be resolved.
pub fn simulate_circuit(
    hugr: &impl HugrView<Node = Node>,
    input_params: &HashMap<usize, f64>,
) -> Result<UnitaryMatrix, SimulationError> {
    let parent = hugr.entrypoint();
    let [input_node, _output_node] = hugr.get_io(parent).expect("No I/O nodes");

    // First pass: determine the number of qubits by counting qubit outputs
    // from the Input node.
    let num_qubits = count_qubit_inputs(hugr);
    if num_qubits == 0 {
        return Err(SimulationError::NoQubits);
    }

    // Track wire to qubit index: a qubit wire is identified by (node, out_port).
    let mut wire_to_qubit: HashMap<(Node, OutgoingPort), usize> = HashMap::new();

    // Assign qubit indices from the input node outputs.
    let mut qubit_idx = 0;
    let sig = hugr.inner_function_type().unwrap();
    for (port_idx, ty) in sig.input().iter().enumerate() {
        if *ty == qb_t() {
            let out_port = OutgoingPort::from(port_idx);
            wire_to_qubit.insert((input_node, out_port), qubit_idx);
            qubit_idx += 1;
        }
    }

    // Track wire to parameter value.
    let mut wire_to_param: HashMap<(Node, OutgoingPort), f64> = HashMap::new();

    // Assign parameter values from function inputs.
    for (port_idx, ty) in sig.input().iter().enumerate() {
        if *ty != qb_t()
            && let Some(&val) = input_params.get(&port_idx)
        {
            let out_port = OutgoingPort::from(port_idx);
            wire_to_param.insert((input_node, out_port), val);
        }
    }

    // Start with identity
    let mut total_unitary = UnitaryMatrix::identity(num_qubits);

    // Process nodes in topological order
    let scheduling_graph = hugr.scheduling_graph(parent);
    let petgraph = scheduling_graph.petgraph();
    let io_nodes = hugr.get_io(parent).unwrap();

    let topo_nodes: Vec<_> = Topo::new(petgraph)
        .iter(petgraph)
        .map(|pg_node| scheduling_graph.pg_to_node(pg_node))
        .filter(|&n| n != io_nodes[0] && n != io_nodes[1])
        .collect();

    for node in topo_nodes {
        let optype = hugr.get_optype(node);

        // Propagate parameter values through LoadConst nodes
        if let Some(val) = try_extract_const_value(hugr, node, optype) {
            let out_port = OutgoingPort::from(0usize);
            wire_to_param.insert((node, out_port), val);
            continue;
        }

        // Propagate parameter values through float arithmetic ops
        if let Some(results) = try_propagate_float_op(hugr, node, optype, &wire_to_param) {
            for (port, val) in results {
                wire_to_param.insert((node, port), val);
            }
            continue;
        }

        // Try to convert rotation to float
        if is_rotation_to_float(optype) {
            // Input is a rotation wire, output is a float wire
            if let Some(val) =
                resolve_input_param(hugr, node, IncomingPort::from(0usize), &wire_to_param)
            {
                let out_port = OutgoingPort::from(0usize);
                wire_to_param.insert((node, out_port), val);
            }
            continue;
        }

        let gate_info = classify_gate(optype);
        match gate_info {
            GateInfo::Unitary {
                gate,
                qubit_ports,
                param_ports,
            } => {
                // Resolve qubit indices
                let mut target_qubits = Vec::with_capacity(qubit_ports.len());
                for &port in &qubit_ports {
                    let (src_node, src_port) = hugr
                        .linked_outputs(node, port)
                        .next()
                        .expect("Qubit wire not connected");
                    let &qb = wire_to_qubit.get(&(src_node, src_port)).unwrap_or_else(|| {
                        panic!(
                            "Qubit wire ({src_node:?}, {src_port:?}) not tracked for node {node:?} op {:?}",
                            optype
                        )
                    });
                    target_qubits.push(qb);
                }

                // Resolve parameter values
                let mut params = Vec::with_capacity(param_ports.len());
                for &port in &param_ports {
                    let val = resolve_input_param(hugr, node, port, &wire_to_param)
                        .ok_or_else(|| {
                            SimulationError::UnresolvedParam(format!(
                                "Cannot resolve param at port {port:?} of node {node:?} ({optype:?})"
                            ))
                        })?;
                    params.push(val);
                }

                // Compute the gate unitary and expand to full system.
                let gate_unitary = gate.unitary(&params);
                let expanded = gate_unitary.expand_to_system(&target_qubits, num_qubits);
                total_unitary = expanded.matmul(&total_unitary);

                // Update wire_to_qubit: output qubit ports map to the same qubit indices.
                for (i, &port) in qubit_ports.iter().enumerate() {
                    // The output port index matches the input port index for qubit wires.
                    let out_port = OutgoingPort::from(port.index());
                    wire_to_qubit.insert((node, out_port), target_qubits[i]);
                }

                // Propagate any parameter outputs (for ops that pass through params).
                propagate_non_qubit_outputs(hugr, node, optype, &mut wire_to_param);
            }
            GateInfo::Skip => {
                // For skipped ops (alloc, free, measure), propagate qubit and param wires.
                propagate_all_wires(hugr, node, optype, &mut wire_to_qubit, &mut wire_to_param);
            }
        }
    }

    Ok(total_unitary)
}

/// Count the number of qubit-typed inputs in the dataflow graph
fn count_qubit_inputs(hugr: &impl HugrView<Node = Node>) -> usize {
    let sig = hugr.inner_function_type().unwrap();
    sig.input().iter().filter(|ty| **ty == qb_t()).count()
}

/// Try to extract a constant float value from a LoadConst or Const node.
fn try_extract_const_value(
    hugr: &impl HugrView<Node = Node>,
    node: Node,
    optype: &OpType,
) -> Option<f64> {
    // Handle LoadConst: follow the static edge to find the Const node.
    if optype.is_load_constant() {
        // The const value is linked via the static input port.
        let static_port = optype.static_input_port()?;
        let (const_node, _) = hugr.linked_outputs(node, static_port).next()?;
        let const_op = hugr.get_optype(const_node);
        return extract_f64_from_const_op(const_op);
    }
    // Handle inline Const nodes (rare but possible).
    if let Some(c) = optype.as_const() {
        return extract_f64_from_value(c.value());
    }
    None
}

/// Extract an f64 value from a Const operation.
fn extract_f64_from_const_op(optype: &OpType) -> Option<f64> {
    optype
        .as_const()
        .and_then(|c| extract_f64_from_value(c.value()))
}

/// Extract an f64 from a hugr Value (either ConstF64 or ConstRotation).
fn extract_f64_from_value(value: &Value) -> Option<f64> {
    // Try as ConstF64
    if let Some(f) = value.get_custom_value::<ConstF64>() {
        return Some(f.value());
    }
    // Try as ConstRotation (stored as half-turns)
    if let Some(r) = value.get_custom_value::<ConstRotation>() {
        return Some(r.half_turns());
    }
    None
}

/// Check if an op is a rotation-to-float conversion.
fn is_rotation_to_float(optype: &OpType) -> bool {
    // The rotation extension has a "to_halfturns" op that converts rotation → f64.
    if let Some(ext_op) = optype.as_extension_op() {
        let name = ext_op.def().name();
        name.as_str() == "to_halfturns" || name.as_str() == "from_halfturns_unchecked"
    } else {
        false
    }
}

/// Try to propagate float values through arithmetic operations.
fn try_propagate_float_op(
    hugr: &impl HugrView<Node = Node>,
    node: Node,
    optype: &OpType,
    wire_to_param: &HashMap<(Node, OutgoingPort), f64>,
) -> Option<Vec<(OutgoingPort, f64)>> {
    let ext_op = optype.as_extension_op()?;
    let name = ext_op.def().name();

    match name.as_str() {
        "fmul" => {
            let a = resolve_input_param(hugr, node, IncomingPort::from(0usize), wire_to_param)?;
            let b = resolve_input_param(hugr, node, IncomingPort::from(1usize), wire_to_param)?;
            Some(vec![(OutgoingPort::from(0usize), a * b)])
        }
        "fadd" => {
            let a = resolve_input_param(hugr, node, IncomingPort::from(0usize), wire_to_param)?;
            let b = resolve_input_param(hugr, node, IncomingPort::from(1usize), wire_to_param)?;
            Some(vec![(OutgoingPort::from(0usize), a + b)])
        }
        "fsub" => {
            let a = resolve_input_param(hugr, node, IncomingPort::from(0usize), wire_to_param)?;
            let b = resolve_input_param(hugr, node, IncomingPort::from(1usize), wire_to_param)?;
            Some(vec![(OutgoingPort::from(0usize), a - b)])
        }
        "fdiv" => {
            let a = resolve_input_param(hugr, node, IncomingPort::from(0usize), wire_to_param)?;
            let b = resolve_input_param(hugr, node, IncomingPort::from(1usize), wire_to_param)?;
            Some(vec![(OutgoingPort::from(0usize), a / b)])
        }
        "fneg" => {
            let a = resolve_input_param(hugr, node, IncomingPort::from(0usize), wire_to_param)?;
            Some(vec![(OutgoingPort::from(0usize), -a)])
        }
        _ => None,
    }
}

/// Resolve the parameter value arriving at a specific input port.
fn resolve_input_param(
    hugr: &impl HugrView<Node = Node>,
    node: Node,
    port: IncomingPort,
    wire_to_param: &HashMap<(Node, OutgoingPort), f64>,
) -> Option<f64> {
    let (src_node, src_port) = hugr.linked_outputs(node, port).next()?;
    wire_to_param.get(&(src_node, src_port)).copied()
}

/// Classify a HUGR node's operation for simulation.
fn classify_gate(optype: &OpType) -> GateInfo {
    // Try TketOp
    if let Some(op) = optype.cast::<TketOp>() {
        return classify_tket_op(op);
    }
    // Try HeliosOp
    if let Some(op) = optype.cast::<HeliosOp>() {
        return classify_helios_op(op);
    }
    // Try SolOp
    if let Some(op) = optype.cast::<SolOp>() {
        return classify_sol_op(op);
    }
    // Everything else is skipped
    GateInfo::Skip
}

fn classify_tket_op(op: TketOp) -> GateInfo {
    match op {
        TketOp::H
        | TketOp::X
        | TketOp::Y
        | TketOp::Z
        | TketOp::S
        | TketOp::Sdg
        | TketOp::T
        | TketOp::Tdg
        | TketOp::V
        | TketOp::Vdg => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize)],
            param_ports: vec![],
        },
        TketOp::Rx | TketOp::Ry | TketOp::Rz => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize)],
            param_ports: vec![IncomingPort::from(1usize)],
        },
        TketOp::CX | TketOp::CY | TketOp::CZ => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize), IncomingPort::from(1usize)],
            param_ports: vec![],
        },
        TketOp::CRz => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize), IncomingPort::from(1usize)],
            param_ports: vec![IncomingPort::from(2usize)],
        },
        TketOp::Toffoli => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![
                IncomingPort::from(0usize),
                IncomingPort::from(1usize),
                IncomingPort::from(2usize),
            ],
            param_ports: vec![],
        },
        // Non-unitary ops: skip
        TketOp::Measure
        | TketOp::MeasureFree
        | TketOp::QAlloc
        | TketOp::TryQAlloc
        | TketOp::QFree
        | TketOp::Reset => GateInfo::Skip,
        _ => GateInfo::Skip, // Other ops are skipped for now
    }
}

fn classify_helios_op(op: HeliosOp) -> GateInfo {
    match op {
        HeliosOp::PhasedX => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize)],
            param_ports: vec![IncomingPort::from(1usize), IncomingPort::from(2usize)],
        },
        HeliosOp::Rz => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize)],
            param_ports: vec![IncomingPort::from(1usize)],
        },
        HeliosOp::ZZPhase => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize), IncomingPort::from(1usize)],
            param_ports: vec![IncomingPort::from(2usize)],
        },
        // Non-unitary ops
        _ => GateInfo::Skip,
    }
}

fn classify_sol_op(op: SolOp) -> GateInfo {
    match op {
        SolOp::PhasedX => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize)],
            param_ports: vec![IncomingPort::from(1usize), IncomingPort::from(2usize)],
        },
        SolOp::Rz => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize)],
            param_ports: vec![IncomingPort::from(1usize)],
        },
        SolOp::PhasedXX => GateInfo::Unitary {
            gate: Box::new(op),
            qubit_ports: vec![IncomingPort::from(0usize), IncomingPort::from(1usize)],
            param_ports: vec![IncomingPort::from(2usize), IncomingPort::from(3usize)],
        },
        // Non-unitary ops
        _ => GateInfo::Skip,
    }
}

/// Propagate non-qubit output wires through a gate node.
fn propagate_non_qubit_outputs(
    _hugr: &impl HugrView<Node = Node>,
    _node: Node,
    _optype: &OpType,
    _wire_to_param: &mut HashMap<(Node, OutgoingPort), f64>,
) {
    // Gate ops don't produce new parameter values; they only pass through qubits.
    // No-op for now.
}

/// Propagate all wires through a skipped node.
fn propagate_all_wires(
    hugr: &impl HugrView<Node = Node>,
    node: Node,
    optype: &OpType,
    wire_to_qubit: &mut HashMap<(Node, OutgoingPort), usize>,
    wire_to_param: &mut HashMap<(Node, OutgoingPort), f64>,
) {
    let n_inputs = optype.value_input_count();
    let n_outputs = optype.value_output_count();

    // For simple pass-through ops, try to propagate qubit and param wires.
    // Heuristic: if input port i corresponds to output port i (same type),
    // propagate the wire mapping.
    let pass_through = n_inputs.min(n_outputs);
    for i in 0..pass_through {
        let in_port = IncomingPort::from(i);
        if let Some((src_node, src_port)) = hugr.linked_outputs(node, in_port).next() {
            let out_port = OutgoingPort::from(i);
            if let Some(&qb) = wire_to_qubit.get(&(src_node, src_port)) {
                wire_to_qubit.insert((node, out_port), qb);
            }
            if let Some(&val) = wire_to_param.get(&(src_node, src_port)) {
                wire_to_param.insert((node, out_port), val);
            }
        }
    }
}
