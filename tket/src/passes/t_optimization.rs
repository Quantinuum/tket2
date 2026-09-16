use crate::passes::normalize::NormalizeErrors;
use crate::passes::inline_funcs::InlineFuncsError;
use crate::passes::pg_convert::{ConversionError, RegisterMap, pauli_graph_to_cmds, serial_circuit_to_pauli_graph };
use crate::passes::{ComposablePass, InlineDFGsPass, InlineFunctionsPass, Normalize, PassScope, WithScope };
use crate::serialize::pytket::{
    default_decoder_config,
    default_encoder_config,
    EncodeOptions,
    EncodedCircuit,
    PytketDecodeError, PytketEncodeError,
};
use crate::{Circuit, CircuitError};
use hugr::hugr::ValidationError;
use hugr::hugr::hugrmut::HugrMut;

use hugr::ops::{OpTag, OpTrait};
use hugr::types::EdgeKind;
use hugr_core::hugr::internal::{HugrInternals, PortgraphNodeMap};
use pauli_graph::{BlackBoxData, GateData, GateType, Op, PauliGraph, PauliGraphPass};
use basic_passes::CanonicalFormPass;
use greedy_synth::{GreedySynthPass, ParallelMode, RebaseTQEToZXPass};
use petgraph::visit as pv;
use pg_optimise::{GroupCommutingOpsPass, RotationMergingPass};
use fast_todd::FastTODDPass;

use hugr::{Hugr, Node};
use hugr::HugrView;
use hugr::hugr::OpType as TketOp;

use tket_json_rs::circuit_json::{Command, Operation};
use tket_json_rs::register::{Bit, ElementId, Qubit};
use tket_json_rs::{OpType, SerialCircuit};

use std::sync::Arc;
use std::cell::Cell;
use std::collections::HashSet;

use std::fs::OpenOptions;
use std::io::Write;
use itertools::Itertools;
use serde_json;

use super::inline_funcs::InlineFuncsHeuristic;

// Only works for purely quantum circuits with no measurements
fn write_pytket_circ(commands: Vec<Command>, qubits:&[Qubit], filename: &str, circ_name: &str) {
        let mut file = OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(true)
        .open(filename)
        .unwrap();

    writeln!(file, "from pytket import Circuit").unwrap();
    writeln!(file, "{} = Circuit({:?})", circ_name, qubits.len()).unwrap();

    for cmd in commands {
        let pytket_cmd = to_pytket(&cmd, qubits, circ_name);
        writeln!(file, "{}", pytket_cmd).unwrap();
    }
}

fn to_pytket(cmd: &Command<String>, qubits: &[Qubit], circ_name: &str) -> String {
    let op = match cmd.op.op_type {
        OpType::H => "H",
        OpType::S => "S",
        OpType::Sdg => "Sdg",
        OpType::Z => "Z",
        OpType::V => "V",
        OpType::Vdg => "Vdg",
        OpType::X => "X",
        OpType::Y => "Y",
        OpType::Z => "Z",
        OpType::CX => "CX",
        OpType::CY => "CY",
        OpType::CZ => "CZ",
        OpType::CRz => "CRz",
        OpType::T => "T",
        OpType::Tdg => "Tdg",
        // OpType::Barrier => return "".to_string(),
        OpType::Barrier => return serde_json::to_string(cmd).unwrap(),
        OpType::Measure => "PYTKET COMPARISON DOES NOT SUPPORT MEASUREMENT",
        _ => return panic!("cannot convert to pytket due to: {}", cmd.op.op_type),
    };

    let args = cmd
        .args
        .iter()
        .filter_map(|id| {
            qubits.iter()
                .position(|q| q.id == *id)
                .map(|n| n.to_string())
        })
        .collect::<Vec<_>>()
        .join(", ");

    format!("{}.{}({})", circ_name, op, args)
}

#[derive(Clone, Debug)]
pub struct TOptimizationPass {
    scope: PassScope,
    ancilla_budget: Option<usize>,
    window_size: Option<usize>,
    pool_size: Option<usize>,
    top_op_size: Option<usize>,
    seed: u64,
    parallel_mode: ParallelMode
}

impl WithScope for TOptimizationPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl Default for TOptimizationPass {
    fn default() -> Self {
        Self { 
            scope: PassScope::default(),
            ancilla_budget: None,
            window_size: None,
            pool_size: None,
            top_op_size: None,
            seed: 0,
            parallel_mode: ParallelMode::Auto
        }
    }
}

impl TOptimizationPass {
    pub fn with_ancilla_budget(mut self, ancilla_budget: usize) -> Self {
        self.ancilla_budget = Some(ancilla_budget);
        self
    }

    pub fn with_window_size(mut self, window_size: usize) -> Self {
        self.window_size = Some(window_size);
        self
    }

    pub fn with_pool_size(mut self, pool_size: usize) -> Self {
        self.pool_size = Some(pool_size);
        self
    }

    pub fn with_top_up_size(mut self, top_op_size: usize) -> Self {
        self.top_op_size = Some(top_op_size);
        self
    }
    
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    pub fn with_parallel_mode(mut self, parallel_mode: ParallelMode) -> Self {
        self.parallel_mode = parallel_mode;
        self
    }
}

pub fn count_order_edges(h: &Hugr) -> (usize, usize) {
    let mut total_edges = 0;
    let mut max_per_node = 0;

    for n in h.nodes() {
        if OpTag::DataflowChild.is_superset(h.get_optype(n).tag()) {
            let p = h.get_optype(n).other_input_port();
            if let Some(p) = p {
                let c = h.linked_outputs(n, p).count();
                total_edges += c;
                max_per_node = max_per_node.max(c);
            }
        }
    }

    (total_edges, max_per_node)
}

impl ComposablePass<Hugr> for TOptimizationPass {
    type Error = GlobalTResynthesisErrors;
    type Result = ();
    fn run(&self, hugr: &mut Hugr) -> Result<Self::Result, Self::Error> {
        InlineFunctionsPass::default().run(hugr).unwrap();

        Normalize::default()
            .run(hugr)?;

        let mut circ = Circuit::try_new(hugr.clone())?;

        let encode_options = EncodeOptions::new()
            .with_subcircuits(true)
            .with_config(default_encoder_config());

        let mut encoded_circs = EncodedCircuit::new(&hugr, encode_options)?;

        for (_, serial_circ) in encoded_circs.iter_mut() {
            let register_map = RegisterMap::new(&serial_circ.qubits, &serial_circ.bits);
            let pauli_graph = serial_circuit_to_pauli_graph(serial_circ, &register_map)?;

            write_pytket_circ(serial_circ.commands.clone(), &serial_circ.qubits, "before.py", "circ_1");

            let canonical_pass = CanonicalFormPass::new().with_forward(true);
            let grouping_pass = GroupCommutingOpsPass::new();
            let rotation_merging_pass = RotationMergingPass::new();
            // let fast_todd_pass = FastTODDPass::new()
            //    .with_ancilla_budget(self.ancilla_budget);
            let rebase_pass = RebaseTQEToZXPass::new()
                .with_allowed_tqes(vec![GateType::ZX]);
            
            let mut synth_pass = GreedySynthPass::new()
                .with_seed(self.seed)
                .with_parallel_mode(self.parallel_mode.clone());
            
            if let Some(ws) = self.window_size {
                synth_pass = synth_pass.with_window_size(ws);
            }
            
            if let Some(ps) = self.pool_size {
                synth_pass = synth_pass.with_pool_size(ps);
            }
            
            if let Some(ts) = self.top_op_size {
                synth_pass = synth_pass.with_top_up_size(ts);
            }

            let pauli_graph = canonical_pass.transform(&pauli_graph);
            let pauli_graph = rotation_merging_pass.transform(&pauli_graph);
            // let pauli_graph = fast_todd_pass.transform(&pauli_graph);
            let pauli_graph = grouping_pass.transform(&pauli_graph);
            let pauli_graph = synth_pass.transform(&pauli_graph);
            let pauli_graph = rebase_pass.transform(&pauli_graph);

            serial_circ.commands = pauli_graph_to_cmds(pauli_graph, &register_map)?;
            
            write_pytket_circ(serial_circ.commands.clone(), &serial_circ.qubits, "after.py", "circ_2");
        }

        encoded_circs
            .reassemble_inplace(
                circ.hugr_mut(),
                Some(Arc::new(default_decoder_config())),
            )?;

        circ.hugr().validate()?;

        let mermaid_string = circ.mermaid_string();

        *hugr = circ.into_hugr();

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
    NormalizeError(NormalizeErrors),
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


// #[cfg(test)]
// mod tests {
//     use super::*;
//     use rstest::*;
//
//     use crate::utils::build_simple_circuit;
//     use crate::TketOp;
//
//     fn count_t_gates_in_mermaid_string(input: &str) ->  usize {
//         input.matches("tket.quantum.T").count()
//     }
//
//     #[fixture]
//     fn identity_circ() -> Circuit {
//         build_simple_circuit(3, |circ| {
//             circ.append(TketOp::Z, [0])?;
//             Ok(())
//         })
//         .unwrap()
//     }
//
//     #[fixture]
//     fn debug_circ() -> Circuit {
//         build_simple_circuit(2, |circ| {
//             circ.append(TketOp::Z, [0])?;
//             circ.append(TketOp::S, [0])?;
//             circ.append(TketOp::H, [1])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::H, [1])?;
//             Ok(())
//         })
//         .unwrap()
//     }
//
//     #[fixture]
//     fn local_clifford_circ() -> Circuit {
//         build_simple_circuit(3, |circ| {
//             circ.append(TketOp::Z, [0])?;
//             circ.append(TketOp::S, [0])?;
//             circ.append(TketOp::X, [1])?;
//             circ.append(TketOp::H, [1])?;
//             circ.append(TketOp::S, [1])?;
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::S, [1])?;
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::H, [0])?;
//             Ok(())
//         })
//         .unwrap()
//     }
//     
//     #[fixture]
//     fn shared_clifford_circ() -> Circuit {
//         build_simple_circuit(3, |circ| {
//             circ.append(TketOp::Z, [0])?;
//             circ.append(TketOp::S, [0])?;
//             circ.append(TketOp::X, [1])?;
//             circ.append(TketOp::H, [1])?;
//             circ.append(TketOp::S, [1])?;
//             // circ.append(TketOp::CX, [0, 1])?;
//             // circ.append(TketOp::S, [1])?;
//             // circ.append(TketOp::CX, [0, 1])?;
//             // circ.append(TketOp::CX, [1, 0])?;
//             // circ.append(TketOp::H, [0])?;
//             Ok(())
//         })
//         .unwrap()
//     }
//
//     #[fixture]
//     fn simple_circ() -> Circuit {
//         build_simple_circuit(3, |circ| {
//             // circ.append(TketOp::Tdg, [2])?;
//             circ.append(TketOp::H, [0])?;
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::T, [1])?;
//             circ.append(TketOp::H, [1])?;
//             circ.append(TketOp::Tdg, [0])?;
//             circ.append(TketOp::H, [2])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::T, [1])?;
//             circ.append(TketOp::H, [2])?;
//             Ok(())
//         })
//         .unwrap()
//     }
//
//     #[fixture]
//     fn hhl_circ() -> Circuit {
//         build_simple_circuit(5, |circ| {
//             circ.append(TketOp::H, [0])?; // h(q0)
//             circ.append(TketOp::H, [1])?; // h(q1)
//             circ.append(TketOp::S, [0])?; // s(q0)
//             circ.append(TketOp::Z, [1])?; // z(q1)
//             // mem_swap(q0, q1)
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::H, [1])?; // h(q1)
//             // csdg(q1, q0)
//             circ.append(TketOp::Tdg, [1])?;
//             circ.append(TketOp::Tdg, [0])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::T, [0])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::H, [0])?; // h(q0)
//
//             circ.append(TketOp::CX, [0, 1])?; // cx(q0, q1)
//             circ.append(TketOp::H, [2])?; // h(q2)
//             circ.append(TketOp::H, [3])?; // h(q3)
//             // cs(q3, q0)
//             circ.append(TketOp::T, [3])?;
//             circ.append(TketOp::T, [0])?;
//             circ.append(TketOp::CX, [3, 0])?;
//             circ.append(TketOp::Tdg, [0])?;
//             circ.append(TketOp::CX, [3, 0])?;
//             // cs(q3, q1)
//             circ.append(TketOp::T, [3])?;
//             circ.append(TketOp::T, [1])?;
//             circ.append(TketOp::CX, [3, 1])?;
//             circ.append(TketOp::Tdg, [1])?;
//             circ.append(TketOp::CX, [3, 1])?;
//             circ.append(TketOp::CX, [0, 1])?; // cx(q0, q1)
//             circ.append(TketOp::CZ, [1, 2])?; // cz(q1, q2)
//
//             circ.append(TketOp::H, [0])?; // h(q0)
//             // cs(q1, q0)
//             circ.append(TketOp::T, [1])?;
//             circ.append(TketOp::T, [0])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::Tdg, [0])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::H, [1])?; // h(q1)
//             // mem_swap(q0, q1)
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::CX, [0, 1])?;
//
//             // mem_swap(q2, q3)
//             circ.append(TketOp::CX, [2, 3])?;
//             circ.append(TketOp::CX, [3, 2])?;
//             circ.append(TketOp::CX, [2, 3])?;
//             circ.append(TketOp::H, [3])?; // h(q3)
//             // csdg(q3, q2)
//             circ.append(TketOp::Tdg, [3])?;
//             circ.append(TketOp::Tdg, [2])?;
//             circ.append(TketOp::CX, [3, 2])?;
//             circ.append(TketOp::T, [2])?;
//             circ.append(TketOp::CX, [3, 2])?;
//             circ.append(TketOp::H, [2])?; // h(q2)
//             circ.append(TketOp::CX, [3, 4])?; // cx(q3, q4)
//             circ.append(TketOp::X, [2])?; // x(q2)
//             circ.append(TketOp::CX, [2, 4])?; // cx(q2, q4)
//             circ.append(TketOp::X, [2])?; // x(q2)
//             circ.append(TketOp::V, [4])?; // v(q4)
//             circ.append(TketOp::T, [4])?; // t(q4)
//             circ.append(TketOp::CX, [2, 4])?; // cx(q2, q4)
//             circ.append(TketOp::Tdg, [4])?; // tdg(q4)
//             circ.append(TketOp::CX, [2, 4])?; // cx(q2, q4)
//             circ.append(TketOp::Vdg, [4])?; // vdg(q4)
//             // mem_swap(q0, q1)
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::H, [1])?; // h(q1)
//             // csdg(q1, q0)
//             circ.append(TketOp::Tdg, [1])?;
//             circ.append(TketOp::Tdg, [0])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::T, [0])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::H, [0])?; // h(q0)
//
//             circ.append(TketOp::H, [2])?; // h(q2)
//             // cs(q3, q2)
//             circ.append(TketOp::T, [3])?;
//             circ.append(TketOp::T, [2])?;
//             circ.append(TketOp::CX, [3, 2])?;
//             circ.append(TketOp::Tdg, [2])?;
//             circ.append(TketOp::CX, [3, 2])?;
//             circ.append(TketOp::H, [3])?; // h(q3)
//             // mem_swap(q2, q3)
//             circ.append(TketOp::CX, [2, 3])?;
//             circ.append(TketOp::CX, [3, 2])?;
//             circ.append(TketOp::CX, [2, 3])?;
//
//             circ.append(TketOp::CZ, [1, 2])?; // cz(q1, q2)
//             circ.append(TketOp::CX, [0, 1])?; // cx(q0, q1)
//             // csdg(q3, q1)
//             circ.append(TketOp::Tdg, [3])?;
//             circ.append(TketOp::Tdg, [1])?;
//             circ.append(TketOp::CX, [3, 1])?;
//             circ.append(TketOp::T, [1])?;
//             circ.append(TketOp::CX, [3, 1])?;
//             // csdg(q3, q0)
//             circ.append(TketOp::Tdg, [3])?;
//             circ.append(TketOp::Tdg, [0])?;
//             circ.append(TketOp::CX, [3, 0])?;
//             circ.append(TketOp::T, [0])?;
//             circ.append(TketOp::CX, [3, 0])?;
//             circ.append(TketOp::H, [2])?; // h(q2)
//             circ.append(TketOp::H, [3])?; // h(q3)
//             circ.append(TketOp::CX, [0, 1])?; // cx(q0, q1)
//
//             circ.append(TketOp::H, [0])?; // h(q0)
//             // cs(q1, q0)
//             circ.append(TketOp::T, [1])?;
//             circ.append(TketOp::T, [0])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::Tdg, [0])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::H, [1])?; // h(q1)
//             // mem_swap(q0, q1)
//             circ.append(TketOp::CX, [0, 1])?;
//             circ.append(TketOp::CX, [1, 0])?;
//             circ.append(TketOp::CX, [0, 1])?;
//
//             Ok(())
//         })
//         .unwrap()
//     }
//
//     #[rstest]
//     fn hhl_test(mut hhl_circ: Circuit) {
//         GlobalTResynthesis::default()
//             .with_ancilla_budget(0)
//             .run(hhl_circ.hugr_mut())
//             .unwrap();
//
//         let t_count = count_t_gates_in_mermaid_string(&hhl_circ.mermaid_string());
//
//         assert_eq!(t_count, 14);
//     }
// }
