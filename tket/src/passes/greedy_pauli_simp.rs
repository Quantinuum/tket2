use crate::passes::guppy::NormalizeGuppyErrors;
use crate::passes::inline_funcs::InlineFuncsError;
use crate::passes::pg_convert::{ConversionError, RegisterMap, pauli_graph_to_cmds, serial_circuit_to_pauli_graph};
use crate::passes::{ComposablePass, InlineFunctionsPass, NormalizeGuppy, PassScope, WithScope};
use crate::CircuitError;
use crate::Circuit;
use crate::serialize::pytket::{
    default_decoder_config,
    default_encoder_config,
    EncodeOptions,
    EncodedCircuit,
    PytketDecodeError, PytketEncodeError,
};

use hugr::hugr::ValidationError;
use pauli_graph::{GateType, PauliGraphPass};
use basic_passes::CanonicalFormPass;
use greedy_synth::{GreedySynthPass, ParallelMode, RebaseTQEToZXPass};
use pg_optimise::{GroupCommutingOpsPass, RotationMergingPass};
use hugr::{Hugr, Node};
use std::sync::Arc;


// - `window_size` (`Option<usize>`) - Size of the sliding window for lookahead during synthesis. Default to 1280.
/// - `pool_size` (`Option<usize>`) - Number of candidate gates to maintain in the pool. Default to max(1000, 0.2*N^2) where N is the number of qubits.
/// - `top_up_size` (`Option<usize>`) - Number of candidates to add after each TQE gate. Default to max(200, pool_size / N) where N is the number of qubits.
/// - `seed` (`u64`) - Random seed for reproducible candidate sampling. Default to `0`.
/// - `parallel_mode` (`ParallelMode`) - Configuration for parallel processing of candidates. Default to `ParallelMode::Auto`.
#[derive(Clone, Debug)]
pub struct GreedyPauliSimpPass {
    scope: PassScope,
    window_size: Option<usize>,
    pool_size: Option<usize>,
    top_up_size: Option<usize>,
    seed: u64,
    parallel_mode: ParallelMode
}

impl WithScope for GreedyPauliSimpPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl Default for GreedyPauliSimpPass {
    fn default() -> Self {
        Self { 
            scope: PassScope::default(),
            window_size: None,
            pool_size: None,
            top_up_size: None,
            seed: 0,
            parallel_mode: ParallelMode::Auto
        }
    }
}

impl GreedyPauliSimpPass {
    pub fn with_window_size(mut self, window_size: usize) -> Self {
        self.window_size = Some(window_size);
        self
    }

    pub fn with_pool_size(mut self, pool_size: usize) -> Self {
        self.pool_size = Some(pool_size);
        self
    }

    pub fn with_top_up_size(mut self, top_op_size: usize) -> Self {
        self.top_up_size = Some(top_op_size);
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

impl ComposablePass<Hugr> for GreedyPauliSimpPass {
    type Error = GreedyPauliSimpErrors;
    type Result = ();
    fn run(&self, hugr: &mut Hugr) -> Result<Self::Result, Self::Error> {
        InlineFunctionsPass::default().run(hugr).unwrap();
        NormalizeGuppy::default().run(hugr)?;
        
        let encode_options = EncodeOptions::new()
            .with_subcircuits(true)
            .with_config(default_encoder_config());

        let mut encoded_circs = EncodedCircuit::new(&hugr, encode_options)?;

        for (_, serial_circ) in encoded_circs.iter_mut() {
            let register_map = RegisterMap::new(&serial_circ.qubits, &serial_circ.bits);
            let pauli_graph = serial_circuit_to_pauli_graph(serial_circ, &register_map)?;

            let canonical_pass = CanonicalFormPass::new().with_forward(true);
            let grouping_pass = GroupCommutingOpsPass::new();
            let rotation_merging_pass = RotationMergingPass::new();
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
            
            if let Some(ts) = self.top_up_size {
                synth_pass = synth_pass.with_top_up_size(ts);
            }

            let pauli_graph = canonical_pass.transform(&pauli_graph);
            let pauli_graph = rotation_merging_pass.transform(&pauli_graph);
            let pauli_graph = grouping_pass.transform(&pauli_graph);
            let pauli_graph = synth_pass.transform(&pauli_graph);
            let pauli_graph = rebase_pass.transform(&pauli_graph);

            serial_circ.commands = pauli_graph_to_cmds(pauli_graph, &register_map)?;
        }

        // encoded_circs
        //     .reassemble_inplace(
        //         circ.hugr_mut(),
        //         Some(Arc::new(default_decoder_config())),
        //     )?;
        
        encoded_circs
            .reassemble_inplace(
                hugr,
                Some(Arc::new(default_decoder_config())),
            )?;

        // circ.hugr().validate()?;
        //
        // let mermaid_string = circ.mermaid_string();
        //
        // *hugr = circ.into_hugr();

        Ok(())
    }
}

/// Errors that can occur during the global-t resynthesis
#[derive(derive_more::Error, Debug, derive_more::Display, derive_more::From)]
pub enum GreedyPauliSimpErrors {
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


#[cfg(test)]
mod tests {
    use super::*;
    use rstest::*;

    use crate::utils::build_simple_circuit;
    use crate::TketOp;

    fn count_t_gates_in_mermaid_string(input: &str) ->  usize {
        input.matches("tket.quantum.T").count()
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

            Ok(())
        })
        .unwrap()
    }

    #[rstest]
    fn hhl_test(mut hhl_circ: Circuit) {
        GreedyPauliSimpPass::default()
            .run(hhl_circ.hugr_mut())
            .unwrap();

        let t_count = count_t_gates_in_mermaid_string(&hhl_circ.mermaid_string());

        assert_eq!(t_count, 14);
    }
}
