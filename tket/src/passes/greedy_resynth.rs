//! Greedy resynthesis of a Clifford + T circuit through a Pauli graph.
//!
//! The [`GreedyResynthPass`] resynthesises a circuit by converting it to a
//! Pauli graph, applying the [`GreedySynthPass`], and converting the result
//! back into a circuit.

use crate::CircuitError;
use crate::passes::inline_funcs::InlineFuncsError;
use crate::passes::normalize::NormalizeErrors;
use crate::passes::pg_convert::{
    ConversionError, RegisterMap, pauli_graph_to_cmds, serial_circuit_to_pauli_graph,
};
use crate::passes::{ComposablePass, InlineFunctionsPass, Normalize, PassScope, WithScope};
use crate::serialize::pytket::{
    EncodeOptions, EncodedCircuit, PytketDecodeError, PytketEncodeError, default_decoder_config,
    default_encoder_config,
};

use hugr::hugr::ValidationError;
use hugr::{Hugr, Node};
use pg_canonical_form::CanonicalFormPass;
use pg_core::{GateType, PGPass};
use pg_greedy_synth::{GreedySynthPass, ParallelMode};
use pg_optimise::{GroupCommutingOpsPass, RotationMergingPass};
use pg_rebase::RebaseTQEToZXPass;
use std::sync::Arc;

// - `window_size` (`Option<usize>`) - Size of the sliding window for lookahead during synthesis. Default to 1280.
/// - `pool_size` (`Option<usize>`) - Number of candidate gates to maintain in the pool. Default to max(1000, 0.2*N^2) where N is the number of qubits.
/// - `top_up_size` (`Option<usize>`) - Number of candidates to add after each TQE gate. Default to max(200, pool_size / N) where N is the number of qubits.
/// - `seed` (`u64`) - Random seed for reproducible candidate sampling. Default to `0`.
/// - `parallel_mode` (`ParallelMode`) - Configuration for parallel processing of candidates. Default to `ParallelMode::Auto`.
#[derive(Clone, Debug)]
pub struct GreedyResynthPass {
    scope: PassScope,
    window_size: Option<usize>,
    pool_size: Option<usize>,
    top_up_size: Option<usize>,
    seed: u64,
    parallel_mode: ParallelMode,
}

impl WithScope for GreedyResynthPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl Default for GreedyResynthPass {
    fn default() -> Self {
        Self {
            scope: PassScope::default(),
            window_size: None,
            pool_size: None,
            top_up_size: None,
            seed: 0,
            parallel_mode: ParallelMode::Auto,
        }
    }
}

impl GreedyResynthPass {
    /// Sets the size of the sliding window used for lookahead during synthesis.
    ///
    /// Defaults to `1280`.
    pub fn with_window_size(mut self, window_size: usize) -> Self {
        self.window_size = Some(window_size);
        self
    }

    /// Sets the number of candidate gates to maintain in the pool.
    ///
    /// Defaults to `max(1000, 0.2 * N^2)` where `N` is the number of qubits.
    pub fn with_pool_size(mut self, pool_size: usize) -> Self {
        self.pool_size = Some(pool_size);
        self
    }

    /// Sets the number of candidate gates to add after each TQE gate.
    ///
    /// Defaults to `max(200, pool_size / N)` where `N` is the number of qubits.
    pub fn with_top_up_size(mut self, top_op_size: usize) -> Self {
        self.top_up_size = Some(top_op_size);
        self
    }

    /// Sets the random seed used to sample candidate gates.
    ///
    /// This allows reproducible synthesis across runs. Defaults to `0`.
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.seed = seed;
        self
    }

    /// Sets the parallel processing configuration for candidate synthesis.
    ///
    /// Defaults to [`ParallelMode::Auto`].
    pub fn with_parallel_mode(mut self, parallel_mode: ParallelMode) -> Self {
        self.parallel_mode = parallel_mode;
        self
    }
}

impl ComposablePass<Hugr> for GreedyResynthPass {
    type Error = GreedyResynthErrors;
    type Result = ();
    fn run(&self, hugr: &mut Hugr) -> Result<Self::Result, Self::Error> {
        let Some(root) = self.scope.root(hugr) else {
            return Ok(());
        };
        InlineFunctionsPass::default()
            .with_scope(self.scope.clone())
            .run(hugr)?;
        Normalize::default()
            .with_scope(self.scope.clone())
            .run(hugr)?;

        let encode_options = EncodeOptions::new()
            .with_subcircuits(self.scope.recursive())
            .with_config(default_encoder_config());

        let mut encoded_circs = EncodedCircuit::new_with_entrypoint(hugr, root, encode_options)?;

        for (_, serial_circ) in encoded_circs.iter_mut() {
            let register_map = RegisterMap::new(&serial_circ.qubits, &serial_circ.bits);
            let pauli_graph = serial_circuit_to_pauli_graph(serial_circ, &register_map)?;

            let canonical_pass = CanonicalFormPass::new().with_forward(true);
            let grouping_pass = GroupCommutingOpsPass::new();
            let rotation_merging_pass = RotationMergingPass::new();
            let rebase_pass = RebaseTQEToZXPass::new().with_allowed_tqes(vec![GateType::ZX]);

            let mut synth_pass = GreedySynthPass::new()
                .with_seed(self.seed)
                .with_parallel_mode(self.parallel_mode);

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
        encoded_circs.reassemble_inplace(hugr, Some(Arc::new(default_decoder_config())))?;

        Ok(())
    }
}

/// Errors that can occur during the global-t resynthesis
#[derive(derive_more::Error, Debug, derive_more::Display, derive_more::From)]
pub enum GreedyResynthErrors {
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

#[cfg(test)]
mod tests {
    use super::*;
    use hugr::HugrView;
    use rstest::rstest;

    use crate::utils::build_simple_circuit;
    use crate::{Circuit, TketOp};

    fn resynthesise(circuit: &mut Circuit) {
        let signature = circuit.circuit_signature().into_owned();

        GreedyResynthPass::default()
            .with_seed(0)
            .with_parallel_mode(ParallelMode::Off)
            .run(circuit.hugr_mut())
            .unwrap();

        circuit.hugr().validate().unwrap();
        assert_eq!(circuit.circuit_signature().as_ref(), &signature);
    }

    fn count_gate(circuit: &Circuit, gate: TketOp) -> usize {
        let gate = gate.into();
        circuit.count_ops(|op| op == &gate)
    }

    // Required because greedy resynth currently doesn't simplify
    // trivial single qubit gate sequences, so S may be synthesised as Sdg Z
    fn assert_s_on_qubit(circuit: &Circuit, target: usize) {
        let hugr = circuit.hugr();
        let input = circuit.input_node();
        let mut target_wire = (input, hugr::OutgoingPort::from(target));
        let mut phase: i32 = 0;

        assert!(target < circuit.qubit_count());
        for node in circuit.toposorted_children(circuit.parent()).unwrap() {
            let op = hugr.get_optype(node);
            phase += if op == &TketOp::S.into() {
                1
            } else if op == &TketOp::Sdg.into() {
                -1
            } else if op == &TketOp::Z.into() {
                2
            } else {
                panic!("expected a diagonal Clifford gate but got {op:?}");
            };
            assert_eq!(hugr.single_linked_output(node, 0), Some(target_wire));
            target_wire = (node, 0.into());
        }
        assert_eq!(phase.rem_euclid(4), 1);
    }

    #[rstest]
    #[case::hadamards(TketOp::H, TketOp::H, vec![0])]
    #[case::inverse_t_gates(TketOp::T, TketOp::Tdg, vec![0])]
    #[case::controlled_nots(TketOp::CX, TketOp::CX, vec![0, 1])]
    fn cancels_inverse_gates(
        #[case] gate: TketOp,
        #[case] inverse: TketOp,
        #[case] qubits: Vec<usize>,
    ) {
        let num_qubits = qubits.len();
        let mut circuit = build_simple_circuit(num_qubits, |circ| {
            circ.append(gate, qubits.clone())?;
            circ.append(inverse, qubits)?;
            Ok(())
        })
        .unwrap();
        let identity = build_simple_circuit(num_qubits, |_| Ok(())).unwrap();

        resynthesise(&mut circuit);

        assert_eq!(circuit.num_operations(), 0);
        assert_eq!(circuit, identity);
    }

    #[test]
    fn merges_two_t_gates_into_s() {
        let mut circuit = build_simple_circuit(1, |circ| {
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::T, [0])?;
            Ok(())
        })
        .unwrap();
        resynthesise(&mut circuit);

        assert_eq!(count_gate(&circuit, TketOp::T), 0);
        assert_eq!(count_gate(&circuit, TketOp::Tdg), 0);
        assert_s_on_qubit(&circuit, 0);
    }

    #[test]
    fn merges_t_gates_across_cx() {
        let mut circuit = build_simple_circuit(2, |circ| {
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::T, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            Ok(())
        })
        .unwrap();
        resynthesise(&mut circuit);

        assert_eq!(count_gate(&circuit, TketOp::T), 0);
        assert_eq!(count_gate(&circuit, TketOp::Tdg), 0);
        assert_eq!(count_gate(&circuit, TketOp::CX), 0);
        assert_s_on_qubit(&circuit, 0);
    }

    #[test]
    fn cancels_phases_across_cz_layer() {
        let mut circuit = build_simple_circuit(6, |circ| {
            for qubit in 0..6 {
                circ.append(TketOp::T, [qubit])?;
            }
            for qubit in 0..5 {
                circ.append(TketOp::CZ, [qubit, qubit + 1])?;
            }
            for qubit in 0..6 {
                circ.append(TketOp::Tdg, [qubit])?;
            }
            Ok(())
        })
        .unwrap();

        resynthesise(&mut circuit);

        // T and Tdg cancel, leaving a nontrivial Clifford circuit to synthesise
        assert_eq!(count_gate(&circuit, TketOp::T), 0);
        assert_eq!(count_gate(&circuit, TketOp::Tdg), 0);
        assert!(count_gate(&circuit, TketOp::CX) > 0);
        assert_eq!(circuit.qubit_count(), 6);
    }
}
