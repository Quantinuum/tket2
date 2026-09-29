//! Resynthesis of a Clifford + T circuit through a Pauli graph.
//!
//! The [`PauliGraphResynthesis`] pass resynthesizes a circuit by converting it to a
//! Pauli graph, applying the [`GreedySynthPass`], and converting the result
//! back into a circuit.

use crate::passes::inline_funcs::InlineFuncsError;
use crate::passes::normalize::NormalizeErrors;
use crate::passes::pg_convert::{
    ConversionError, RegisterMap, pauli_graph_to_cmds, serial_circuit_to_pauli_graph,
};
use crate::passes::{ComposablePass, InlineFunctionsPass, Normalize, PassScope, WithScope};
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::extension::PytketDecoder;
use crate::serialize::pytket::{
    EncodeOptions, EncodedCircuit, PytketDecodeError, PytketEncodeError, default_decoder_config,
    default_encoder_config,
};
use crate::{CircuitError, TketOp, metadata};

use hugr::builder::{Dataflow, DataflowSubContainer, SubContainer};
use hugr::extension::prelude::{bool_t, qb_t};
use hugr::hugr::ValidationError;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::handle::NodeHandle;
use hugr::types::Signature;
use hugr::{Hugr, HugrView, Node, type_row};
use pg_canonical_form::CanonicalFormPass;
use pg_core::{GateType, Op, PGPass, PauliGraph};
use pg_greedy_synth::{GreedySynthPass, ParallelMode};
use pg_optimise::{GroupCommutingOpsPass, RotationMergingPass};
use pg_rebase::RebaseTQEToZXPass;
use pg_t_optimize::TOptimizationPass;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use tket_json_rs::circuit_json::Operation;
use tket_json_rs::register::{Bit, ElementId};
use tket_json_rs::{OpType as SerialOpType, SerialCircuit};

const ANCILLA_MARKER: &str = "__tket_ancilla";

/// Resynthesize a Clifford + T circuit by converting it to a Pauli graph and applying various
/// optimisation techniques such as:
/// - phase folding
/// - a synthesis algorithm from pauli graph to Clifford + T aimed at reducing the number of 2
///   qubit gates
///
///
/// - `window_size` (`Option<usize>`) - Size of the sliding window for lookahead during synthesis. Default to 1280.
/// - `pool_size` (`Option<usize>`) - Number of candidate gates to maintain in the pool. Default to max(1000, 0.2*N^2) where N is the number of qubits.
/// - `top_up_size` (`Option<usize>`) - Number of candidates to add after each TQE gate. Default to max(200, pool_size / N) where N is the number of qubits.
/// - `seed` (`u64`) - Random seed for reproducible candidate sampling. Default to `0`.
/// - `parallel_mode` (`ParallelMode`) - Configuration for parallel processing of candidates. Default to `ParallelMode::Auto`.
///
/// Explicit sizes must be greater than zero. Invalid sizes cause `run` to return
/// [`PauliGraphResynthesisErrors::InvalidParameters`] before modifying the circuit.
#[derive(Clone, Debug)]
pub struct PauliGraphResynthesis {
    scope: PassScope,
    t_optimization: bool,
    ancilla_budget: Option<usize>,
    window_size: Option<usize>,
    pool_size: Option<usize>,
    top_up_size: Option<usize>,
    seed: u64,
    parallel_mode: ParallelMode,
}

impl WithScope for PauliGraphResynthesis {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl Default for PauliGraphResynthesis {
    fn default() -> Self {
        Self {
            scope: PassScope::default(),
            t_optimization: false,
            ancilla_budget: None,
            window_size: None,
            pool_size: None,
            top_up_size: None,
            seed: 0,
            parallel_mode: ParallelMode::Auto,
        }
    }
}

impl PauliGraphResynthesis {
    /// Enables T-optimization.
    ///
    /// Defaults to `false`.
    pub fn with_t_optimization(mut self, t_optimization: bool) -> Self {
        self.t_optimization = t_optimization;
        self
    }

    /// Sets the number of ancilla qubits to use for T-optimization.
    ///
    /// Defaults to the number of Hadamard gates in each dataflow region.
    /// When T-optimization is not enabled, this parameter is ignored.
    pub fn with_ancilla_budget(mut self, ancilla_budget: usize) -> Self {
        self.ancilla_budget = Some(ancilla_budget);
        self
    }

    /// Sets the size of the sliding window used for lookahead during synthesis.
    ///
    /// Defaults to `1280`. Must be greater than zero.
    pub fn with_window_size(mut self, window_size: usize) -> Self {
        self.window_size = Some(window_size);
        self
    }

    /// Sets the number of candidate gates to maintain in the pool.
    ///
    /// Defaults to `max(1000, 0.2 * N^2)` where `N` is the number of qubits.
    /// Must be greater than zero.
    pub fn with_pool_size(mut self, pool_size: usize) -> Self {
        self.pool_size = Some(pool_size);
        self
    }

    /// Sets the number of candidate gates to add after each TQE gate.
    ///
    /// Defaults to `max(200, pool_size / N)` where `N` is the number of qubits.
    /// Must be greater than zero.
    pub fn with_top_up_size(mut self, top_up_size: usize) -> Self {
        self.top_up_size = Some(top_up_size);
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

impl ComposablePass<Hugr> for PauliGraphResynthesis {
    type Error = PauliGraphResynthesisErrors;
    type Result = ();
    fn run(&self, hugr: &mut Hugr) -> Result<Self::Result, Self::Error> {
        for (parameter, size) in [
            ("window_size", self.window_size),
            ("pool_size", self.pool_size),
            ("top_up_size", self.top_up_size),
        ] {
            if size == Some(0) {
                return Err(PauliGraphResynthesisErrors::InvalidParameters { parameter });
            }
        }

        let Some(root) = self.scope.root(hugr) else {
            return Ok(());
        };
        InlineFunctionsPass::default()
            .with_scope(self.scope.clone())
            .run(hugr)?;
        Normalize::default()
            .with_scope(self.scope.clone())
            .run(hugr)?;

        if self.t_optimization {
            let regions: Vec<_> = self
                .scope
                .regions(hugr)
                .filter(|&region| hugr.get_io(region).is_some())
                .collect();
            for region in regions {
                let budget = self.ancilla_budget.unwrap_or_else(|| {
                    hugr.children(region)
                        .filter(|&node| hugr.get_optype(node) == &TketOp::H.into())
                        .count()
                });
                for _ in 0..budget {
                    let alloc = hugr.add_node_with_parent(region, TketOp::QAlloc);
                    let reset = hugr.add_node_with_parent(region, TketOp::Reset);
                    let free = hugr.add_node_with_parent(region, TketOp::QFree);
                    hugr.connect(alloc, 0, reset, 0);
                    hugr.connect(reset, 0, free, 0);
                    hugr.set_metadata::<metadata::PytketOpGroup>(reset, ANCILLA_MARKER);
                }
            }
        }

        let encode_options = EncodeOptions::new()
            .with_subcircuits(self.scope.recursive())
            .with_config(default_encoder_config());

        let mut encoded_circs = EncodedCircuit::new_with_entrypoint(hugr, root, encode_options)?;

        let mut ancillas: HashMap<Node, HashSet<ElementId>> = HashMap::new();
        if self.t_optimization {
            for (region, serial_circ) in encoded_circs.iter_mut() {
                let registers = ancillas.entry(region).or_default();
                serial_circ.commands.retain(|cmd| {
                    if cmd.opgroup.as_deref() == Some(ANCILLA_MARKER) {
                        registers.insert(cmd.args[0].clone());
                        false
                    } else {
                        true
                    }
                });
            }
        }

        for (region, serial_circ) in encoded_circs.iter_mut() {
            if self.t_optimization {
                let registers = &ancillas[&region];
                serial_circ
                    .qubits
                    .sort_by_key(|q| registers.contains(&q.id));
            }
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
            let pauli_graph = if self.t_optimization {
                let registers = &ancillas[&region];
                let budget = serial_circ
                    .qubits
                    .iter()
                    .filter(|q| registers.contains(&q.id))
                    .count();

                let optimized = TOptimizationPass::new()
                    .with_ancilla_budget(budget)
                    .with_first_bit(serial_circ.bits.len())
                    .transform(&pauli_graph);
                // Lower the new measurements, resets and conditional corrections
                // into the operations accepted by greedy synthesis.
                let optimized = canonical_pass.transform(&optimized);
                allocate_measurement_bits(serial_circ, &optimized);
                optimized
            } else {
                pauli_graph
            };

            let pauli_graph = grouping_pass.transform(&pauli_graph);
            let pauli_graph = synth_pass.transform(&pauli_graph);
            let pauli_graph = rebase_pass.transform(&pauli_graph);

            let register_map = RegisterMap::new(&serial_circ.qubits, &serial_circ.bits);
            serial_circ.commands = pauli_graph_to_cmds(pauli_graph, &register_map)?;
        }
        let mut decoder_config = default_decoder_config();
        decoder_config.add_decoder(ResynthesisDecoder);
        encoded_circs.reassemble_inplace(hugr, Some(Arc::new(decoder_config)))?;

        Ok(())
    }
}

/// Decode synthesized swaps and single-bit Clifford corrections into native HUGR.
struct ResynthesisDecoder;

impl PytketDecoder for ResynthesisDecoder {
    fn op_types(&self) -> Vec<SerialOpType> {
        vec![SerialOpType::Conditional, SerialOpType::SWAP]
    }

    fn op_to_hugr<'h>(
        &self,
        op: &Operation,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
        _opgroup: Option<&str>,
        decoder: &mut PytketDecoderContext<'h>,
    ) -> Result<DecodeStatus, PytketDecodeError> {
        if op.op_type == SerialOpType::SWAP {
            if qubits.len() != 2 || !bits.is_empty() || !params.is_empty() {
                return Err(PytketDecodeError::custom("Unexpected arguments for SWAP"));
            }
            let tracked = decoder.find_typed_wires(&[qb_t(), qb_t()], qubits, bits, params)?;
            let swap = decoder
                .builder
                .dfg_builder(
                    Signature::new(vec![qb_t(); 2], vec![qb_t(); 2]),
                    tracked.value_wires(),
                )
                .map_err(PytketDecodeError::custom)?;
            let [a, b] = swap.input_wires_arr();
            let node = swap
                .finish_with_outputs([b, a])
                .map_err(PytketDecodeError::custom)?
                .node();
            decoder.register_node_outputs(node, qubits.iter().cloned(), [])?;
            return Ok(DecodeStatus::Success);
        }
        let condition = op.conditional.as_ref().ok_or_else(|| {
            PytketDecodeError::custom("Missing condition on a conditional correction")
        })?;
        if condition.width != 1 || condition.value > 1 || bits.len() != 1 {
            return Err(PytketDecodeError::custom(
                "Only single-bit conditional corrections are supported",
            ));
        }

        let (gate, arity) = match condition.op.op_type {
            SerialOpType::H => (Some(TketOp::H), 1),
            SerialOpType::X => (Some(TketOp::X), 1),
            SerialOpType::Y => (Some(TketOp::Y), 1),
            SerialOpType::Z => (Some(TketOp::Z), 1),
            SerialOpType::S => (Some(TketOp::S), 1),
            SerialOpType::Sdg => (Some(TketOp::Sdg), 1),
            SerialOpType::V => (Some(TketOp::V), 1),
            SerialOpType::Vdg => (Some(TketOp::Vdg), 1),
            SerialOpType::CX => (Some(TketOp::CX), 2),
            SerialOpType::CY => (Some(TketOp::CY), 2),
            SerialOpType::CZ => (Some(TketOp::CZ), 2),
            // SWAP needs only crossed wires inside the selected branch.
            SerialOpType::SWAP => (None, 2),
            _ => {
                return Err(PytketDecodeError::custom(format!(
                    "Unsupported conditional correction gate: {:?}",
                    condition.op.op_type,
                )));
            }
        };
        if qubits.len() != arity
            || !params.is_empty()
            || condition.op.params.as_ref().is_some_and(|p| !p.is_empty())
        {
            return Err(PytketDecodeError::custom(
                "Unexpected arguments for conditional correction",
            ));
        }

        let types = [vec![bool_t()], vec![qb_t(); arity]].concat();
        let tracked = decoder.find_typed_wires(&types, qubits, bits, &[])?;
        let mut wires = tracked.value_wires();
        let control = wires.next().unwrap();
        let targets: Vec<_> = wires.map(|wire| (qb_t(), wire)).collect();
        let mut conditional = decoder
            .builder
            .conditional_builder(
                (vec![type_row![]; 2], control),
                targets,
                vec![qb_t(); arity].into(),
            )
            .map_err(PytketDecodeError::custom)?;

        for value in 0..2 {
            let mut branch = conditional
                .case_builder(value)
                .map_err(PytketDecodeError::custom)?;
            let mut inputs: Vec<_> = branch.input_wires().collect();
            let outputs = if value == condition.value as usize {
                if let Some(gate) = gate {
                    branch
                        .add_dataflow_op(gate, inputs)
                        .map_err(PytketDecodeError::custom)?
                        .outputs()
                        .collect()
                } else {
                    inputs.reverse();
                    inputs
                }
            } else {
                inputs
            };
            branch
                .finish_with_outputs(outputs)
                .map_err(PytketDecodeError::custom)?;
        }

        let node = conditional
            .finish_sub_container()
            .map_err(PytketDecodeError::custom)?
            .node();
        // The condition bit is read-only; subsequent corrections may reuse it.
        decoder.register_node_outputs(node, qubits.iter().cloned(), [])?;
        Ok(DecodeStatus::Success)
    }
}

/// Allocate serial registers for measurement results introduced by T optimization.
fn allocate_measurement_bits(circuit: &mut SerialCircuit, graph: &PauliGraph) {
    let num_bits = graph
        .get_ops()
        .iter()
        .filter_map(|op| match op {
            Op::Measure { data } => Some(data.get_cbit() + 1),
            _ => None,
        })
        .max()
        .unwrap_or(circuit.bits.len());

    let mut used_ids: HashSet<_> = circuit
        .bits
        .iter()
        .map(|b| b.id.clone())
        .chain(circuit.qubits.iter().map(|q| q.id.clone()))
        .collect();
    let mut index = 0;
    while circuit.bits.len() < num_bits {
        let id = ElementId("__tket_measurement".to_owned(), vec![index]);
        index += 1;
        if used_ids.insert(id.clone()) {
            circuit.bits.push(Bit { id });
        }
    }
}

/// Errors that can occur during Pauli graph resynthesis.
#[derive(derive_more::Error, Debug, derive_more::Display, derive_more::From)]
pub enum PauliGraphResynthesisErrors {
    /// An explicitly configured size is zero.
    #[display("Invalid parameters: {parameter} must be greater than zero")]
    InvalidParameters {
        /// Name of the invalid size parameter.
        parameter: &'static str,
    },
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

    fn resynthesize(circuit: &mut Circuit) {
        let signature = circuit.circuit_signature().into_owned();

        PauliGraphResynthesis::default()
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

    fn assert_no_opaque_ops(hugr: &Hugr) {
        for node in hugr.nodes() {
            if let Some(op) = hugr.get_optype(node).as_extension_op() {
                assert_ne!(
                    op.extension_id(),
                    &crate::extension::TKET1_EXTENSION_ID,
                    "{op:?}"
                );
            }
        }
    }

    #[rstest]
    fn resynthesizes_with_ancillas(#[values(Some(1), Some(2), None)] budget: Option<usize>) {
        let mut circuit = build_simple_circuit(1, |circ| {
            for _ in 0..4 {
                circ.append(TketOp::T, [0])?;
                circ.append(TketOp::H, [0])?;
            }
            Ok(())
        })
        .unwrap();
        let signature = circuit.circuit_signature().into_owned();
        let mut pass = PauliGraphResynthesis::default()
            .with_t_optimization(true)
            .with_parallel_mode(ParallelMode::Off);
        pass.ancilla_budget = budget;
        pass.run(circuit.hugr_mut()).unwrap();
        circuit.hugr().validate().unwrap();
        assert_eq!(circuit.circuit_signature().as_ref(), &signature);
        assert!(count_gate(&circuit, TketOp::Measure) > 0);
        assert!(
            circuit
                .hugr()
                .nodes()
                .any(|n| circuit.hugr().get_optype(n).is_conditional())
        );
        assert_no_opaque_ops(circuit.hugr());
    }

    fn conditional_circuit(gate: SerialOpType, arity: usize, value: u32) -> SerialCircuit {
        use tket_json_rs::circuit_json::{Command, Conditional};
        let mut circuit = SerialCircuit::new(None, "0".to_owned());
        circuit.qubits = (0..3)
            .map(|i| ElementId("q".to_owned(), vec![i]).into())
            .collect();
        circuit.bits = vec![ElementId("c".to_owned(), vec![0]).into()];
        circuit.commands.push(Command {
            op: Operation::from_optype(SerialOpType::Measure),
            args: vec![circuit.qubits[0].id.clone(), circuit.bits[0].id.clone()],
            opgroup: None,
        });
        // Two corrections share a measurement, with reversed target order.
        for _ in 0..2 {
            let mut op = Operation::from_optype(SerialOpType::Conditional);
            op.conditional = Some(Conditional {
                op: Box::new(Operation::from_optype(gate)),
                width: 1,
                value,
            });
            let mut args = vec![circuit.bits[0].id.clone()];
            args.extend(
                circuit
                    .qubits
                    .iter()
                    .rev()
                    .take(arity)
                    .map(|q| q.id.clone()),
            );
            circuit.commands.push(Command {
                op,
                args,
                opgroup: None,
            });
        }
        circuit
    }

    fn decode_conditionals(circuit: &SerialCircuit) -> Result<Hugr, PytketDecodeError> {
        use crate::serialize::pytket::{DecodeOptions, load_tk1_json_str};
        let mut config = default_decoder_config();
        config.add_decoder(ResynthesisDecoder);
        load_tk1_json_str(
            &serde_json::to_string(circuit).unwrap(),
            DecodeOptions::new().with_config(Arc::new(config)),
        )
    }

    #[rstest]
    #[case::x(SerialOpType::X, Some(TketOp::X), 1)]
    #[case::cx(SerialOpType::CX, Some(TketOp::CX), 2)]
    #[case::swap(SerialOpType::SWAP, None, 2)]
    fn decodes_native_conditions(
        #[case] gate: SerialOpType,
        #[case] native_gate: Option<TketOp>,
        #[case] arity: usize,
        #[values(0, 1)] value: u32,
    ) {
        let hugr = decode_conditionals(&conditional_circuit(gate, arity, value)).unwrap();
        hugr.validate().unwrap();
        assert_no_opaque_ops(&hugr);
        let measure = hugr
            .nodes()
            .find(|&n| hugr.get_optype(n) == &TketOp::Measure.into())
            .unwrap();
        let conditions: Vec<_> = hugr
            .nodes()
            .filter(|&n| hugr.get_optype(n).is_conditional())
            .collect();
        assert_eq!(conditions.len(), 2);
        let [input, _] = hugr.get_io(hugr.entrypoint()).unwrap();
        for port in 0..arity {
            assert_eq!(
                hugr.single_linked_output(conditions[0], port + 1),
                Some((input, (2 - port).into()))
            );
        }
        for &condition in &conditions {
            assert_eq!(
                hugr.single_linked_output(condition, 0),
                Some((measure, 1.into()))
            );
            for (index, case) in hugr.children(condition).enumerate() {
                let [input, output] = hugr.get_io(case).unwrap();
                let gates: Vec<_> = hugr
                    .children(case)
                    .filter(|&n| n != input && n != output)
                    .collect();
                if index == value as usize
                    && let Some(native_gate) = native_gate
                {
                    assert_eq!(gates.len(), 1);
                    assert_eq!(hugr.get_optype(gates[0]), &native_gate.into());
                    for port in 0..arity {
                        assert_eq!(
                            hugr.single_linked_output(gates[0], port),
                            Some((input, port.into()))
                        );
                        assert_eq!(
                            hugr.single_linked_output(output, port),
                            Some((gates[0], port.into()))
                        );
                    }
                } else {
                    assert!(gates.is_empty());
                    for port in 0..arity {
                        let source = if index == value as usize {
                            arity - 1 - port
                        } else {
                            port
                        };
                        assert_eq!(
                            hugr.single_linked_output(output, port),
                            Some((input, source.into()))
                        );
                    }
                }
            }
        }
        // Each target's output from the first correction feeds the same target
        // in the second correction, regardless of its register-list position.
        for port in 0..arity {
            assert_eq!(
                hugr.single_linked_output(conditions[1], port + 1),
                Some((conditions[0], port.into()))
            );
        }
    }

    #[test]
    fn rejects_unsupported_conditions_instead_of_making_them_opaque() {
        let mut circuit = conditional_circuit(SerialOpType::X, 1, 1);
        circuit.commands[1].op.conditional.as_mut().unwrap().width = 2;
        assert!(decode_conditionals(&circuit).is_err());
        let circuit = conditional_circuit(SerialOpType::Rx, 1, 1);
        assert!(decode_conditionals(&circuit).is_err());
    }

    #[test]
    fn decodes_swap_as_crossed_wires() {
        use tket_json_rs::circuit_json::Command;
        let mut circuit = conditional_circuit(SerialOpType::X, 1, 1);
        circuit.commands = vec![Command {
            op: Operation::from_optype(SerialOpType::SWAP),
            args: vec![circuit.qubits[2].id.clone(), circuit.qubits[1].id.clone()],
            opgroup: None,
        }];
        let hugr = decode_conditionals(&circuit).unwrap();
        hugr.validate().unwrap();
        assert_no_opaque_ops(&hugr);
        let swap = hugr.nodes().find(|&n| hugr.get_optype(n).is_dfg()).unwrap();
        let [input, output] = hugr.get_io(swap).unwrap();
        assert_eq!(hugr.children(swap).count(), 2);
        assert_eq!(
            hugr.single_linked_output(output, 0),
            Some((input, 1.into()))
        );
        assert_eq!(
            hugr.single_linked_output(output, 1),
            Some((input, 0.into()))
        );
    }

    #[test]
    fn allocates_fresh_measurement_bits_across_batches() {
        use pg_core::{Pauli, RotationData};
        use tket_json_rs::{OpType, register::Qubit};

        let mut circuit = SerialCircuit::new(None, "0".to_owned());
        circuit.qubits = vec![
            Qubit::from(ElementId("q".to_owned(), vec![0])),
            Qubit::from(ElementId("__tket_measurement".to_owned(), vec![1])),
        ];
        circuit.bits = vec![Bit::from(ElementId(
            "__tket_measurement".to_owned(),
            vec![0],
        ))];
        let original_bits = circuit.bits.clone();
        let graph = PauliGraph::new(2).with_ops(
            (0..6)
                .map(|i| Op::Rotation {
                    data: RotationData::new(
                        vec![if i % 2 == 0 { Pauli::Z } else { Pauli::X }, Pauli::I],
                        0.25,
                    ),
                })
                .collect(),
        );
        let optimized = TOptimizationPass::new()
            .with_ancilla_budget(1)
            .with_first_bit(circuit.bits.len())
            .transform(&graph);
        let canonical = CanonicalFormPass::new().transform(&optimized);
        allocate_measurement_bits(&mut circuit, &canonical);
        assert_eq!(&circuit.bits[..original_bits.len()], &original_bits);
        // The single ancilla is reused across batches, with a fresh result each time.
        assert!(circuit.bits.len() > original_bits.len() + 1);
        assert_eq!(
            circuit.bits[1].id,
            ElementId("__tket_measurement".to_owned(), vec![2])
        );

        let grouped = GroupCommutingOpsPass::new().transform(&canonical);
        let gates = GreedySynthPass::new()
            .with_parallel_mode(ParallelMode::Off)
            .transform(&grouped);
        let gates = RebaseTQEToZXPass::new()
            .with_allowed_tqes(vec![GateType::ZX])
            .transform(&gates);
        let map = RegisterMap::new(&circuit.qubits, &circuit.bits);
        let commands = pauli_graph_to_cmds(gates, &map).unwrap();
        let mut measured = HashSet::new();
        let mut conditions = 0;
        for command in commands {
            if command.op.op_type == OpType::Measure {
                let bit = &command.args[1];
                assert!(!original_bits.iter().any(|b| &b.id == bit));
                assert!(measured.insert(bit.clone()));
            } else if command.op.op_type == OpType::Conditional {
                assert!(measured.contains(&command.args[0]));
                conditions += 1;
            }
        }
        assert_eq!(measured.len(), circuit.bits.len() - original_bits.len());
        assert!(conditions > 0);
    }

    #[rstest]
    #[case::window_size(PauliGraphResynthesis::default().with_window_size(0), "window_size")]
    #[case::pool_size(PauliGraphResynthesis::default().with_pool_size(0), "pool_size")]
    #[case::top_up_size(PauliGraphResynthesis::default().with_top_up_size(0), "top_up_size")]
    fn rejects_invalid_parameters(
        #[case] pass: PauliGraphResynthesis,
        #[case] expected_parameter: &str,
    ) {
        let mut circuit = build_simple_circuit(1, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::H, [0])?;
            Ok(())
        })
        .unwrap();
        let original = circuit.clone();

        let error = pass.run(circuit.hugr_mut()).unwrap_err();

        assert!(matches!(
            error,
            PauliGraphResynthesisErrors::InvalidParameters { parameter }
                if parameter == expected_parameter
        ));
        assert_eq!(circuit, original);
    }

    // Required because Pauli graph resynthesis currently doesn't simplify
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

        resynthesize(&mut circuit);

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
        resynthesize(&mut circuit);

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
        resynthesize(&mut circuit);

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

        resynthesize(&mut circuit);

        // T and Tdg cancel, leaving a nontrivial Clifford circuit to synthesise
        assert_eq!(count_gate(&circuit, TketOp::T), 0);
        assert_eq!(count_gate(&circuit, TketOp::Tdg), 0);
        assert!(count_gate(&circuit, TketOp::CX) > 0);
        assert_eq!(circuit.qubit_count(), 6);
    }
}
