//! Resynthesis of a Clifford + Rz circuit through a Pauli graph.
//!
//! The [`PauliGraphResynthesis`] pass optimises a circuit by converting it to a Pauli graph, and applying:
//! - Phase folding through the [`RotationMergingPass`]
//! - Optional phase polynomial resynthesis for further T count reduction through the [`TOptimizationPass`]
//! - Synthesis of the pauli graph as a circuit, aiming to minimize 2 qubit gates, through the [`GreedySynthPass`]

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

/// Resynthesize a Clifford + Rz circuit by converting it to a Pauli graph and applying various
/// optimisation techniques such as:
/// - phase folding
/// - optional phase polynomial resynthesis for T gate reduction
/// - a synthesis algorithm from pauli graph to Clifford + Rz aimed at reducing the number of 2
///   qubit gates
///
/// Rotation angles must be numeric as symbolic angles are not supported currently.
/// Circuits must be Clifford + T when `t_optimization` is enabled.
///
/// - `window_size` (`Option<usize>`) - Size of the sliding window for lookahead during synthesis. Default to 1280.
/// - `pool_size` (`Option<usize>`) - Number of candidate gates to maintain in the pool. Default to max(1000, 0.2*N^2) where N is the number of qubits.
/// - `top_up_size` (`Option<usize>`) - Number of candidates to add after each TQE gate. Default to max(200, pool_size / N) where N is the number of qubits.
/// - `seed` (`u64`) - Random seed for reproducible candidate sampling. Default to `0`.
/// - `parallel_mode` (`ParallelMode`) - Configuration for parallel processing of candidates. Default to `ParallelMode::Auto`.
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
    /// Allocate a pool in each outer circuit. Nested region interfaces are unchanged;
    /// regions without ancillas use a budget of zero.
    /// Defaults to the largest Hadamard count among the selected dataflow regions.
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

        let marker = ancilla_marker(hugr);
        if self.t_optimization {
            let regions: Vec<_> = self
                .scope
                .regions(hugr)
                .filter(|&region| hugr.get_io(region).is_some())
                .collect();

            let budget = self
                .ancilla_budget
                .unwrap_or_else(|| max_hadamards(hugr, &regions));

            for region in regions {
                if region == root || hugr.get_parent(region) == Some(hugr.module_root()) {
                    allocate_ancillas(hugr, region, budget, &marker);
                }
            }
        }

        let encode_options = EncodeOptions::new()
            .with_subcircuits(self.scope.recursive())
            .keep_empty_circuits(self.t_optimization);

        let mut encoded_circs = EncodedCircuit::new_with_entrypoint(hugr, root, encode_options)?;

        for node in hugr.descendants(root).collect::<Vec<_>>() {
            if hugr.get_metadata::<metadata::PytketOpGroup>(node) == Some(marker.as_str()) {
                hugr.remove_metadata::<metadata::PytketOpGroup>(node);
            }
        }

        let mut ancillas: HashMap<Node, HashSet<ElementId>> = HashMap::new();
        for (region, serial_circ) in encoded_circs.iter_mut() {
            let registers = ancillas.entry(region).or_default();
            serial_circ.commands.retain(|cmd| {
                if cmd.opgroup.as_deref() == Some(marker.as_str()) {
                    registers.extend(cmd.args.iter().cloned());
                    false
                } else {
                    true
                }
            });
        }

        for (region, serial_circ) in encoded_circs.iter_mut() {
            if serial_circ.commands.is_empty() {
                continue;
            }
            let registers = &ancillas[&region];
            let mut qubits = serial_circ.qubits.clone();
            qubits.sort_by_key(|q| registers.contains(&q.id));

            let register_map = RegisterMap::new(&qubits, &serial_circ.bits);
            let mut pauli_graph = serial_circuit_to_pauli_graph(serial_circ, &register_map)?;

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

            pauli_graph = canonical_pass.transform(&pauli_graph);
            pauli_graph = rotation_merging_pass.transform(&pauli_graph);

            if self.t_optimization {
                let unsupported = pauli_graph.get_ops().iter().find_map(|op| match op {
                    Op::Measure { .. } => Some("measurements"),
                    Op::Reset { .. } => Some("resets"),
                    Op::BlackBox { .. } => Some("black boxes"),
                    Op::ConditionalBox { .. } => Some("conditional operations"),
                    _ => None,
                });
                if let Some(operation) = unsupported {
                    return Err(PauliGraphResynthesisErrors::UnsupportedTOptimizationInput {
                        operation,
                    });
                }

                let budget = serial_circ
                    .qubits
                    .iter()
                    .filter(|q| registers.contains(&q.id))
                    .count();

                let optimized = TOptimizationPass::new()
                    .with_ancilla_budget(budget)
                    .with_first_bit(serial_circ.bits.len())
                    .transform(&pauli_graph);

                let optimized = canonical_pass.transform(&optimized);
                allocate_measurement_bits(serial_circ, &optimized);

                pauli_graph = optimized;
            }

            pauli_graph = grouping_pass.transform(&pauli_graph);
            pauli_graph = synth_pass.transform(&pauli_graph);
            pauli_graph = rebase_pass.transform(&pauli_graph);

            let register_map = RegisterMap::new(&qubits, &serial_circ.bits);
            serial_circ.commands = pauli_graph_to_cmds(pauli_graph, &register_map)?;

            for qubit in qubits.iter().filter(|q| registers.contains(&q.id)) {
                serial_circ
                    .commands
                    .push(tket_json_rs::circuit_json::Command {
                        op: Operation::from_optype(SerialOpType::Reset),
                        args: vec![qubit.id.clone()],
                        opgroup: None,
                    });
            }
        }

        let mut decoder_config = default_decoder_config();
        decoder_config.add_decoder(ResynthesisDecoder);
        encoded_circs.reassemble_inplace(hugr, Some(Arc::new(decoder_config)))?;
        Ok(())
    }
}

/// Returns the maximum number of Hadamards in a region from the given list.
fn max_hadamards(hugr: &Hugr, regions: &[Node]) -> usize {
    regions
        .iter()
        .map(|&region| {
            hugr.children(region)
                .filter(|&node| hugr.get_optype(node) == &TketOp::H.into())
                .count()
        })
        .max()
        .unwrap_or(0)
}

/// Adds labels to added ancillas to avoid name collisions.
fn ancilla_marker(hugr: &Hugr) -> String {
    let mut marker = "__tket_ancilla_pool".to_owned();
    while hugr
        .nodes()
        .any(|n| hugr.get_metadata::<metadata::PytketOpGroup>(n) == Some(marker.as_str()))
    {
        marker.push('_');
    }
    marker
}

/// Allocate a pool locally, without changing any region interfaces.
fn allocate_ancillas(hugr: &mut Hugr, region: Node, budget: usize, label: &str) {
    for _ in 0..budget {
        let alloc = hugr.add_node_with_parent(region, TketOp::QAlloc);
        let marker =
            hugr.add_node_with_parent(region, hugr::extension::prelude::Barrier::new(vec![qb_t()]));
        let free = hugr.add_node_with_parent(region, TketOp::QFree);
        hugr.connect(alloc, 0, marker, 0);
        hugr.connect(marker, 0, free, 0);
        hugr.set_metadata::<metadata::PytketOpGroup>(marker, label);
    }
}

/// Extends the decoder to support the classically controlled Clifford gates
/// introduced by the hadamard gadgets in T optimization, as well as SWAP gates
/// to further reduce the 2 qubit gate count.
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
            let mut outputs: Vec<_> = branch.input_wires().collect();

            if value == condition.value as usize {
                if let Some(gate) = gate {
                    outputs = branch
                        .add_dataflow_op(gate, outputs)
                        .map_err(PytketDecodeError::custom)?
                        .outputs()
                        .collect();
                } else {
                    outputs.swap(0, 1);
                }
            }
            branch
                .finish_with_outputs(outputs)
                .map_err(PytketDecodeError::custom)?;
        }

        let node = conditional
            .finish_sub_container()
            .map_err(PytketDecodeError::custom)?
            .node();

        decoder.register_node_outputs(node, qubits.iter().cloned(), [])?;
        Ok(DecodeStatus::Success)
    }
}

/// Allocate classical registers for measurement results introduced by T optimization's
/// hadamard gadgets.
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
    /// The input contains operations unsupported by T optimization.
    #[display(
        "T optimization does not support {operation} in the input circuit. \
         Disable t_optimization or apply it to a unitary Clifford + T region."
    )]
    UnsupportedTOptimizationInput {
        /// Kind of unsupported operation.
        operation: &'static str,
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
    use hugr::{CircuitUnit, HugrView};
    use rstest::rstest;

    use crate::extension::rotation::ConstRotation;
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

    #[rstest]
    #[case::different_angles([0.1, 0.2], 0.3)]
    #[case::opposite_angles([0.1, -0.1], 0.0)]
    fn merges_arbitrary_rz_angles(#[case] angles: [f64; 2], #[case] expected: f64) {
        let mut circuit = build_simple_circuit(1, |circ| {
            for angle in angles {
                let angle = circ.add_constant(ConstRotation::new(angle).unwrap());
                circ.append_and_consume(
                    TketOp::Rz,
                    [CircuitUnit::Linear(0), CircuitUnit::Wire(angle)],
                )?;
            }
            Ok(())
        })
        .unwrap();
        resynthesize(&mut circuit);

        if expected == 0.0 {
            let identity = build_simple_circuit(1, |_| Ok(())).unwrap();
            assert_eq!(circuit.num_operations(), 0);
            assert_eq!(circuit, identity);
            return;
        }

        assert_eq!(circuit.num_operations(), 1);
        assert_eq!(count_gate(&circuit, TketOp::Rz), 1);
        let hugr = circuit.hugr();
        let angles: Vec<_> = hugr
            .nodes()
            .filter_map(|node| hugr.get_optype(node).as_const())
            .filter_map(|constant| constant.value().get_custom_value::<ConstRotation>())
            .map(ConstRotation::half_turns)
            .collect();
        assert_eq!(angles.len(), 1);
        assert!((angles[0] - expected).abs() < 1e-10);
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
