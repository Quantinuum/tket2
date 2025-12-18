//! Tracker for pytket values associated to wires in a hugr being encoded.
//!
//! Values can be qubits or bits (identified by a [`tket_json_rs::register::ElementId`]),
//! or a string-encoded parameter expression.
//!
//! Wires in the hugr may be associated with multiple values.
//! Qubit and bit wires map to a single register element, and float/rotation wires map to a string parameter.
//! But custom operations (e.g. arrays / sums) may map to multiple things.
//!
//! Extensions can define which elements they map to

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use hugr::core::HugrNode;
use hugr::ops::OpParent;
use hugr::{HugrView, Wire};
use hugr_core::metadata::Metadata;
use itertools::Itertools;
use tket_json_rs::circuit_json;
use tket_json_rs::register::ElementId as RegisterUnit;

use crate::circuit::Circuit;
use crate::metadata;
use crate::serialize::pytket::circuit::StraightThroughWire;
use crate::serialize::pytket::extension::RegisterCount;
use crate::serialize::pytket::{PytketEncodeError, PytketEncodeOpError, RegisterHash};

use super::PytketEncoderConfig;
use super::unit_generator::RegisterUnitGenerator;

/// A structure for tracking qubits used in the circuit being encoded.
///
/// Nb: Although `tket-json-rs` has a "Register" struct, it's actually
/// an identifier for single qubits/bits in the `Register::0` register.
/// We rename it to `RegisterUnit` here to avoid confusion.
#[derive(derive_more::Debug, Clone)]
#[debug(bounds(N: std::fmt::Debug))]
pub struct ValueTracker<N> {
    /// List of generated qubit register names.
    qubits: Vec<RegisterUnit>,
    /// List of generated bit register names.
    bits: Vec<RegisterUnit>,
    /// List of seen parameters.
    params: Vec<String>,

    /// The tracked data for a wire in the hugr.
    ///
    /// Contains an ordered list of values associated with it,
    /// and a counter of unexplored neighbours used to prune the map
    /// once the wire is fully explored.
    wires: BTreeMap<Wire<N>, TrackedWire>,

    /// Qubits in `qubits` that are not currently registered to any wire.
    ///
    /// We draw names from here when a new qubit name is needed, before
    /// resorting to the `qubit_reg_generator`.
    unused_qubits: BTreeSet<TrackedQubit>,
    /// Bits in `bits` that are not currently registered to any wire.
    ///
    /// We draw names from here when a new bit name is needed, before
    /// resorting to the `bit_reg_generator`.
    unused_bits: BTreeSet<TrackedBit>,

    /// A generator of new registers units to use for qubit wires.
    qubit_reg_generator: RegisterUnitGenerator,
    /// A generator of new registers units to use for bit wires.
    bit_reg_generator: RegisterUnitGenerator,

    /// The parameter names in the region's input.
    ///
    /// This list contains entries from the circuit's [`METADATA_INPUT_PARAMETERS`] metadata,
    /// plus fresh variable names generated as needed.
    input_params: Vec<String>,
}

/// A lightweight identifier for a qubit value.
///
/// Contains an index into the `qubits` array of [`ValueTracker`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, derive_more::Display,
)]
#[display("qubit#{}", self.0)]
pub struct TrackedQubit(usize);

/// A lightweight identifier for a bit value.
///
/// Contains an index into the `bits` array of [`ValueTracker`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, derive_more::Display,
)]
#[display("bit#{}", self.0)]
pub struct TrackedBit(usize);

/// A lightweight identifier for a parameter value.
///
/// Contains an index into the `params` array of [`ValueTracker`].
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default, derive_more::Display,
)]
#[display("param#{}", self.0)]
pub struct TrackedParam(usize);

/// A lightweight identifier for a qubit/bit/parameter value.
///
/// Contains an index into the corresponding value array in [`ValueTracker`].
#[derive(
    Debug,
    Clone,
    Copy,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    derive_more::From,
    derive_more::Display,
)]
#[non_exhaustive]
pub enum TrackedValue {
    /// A qubit value.
    ///
    /// Index into the `qubits` array of [`ValueTracker`].
    Qubit(TrackedQubit),
    /// A bit value.
    ///
    /// Index into the `bits` array of [`ValueTracker`].
    Bit(TrackedBit),
    /// A parameter value.
    ///
    /// Index into the `params` array of [`ValueTracker`].
    Param(TrackedParam),
}

/// Lists of tracked values, separated by type.
#[derive(Debug, Clone, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct TrackedValues {
    /// Tracked qubit values.
    pub qubits: Vec<TrackedQubit>,
    /// Tracked bit values.
    pub bits: Vec<TrackedBit>,
    /// Tracked parameter values.
    pub params: Vec<TrackedParam>,
}

impl Extend<TrackedValue> for TrackedValues {
    fn extend<T: IntoIterator<Item = TrackedValue>>(&mut self, iter: T) {
        for v in iter {
            match v {
                TrackedValue::Qubit(qb) => self.qubits.push(qb),
                TrackedValue::Bit(bit) => self.bits.push(bit),
                TrackedValue::Param(param) => self.params.push(param),
            }
        }
    }
}

/// Data associated with a tracked wire in the hugr.
#[derive(Debug, Clone)]
struct TrackedWire {
    /// The values associated with the wire.
    ///
    /// This is a list of [`TrackedValue`]s, which can be qubits, bits, or
    /// parameters.
    ///
    /// If the wire type was not translatable to pytket values, this attribute
    /// will be `None`.
    pub(self) values: Option<Vec<TrackedValue>>,
    /// The number of unexplored neighbours of the wire.
    ///
    /// This is used to prune the [`ValueTracker::wires`] map once the wire is
    /// fully explored.
    pub(self) unexplored_neighbours: usize,
}

/// The result finalizing the value tracker.
///
/// Contains the final list of qubit and bit registers, and the implicit
/// permutation of the output registers.
#[derive(Debug, Clone)]
pub struct ValueTrackerResult {
    /// The final list of qubit registers at the input.
    pub qubits: Vec<RegisterUnit>,
    /// The final list of bit registers.
    pub bits: Vec<RegisterUnit>,
    /// The final list of parameter expressions at the output.
    pub params: Vec<String>,
    /// The ordered list of qubit registers seen at the output of the region.
    pub qubit_outputs: Vec<RegisterUnit>,
    /// The ordered list of bit registers seen at the output of the region.
    pub bit_outputs: Vec<RegisterUnit>,
    /// The implicit permutation of the qubit registers.
    pub qubit_permutation: Vec<circuit_json::ImplicitPermutation>,
    /// A list of parameter variables seen at the input node of the region.
    pub input_params: Vec<String>,
    /// A list of wires that were not tracked but originated directly
    /// from the input node (so they don't appear in an unsupported graph).
    pub straight_through_wires: Vec<StraightThroughWire>,
}

impl<N: HugrNode> ValueTracker<N> {
    /// Create a new [`ValueTracker`] from the inputs of a [`Circuit`].
    ///
    /// Reads a number of metadata values from the circuit root node, if present, to preserve information on circuits produced by
    /// decoding a pytket circuit:
    ///
    /// - `METADATA_Q_REGISTERS`: The qubit input register names.
    /// - `METADATA_Q_OUTPUT_REGISTERS`: The reordered qubit output register names.
    /// - `METADATA_B_REGISTERS`: The bit input register names.
    /// - `METADATA_B_OUTPUT_REGISTERS`: The reordered bit output register names.
    /// - `METADATA_INPUT_PARAMETERS`: The input parameter names.
    ///
    pub(super) fn new<H: HugrView<Node = N>>(
        circ: &Circuit<H>,
        region: N,
        config: &PytketEncoderConfig<H>,
    ) -> Result<Self, PytketEncodeError<N>> {
        let param_variable_names: Vec<String> =
            read_metadata_json_list::<_, _, metadata::InputParameters>(circ, region);
        let mut tracker = ValueTracker {
            qubits: read_metadata_json_list::<_, _, metadata::QubitRegisters>(circ, region)
                .into_iter()
                .map(|q| q.id)
                .collect_vec(),
            bits: read_metadata_json_list::<_, _, metadata::BitRegisters>(circ, region)
                .into_iter()
                .map(|b| b.id)
                .collect_vec(),
            params: Vec::with_capacity(param_variable_names.len()),
            wires: BTreeMap::new(),
            unused_qubits: BTreeSet::new(),
            unused_bits: BTreeSet::new(),
            qubit_reg_generator: RegisterUnitGenerator::default(),
            bit_reg_generator: RegisterUnitGenerator::default(),
            input_params: Vec::with_capacity(param_variable_names.len()),
        };

        tracker.unused_qubits = (0..tracker.qubits.len()).map(TrackedQubit).collect();
        tracker.unused_bits = (0..tracker.bits.len()).map(TrackedBit).collect();
        tracker.qubit_reg_generator = RegisterUnitGenerator::new("q", tracker.qubits.iter());
        tracker.bit_reg_generator = RegisterUnitGenerator::new("c", tracker.bits.iter());

        // Generator of input parameter variable names.
        let existing_param_vars: HashSet<String> = param_variable_names.iter().cloned().collect();
        let mut param_gen = param_variable_names.into_iter().chain(
            (0..)
                .map(|i| format!("f{i}"))
                .filter(|name| !existing_param_vars.contains(name)),
        );

        // Register the circuit's inputs with the tracker.
        let region_optype = circ.hugr().get_optype(region);
        let signature = region_optype.inner_function_type().ok_or_else(|| {
            let optype = circ.hugr().get_optype(region).to_string();
            PytketEncodeError::NonDataflowRegion { region, optype }
        })?;
        let inp_node = circ.hugr().get_io(region).unwrap()[0];
        for (port, typ) in circ
            .hugr()
            .node_outputs(inp_node)
            .zip(signature.input().iter())
        {
            let wire = Wire::new(inp_node, port);
            let Some(count) = config.type_to_pytket(typ) else {
                // If the input has a non-serializable type, it gets skipped.
                //
                // We will store the connection outside the serialized circuit,
                // either as an unsupported subgraph or as a
                // [StraightThroughWire].
                continue;
            };

            let mut wire_values = Vec::with_capacity(count.total());
            for _ in 0..count.qubits {
                let qb = tracker.new_qubit();
                wire_values.push(TrackedValue::Qubit(qb));
            }
            for _ in 0..count.bits {
                let bit = tracker.new_bit();
                wire_values.push(TrackedValue::Bit(bit));
            }
            for _ in 0..count.params {
                let param_name = param_gen.next().unwrap();
                tracker.input_params.push(param_name.clone());
                let param = tracker.new_param(param_name);
                wire_values.push(TrackedValue::Param(param));
            }

            tracker.register_wire(wire, wire_values, circ)?;
        }

        Ok(tracker)
    }

    /// Create a new qubit register name.
    ///
    /// Picks unused names from the `qubits` list, if available, or generates
    /// a new one with the internal generator.
    pub fn new_qubit(&mut self) -> TrackedQubit {
        self.unused_qubits.pop_first().unwrap_or_else(|| {
            self.qubits.push(self.qubit_reg_generator.next());
            TrackedQubit(self.qubits.len() - 1)
        })
    }

    /// Create a new bit register name.
    ///
    /// Picks unused names from the `bits` list, if available, or generates
    /// a new one with the internal generator.
    pub fn new_bit(&mut self) -> TrackedBit {
        self.unused_bits.pop_first().unwrap_or_else(|| {
            self.bits.push(self.bit_reg_generator.next());
            TrackedBit(self.bits.len() - 1)
        })
    }

    /// Register a new parameter string expression.
    ///
    /// Returns a unique identifier for the expression.
    pub fn new_param(&mut self, expression: impl ToString) -> TrackedParam {
        self.params.push(expression.to_string());
        TrackedParam(self.params.len() - 1)
    }

    /// Associate a list of values with a wire.
    ///
    /// Linear qubit IDs can be reused to mark the new position of the qubit in the
    /// circuit.
    /// Bit types are not linear, so each [`TrackedBit`] is associated with a unique bit
    /// state in the circuit. The IDs may only be reused when no more users of the bit are
    /// present in the circuit.
    ///
    /// ### Panics
    ///
    /// If the wire is already associated with a different set of values.
    pub fn register_wire<Val: Into<TrackedValue>>(
        &mut self,
        wire: Wire<N>,
        values: impl IntoIterator<Item = Val>,
        circ: &Circuit<impl HugrView<Node = N>>,
    ) -> Result<(), PytketEncodeOpError<N>> {
        let values = values.into_iter().map(|v| v.into()).collect_vec();

        // Remove any qubit/bit used here from the unused set.
        for value in &values {
            match value {
                TrackedValue::Qubit(qb) => {
                    self.unused_qubits.remove(qb);
                }
                TrackedValue::Bit(bit) => {
                    self.unused_bits.remove(bit);
                }
                TrackedValue::Param(_) => {}
            }
        }

        let unexplored_neighbours = circ.hugr().linked_ports(wire.node(), wire.source()).count();
        let tracked = TrackedWire {
            values: Some(values),
            unexplored_neighbours,
        };
        if self.wires.insert(wire, tracked).is_some() {
            return Err(PytketEncodeOpError::WireAlreadyHasValues { wire });
        }

        if unexplored_neighbours == 0 {
            // We can unregister the wire immediately, since it has no unexplored
            // neighbours. This will free up the qubit and bit registers associated with it.
            self.unregister_wire(wire)
                .expect("Wire should be registered in the tracker");
        }

        Ok(())
    }

    /// Returns the values associated with a wire.
    ///
    /// Marks the port connection as explored. When all ports connected to the wire
    /// are explored, the wire is removed from the tracker.
    ///
    /// To avoid this use `peek_wire_values` instead.
    ///
    /// Returns `None` if the wire did not have any values associated with it,
    /// or if it had a type that cannot be translated into pytket values.
    pub(super) fn wire_values(&mut self, wire: Wire<N>) -> Option<Cow<'_, [TrackedValue]>> {
        let values = self.wires.get(&wire)?;
        if values.unexplored_neighbours != 1 {
            let wire = self.wires.get_mut(&wire).unwrap();
            wire.unexplored_neighbours -= 1;
            let values = wire.values.as_ref()?;
            return Some(Cow::Borrowed(values));
        }
        let values = self.unregister_wire(wire)?;
        Some(Cow::Owned(values))
    }

    /// Returns the values associated with a wire.
    ///
    /// The wire is not marked as explored. To improve performance, make sure to call
    /// [`ValueTracker::wire_values`] once per wire connection.
    ///
    /// Returns `None` if the wire did not have any values associated with it,
    /// or if it had a type that cannot be translated into pytket values.
    pub(super) fn peek_wire_values(&self, wire: Wire<N>) -> Option<&[TrackedValue]> {
        let wire = self.wires.get(&wire)?;
        let values = wire.values.as_ref()?;
        Some(&values[..])
    }

    /// Unregister a wire, freeing up the qubit and bit registers associated with it.
    ///
    /// Panics if the wire is not registered.
    fn unregister_wire(&mut self, wire: Wire<N>) -> Option<Vec<TrackedValue>> {
        let wire = self.wires.remove(&wire).unwrap();
        let values = wire.values?;

        // Free up the qubit and bit registers associated with the wire.
        for value in &values {
            match value {
                TrackedValue::Qubit(qb) => {
                    self.unused_qubits.insert(*qb);
                }
                TrackedValue::Bit(bit) => {
                    self.unused_bits.insert(*bit);
                }
                TrackedValue::Param(_) => {}
            }
        }

        Some(values)
    }

    /// Returns the qubit register associated with a qubit value.
    pub fn qubit_register(&self, qb: TrackedQubit) -> &RegisterUnit {
        &self.qubits[qb.0]
    }

    /// Returns the bit register associated with a bit value.
    pub fn bit_register(&self, bit: TrackedBit) -> &RegisterUnit {
        &self.bits[bit.0]
    }

    /// Returns the string-encoded parameter expression associated with a parameter value.
    pub fn param_expression(&self, param: TrackedParam) -> &str {
        &self.params[param.0]
    }

    /// Finish the tracker and return the final list of qubit and bit registers.
    ///
    /// Looks at the circuit's output node to determine the final order of
    /// output.
    pub(super) fn finish(
        self,
        circ: &Circuit<impl HugrView<Node = N>>,
        region: N,
    ) -> Result<ValueTrackerResult, PytketEncodeOpError<N>> {
        let [input_node, output_node] = circ.hugr().get_io(region).unwrap();

        // Ordered list of qubits and bits at the output of the circuit.
        let mut straight_through_wires = Vec::new();
        let mut qubit_outputs = Vec::with_capacity(self.qubits.len() - self.unused_qubits.len());
        let mut bit_outputs = Vec::with_capacity(self.bits.len() - self.unused_bits.len());
        let mut param_outputs = Vec::new();
        for tgt_port in circ.hugr().node_inputs(output_node) {
            for (src_node, src_port) in circ.hugr().linked_outputs(output_node, tgt_port) {
                let wire = Wire::new(src_node, src_port);
                let Some(values) = self.peek_wire_values(wire) else {
                    // If the wire originates from the input node, track it as a straight through wire.
                    // Otherwise, ignore it (it originates in an unsupported subgraph)
                    if src_node == input_node {
                        straight_through_wires.push(StraightThroughWire {
                            input_source: src_port,
                            output_target: tgt_port,
                        });
                    }
                    continue;
                };
                for value in values {
                    match value {
                        TrackedValue::Qubit(qb) => {
                            qubit_outputs.push(self.qubit_register(*qb).clone())
                        }
                        TrackedValue::Bit(bit) => bit_outputs.push(self.bit_register(*bit).clone()),
                        TrackedValue::Param(param) => {
                            param_outputs.push(self.param_expression(*param).to_string())
                        }
                    }
                }
            }
        }

        // Compute the final register permutations.
        let qubit_permutation = compute_final_permutation(qubit_outputs.clone(), &self.qubits);

        Ok(ValueTrackerResult {
            qubits: self.qubits,
            bits: self.bits,
            params: param_outputs,
            qubit_outputs,
            bit_outputs,
            qubit_permutation,
            input_params: self.input_params,
            straight_through_wires,
        })
    }
}

impl TrackedValues {
    /// Return a new container with a list of tracked qubits.
    pub fn new_qubits(qubits: impl IntoIterator<Item = TrackedQubit>) -> Self {
        let qubits = qubits.into_iter().collect();
        Self {
            qubits,
            bits: Vec::new(),
            params: Vec::new(),
        }
    }

    /// Return a new container with a list of tracked bits.
    pub fn new_bits(bits: impl IntoIterator<Item = TrackedBit>) -> Self {
        let bits = bits.into_iter().collect();
        Self {
            qubits: Vec::new(),
            bits,
            params: Vec::new(),
        }
    }

    /// Return a new container with a list of tracked parameters.
    pub fn new_params(params: impl IntoIterator<Item = TrackedParam>) -> Self {
        let params = params.into_iter().collect();
        Self {
            qubits: Vec::new(),
            bits: Vec::new(),
            params,
        }
    }

    /// Returns the number of qubits, bits, and parameters in the list.
    pub fn count(&self) -> RegisterCount {
        RegisterCount::new(self.qubits.len(), self.bits.len(), self.params.len())
    }

    /// Iterate over the values in the list.
    pub fn iter(&self) -> impl Iterator<Item = TrackedValue> + '_ {
        self.qubits
            .iter()
            .map(|&qb| TrackedValue::Qubit(qb))
            .chain(self.bits.iter().map(|&bit| TrackedValue::Bit(bit)))
            .chain(self.params.iter().map(|&param| TrackedValue::Param(param)))
    }

    /// Append tracked values to the list.
    pub fn append(&mut self, other: TrackedValues) {
        self.qubits.extend(other.qubits);
        self.bits.extend(other.bits);
        self.params.extend(other.params);
    }
}

impl IntoIterator for TrackedValues {
    type Item = TrackedValue;

    type IntoIter = std::iter::Chain<
        std::iter::Chain<
            itertools::MapInto<std::vec::IntoIter<TrackedQubit>, TrackedValue>,
            itertools::MapInto<std::vec::IntoIter<TrackedBit>, TrackedValue>,
        >,
        itertools::MapInto<std::vec::IntoIter<TrackedParam>, TrackedValue>,
    >;

    fn into_iter(self) -> Self::IntoIter {
        self.qubits
            .into_iter()
            .map_into()
            .chain(self.bits.into_iter().map_into())
            .chain(self.params.into_iter().map_into())
    }
}

/// Read a json-encoded vector of values from the circuit's root metadata.
fn read_metadata_json_list<T: serde::de::DeserializeOwned, H: HugrView, K: Metadata>(
    circ: &Circuit<H>,
    region: H::Node,
) -> Vec<T>
where
    for<'hugr> K::Type<'hugr>: Into<Vec<T>>,
{
    circ.hugr()
        .get_metadata::<K>(region)
        .map(Into::into)
        .unwrap_or_default()
}

/// Compute the final unit permutation for a circuit.
///
/// Arguments:
/// - `all_inputs`: The ordered list of registers declared in the circuit.
/// - `actual_outputs`: The final order of output registers, computed from the
///   wires at the output node of the circuit.
///
/// Returns the final permutation of the output registers.
pub(super) fn compute_final_permutation(
    mut actual_outputs: Vec<RegisterUnit>,
    all_inputs: &[RegisterUnit],
) -> Vec<circuit_json::ImplicitPermutation> {
    let declared_outputs: Vec<&RegisterUnit> = all_inputs.iter().collect();
    let mut actual_outputs_hashes: HashSet<RegisterHash> =
        actual_outputs.iter().map(RegisterHash::from).collect();
    let mut input_hashes: HashMap<RegisterHash, usize> = HashMap::default();
    for (i, inp) in all_inputs.iter().enumerate() {
        let hash = inp.into();
        input_hashes.insert(hash, i);
    }
    // Extend `actual_outputs` with extra registers seen in the circuit.
    for reg in all_inputs {
        let hash = reg.into();
        if !actual_outputs_hashes.contains(&hash) {
            actual_outputs.push(reg.clone());
            actual_outputs_hashes.insert(hash);
        }
    }

    // Compute the final permutation.
    //
    // For each element `reg` at the output of the circuit, we find its position `i` at the input,
    // and find out the pytket output register associated with that position in the `declared_outputs` list.

    actual_outputs
        .iter()
        .map(|reg| {
            let hash = reg.into();
            let i = input_hashes.get(&hash).unwrap();
            let out = declared_outputs[*i].clone();
            circuit_json::ImplicitPermutation(
                tket_json_rs::register::Qubit { id: reg.clone() },
                tket_json_rs::register::Qubit { id: out },
            )
        })
        .collect_vec()
}
