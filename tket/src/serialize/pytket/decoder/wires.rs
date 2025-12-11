//! Structures to keep track of pytket [`ElementId`][tket_json_rs::register::ElementId]s and
//! their correspondence to wires in the hugr being defined.
use std::collections::{BTreeMap, VecDeque};
use std::sync::Arc;

use hugr::builder::{DFGBuilder, Dataflow as _};
use hugr::extension::prelude::{bool_t, qb_t};
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::Value;
use hugr::std_extensions::arithmetic::float_types::{float64_type, ConstF64};
use hugr::types::Type;
use hugr::{Hugr, IncomingPort, Node, Wire};
use indexmap::{IndexMap, IndexSet};
use itertools::Itertools;
use tket_json_rs::circuit_json::ImplicitPermutation;
use tket_json_rs::register::ElementId as PytketRegister;

use crate::extension::bool::bool_type;
use crate::extension::rotation::{rotation_type, ConstRotation};
use crate::serialize::pytket::decoder::param::parser::{parse_pytket_param, PytketParam};
use crate::serialize::pytket::decoder::{
    LoadedParameter, ParameterType, PytketDecoderContext, TrackedBit, TrackedBitId, TrackedQubit,
    TrackedQubitId,
};
use crate::serialize::pytket::extension::RegisterCount;
use crate::serialize::pytket::opaque::EncodedEdgeID;
use crate::serialize::pytket::{
    PytketDecodeError, PytketDecodeErrorInner, PytketDecoderConfig, RegisterHash, PARAMETER_TYPES,
};
use crate::{symbolic_constant_op, TketOp};

/// Tracked data for a wire in [`TrackedWires`].
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct WireData {
    /// The identifier in the hugr.
    wire: Wire,
    /// The type of the wire.
    ty: Arc<Type>,
    /// List of pytket qubit arguments corresponding to this wire.
    qubits: Vec<TrackedQubitId>,
    /// List of pytket bit arguments corresponding to this wire.
    bits: Vec<TrackedBitId>,
}

impl WireData {
    /// The wire identifier.
    pub fn wire(&self) -> Wire {
        self.wire
    }

    /// The HUGR type for the wire.
    pub fn ty(&self) -> &Type {
        &self.ty
    }

    /// The pytket qubit arguments corresponding to this wire.
    pub fn qubits<'d>(
        &'d self,
        decoder: &'d PytketDecoderContext<'d>,
    ) -> impl Iterator<Item = TrackedQubit> + 'd {
        self.qubits
            .iter()
            .map(move |elem_id| decoder.wire_tracker.get_qubit(*elem_id))
            .cloned()
    }

    /// The pytket bit arguments corresponding to this wire.
    pub fn bits<'d>(
        &'d self,
        decoder: &'d PytketDecoderContext<'d>,
    ) -> impl Iterator<Item = TrackedBit> + 'd {
        self.bits
            .iter()
            .map(move |elem_id| decoder.wire_tracker.get_bit(*elem_id))
            .cloned()
    }

    /// Returns the number of qubits carried by this wire.
    pub fn num_qubits(&self) -> usize {
        self.qubits.len()
    }

    /// Returns the number of bits carried by this wire.
    pub fn num_bits(&self) -> usize {
        self.bits.len()
    }
}

/// Set of wires related to a pytket operation being decoded.
///
/// Contains both _parameter_ and _value_ wires.
///
/// The _parameter_ wires are wires that contain a single [`LoadedParameter`]
/// (either a float or a rotation) corresponding to the sympy expressions in the
/// operation arguments.
///
/// The _value_ wires are wires that contain a collection of [`TrackedQubit`]s
/// and [`TrackedBit`]s.
///
/// This set is passed to the implementer of `PytketDecoder` with the wires that
/// were found to contain the pytket registers used by the operation.
#[derive(Debug, Clone)]
pub struct TrackedWires {
    /// Computed list of wires corresponding to the arguments,
    /// along with their types.
    value_wires: Vec<WireData>,
    /// List of wires corresponding to the parameters.
    parameter_wires: Vec<LoadedParameter>,
}

impl TrackedWires {
    /// Return the number of value wires tracked.
    #[inline]
    #[must_use]
    pub fn value_count(&self) -> usize {
        self.value_wires.len()
    }

    /// Return the number of parameter wires tracked.
    #[inline]
    #[must_use]
    pub fn parameter_count(&self) -> usize {
        self.parameter_wires.len()
    }

    /// Return the number of wires tracked.
    #[inline]
    #[must_use]
    pub fn len(&self) -> usize {
        self.value_wires.len() + self.parameter_wires.len()
    }

    /// Return whether there are no tracked wires.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.value_wires.is_empty() && self.parameter_wires.is_empty()
    }

    /// Return an iterator over the wires and their types.
    #[inline]
    pub(super) fn iter_values(&self) -> impl Iterator<Item = &'_ WireData> + Clone + '_ {
        self.value_wires.iter()
    }

    /// Return an iterator over the parameters.
    #[inline]
    pub fn iter_parameters(&self) -> impl Iterator<Item = &'_ LoadedParameter> + Clone + '_ {
        self.parameter_wires.iter()
    }

    /// Returns the types of the value wires.
    #[inline]
    pub fn value_types(&self) -> impl Iterator<Item = &'_ Type> + Clone + '_ {
        self.value_wires.iter().map(|wd| wd.ty())
    }

    /// Returns the types of the parameter wires.
    #[inline]
    pub fn parameter_types(&self) -> impl Iterator<Item = &'_ Type> + Clone + '_ {
        self.parameter_wires.iter().map(|p| p.wire_type())
    }

    /// Returns the wire types in this tracked wires.
    #[inline]
    pub fn wire_types(&self) -> impl Iterator<Item = &'_ Type> + Clone + '_ {
        self.value_types().chain(self.parameter_types())
    }

    /// Returns the tracked qubit elements in the set of wires.
    #[inline]
    pub fn qubits<'d>(
        &'d self,
        decoder: &'d PytketDecoderContext<'d>,
    ) -> impl Iterator<Item = TrackedQubit> + 'd {
        self.value_wires
            .iter()
            .flat_map(move |wd| wd.qubits(decoder))
    }

    /// Returns the tracked bit elements in the set of wires.
    #[inline]
    pub fn bits<'d>(
        &'d self,
        decoder: &'d PytketDecoderContext<'d>,
    ) -> impl Iterator<Item = TrackedBit> + 'd {
        self.value_wires.iter().flat_map(move |wd| wd.bits(decoder))
    }

    /// Return the tracked value wires in this tracked wires.
    #[inline]
    pub fn value_wires(&self) -> impl Iterator<Item = Wire> + Clone + '_ {
        self.value_wires.iter().map(|wd| wd.wire())
    }

    /// Return the tracked parameter wires in this tracked wires.
    #[inline]
    pub fn parameter_wires(&self) -> impl Iterator<Item = Wire> + Clone + '_ {
        self.parameter_wires.iter().map(|p| p.wire())
    }

    /// Returns the wires in this tracked wires.
    #[inline]
    pub fn wires(&self) -> impl Iterator<Item = Wire> + Clone + '_ {
        self.value_wires().chain(self.parameter_wires())
    }

    /// Returns the wires in this tracked wires as an array of types.
    ///
    /// Returns an error if the number of wires is not equal to `N`.
    #[inline]
    pub fn wires_arr<const N: usize>(&self) -> Result<[Wire; N], PytketDecodeError> {
        let expected_values = N.saturating_sub(self.parameter_count());
        let expected_params = N - expected_values;
        self.check_len(expected_values, expected_params)?;
        Ok(self
            .wires()
            .collect_array()
            .expect("check_len should have failed"))
    }

    /// Returns the amount of qubits, bits, and parameters carried by this tracked wires.
    #[inline]
    #[must_use]
    pub fn register_count(&self) -> RegisterCount {
        let mut counts: RegisterCount = self
            .iter_values()
            .map(|w| RegisterCount::new(w.num_qubits(), w.num_bits(), 0))
            .sum();
        counts.params += self.parameter_count();
        counts
    }

    /// Checks that we have the expected number of wires, and returns an error otherwise.
    ///
    /// # Arguments
    ///
    /// * `expected_values` - The expected number of value wires.
    /// * `expected_params` - The expected number of parameter wires.
    pub fn check_len(
        &self,
        expected_values: usize,
        expected_params: usize,
    ) -> Result<(), PytketDecodeError> {
        if self.value_count() != expected_values || self.parameter_count() != expected_params {
            let types = self.wire_types().map(|ty| ty.to_string()).collect_vec();
            Err(PytketDecodeErrorInner::UnexpectedInputWires {
                expected_values,
                expected_params,
                actual_values: self.value_count(),
                actual_params: self.parameter_count(),
                expected_types: None,
                actual_types: Some(types),
            }
            .into())
        } else {
            Ok(())
        }
    }

    /// Checks that we have the expected wire types, and returns an error otherwise.
    ///
    /// # Arguments
    ///
    /// * `expected_values` - The expected types of the value wires.
    /// * `expected_params` - The expected number of parameters. Note that these may be either `float` or `rotation`-typed.
    ///   Use [`LoadedParameter::with_type`] to cast them as needed.
    pub fn check_types(
        &self,
        expected_values: &[Type],
        expected_params: usize,
    ) -> Result<(), PytketDecodeError> {
        let vals = expected_values.iter();
        if !itertools::equal(self.value_types(), vals) || self.parameter_count() != expected_params
        {
            let actual = self.value_types().collect_vec();
            Err(PytketDecodeErrorInner::UnexpectedInputWires {
                expected_values: expected_values.len(),
                expected_params,
                actual_values: self.value_count(),
                actual_params: self.parameter_count(),
                expected_types: Some(
                    expected_values
                        .iter()
                        .map(|ty| ty.to_string())
                        .collect_vec(),
                ),
                actual_types: Some(actual.iter().map(|ty| ty.to_string()).collect_vec()),
            }
            .into())
        } else {
            Ok(())
        }
    }
}

/// Tracker for wires added to a hugr.
///
/// Keeps track of the wires added to the hugr, and the qubit/bit/parameters
/// that they contain.
///
/// Wire may contain either a single [`LoadedParameter`] or a collection of
/// [`TrackedQubit`]s and [`TrackedBit`]s. Each tracked
/// element in a wire is said to be "up to date" if it is the latest reference
/// to that pytket register. Once the register is seen in the output of an
/// operation, all previous references to it become "outdated".
#[derive(Debug, Clone, Default)]
pub(crate) struct WireTracker {
    /// A map of wires being tracked, with their type and list of
    /// tracked pytket registers and parameters.
    wires: IndexMap<Wire, WireData>,
    /// The list of tracked qubit elements.
    ///
    /// Indexed by [`TrackedQubitId`].
    qubits: Vec<TrackedQubit>,
    /// The list of tracked bit elements.
    ///
    /// Indexed by [`TrackedBitId`].
    bits: Vec<TrackedBit>,
    /// A map from pytket register hashes to the latest up-to-date [`TrackedQubit`] referencing it.
    ///
    /// The map keys are kept in the order they were defined in the circuit.
    latest_qubit_tracker: IndexMap<RegisterHash, TrackedQubitId>,
    /// A map from pytket register hashes to the latest up-to-date [`TrackedBit`] referencing it.
    ///
    /// The map keys are kept in the order they were defined in the circuit.
    latest_bit_tracker: IndexMap<RegisterHash, TrackedBitId>,
    /// For each tracked qubit, the list of wires that contain it.
    qubit_wires: IndexMap<TrackedQubitId, Vec<Wire>>,
    /// For each tracked bit, the list of wires that contain it.
    bit_wires: IndexMap<TrackedBitId, Vec<Wire>>,
    /// An ordered set of parameters found in operation arguments, and added as
    /// new region inputs.
    parameters: IndexMap<String, LoadedParameter>,
    /// Parameter inputs to the region with no associated variable.
    ///
    /// These will be reused as needed if new parameter names are found in the command arguments.
    unused_parameter_inputs: VecDeque<LoadedParameter>,
    /// A list of input variables added to the hugr.
    ///
    /// Ordered according to their order in the function input.
    parameter_vars: IndexSet<String>,
    /// A permutation of qubit registers in `latest_qubit_tracker` that we
    /// expect to see at the output.
    ///
    /// This originates from pytket's `implicit_permutation` field.
    ///
    /// For a circuit with three qubit registers [q0, q1, q2] and an implicit
    /// permutation {q0 -> q1, q1 -> q2, q2 -> q0}, `output_qubit_permutation`
    /// will be {1 -> 0, 2 -> 1, 0 -> 2} and the output order will be [2, 0, 1].
    /// That is, at position 0 of the output we'll see the register originally
    /// named q2, at position 1 the register originally named q0, and so on.
    ///
    /// Registers outside the range of the array are not affected, and will
    /// appear in the same order as they were added to `latest_qubit_tracker`.
    output_qubit_permutation: Vec<usize>,
    /// Wires with unsupported types, created from the input node or from decoded opaque barriers.
    ///
    /// See [`EncodedEdgeID`], [`UnsupportedWireState`]
    unsupported_wires: IndexMap<EncodedEdgeID, UnsupportedWireState>,
}

/// Possible states for the entries in [`WireTracker::unsupported_wires`].
#[derive(Debug, Clone)]
enum UnsupportedWireState {
    /// The wire has been associated with a [`Wire`].
    Associated(Wire),
    /// The wire has not been associated with a [`Wire`] yet.
    ///
    /// We store target ports that need to be connected once the source is
    /// added.
    ///
    /// This is used when decoding unsupported inline subgraphs out-of-order,
    /// where we may see the inputs before the outputs.
    Pending(Vec<(Node, IncomingPort)>),
}

impl Default for UnsupportedWireState {
    fn default() -> Self {
        Self::Pending(Vec::new())
    }
}

impl WireTracker {
    /// Returns a new WireTracker with the given capacity.
    pub fn with_capacity(qubit_count: usize, bit_count: usize) -> Self {
        WireTracker {
            wires: IndexMap::new(),
            qubits: Vec::with_capacity(qubit_count),
            bits: Vec::with_capacity(bit_count),
            latest_qubit_tracker: IndexMap::with_capacity(qubit_count),
            latest_bit_tracker: IndexMap::with_capacity(bit_count),
            qubit_wires: IndexMap::with_capacity(qubit_count),
            bit_wires: IndexMap::with_capacity(bit_count),
            parameters: IndexMap::new(),
            unused_parameter_inputs: VecDeque::new(),
            parameter_vars: IndexSet::new(),
            output_qubit_permutation: Vec::with_capacity(qubit_count),
            unsupported_wires: IndexMap::new(),
        }
    }

    /// Closes the WireTracker.
    ///
    /// Returns a list of input parameter added to the hugr, in the order they
    /// were added.
    ///
    /// For the ordered qubit and bit elements, see
    /// [`WireTracker::known_pytket_qubits`] and
    /// [`WireTracker::known_pytket_bits`].
    pub(super) fn finish(self) -> IndexSet<String> {
        self.parameter_vars
    }

    /// Set the output qubit permutation.
    ///
    /// This is used to reorder the qubit registers at the output, according to
    /// pytket's implicit permutation.
    pub(super) fn compute_output_permutation(&mut self, permutation: &Vec<ImplicitPermutation>) {
        let mut reordered: BTreeMap<usize, usize> = BTreeMap::new();

        let position = |id: &tket_json_rs::register::Qubit| {
            let hash = RegisterHash::from(id);
            self.latest_qubit_tracker.get_index_of(&hash).unwrap()
        };

        for ImplicitPermutation(input, output) in permutation {
            let input_pos = position(input);
            let output_pos = position(output);
            reordered.insert(output_pos, input_pos);
        }
        self.output_qubit_permutation = reordered.values().copied().collect();
    }

    /// Returns a reference to the tracked qubit at the given index.
    fn get_qubit(&self, id: TrackedQubitId) -> &TrackedQubit {
        &self.qubits[id.0]
    }

    /// Returns a reference to the tracked bit at the given index.
    fn get_bit(&self, id: TrackedBitId) -> &TrackedBit {
        &self.bits[id.0]
    }

    /// Returns `true` if the tracked qubit has been initialized.
    ///
    /// Qubits that have been registered but not associated to any
    /// wires do not need to be consumed at the end of the decoding.
    pub(super) fn qubit_is_initialized(&self, qubit: &TrackedQubit) -> bool {
        self.qubit_wires
            .get(&qubit.id())
            .is_some_and(|ws| !ws.is_empty())
    }

    /// Returns the list of known pytket registers, in the order we expect to
    /// see them at the output.
    ///
    /// This is the ordered they were registered, permuted according to
    /// [`WireTracker::output_qubit_permutation`].
    pub(super) fn known_pytket_qubits(&self) -> impl Iterator<Item = &TrackedQubit> {
        (0..self.latest_qubit_tracker.len()).map(|i| {
            let i = self.output_qubit_permutation.get(i).copied().unwrap_or(i);
            let (_, &elem_id) = self.latest_qubit_tracker.get_index(i).unwrap();
            self.get_qubit(elem_id)
        })
    }

    /// Returns the list of known pytket bit registers, in the order they were registered.
    pub(super) fn known_pytket_bits(&self) -> impl Iterator<Item = &TrackedBit> {
        self.latest_bit_tracker
            .iter()
            .map(|(_, &elem_id)| self.get_bit(elem_id))
    }

    /// Track a new pytket qubit register.
    ///
    /// If the pytket register was already in the tracker,
    /// marks the previous element as outdated.
    ///
    /// If the [`RegisterHash`] has already been computed, it can be passed in
    /// to avoid recomputing it.
    pub(super) fn track_qubit(
        &mut self,
        qubit_reg: Arc<PytketRegister>,
        reg_hash: Option<RegisterHash>,
    ) -> Result<&TrackedQubit, PytketDecodeError> {
        check_register(&qubit_reg)?;

        let id = TrackedQubitId(self.qubits.len());
        let hash = reg_hash.unwrap_or_else(|| RegisterHash::from(qubit_reg.as_ref()));
        self.qubits
            .push(TrackedQubit::new_with_hash(id, qubit_reg, hash));
        if let Some(previous_id) = self.latest_qubit_tracker.insert(hash, id) {
            self.qubits[previous_id.0].mark_outdated();
        }
        self.qubit_wires.insert(id, Vec::new());
        Ok(self.get_qubit(id))
    }

    /// Track a new pytket bit register.
    ///
    /// If the pytket register was already in the tracker,
    /// marks the previous element as outdated.
    ///
    /// If the [`RegisterHash`] has already been computed, it can be passed in
    /// to avoid recomputing it.
    pub(super) fn track_bit(
        &mut self,
        bit_reg: Arc<PytketRegister>,
        reg_hash: Option<RegisterHash>,
    ) -> Result<&TrackedBit, PytketDecodeError> {
        check_register(&bit_reg)?;

        let id = TrackedBitId(self.bits.len());
        let hash = reg_hash.unwrap_or_else(|| RegisterHash::from(bit_reg.as_ref()));
        self.bits.push(TrackedBit::new_with_hash(id, bit_reg, hash));
        if let Some(previous_id) = self.latest_bit_tracker.insert(hash, id) {
            self.bits[previous_id.0].mark_outdated();
        }
        self.bit_wires.insert(id, Vec::new());
        Ok(self.get_bit(id))
    }

    /// Mark all the values in a wire as outdated.
    fn mark_wire_outdated(&mut self, wire: Wire) {
        let wire_data = &self.wires[&wire];

        for qubit in &wire_data.qubits {
            self.qubits[qubit.0].mark_outdated();
        }
        for bit in &wire_data.bits {
            self.bits[bit.0].mark_outdated();
        }
    }

    /// Mark a qubit as outdated, without adding a new wire containing the fresh value.
    ///
    /// This is used when a hugr operation consumes pytket registers as its inputs, but doesn't use them in the outputs.
    pub fn mark_qubit_outdated(&mut self, mut qubit: TrackedQubit) -> TrackedQubit {
        self.qubits[qubit.id().0].mark_outdated();
        qubit.mark_outdated();
        qubit
    }

    /// Mark a bit as outdated, without adding a new wire containing the fresh value.
    ///
    /// This is used when a hugr operation consumes pytket registers as its inputs, but doesn't use them in the outputs.
    pub fn mark_bit_outdated(&mut self, mut bit: TrackedBit) -> TrackedBit {
        self.bits[bit.id().0].mark_outdated();
        bit.mark_outdated();
        bit
    }

    /// Returns the latest tracked qubit for a pytket register.
    ///
    /// Returns an error if the register is not known.
    ///
    /// The returned element is guaranteed to be up to date (See [`TrackedQubit::is_outdated`]).
    pub fn tracked_qubit_for_register(
        &self,
        register: &PytketRegister,
    ) -> Result<&TrackedQubit, PytketDecodeError> {
        let hash = RegisterHash::from(register);
        let Some(id) = self.latest_qubit_tracker.get(&hash) else {
            return Err(PytketDecodeError::unknown_qubit_reg(register));
        };
        Ok(self.get_qubit(*id))
    }

    /// Returns the latest tracked bit for a pytket register.
    ///
    /// Returns an error if the register is not known.
    ///
    /// The returned element is guaranteed to be up to date (See [`TrackedBit::is_outdated`]).
    pub fn tracked_bit_for_register(
        &self,
        register: &PytketRegister,
    ) -> Result<&TrackedBit, PytketDecodeError> {
        let hash = RegisterHash::from(register);
        let Some(id) = self.latest_bit_tracker.get(&hash) else {
            return Err(PytketDecodeError::unknown_bit_reg(register));
        };
        Ok(self.get_bit(*id))
    }

    /// Returns the list of wires that contain the given qubit.
    fn qubit_wires(&self, qubit: &TrackedQubit) -> impl Iterator<Item = Wire> + '_ + use<'_> {
        self.qubit_wires[&qubit.id()].iter().copied()
    }

    /// Returns the list of wires that contain the given bit.
    fn bit_wires(&self, bit: &TrackedBit) -> impl Iterator<Item = Wire> + '_ + use<'_> {
        self.bit_wires[&bit.id()].iter().copied()
    }

    /// Given a list of pytket registers, splits them into qubit and bits and
    /// returns the latest tracked elements for each.
    pub(super) fn pytket_args_to_tracked_elems(
        &self,
        args: &[PytketRegister],
    ) -> Result<(Vec<TrackedQubit>, Vec<TrackedBit>), PytketDecodeError> {
        let mut qubit_args = Vec::with_capacity(args.len());
        let mut bit_args = Vec::new();

        for arg in args {
            let reg_hash = RegisterHash::from(arg);
            let is_bit = self.latest_bit_tracker.contains_key(&reg_hash);
            if is_bit {
                bit_args.push(self.tracked_bit_for_register(arg)?.clone());
            } else {
                qubit_args.push(self.tracked_qubit_for_register(arg)?.clone());
            }
        }
        Ok((qubit_args, bit_args))
    }

    /// Returns a tracked wire of the given type, containing registers from the
    /// [`TrackedQubit`]s, [`TrackedBit`]s, and [`LoadedParameter`]s in their
    /// given order.
    ///
    /// Returns an error if a valid wire cannot be found.
    ///
    /// The qubit and bit arguments are only consumed as required by the type,
    /// some registers may be left unused.
    ///
    /// If the wire type require additional conversion, some operations will be
    /// added to the Hugr to perform it.
    ///
    /// # Arguments
    ///
    /// * `config` - The configuration for the decoder, used to count the qubits
    ///   and bits required by each type.
    /// * `ty` - The type of the arguments we require in the wire.
    /// * `qubit_args` - The list of tracked qubits we require in the wire.
    ///   Values are consumed from the front and removed from the slice.
    /// * `bit_args` - The list of tracked bits we require in the wire.
    /// * `params` - The list of parameters to load to wire. See
    ///   [`WireTracker::load_half_turns_parameter`] for more details. Values
    ///   are consumed from the front and removed from the slice.
    /// * `unsupported_wire` - The id of an unsupported wire, if known.
    ///
    /// # Errors
    ///
    /// See [`WireTracker::find_typed_wires`] for possible errors.
    #[allow(clippy::too_many_arguments)]
    pub(in crate::serialize::pytket) fn find_typed_wire(
        &mut self,
        config: &PytketDecoderConfig,
        builder: &mut DFGBuilder<&mut Hugr>,
        ty: &Type,
        qubit_args: &mut &[TrackedQubit],
        bit_args: &mut &[TrackedBit],
        params: &mut &[LoadedParameter],
        unsupported_wire: Option<EncodedEdgeID>,
    ) -> Result<FoundWire, PytketDecodeError> {
        // TODO: Use the slice `split_off_first` method once MSRV is ≥1.87
        fn split_off_first<'a, T>(slice: &mut &'a [T]) -> Option<&'a T> {
            let (first, rem) = slice.split_first()?;
            *slice = rem;
            Some(first)
        }

        // Return a parameter input if the type is a float or rotation.
        if PARAMETER_TYPES.contains(ty) {
            let Some(param) = split_off_first(params) else {
                return Err(
                    PytketDecodeErrorInner::NoMatchingParameter { ty: ty.to_string() }.wrap(),
                );
            };
            if ty == param.wire_type() {
                return Ok(FoundWire::Parameter(*param));
            }
            // Convert between half-turn floats and rotations as needed.
            let param_ty = if ty == &float64_type() {
                ParameterType::FloatHalfTurns
            } else {
                ParameterType::Rotation
            };
            return Ok(FoundWire::Parameter(param.with_type(param_ty, builder)));
        }

        // Translate the wire type to a pytket register count.
        let Some(reg_count) = config.type_to_pytket(ty) else {
            return unsupported_wire
                .map(|id| FoundWire::Unsupported { id })
                .ok_or_else(|| {
                    PytketDecodeErrorInner::UnexpectedInputType {
                        unknown_type: ty.to_string(),
                        all_types: vec![ty.to_string()],
                    }
                    .wrap()
                });
        };

        // List candidate wires that contain the qubits and bits we need.
        let qubit_candidates = qubit_args
            .first()
            .into_iter()
            .flat_map(|qb| self.qubit_wires(qb));
        let bit_candidates = bit_args
            .first()
            .into_iter()
            .flat_map(|bit| self.bit_wires(bit));
        let candidates = qubit_candidates.chain(bit_candidates).collect_vec();

        // The bits and qubits we expect the wire to contain.
        let wire_qubits = qubit_args
            .iter()
            .take(reg_count.qubits)
            .cloned()
            .collect_vec();
        let wire_qubit_ids = wire_qubits.iter().map(|q| q.id()).collect_vec();
        let wire_bits = bit_args.iter().take(reg_count.bits).cloned().collect_vec();
        let wire_bit_ids = wire_bits.iter().map(|bit| bit.id()).collect_vec();

        // Find a wire that contains the correct type..
        let check_wire = |w: &Wire| {
            let wire_data = &self.wires[w];
            wire_data.qubits == wire_qubit_ids
                && wire_data.bits == wire_bit_ids
                && config.types_are_isomorphic(wire_data.ty(), ty)
        };
        let wire = match candidates.into_iter().find(check_wire) {
            Some(wire) => wire,
            // Handle lazy initialization of qubit and bit wires. These are
            // normally qubits/bits present in the pytket circuit definition,
            // but not in the region's input.
            _ if ty == &qb_t() => self.initialize_qubit_wire(builder, qubit_args[0].clone())?,
            _ if ty == &bool_t() || ty == &bool_type() => {
                self.initialize_bit_wire(builder, bit_args[0].clone())?
            }
            _ => {
                return Err(PytketDecodeErrorInner::NoMatchingWire {
                    ty: ty.to_string(),
                    qubit_args: qubit_args
                        .iter()
                        .map(|q| q.pytket_register().to_string())
                        .collect(),
                    bit_args: bit_args
                        .iter()
                        .map(|bit| bit.pytket_register().to_string())
                        .collect(),
                }
                .wrap());
            }
        };

        // Check that none of the selected qubit or bit has been marked as outdated.
        if let Some(qubit) = qubit_args
            .iter()
            .take(reg_count.qubits)
            .find(|q| q.is_outdated())
        {
            return Err(PytketDecodeErrorInner::OutdatedQubit {
                qubit: qubit.pytket_register().to_string(),
            }
            .wrap());
        }
        if let Some(bit) = bit_args
            .iter()
            .take(reg_count.bits)
            .find(|b| b.is_outdated())
        {
            return Err(PytketDecodeErrorInner::OutdatedBit {
                bit: bit.pytket_register().to_string(),
            }
            .wrap());
        }

        // Mark the qubits and bits as used.
        *qubit_args = &qubit_args[reg_count.qubits..];
        *bit_args = &bit_args[reg_count.bits..];

        // Convert the wire type, if needed.
        let wire_data = &self.wires[&wire];
        let new_wire = config.transform_typed_value(wire, wire_data.ty(), ty, builder)?;

        if wire == new_wire {
            Ok(FoundWire::Register(self.wires[&wire].clone()))
        } else {
            let ty: Arc<Type> = wire_data.ty.clone();
            self.track_wire(new_wire, ty, wire_qubits, wire_bits)?;
            self.mark_wire_outdated(wire);
            Ok(FoundWire::Register(self.wires[&new_wire].clone()))
        }
    }

    /// Returns a new [TrackedWires] set for a list of [`TrackedQubit`]s,
    /// [`TrackedBit`]s, and [`LoadedParameter`]s following the required types.
    ///
    /// Returns an error if a valid set of wires with the given types cannot be
    /// found.
    ///
    /// The qubit and bit arguments are only consumed as required by the types.
    /// Some registers may be left unused.
    ///
    /// If the wire type require additional conversion, some operations will be
    /// added to the Hugr to perform it.
    ///
    /// # Arguments
    ///
    /// * `config` - The configuration for the decoder, used to count the qubits and bits required by each type.
    /// * `types` - The types of the arguments we require in the wires.
    /// * `qubit_args` - The list of tracked qubits we require in the wires.
    /// * `bit_args` - The list of tracked bits we require in the wire.
    /// * `params` - The list of parameters to load to wires. See
    ///   [`WireTracker::load_half_turns_parameter`] for more details.
    ///
    /// # Errors
    ///
    /// - [`PytketDecodeErrorInner::OutdatedQubit`] if a qubit in `qubit_args` was marked as outdated.
    /// - [`PytketDecodeErrorInner::OutdatedBit`] if a bit in `bit_args` was marked as outdated.
    /// - [`PytketDecodeErrorInner::UnexpectedInputType`] if a type in `types` cannot be mapped to a [`RegisterCount`]
    /// - [`PytketDecodeErrorInner::NoMatchingWire`] if there is no wire with the requested type for the given qubit/bit arguments.
    pub(super) fn find_typed_wires(
        &mut self,
        config: &PytketDecoderConfig,
        builder: &mut DFGBuilder<&mut Hugr>,
        types: &[Type],
        mut qubit_args: &[TrackedQubit],
        mut bit_args: &[TrackedBit],
        mut params: &[LoadedParameter],
    ) -> Result<TrackedWires, PytketDecodeError> {
        // Map each requested type to a wire.
        //
        // Ignore parameter inputs.
        let mut tracked_wires = TrackedWires {
            value_wires: Vec::with_capacity(types.len() - params.len()),
            parameter_wires: Vec::with_capacity(params.len()),
        };
        for ty in types {
            match self.find_typed_wire(
                config,
                builder,
                ty,
                &mut qubit_args,
                &mut bit_args,
                &mut params,
                None,
            ) {
                Ok(FoundWire::Register(wire)) => tracked_wires.value_wires.push(wire),
                Ok(FoundWire::Parameter(param)) => tracked_wires.parameter_wires.push(param),
                Ok(FoundWire::Unsupported { .. }) => {
                    unreachable!("unsupported_wire was not defined")
                }
                // Add additional context to UnexpectedInputType errors.
                Err(PytketDecodeError {
                    inner: PytketDecodeErrorInner::UnexpectedInputType { unknown_type, .. },
                    pytket_op,
                    hugr_op,
                }) => {
                    let inner = PytketDecodeErrorInner::UnexpectedInputType {
                        unknown_type,
                        all_types: types.iter().map(ToString::to_string).collect(),
                    };
                    return Err(PytketDecodeError {
                        inner,
                        pytket_op,
                        hugr_op,
                    });
                }
                Err(e) => return Err(e),
            };
        }

        Ok(tracked_wires)
    }

    /// Loads the given parameter half-turns expression as a [`LoadedParameter`]
    /// in the hugr.
    ///
    /// - If the parameter is a known algebraic operation, adds the required op
    ///   and recurses on its inputs.
    /// - If the parameter is a constant, a constant definition is added to the
    ///   Hugr.
    /// - If the parameter is a variable, adds a new `rotation` input to the
    ///   region.
    /// - If the parameter is a sympy expressions, adds it as a
    ///   [`SympyOpDef`][crate::extension::sympy::SympyOpDef] black box.
    ///
    /// # Arguments
    ///
    /// * `hugr` - The hugr to load the parameters to.
    /// * `param` - The parameter expression to load.
    /// * `type_hint` - A hint for the type of the parameter we want to load.
    ///   This lets us decide between using [`ConstRotation`] and [`ConstF64`]
    ///   for constants. The actual returned type may be different.
    ///
    /// # Panics
    ///
    /// If the hugr builder does not support adding input wires.
    /// (That is, we're not building a FuncDefn or a DFG).
    pub fn load_half_turns_parameter(
        &mut self,
        hugr: &mut DFGBuilder<&mut Hugr>,
        param: &str,
        type_hint: Option<ParameterType>,
    ) -> LoadedParameter {
        /// Recursive parameter loading.
        ///
        /// `type_hint` is a hint for the type of the parameter we want to load.
        /// The actual returned type may be different.
        fn process(
            hugr: &mut DFGBuilder<&mut Hugr>,
            input_params: &mut IndexMap<String, LoadedParameter>,
            param_vars: &mut IndexSet<String>,
            unused_param_inputs: &mut VecDeque<LoadedParameter>,
            parsed: PytketParam,
            param: &str,
            type_hint: Option<ParameterType>,
        ) -> LoadedParameter {
            match parsed {
                PytketParam::Constant(half_turns) => match type_hint {
                    Some(ParameterType::FloatHalfTurns) | Some(ParameterType::FloatRadians) => {
                        let value: Value = ConstF64::new(half_turns).into();
                        let wire = hugr.add_load_const(value);
                        LoadedParameter::float_half_turns(wire)
                    }
                    _ => {
                        let value: Value = ConstRotation::new(half_turns).unwrap().into();
                        let wire = hugr.add_load_const(value);
                        LoadedParameter::rotation(wire)
                    }
                },
                PytketParam::Sympy(expr) => {
                    // store string in custom op.
                    let symb_op = symbolic_constant_op(expr.to_string());
                    let wire = hugr.add_dataflow_op(symb_op, []).unwrap().out_wire(0);
                    LoadedParameter::rotation(wire)
                }
                PytketParam::InputVariable { name } => {
                    // Special case for the name "pi": inserts a constant definition instead.
                    match (name, type_hint) {
                        ("pi", Some(ParameterType::FloatHalfTurns))
                        | ("pi", Some(ParameterType::FloatRadians)) => {
                            let value: Value = ConstF64::new(std::f64::consts::PI).into();
                            let wire = hugr.add_load_const(value);
                            LoadedParameter::float_half_turns(wire)
                        }
                        ("pi", _) => {
                            let value: Value =
                                ConstRotation::new(std::f64::consts::PI).unwrap().into();
                            let wire = hugr.add_load_const(value);
                            LoadedParameter::rotation(wire)
                        }
                        _ => {
                            // Look it up in the input parameters to the circuit, and add a new float input if needed.
                            *input_params.entry(name.to_string()).or_insert_with(|| {
                                param_vars.insert(name.to_string());
                                match unused_param_inputs.pop_front() {
                                    Some(loaded) => loaded,
                                    None => {
                                        let wire = hugr
                                            .add_input(rotation_type())
                                            .expect("Must be building a FuncDefn or a DFG");
                                        LoadedParameter::rotation(wire)
                                    }
                                }
                            })
                        }
                    }
                }
                PytketParam::Operation { op, args, param_ty } => {
                    // We assume all operations take float inputs.
                    let input_wires = args
                        .into_iter()
                        .map(|arg| {
                            let param = process(
                                hugr,
                                input_params,
                                param_vars,
                                unused_param_inputs,
                                arg,
                                param,
                                Some(param_ty),
                            );
                            param.with_type(param_ty, hugr).wire()
                        })
                        .collect_vec();
                    // If any of the following asserts panics, it means we added invalid ops to the sympy parser.
                    let res = hugr.add_dataflow_op(op, input_wires).unwrap_or_else(|e| {
                        panic!("Error while decoding pytket operation parameter \"{param}\". {e}",)
                    });
                    assert_eq!(res.num_value_outputs(), 1, "An operation decoded from the pytket op parameter \"{param}\" had {} outputs", res.num_value_outputs());
                    LoadedParameter::new(param_ty, res.out_wire(0))
                }
            }
        }

        process(
            hugr,
            &mut self.parameters,
            &mut self.parameter_vars,
            &mut self.unused_parameter_inputs,
            parse_pytket_param(param),
            param,
            type_hint,
        )
    }

    /// Track a new wire, updating any tracked elements that are present in it.
    pub fn track_wire(
        &mut self,
        wire: Wire,
        ty: Arc<Type>,
        qubits: impl IntoIterator<Item = TrackedQubit>,
        bits: impl IntoIterator<Item = TrackedBit>,
    ) -> Result<(), PytketDecodeError> {
        let qubits = qubits
            .into_iter()
            .map(|q| {
                self.track_qubit(q.pytket_register_arc(), None)
                    .map(TrackedQubit::id)
            })
            .collect::<Result<_, _>>()?;
        let bits = bits
            .into_iter()
            .map(|b| {
                self.track_bit(b.pytket_register_arc(), None)
                    .map(TrackedBit::id)
            })
            .collect::<Result<_, _>>()?;

        for &q in &qubits {
            self.qubit_wires[&q].push(wire);
        }
        for &b in &bits {
            self.bit_wires[&b].push(wire);
        }

        let wire_data = WireData {
            wire,
            ty,
            qubits,
            bits,
        };
        self.wires.insert(wire, wire_data);

        Ok(())
    }

    /// Associate an input wire to the region with a parameter.
    pub(super) fn register_input_parameter(
        &mut self,
        loaded: LoadedParameter,
        param: String,
    ) -> Result<(), PytketDecodeError> {
        let entry = self.parameters.entry(param.clone());
        if let indexmap::map::Entry::Occupied(_) = &entry {
            return Err(PytketDecodeErrorInner::DuplicatedParameter {
                param: entry.key().clone(),
            }
            .into());
        }
        self.parameter_vars.insert(param);
        entry.insert_entry(loaded);
        Ok(())
    }

    /// Track a parameter input to the region for which we don't have a variable name yet.
    pub(super) fn register_unused_parameter_input(&mut self, loaded: LoadedParameter) {
        self.unused_parameter_inputs.push_back(loaded);
    }

    /// Declare an `EncodeEdgeID` for a wire target into an inline subgraph
    /// payload's input.
    ///
    /// If the `EncodedEdgeID` has been registered before with
    /// [`Self::connect_unsupported_wire_source`], make the connection.
    ///
    /// Otherwise, register the edge id and the targets to be connected
    /// later.
    pub fn connect_unsupported_wire_targets(
        &mut self,
        id: EncodedEdgeID,
        targets: impl IntoIterator<Item = (Node, IncomingPort)>,
        hugr: &mut Hugr,
    ) {
        match self.unsupported_wires.entry(id).or_default() {
            UnsupportedWireState::Associated(wire) => {
                for (node, port) in targets {
                    hugr.connect(wire.node(), wire.source(), node, port);
                }
            }
            UnsupportedWireState::Pending(existing_targets) => {
                existing_targets.extend(targets);
            }
        }
    }

    /// Declare an `EncodeEdgeID` for a wire source from an inline subgraph
    /// payload's output.
    ///
    /// If any wire targets have been registered with
    /// [`Self::connect_unsupported_wire_target`], make the connections.
    pub fn connect_unsupported_wire_source(
        &mut self,
        id: EncodedEdgeID,
        wire: Wire,
        hugr: &mut Hugr,
    ) {
        match self
            .unsupported_wires
            .insert(id, UnsupportedWireState::Associated(wire))
        {
            None => {}
            Some(UnsupportedWireState::Pending(targets)) => {
                for (node, port) in targets {
                    hugr.connect(wire.node(), wire.source(), node, port);
                }
            }
            Some(UnsupportedWireState::Associated(existing_wire)) => {
                panic!(
                    "Tried to associate unsupported wire {id} with {wire}, but it has already been associated with {existing_wire}"
                );
            }
        }
    }

    /// Initialize a qubit wire that has been declared earlier.
    ///
    /// This is used when a qubit is declared in the pytket circuit definition,
    /// but not in the region's input.
    fn initialize_qubit_wire(
        &mut self,
        builder: &mut DFGBuilder<&mut Hugr>,
        qubit: TrackedQubit,
    ) -> Result<Wire, PytketDecodeError> {
        let wire = builder
            .add_dataflow_op(TketOp::QAlloc, [])
            .unwrap()
            .out_wire(0);
        self.track_wire(wire, qubit.ty(), [qubit], [])?;
        Ok(wire)
    }

    /// Initialize a bit wire that has been declared earlier.
    ///
    /// This is used when a bit is declared in the pytket circuit definition,
    /// but not in the region's input.
    fn initialize_bit_wire(
        &mut self,
        builder: &mut DFGBuilder<&mut Hugr>,
        bit: TrackedBit,
    ) -> Result<Wire, PytketDecodeError> {
        let wire = builder.add_load_const(Value::false_val());
        self.track_wire(wire, bit.ty(), [], [bit])?;
        Ok(wire)
    }
}

/// Only single-indexed registers are supported.
fn check_register(register: &PytketRegister) -> Result<(), PytketDecodeError> {
    if register.1.len() != 1 {
        Err(PytketDecodeErrorInner::MultiIndexedRegister {
            register: register.to_string(),
        }
        .into())
    } else {
        Ok(())
    }
}

/// Result type of [`WireTracker::find_typed_wire`].
///
/// Returns either a value to append to a [`TrackedWires`] instance, or a wire
/// for an edge in an unsupported subgraph.
///
/// The latter is only used internally when decoding unsupported subgraphs from
/// opaque pytket barriers. Users will see
/// [`PytketDecodeErrorInner::UnexpectedInputType`] if they try to decode such a
/// wire.
#[derive(Debug, Clone, PartialEq)]
pub(in crate::serialize::pytket) enum FoundWire {
    /// Found a type carrying bit/qubit registers.
    Register(WireData),
    /// Found a parameter input.
    Parameter(LoadedParameter),
    /// Found an unsupported wire, registered to an existing wire.
    ///
    /// This variant is only used when decoding unsupported subgraphs from
    /// opaque pytket barriers.
    Unsupported {
        /// The id of the unsupported wire.
        id: EncodedEdgeID,
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use hugr::extension::prelude::{bool_t, qb_t};
    use hugr::types::SumType;
    use hugr::Node;
    use rstest::{fixture, rstest};
    use std::sync::Arc;
    use tket_json_rs::register::ElementId;

    #[fixture]
    fn sample_wire(#[default(0)] wire_idx: usize) -> Wire {
        Wire::new(Node::from(portgraph::NodeIndex::new(wire_idx)), 0)
    }

    // Test basic WireTracker creation
    #[rstest]
    fn tracker_properties() {
        let mut tracker = WireTracker::with_capacity(5, 3);
        let qubit_reg = Arc::new(ElementId("q".to_string(), vec![0]));
        let bit_reg = Arc::new(ElementId("c".to_string(), vec![0]));
        let multi_indexed_reg = Arc::new(ElementId("q".to_string(), vec![0, 1]));
        let wire1 = sample_wire(1);

        // Initially, everything is empty - test through public methods
        assert_eq!(tracker.known_pytket_qubits().count(), 0);
        assert_eq!(tracker.known_pytket_bits().count(), 0);

        // Track an invalid register name.
        match tracker.track_qubit(multi_indexed_reg.clone(), None) {
            Err(PytketDecodeError {
                inner: PytketDecodeErrorInner::MultiIndexedRegister { register },
                ..
            }) => {
                assert_eq!(register, multi_indexed_reg.to_string());
            }
            e => panic!("Expected MultiIndexedRegister error, got {e:?}"),
        }

        // Getting the tracked qubits or bits for an unknown register should fail.
        match tracker.tracked_qubit_for_register(&qubit_reg) {
            Err(PytketDecodeError {
                inner: PytketDecodeErrorInner::UnknownQubitRegister { register },
                ..
            }) => {
                assert_eq!(register, qubit_reg.to_string());
            }
            e => panic!("Expected UnknownQubitRegister error, got {e:?}"),
        }
        match tracker.tracked_bit_for_register(&bit_reg) {
            Err(PytketDecodeError {
                inner: PytketDecodeErrorInner::UnknownBitRegister { register },
                ..
            }) => {
                assert_eq!(register, bit_reg.to_string());
            }
            e => panic!("Expected UnknownBitRegister error, got {e:?}"),
        }

        // Track a new qubit
        let tracked_q_0 = tracker
            .track_qubit(qubit_reg.clone(), None)
            .expect("Should track qubit")
            .clone();
        assert_eq!(tracker.known_pytket_qubits().count(), 1);
        assert_eq!(tracker.known_pytket_bits().count(), 0);
        let tracked_qubit = tracker
            .tracked_qubit_for_register(&qubit_reg)
            .expect("Should find tracked qubit")
            .clone();
        assert!(!tracked_qubit.is_outdated());
        assert_eq!(tracked_qubit, tracked_q_0);

        // Track the same qubit again, it should add a new TrackedQubit and mark the previous one as outdated
        let tracked_q_1 = tracker
            .track_qubit(qubit_reg.clone(), None)
            .expect("Should track qubit again")
            .clone();
        let tracked_q_0 = tracker.get_qubit(tracked_q_0.id());
        assert_eq!(tracker.known_pytket_qubits().count(), 1); // still only one unique register
        assert!(tracked_q_0.is_outdated());
        assert!(!tracked_q_1.is_outdated());
        let tracked_qubit = tracker
            .tracked_qubit_for_register(&qubit_reg)
            .expect("Should find latest tracked qubit")
            .clone();
        assert_eq!(tracked_qubit, tracked_q_1);

        // Track a bit
        let bit_id = tracker
            .track_bit(bit_reg.clone(), None)
            .expect("Should track bit")
            .clone();
        assert_eq!(tracker.known_pytket_bits().count(), 1);
        assert!(!bit_id.is_outdated());
        let tracked_bit = tracker
            .tracked_bit_for_register(&bit_reg)
            .expect("Should find tracked bit")
            .clone();
        assert_eq!(tracked_bit, bit_id);

        // Associate the bit and qubit with a wire.
        tracker
            .track_wire(
                wire1,
                Arc::new(SumType::new_tuple(vec![qb_t(), bool_t()]).into()),
                vec![tracked_qubit.clone()],
                vec![tracked_bit.clone()],
            )
            .expect("Should track wire");
    }
}
