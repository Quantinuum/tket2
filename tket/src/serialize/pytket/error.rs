//! Error definitions for the pytket serialization.

use derive_more::{Display, Error, From};
use hugr::core::HugrNode;
use hugr::envelope::EnvelopeError;
use hugr::ops::OpType;
use hugr::Wire;
use itertools::Itertools;
use tket_json_rs::register::ElementId;

use crate::serialize::pytket::extension::RegisterCount;
use crate::serialize::pytket::opaque::SubgraphId;

/// Error type for conversion between pytket operations and tket ops.
#[derive(Display, derive_more::Debug, Error)]
#[non_exhaustive]
#[debug(bounds(N: HugrNode))]
pub enum PytketEncodeOpError<N: HugrNode = hugr::Node> {
    /// Tried to decode a tket1 operation with not enough parameters.
    #[display(
        "Operation {} is missing encoded parameters. Expected at least {expected} but only \"{}\" were specified.",
        optype,
        params.iter().join(", "),
    )]
    MissingSerialisedParams {
        /// The operation name.
        optype: OpType,
        /// The expected number of parameters.
        expected: usize,
        /// The given of parameters.
        params: Vec<String>,
    },
    /// Tried to decode a tket1 operation with not enough qubit/bit arguments.
    #[display(
        "Operation {} is missing encoded arguments. Expected {expected_qubits} and {expected_bits}, but only \"{args:?}\" were specified.",
        optype,
    )]
    MissingSerialisedArguments {
        /// The operation name.
        optype: OpType,
        /// The expected number of qubits.
        expected_qubits: usize,
        /// The expected number of bits.
        expected_bits: usize,
        /// The given of parameters.
        args: Vec<ElementId>,
    },
    /// Tried to query the values associated with an unexplored wire.
    ///
    /// This reflects a bug in the operation encoding logic of an operation.
    #[display("Could not find values associated with {wire}.")]
    WireHasNoValues {
        /// The wire that has no values.
        wire: Wire<N>,
    },
    /// Tried to add values to an already registered wire.
    ///
    /// This reflects a bug in the operation encoding logic of an operation.
    #[display("Tried to register values for wire {wire}, but it already has associated values.")]
    WireAlreadyHasValues {
        /// The wire that already has values.
        wire: Wire<N>,
    },
    /// Cannot encode subgraphs with nested structure or non-local edges in an standalone circuit.
    #[display("Cannot encode subgraphs with nested structure or non-local edges in an standalone circuit. Unsupported nodes: {}",
        nodes.iter().join(", "),
    )]
    UnsupportedStandaloneSubgraph {
        /// The nodes that are part of the unsupported subgraph.
        nodes: Vec<N>,
    },
}

/// Error type for conversion between tket ops and pytket operations.
#[derive(derive_more::Debug, Display, Error, From)]
#[non_exhaustive]
pub enum PytketEncodeError<N: HugrNode = hugr::Node> {
    /// Tried to encode a non-dataflow region.
    #[display("Cannot encode non-dataflow region at {region} with type {optype}.")]
    NonDataflowRegion {
        /// The region that is not a dataflow region.
        region: N,
        /// The operation type of the region.
        optype: String,
    },
    /// Operation conversion error.
    #[from]
    OpEncoding(PytketEncodeOpError<N>),
    /// Custom user-defined error raised while encoding an operation.
    #[display("Error while encoding operation: {msg}")]
    CustomError {
        /// The custom error message
        msg: String,
    },
    /// Tried to extract an standalone circuit from an
    /// [`EncodedCircuit`][super::circuit::EncodedCircuit] whose head region is
    /// not a dataflow container in the original hugr.
    #[display("Tried to extract an standalone circuit from an `EncodedCircuit` whose head region is not a dataflow container in the original hugr. Head operation {head_op}")]
    InvalidStandaloneHeadRegion {
        /// The head region operation that is not a dataflow container.
        head_op: String,
    },
    /// No qubits or bits to attach the barrier command to for unsupported nodes.
    #[display("An unsupported subgraph has no qubits or bits to attach the barrier command to{}",
        if params.is_empty() {"".to_string()} else {format!(" alongside its parameters [{}]", params.iter().join(", "))}
    )]
    #[deprecated(since = "0.16.1", note = "No longer emitted since 0.16.1")]
    #[from(ignore)]
    UnsupportedSubgraphHasNoRegisters {
        /// Parameter inputs to the unsupported subgraph.
        params: Vec<String>,
    },
}

impl<N: HugrNode> PytketEncodeError<N> {
    /// Create a new error with a custom message.
    pub fn custom(msg: impl ToString) -> Self {
        Self::CustomError {
            msg: msg.to_string(),
        }
    }
}

/// Error type for conversion between tket2 ops and pytket operations.
#[derive(derive_more::Debug, Display, Error)]
#[non_exhaustive]
#[display(
    "{inner}{context}",
    context = {
        match (pytket_op, hugr_op) {
            (Some(pytket_op), Some(hugr_op)) => format!(". While decoding a pytket {pytket_op} as a hugr {hugr_op}"),
            (Some(pytket_op), None) => format!(". While decoding a pytket {pytket_op}"),
            (None, Some(hugr_op)) => format!(". While decoding a hugr {hugr_op}"),
            (None, None) => String::new(),
        }
    },
)]
pub struct PytketDecodeError {
    /// The kind of error.
    pub inner: PytketDecodeErrorInner,
    /// The pytket operation that caused the error, if applicable.
    pub pytket_op: Option<String>,
    /// The hugr operation that caused the error, if applicable.
    pub hugr_op: Option<String>,
}

impl PytketDecodeError {
    /// Create a new error with a custom message.
    pub fn custom(msg: impl ToString) -> Self {
        PytketDecodeErrorInner::CustomError {
            msg: msg.to_string(),
        }
        .into()
    }

    /// Create an error for an unknown qubit register.
    pub fn unknown_qubit_reg(register: &tket_json_rs::register::ElementId) -> Self {
        PytketDecodeErrorInner::UnknownQubitRegister {
            register: register.to_string(),
        }
        .into()
    }

    /// Create an error for an unknown bit register.
    pub fn unknown_bit_reg(register: &tket_json_rs::register::ElementId) -> Self {
        PytketDecodeErrorInner::UnknownBitRegister {
            register: register.to_string(),
        }
        .into()
    }

    /// Add the pytket operation name to the error.
    pub fn pytket_op(mut self, op: &tket_json_rs::OpType) -> Self {
        self.pytket_op = Some(format!("{op}"));
        self
    }

    /// Add the hugr operation name to the error.
    pub fn hugr_op(mut self, op: impl ToString) -> Self {
        self.hugr_op = Some(op.to_string());
        self
    }
}

impl From<PytketDecodeErrorInner> for PytketDecodeError {
    fn from(inner: PytketDecodeErrorInner) -> Self {
        Self {
            inner,
            pytket_op: None,
            hugr_op: None,
        }
    }
}

/// Error variants of [`PytketDecodeError`], signalling errors during the
/// conversion between tket2 ops and pytket operations.
#[derive(derive_more::Debug, Display, Error)]
#[non_exhaustive]
pub enum PytketDecodeErrorInner {
    /// The pytket circuit uses multi-indexed registers.
    //
    // This could be supported in the future, if there is a need for it.
    #[display("Register {register} in the circuit has multiple indices. Tket2 does not support multi-indexed registers")]
    MultiIndexedRegister {
        /// The register name.
        register: String,
    },
    /// Found an unexpected register name.
    #[display("Found an unknown qubit register name: {register}")]
    UnknownQubitRegister {
        /// The unknown register name.
        register: String,
    },
    /// Found an unexpected bit register name.
    #[display("Found an unknown bit register name: {register}")]
    UnknownBitRegister {
        /// The unknown register name.
        register: String,
    },
    /// The given signature to use for the HUGR's input wires is not compatible with the number of qubits and bits in the pytket circuit.
    ///
    /// The expected number of qubits and bits may be different depending on the [`PytketTypeTranslator`][super::extension::PytketTypeTranslator]s used in the decoder config.
    #[display(
        "The given input types {input_types} to use for the HUGR's input wires are not compatible with the number of qubits and bits in the pytket circuit. Expected {expected_qubits} qubits and {expected_bits} bits, but found {circ_qubits} qubits and {circ_bits} bits",
        input_types = input_types.iter().join(", "),
    )]
    InvalidInputSignature {
        /// The given input types.
        input_types: Vec<String>,
        /// The expected number of qubits in the signature.
        expected_qubits: usize,
        /// The expected number of bits in the signature.
        expected_bits: usize,
        /// The number of qubits in the pytket circuit.
        circ_qubits: usize,
        /// The number of bits in the pytket circuit.
        circ_bits: usize,
    },
    /// The signature to use for the HUGR's output wires is not compatible with the number of qubits and bits in the pytket circuit.
    ///
    /// We don't do any kind of type conversion, so this depends solely on the last operation to update each register.
    #[display(
        "The expected output types {expected_types} are not compatible with the pytket circuit definition",
        expected_types = expected_types.iter().join(", "),
    )]
    InvalidOutputSignature {
        /// The expected types of the input wires.
        expected_types: Vec<String>,
    },
    /// A pytket operation had some input registers that couldn't be mapped to hugr wires.
    //
    // Some of this errors will be avoided in the future once we are able to decompose complex types automatically.
    #[display(
        "Could not find a wire with the required qubit arguments [{qubit_args:?}] and bit arguments [{bit_args:?}]",
        qubit_args = qubit_args.iter().join(", "),
        bit_args = bit_args.iter().join(", "),
    )]
    ArgumentCouldNotBeMapped {
        /// The qubit arguments that couldn't be mapped.
        qubit_args: Vec<String>,
        /// The bit arguments that couldn't be mapped.
        bit_args: Vec<String>,
    },
    /// Found an unexpected number of input wires when decoding an operation.
    #[display(
            "Expected {expected_values} input value wires{expected_types} and {expected_params} input parameters, but found {actual_values} values{actual_types} and {actual_params} parameters",
            expected_types = match expected_types {
                None => "".to_string(),
                Some(tys) => format!(" with types [{}]", tys.iter().join(", ")),
            },
            actual_types = match actual_types {
                None => "".to_string(),
                Some(tys) => format!(" with types [{}]", tys.iter().join(", ")),
            },
        )]
    UnexpectedInputWires {
        /// The expected amount of input wires.
        expected_values: usize,
        /// The expected amount of input parameters.
        expected_params: usize,
        /// The actual amount of input wires.
        actual_values: usize,
        /// The actual amount of input parameters.
        actual_params: usize,
        /// The expected types of the input wires.
        expected_types: Option<Vec<String>>,
        /// The actual types of the input wires.
        actual_types: Option<Vec<String>>,
    },
    /// Found an unexpected input type when decoding an operation.
    #[display(
        "Found an unexpected type {unknown_type} in the input wires, in input signature ({all_types})",
        all_types = all_types.iter().join(", "),
    )]
    UnexpectedInputType {
        /// The unknown type.
        unknown_type: String,
        /// All the input types specified for the operation.
        all_types: Vec<String>,
    },
    /// Tried to track the output wires of a node, but the number of tracked elements didn't match the ones in the output wires.
    #[display(
        "Tried to track the output wires of a node, but the number of tracked elements didn't match the ones in the output wires. Expected {expected_qubits} qubits and {expected_bits} bits, but found {circ_qubits} qubits and {circ_bits} bits in the node outputs"
    )]
    UnexpectedNodeOutput {
        /// The expected number of qubits.
        expected_qubits: usize,
        /// The expected number of bits.
        expected_bits: usize,
        /// The number of qubits in HUGR node outputs.
        circ_qubits: usize,
        /// The number of bits in HUGR node output.
        circ_bits: usize,
    },
    /// Custom user-defined error raised while encoding an operation.
    #[display("Error while decoding operation: {msg}")]
    CustomError {
        /// The custom error message
        msg: String,
    },
    /// Input parameter was defined multiple times.
    #[display("Parameter {param} was defined multiple times in the input signature")]
    DuplicatedParameter {
        /// The parameter name.
        param: String,
    },
    /// Not enough parameter names given for the input signature.
    #[display("Tried to initialize a pytket circuit decoder with {num_params_given} given parameter names, but more were required by the input signature")]
    MissingParametersInInput {
        /// The number of parameters given.
        num_params_given: usize,
    },
    /// We don't support complex types containing parameters in the input.
    //
    // This restriction may be relaxed in the future.
    #[display("Complex type {ty} contains {num_params} inside it. We only support input parameters in standalone 'float' or 'rotation'-typed wires")]
    UnsupportedParametersInInput {
        /// The type that contains the parameters.
        ty: String,
        /// The number of parameters in the type.
        num_params: usize,
    },
    /// We couldn't find a wire that contains the required type.
    #[display(
        "Could not find a wire with type {ty} that contains {expected_arguments}",
        expected_arguments = match (qubit_args.is_empty(), bit_args.is_empty()) {
            (true, true) => "no arguments".to_string(),
            (true, false) => format!("pytket bit arguments [{}]", bit_args.iter().join(", ")),
            (false, true) => format!("pytket qubit arguments [{}]", qubit_args.iter().join(", ")),
            (false, false) => format!("pytket qubit and bit arguments [{}] and [{}]", qubit_args.iter().join(", "), bit_args.iter().join(", ")),
        },
    )]
    NoMatchingWire {
        /// The type that couldn't be found.
        ty: String,
        /// The qubit registers expected in the wire.
        qubit_args: Vec<String>,
        /// The bit registers expected in the wire.
        bit_args: Vec<String>,
    },
    /// We couldn't find a parameter of the required input type.
    #[display("Could not find a parameter of the required input type '{ty}'")]
    NoMatchingParameter {
        /// The type that couldn't be found.
        ty: String,
    },
    /// The number of pytket registers passed to
    /// `PytketDecodeContext::wire_up_node` or `add_node_with_wires` does not
    /// match the number of registers required by the operation.
    #[display(
        "The operation input requires {expected_count} registers to cover input wires types [{expected_types}], but only got {actual_count}",
        expected_types = expected_types.iter().join(", "),
    )]
    NotEnoughInputRegisters {
        /// The types we tried to get wires for.
        expected_types: Vec<String>,
        /// The number of registers required by the types.
        expected_count: RegisterCount,
        /// The number of registers we actually got.
        actual_count: RegisterCount,
    },
    /// The number of pytket registers passed to
    /// `PytketDecodeContext::wire_up_node` or `add_node_with_wires` does not
    /// match the number of registers required by the operation.
    #[display(
        "The operation output requires {expected_count} registers to cover output wires types [{expected_types}], but only got {actual_count}",
           expected_types = expected_types.iter().join(", "),
       )]
    NotEnoughOutputRegisters {
        /// The types we tried to get wires for.
        expected_types: Vec<String>,
        /// The number of registers required by the types.
        expected_count: RegisterCount,
        /// The number of registers we actually got.
        actual_count: RegisterCount,
    },
    /// A qubit was marked as outdated, but was expected to be fresh.
    #[display("Discarded qubit {qubit} cannot be used as an input")]
    OutdatedQubit {
        /// The qubit that was marked as outdated.
        qubit: String,
    },
    /// A bit was marked as outdated, but was expected to be fresh.
    #[display("Discarded bit {bit} cannot be used as an input")]
    OutdatedBit {
        /// The bit that was marked as outdated.
        bit: String,
    },
    /// Tried to reassemble a circuit from a region that was not contained in the [`EncodedCircuit`][super::circuit::EncodedCircuit].
    #[display("Tried to reassemble a circuit from region {region}, but the circuit was not found in the `EncodedCircuit`")]
    NotAnEncodedRegion {
        /// The region we tried to decode
        region: String,
    },
    /// Tried to decode a circuit into an existing region, but the region was modified since creating the [`EncodedCircuit`][super::circuit::EncodedCircuit].
    #[display("Tried to decode a circuit into region {region}, but the region was modified since creating the `EncodedCircuit`. New region optype: {new_optype}")]
    IncompatibleTargetRegion {
        /// The region we tried to decode
        region: hugr::Node,
        /// The new region optype
        new_optype: OpType,
    },
    /// The pytket circuit contains an opaque barrier representing a unsupported subgraph in the original HUGR,
    /// but the corresponding subgraph is not present in the [`EncodedCircuit`][super::circuit::EncodedCircuit] structure.
    #[display("The pytket circuit contains a barrier representing an opaque subgraph in the original HUGR, but the corresponding subgraph is not present in the `EncodedCircuit` structure. Subgraph ID {id}")]
    OpaqueSubgraphNotFound {
        /// The ID of the opaque subgraph.
        id: SubgraphId,
    },
    /// The stored subgraph payload was not a valid flat subgraph in a dataflow region of the target hugr.
    #[display("The stored subgraph {id} was not a valid flat subgraph in a dataflow region of the target hugr.")]
    ExternalSubgraphWasModified {
        /// The ID of the opaque subgraph.
        id: SubgraphId,
    },
    /// Cannot decode Hugr from an unsupported subgraph payload in a pytket barrier operation.
    #[display("Cannot decode Hugr from an inline subgraph payload in a pytket barrier operation. {source}")]
    UnsupportedSubgraphInlinePayload {
        /// The envelope decoding error.
        source: EnvelopeError,
    },
    /// Cannot translate a wire from one type to another.
    #[display("Cannot translate {wire} from type {initial_type} to type {target_type}{}",
        context.as_ref().map(|s| format!(". {s}")).unwrap_or_default()
    )]
    CannotTranslateWire {
        /// The wire that couldn't be translated.
        wire: Wire,
        /// The initial type of the wire.
        initial_type: String,
        /// The target type of the wire.
        target_type: String,
        /// The error that occurred while translating the wire.
        context: Option<String>,
    },
}

impl PytketDecodeErrorInner {
    /// Wrap the error in a [`PytketDecodeError`].
    pub fn wrap(self) -> PytketDecodeError {
        PytketDecodeError::from(self)
    }
}
