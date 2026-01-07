//! Extension encoder/decoders for the tket <-> `pytket` conversion.
//!
//! To add a new extension encoder, implement the [`PytketEmitter`] trait and add
//! it to the [`PytketEncoderConfig`](crate::serialize::pytket::PytketEncoderConfig)
//! used for decoding.
//!
//! To add a new extension decoder, implement the [`PytketDecoder`] trait and
//! add it to the [`PytketDecoderConfig`](crate::serialize::pytket::PytketDecoderConfig)
//! used for encoding.
//!
//! This module contains decoders for some common extensions. The
//! [`default_encoder_config`](crate::serialize::pytket::default_encoder_config)
//! creates a configuration with the decoders for the standard library and tket
//! extension.

mod bool;
mod core;
mod float;
mod prelude;
mod rotation;
mod tk1;
mod tket;

pub use bool::BoolEmitter;
pub use core::CoreDecoder;
pub use float::FloatEmitter;
pub use prelude::PreludeEmitter;
pub use rotation::RotationEmitter;
pub use tk1::Tk1Emitter;
pub use tket::TketOpEmitter;

pub(crate) use bool::set_bits_op;
pub(crate) use tk1::{OpaqueTk1Op, build_opaque_tket_op};

use super::encoder::TrackedValues;
use crate::Circuit;
use crate::serialize::pytket::config::TypeTranslatorSet;
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::encoder::{EncodeStatus, PytketEncoderContext};
use crate::serialize::pytket::{PytketDecodeError, PytketEncodeError};
use hugr::HugrView;
use hugr::extension::ExtensionId;
use hugr::ops::ExtensionOp;
use hugr::ops::constant::OpaqueValue;
use hugr::types::CustomType;

/// An encoder of HUGR operations and types that transforms them into pytket
/// primitives.
///
/// An [encoder configuration](crate::serialize::pytket::PytketEncoderConfig)
/// contains a list of such encoders. When encountering a type, operation, or
/// constant in the HUGR being encoded, the configuration will call each of
/// the encoders declaring support for the specific extension sequentially until
/// one of them indicates a successful conversion.
//
// Note: The HugrView type needs to be fixed at the type level rather than on
// the method signatures to ensure the trait is dyn compatible.
pub trait PytketEmitter<H: HugrView> {
    /// The name of the extension this encoder is for.
    ///
    /// [`PytketEmitter::op_to_pytket`] will only be called for operations/types
    /// of these extensions.
    ///
    /// If the function returns `None`, the encoder will be called for all
    /// operations/types irrespective of their extension.
    fn extensions(&self) -> Option<Vec<ExtensionId>>;

    /// Given a node in the HUGR circuit and its operation type, try to convert
    /// it to a pytket operation and add it to the pytket encoder.
    ///
    /// Returns an [`EncodeStatus`] indicating if the operation was successfully
    /// converted. If the operation is not supported by the encoder, it's
    /// important to **not** modify the `encoder` context as that may invalidate
    /// the context for other encoders that may be called afterwards.
    ///
    /// If [`PytketEmitter::extensions`] returns a list of extensions,
    /// `op` will be a custom operation from one of them.
    fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let _ = (node, op, circ, encoder);
        Ok(EncodeStatus::Unsupported)
    }

    /// Given an opaque constant value, add it to the pytket encoder and return
    /// the values to associate to the loaded constant.
    fn const_to_pytket(
        &self,
        value: &OpaqueValue,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<Option<TrackedValues>, PytketEncodeError<H::Node>> {
        let _ = (value, encoder);
        Ok(None)
    }
}

/// A decoder of pytket operations and types that transforms them into HUGR
/// primitives.
///
/// A [decoder configuration](crate::serialize::pytket::PytketDecoderConfig)
/// contains a list of such decoders.
pub trait PytketDecoder {
    /// A list of pytket's [`tket_json_rs::OpType`] supported by this decoder.
    ///
    /// [`PytketDecoder::op_to_hugr`] will only be called for commands
    /// containing these.
    fn op_types(&self) -> Vec<tket_json_rs::OpType>;

    /// Given a pytket [`tket_json_rs::circuit_json::Operation`] node in the
    /// HUGR circuit and its operation type, try to convert it to a pytket
    /// operation and add it to the pytket decoder.
    ///
    /// Returns an [`DecodeStatus`] indicating if the operation was successfully
    /// converted. If the operation is not supported by the decoder, it's
    /// important to **not** modify the `decoder` context as that may invalidate
    /// the context for other decoders that may be called afterwards.
    ///
    /// `op` will always have one of the [`tket_json_rs::OpType`]s specified in
    /// [`PytketDecoder::op_types`].
    fn op_to_hugr<'h>(
        &self,
        op: &tket_json_rs::circuit_json::Operation,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
        opgroup: Option<&str>,
        decoder: &mut PytketDecoderContext<'h>,
    ) -> Result<DecodeStatus, PytketDecodeError> {
        let _ = (op, qubits, bits, params, opgroup, decoder);
        Ok(DecodeStatus::Unsupported)
    }
}

/// A translator of HUGR types that describes how to encode them as pytket
/// registers (qubits, bits, and parameter expressions).
///
/// This is used both during encoding and decoding of pytket operations and
/// types.
pub trait PytketTypeTranslator {
    /// The name of the extensions this translator is for.
    ///
    /// [`PytketTypeTranslator::type_to_pytket`] will only be called for
    /// operations/types of these extensions.
    fn extensions(&self) -> Vec<ExtensionId>;

    /// Given a HUGR opaque type, return the number of qubits, bits, and
    /// parameter expressions of its pytket counterpart.
    ///
    /// If the type cannot be fully translated into a list of the aforementioned
    /// values, return `None`. Operations dealing with such types will be marked
    /// as unsupported and will be serialized as opaque operations.
    ///
    /// The `type_translators` argument is a set of known type translators. This
    /// can be used when translating types that wrap over other types.
    fn type_to_pytket(
        &self,
        typ: &CustomType,
        type_translators: &TypeTranslatorSet,
    ) -> Option<RegisterCount> {
        let _ = (typ, type_translators);
        None
    }
}

/// A count of pytket qubits, bits, and sympy parameters.
///
/// Used as return value for [`TrackedValues::count`].
#[derive(
    Clone,
    Copy,
    PartialEq,
    Eq,
    Hash,
    Debug,
    Default,
    derive_more::Display,
    derive_more::Add,
    derive_more::AddAssign,
    derive_more::Sub,
    derive_more::SubAssign,
    derive_more::Sum,
)]
#[display("{qubits} qubits, {bits} bits, {params} parameters")]
#[non_exhaustive]
pub struct RegisterCount {
    /// Amount of qubits.
    pub qubits: usize,
    /// Amount of bits.
    pub bits: usize,
    /// Amount of sympy parameters.
    pub params: usize,
}

impl RegisterCount {
    /// Create a new [`RegisterCount`] from the number of qubits, bits, and parameters.
    pub const fn new(qubits: usize, bits: usize, params: usize) -> Self {
        RegisterCount {
            qubits,
            bits,
            params,
        }
    }

    /// Create a new [`RegisterCount`] containing only qubits.
    pub const fn only_qubits(qubits: usize) -> Self {
        RegisterCount {
            qubits,
            bits: 0,
            params: 0,
        }
    }

    /// Create a new [`RegisterCount`] containing only bits.
    pub const fn only_bits(bits: usize) -> Self {
        RegisterCount {
            qubits: 0,
            bits,
            params: 0,
        }
    }

    /// Create a new [`RegisterCount`] containing only parameters.
    pub const fn only_params(params: usize) -> Self {
        RegisterCount {
            qubits: 0,
            bits: 0,
            params,
        }
    }

    /// Returns the number of qubits, bits, and parameters associated with the wire.
    pub const fn total(&self) -> usize {
        self.qubits + self.bits + self.params
    }

    /// Returns `true` if the register count is empty.
    pub const fn is_empty(&self) -> bool {
        self.qubits == 0 && self.bits == 0 && self.params == 0
    }
}
