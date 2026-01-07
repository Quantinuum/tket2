//! Configuration for converting [`tket_json_rs::circuit_json::SerialCircuit`]
//! into [`Circuit`]s.
//!
//! A configuration struct contains a list of custom decoders that define
//! translations of legacy tket primitives into HUGR operations.

use hugr::builder::DFGBuilder;
use hugr::types::Type;
use hugr::{Hugr, Wire};
use itertools::Itertools;
use std::collections::HashMap;

use crate::serialize::pytket::PytketDecodeError;
use crate::serialize::pytket::decoder::{
    DecodeStatus, LoadedParameter, PytketDecoderContext, TrackedBit, TrackedQubit,
};
use crate::serialize::pytket::extension::{PytketDecoder, PytketTypeTranslator, RegisterCount};

use super::TypeTranslatorSet;

/// Configuration for converting [`tket_json_rs::circuit_json::SerialCircuit`]
/// into [`Circuit`][crate::Circuit].
///
/// Contains custom decoders that define translations for HUGR operations,
/// types, and consts into pytket primitives.
#[derive(Default, derive_more::Debug)]
pub struct PytketDecoderConfig {
    /// Operation emitters
    #[debug(skip)]
    pub(super) decoders: Vec<Box<dyn PytketDecoder + Send + Sync>>,
    /// Pre-computed map from pytket optypes to corresponding decoders in
    /// `decoders`, identified by their index.
    #[debug("{:?}", optype_decoders.keys().collect_vec())]
    optype_decoders: HashMap<tket_json_rs::OpType, Vec<usize>>,
    /// Set of type translators used to translate HUGR types into pytket registers.
    type_translators: TypeTranslatorSet,
}

impl PytketDecoderConfig {
    /// Create a new [`PytketDecoderConfig`] with no decoders.
    pub fn new() -> Self {
        Self {
            decoders: vec![],
            optype_decoders: HashMap::new(),
            type_translators: TypeTranslatorSet::default(),
        }
    }

    /// Add a decoder to the configuration.
    pub fn add_decoder(&mut self, decoder: impl PytketDecoder + Send + Sync + 'static) {
        let idx = self.decoders.len();

        for optype in decoder.op_types() {
            self.optype_decoders.entry(optype).or_default().push(idx);
        }

        self.decoders.push(Box::new(decoder));
    }

    /// Add a type translator to the configuration.
    pub fn add_type_translator(
        &mut self,
        translator: impl PytketTypeTranslator + Send + Sync + 'static,
    ) {
        self.type_translators.add_type_translator(translator);
    }

    /// Encode a HUGR operation using the registered custom encoders.
    ///
    /// Returns `true` if the operation was successfully converted and no further
    /// encoders should be called.
    pub(in crate::serialize::pytket) fn op_to_hugr<'a>(
        &self,
        op: &tket_json_rs::circuit_json::Operation,
        qubits: &[TrackedQubit],
        bits: &[TrackedBit],
        params: &[LoadedParameter],
        opgroup: &Option<String>,
        decoder: &mut PytketDecoderContext<'a>,
    ) -> Result<DecodeStatus, PytketDecodeError> {
        let mut result = DecodeStatus::Unsupported;
        let opgroup = opgroup.as_deref();
        for enc in self.decoders_for_optype(&op.op_type) {
            result = enc.op_to_hugr(op, qubits, bits, params, opgroup, decoder)?;
            if result == DecodeStatus::Success {
                break;
            }
        }
        Ok(result)
    }

    /// Lists the decoder that can handle a given pytket optype.
    fn decoders_for_optype(
        &self,
        optype: &tket_json_rs::OpType,
    ) -> impl Iterator<Item = &Box<dyn PytketDecoder + Send + Sync>> + use<'_> {
        self.optype_decoders
            .get(optype)
            .into_iter()
            .flatten()
            .map(move |idx| &self.decoders[*idx])
    }

    /// Translate a HUGR type into a count of qubits, bits, and parameters,
    /// using the registered custom translator.
    ///
    /// Only tuple sums, bools, and custom types are supported.
    /// Other types will return `None`.
    pub fn type_to_pytket(&self, typ: &Type) -> Option<RegisterCount> {
        self.type_translators.type_to_pytket(typ)
    }

    /// Returns `true` if the two types are isomorphic. I.e. they can be translated
    /// into each other without losing information.
    pub fn types_are_isomorphic(&self, typ1: &Type, typ2: &Type) -> bool {
        self.type_translators.types_are_isomorphic(typ1, typ2)
    }

    /// Inserts the necessary operations to translate a type into an isomorphic
    /// type.
    ///
    /// This operation fails if [`Self::types_are_isomorphic`] returns `false`.
    pub(in crate::serialize::pytket) fn transform_typed_value(
        &self,
        wire: Wire,
        initial_type: &Type,
        target_type: &Type,
        builder: &mut DFGBuilder<&mut Hugr>,
    ) -> Result<Wire, PytketDecodeError> {
        self.type_translators
            .transform_typed_value(wire, initial_type, target_type, builder)
    }
}
