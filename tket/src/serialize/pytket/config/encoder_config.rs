//! Configuration for converting [`Circuit`]s into
//! [`tket_json_rs::circuit_json::SerialCircuit`]
//!
//! A configuration struct contains a list of custom emitters that define
//! translations of HUGR operations and types into pytket primitives.

use std::collections::{BTreeSet, HashMap, VecDeque};

use hugr::extension::{ExtensionId, ExtensionSet};
use hugr::ops::{ExtensionOp, Value};
use hugr::types::{SumType, Type};

use crate::serialize::pytket::encoder::EncodeStatus;
use crate::serialize::pytket::extension::{set_bits_op, PytketTypeTranslator, RegisterCount};
use crate::serialize::pytket::{PytketEmitter, PytketEncodeError};
use crate::Circuit;

use super::super::encoder::{PytketEncoderContext, TrackedValues};
use super::TypeTranslatorSet;
use hugr::HugrView;
use itertools::Itertools;

/// Configuration for converting [`Circuit`] into
/// [`tket_json_rs::circuit_json::SerialCircuit`].
///
/// Contains custom emitters that define translations for HUGR operations,
/// types, and consts into pytket primitives.
#[derive(derive_more::Debug)]
#[debug(bounds(H: HugrView))]
pub struct PytketEncoderConfig<H: HugrView> {
    /// Operation emitters
    #[debug(skip)]
    pub(super) emitters: Vec<Box<dyn PytketEmitter<H>>>,
    /// Pre-computed map from extension ids to corresponding emitters in
    /// `emitters`, identified by their index.
    #[debug("{:?}", extension_emitters.keys().collect_vec())]
    extension_emitters: HashMap<ExtensionId, Vec<usize>>,
    /// Emitters that request to be called for all operations.
    no_extension_emitters: Vec<usize>,
    /// Set of type translators used to translate HUGR types into pytket registers.
    type_translators: TypeTranslatorSet,
}

impl<H: HugrView> PytketEncoderConfig<H> {
    /// Create a new [`PytketEncoderConfig`] with no encoders.
    pub fn new() -> Self {
        Self::default()
    }

    /// Add an encoder to the configuration.
    pub fn add_emitter(&mut self, encoder: impl PytketEmitter<H> + 'static) {
        let idx = self.emitters.len();

        match encoder.extensions() {
            Some(extensions) => {
                for ext in extensions {
                    self.extension_emitters.entry(ext).or_default().push(idx);
                }
            }
            // If the encoder does not specify an extension, it will be called
            // for all operations.
            None => self.no_extension_emitters.push(idx),
        }

        self.emitters.push(Box::new(encoder));
    }

    /// Add a type translator to the configuration.
    pub fn add_type_translator(
        &mut self,
        translator: impl PytketTypeTranslator + Send + Sync + 'static,
    ) {
        self.type_translators.add_type_translator(translator);
    }

    /// List the extensions supported by the encoders.
    ///
    /// Some encoders may not specify an extension, in which case they will be called
    /// for all operations irrespectively of the list returned here.
    ///
    /// Use [`PytketEncoderConfig::add_emitter`] to extend this list.
    pub fn supported_extensions(&self) -> impl Iterator<Item = &ExtensionId> {
        self.extension_emitters.keys()
    }

    /// Encode a HUGR operation using the registered custom encoders.
    ///
    /// Returns `true` if the operation was successfully converted and no further
    /// encoders should be called.
    pub fn op_to_pytket(
        &self,
        node: H::Node,
        op: &ExtensionOp,
        circ: &Circuit<H>,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        let mut result = EncodeStatus::Unsupported;
        let extension = op.def().extension_id();
        for enc in self.emitters_for_extension(extension) {
            if enc.op_to_pytket(node, op, circ, encoder)? == EncodeStatus::Success {
                result = EncodeStatus::Success;
                break;
            }
        }
        Ok(result)
    }

    /// Encode a const value into the pytket context using the registered custom
    /// encoders.
    ///
    /// Returns the values associated to the loaded constant, or `None` if the
    /// constant could not be encoded.
    pub fn const_to_pytket(
        &self,
        value: &Value,
        encoder: &mut PytketEncoderContext<H>,
    ) -> Result<Option<TrackedValues>, PytketEncodeError<H::Node>> {
        let mut values = TrackedValues::default();
        let mut queue = VecDeque::from([value]);
        while let Some(value) = queue.pop_front() {
            match value {
                Value::Sum(sum) => {
                    if sum.sum_type == SumType::new_unary(2) {
                        let new_bit = encoder.values.new_bit();
                        if value == &Value::true_val() {
                            let op = set_bits_op(&[true]);
                            encoder.emit_command(op, &[], &[new_bit], None);
                        }
                        return Ok(Some(TrackedValues::new_bits([new_bit])));
                    }
                    if sum.sum_type.as_tuple().is_some() {
                        for v in sum.values.iter() {
                            queue.push_back(v);
                        }
                    }
                }
                Value::Extension { e: opaque } => {
                    // Collect all extensions required to define the type.
                    let typ = opaque.value().get_type();
                    let type_exts = typ.used_extensions().unwrap_or_else(|e| {
                        panic!("Tried to encode a type with partially initialized extension. {e}");
                    });
                    let exts_set = ExtensionSet::from_iter(type_exts.ids().cloned());

                    let mut encoded = false;
                    for e in self.emitters_for_extensions(&exts_set) {
                        if let Some(vs) = e.const_to_pytket(opaque, encoder)? {
                            values.append(vs);
                            encoded = true;
                            break;
                        }
                    }
                    if !encoded {
                        return Ok(None);
                    }
                }
                _ => return Ok(None),
            }
        }
        Ok(Some(values))
    }

    /// Translate a HUGR type into a count of qubits, bits, and parameters,
    /// using the registered custom translator.
    ///
    /// Only tuple sums, bools, and custom types are supported.
    /// Other types will return `None`.
    pub fn type_to_pytket(&self, typ: &Type) -> Option<RegisterCount> {
        self.type_translators.type_to_pytket(typ)
    }

    /// Lists the emitters that can handle a given extension.
    fn emitters_for_extension(
        &self,
        ext: &ExtensionId,
    ) -> impl Iterator<Item = &Box<dyn PytketEmitter<H>>> + use<'_, H> {
        self.extension_emitters
            .get(ext)
            .into_iter()
            .flatten()
            .chain(self.no_extension_emitters.iter())
            .map(move |idx| &self.emitters[*idx])
    }

    /// Lists the emitters that can handle a given set of extensions.
    fn emitters_for_extensions(
        &self,
        exts: &ExtensionSet,
    ) -> impl Iterator<Item = &Box<dyn PytketEmitter<H>>> + use<'_, H> {
        let emitter_ids: BTreeSet<usize> = exts
            .iter()
            .flat_map(|ext| self.extension_emitters.get(ext).into_iter().flatten())
            .chain(self.no_extension_emitters.iter())
            .copied()
            .collect();
        emitter_ids.into_iter().map(move |idx| &self.emitters[idx])
    }
}

impl<H: HugrView> Default for PytketEncoderConfig<H> {
    fn default() -> Self {
        Self {
            emitters: Default::default(),
            extension_emitters: Default::default(),
            no_extension_emitters: Default::default(),
            type_translators: TypeTranslatorSet::default(),
        }
    }
}
