//! Option structs for the [TKETDecode][super::TKETDecode] trait methods.

use std::sync::Arc;

use hugr::extension::ExtensionRegistry;
use hugr::types::Signature;
use hugr::{Hugr, HugrView, Node};

use crate::serialize::pytket::{PytketDecoderConfig, PytketEncoderConfig};

use super::default_decoder_config;

/// Options used when decoding a pytket
/// [`SerialCircuit`][tket_json_rs::circuit_json::SerialCircuit] into a HUGR.
///
/// See [TKETDecode::decode][super::TKETDecode::decode].
///
/// In contrast to [PytketDecoderConfig] which is normally statically defined by
/// a library, these options may vary between calls.
///
/// The generic parameter `H` is the HugrView type of the Hugr that was encoded
/// into the pytket circuit, if any. This is required when the encoded pytket
/// circuit contains opaque barriers that reference subgraphs in the original
/// HUGR. See
/// [`OpaqueSubgraphPayload`][super::opaque::OpaqueSubgraphPayload]
/// for more details.
#[derive(Clone, Debug, Default)]
#[non_exhaustive]
pub struct DecodeOptions {
    /// The configuration for the decoder, containing custom
    /// operation decoders.
    ///
    /// When `None`, we will use [`default_decoder_config`][super::default_decoder_config].
    pub config: Option<Arc<PytketDecoderConfig>>,
    /// The signature of the function to create.
    ///
    /// The number of qubits in the input types must be less than or equal to the
    /// number of qubits in the circuit. Qubits not present in the input will
    /// will be allocated in the |0> state.
    ///
    /// If the signature input types contain fewer bits than those defined in the
    /// circuit, the remaining ones will be initialized to false internally.
    ///
    /// Float and rotation inputs in the signature will be associated with
    /// parameter names in `input_params`, or bound to variables in the
    /// circuit as they are found. The final circuit may contain additional
    /// parameter inputs, if required by the circuit arguments.
    ///
    /// If `None`, we will use qubits, bools, and
    /// [rotation_type][crate::extension::rotation::rotation_type] parameters.
    pub signature: Option<Signature>,
    /// A list of parameter names to add to the function input.
    ///
    /// If additional parameters are found in the circuit, they will be added
    /// after these using generic names.
    pub input_params: Vec<String>,
    /// The extensions to use when loading the HUGR envelope.
    ///
    /// When `None`, we will use a default registry that includes the prelude,
    /// std, TKET1, and TketOps extensions.
    pub extensions: Option<ExtensionRegistry>,
}

impl DecodeOptions {
    /// Create a new [`DecodeOptions`] with the default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set a decoder configuration.
    #[must_use]
    pub fn with_config(mut self, config: impl Into<Arc<PytketDecoderConfig>>) -> Self {
        self.config = Some(config.into());
        self
    }

    /// Set `DecodeOptions::config` to use [`default_decoder_config`].
    #[must_use]
    pub fn with_default_config(mut self) -> Self {
        self.config = Some(Arc::new(default_decoder_config()));
        self
    }

    /// Set the signature of the function to create.
    #[must_use]
    pub fn with_signature(mut self, signature: Signature) -> Self {
        self.signature = Some(signature);
        self
    }

    /// Set the input parameter names.
    #[must_use]
    pub fn with_input_params(mut self, input_params: impl IntoIterator<Item = String>) -> Self {
        self.input_params = input_params.into_iter().collect();
        self
    }

    /// Set the extensions to use when loading the HUGR envelope.
    #[must_use]
    pub fn with_extensions(mut self, extensions: ExtensionRegistry) -> Self {
        self.extensions = Some(extensions);
        self
    }

    /// Returns the extensions to use when loading the HUGR envelope.
    ///
    /// If the option is `None`, we will use a default registry that includes
    /// the prelude, std, TKET1, and TketOps extensions.
    pub fn extension_registry(&self) -> &ExtensionRegistry {
        self.extensions
            .as_ref()
            .unwrap_or(&crate::extension::REGISTRY)
    }
}

/// Where to insert the decoded circuit when calling
/// [`TKETDecode::decode_into`][super::TKETDecode::decode_into].
#[derive(Debug, derive_more::Display, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum DecodeInsertionTarget {
    /// Insert the decoded circuit as a new function in the HUGR.
    #[display("{}",
        match fn_name {
            Some(fn_name) => format!("Function({fn_name})"),
            None => "Function".to_string(),
        }
    )]
    Function {
        /// The name of the function to create.
        ///
        /// If `None`, we will use the encoded circuit's name, or "main" if the circuit has no name.
        fn_name: Option<String>,
    },
    /// Insert the decoded circuit as a dataflow region in the HUGR under the given parent.
    #[display("Region({parent})")]
    Region {
        /// The parent node that will contain the circuit's decoded DFG.
        parent: Node,
    },
}

impl DecodeInsertionTarget {
    /// Create a new [`DecodeInsertionTarget::Function`] with the default values.
    pub fn function(fn_name: impl ToString) -> Self {
        Self::Function {
            fn_name: Some(fn_name.to_string()),
        }
    }
}

impl Default for DecodeInsertionTarget {
    fn default() -> Self {
        Self::Function { fn_name: None }
    }
}

/// Options used when encoding a HUGR into a pytket
/// [`SerialCircuit`][tket_json_rs::circuit_json::SerialCircuit].
///
/// See [TKETDecode::encode][super::TKETDecode::encode].
///
/// In contrast to [PytketEncoderConfig] which is normally statically defined by
/// a library, these options may vary between calls.
#[derive(Clone)]
#[non_exhaustive]
pub struct EncodeOptions<H: HugrView = Hugr> {
    /// The configuration for the decoder, containing custom
    /// operation decoders.
    ///
    /// When `None`, we will use [`default_encoder_config`][super::default_encoder_config].
    pub config: Option<Arc<PytketEncoderConfig<H>>>,
    /// Whether to encode independent subcircuits for subregions of the HUGR
    /// that are descendants of unsupported operations.
    pub encode_subcircuits: bool,
}

impl<H: HugrView> EncodeOptions<H> {
    /// Create a new [`EncodeOptions`] with the default values.
    pub fn new() -> Self {
        Self::default()
    }

    /// Set a encoder configuration.
    pub fn with_config(mut self, config: impl Into<Arc<PytketEncoderConfig<H>>>) -> Self {
        self.config = Some(config.into());
        self
    }

    /// Set whether to encode independent subcircuits for subregions of the HUGR
    /// that are descendants of unsupported operations.
    pub fn with_subcircuits(mut self, encode_subcircuits: bool) -> Self {
        self.encode_subcircuits = encode_subcircuits;
        self
    }
}

impl<H: HugrView> Default for EncodeOptions<H> {
    fn default() -> Self {
        Self {
            config: Default::default(),
            encode_subcircuits: Default::default(),
        }
    }
}
