//! Definitions of the payloads for opaque barrier metadata in pytket circuits.

use hugr::core::HugrNode;
use hugr::envelope::{EnvelopeConfig, EnvelopeError};
use hugr::extension::resolution::{resolve_type_extensions, WeakExtensionRegistry};
use hugr::extension::{ExtensionRegistry, ExtensionRegistryLoadError};
use hugr::package::Package;
use hugr::types::Type;
use hugr::{HugrView, Wire};
use itertools::Itertools;

use crate::serialize::pytket::opaque::OpaqueSubgraph;
use crate::serialize::pytket::{
    PytketDecodeError, PytketDecodeErrorInner, PytketEncodeError, PytketEncodeOpError,
};

use super::SubgraphId;

/// Pytket opgroup used to identify opaque barrier operations that encode opaque HUGR subgraphs.
///
/// See [`OpaqueSubgraphPayload`].
#[deprecated(
    note = "Opaque barriers do not set an opgroup anymore",
    since = "0.16.1"
)]
pub const OPGROUP_OPAQUE_HUGR: &str = "OPAQUE_HUGR";

/// Identifier for a wire in the Hugr, encoded as a 64-bit hash that is
/// detached from the node IDs of the in-memory Hugr.
///
/// These are used to identify edges in the [`OpaqueSubgraphPayload::Inline`]
/// payloads encoded in opaque barriers on the encoded pytket circuits.
///
/// We require them to reconstruct the edges of the hugr that are not reflected
/// as pytket register/parameter dependencies. This is the case for edges with
/// unsupported types between opaque subgraphs, or between an opaque
/// subgraph and a HUGR input/output node.
#[derive(
    Debug,
    derive_more::Display,
    Default,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Hash,
    serde::Serialize,
    serde::Deserialize,
)]
#[display("Edge#{id}", id = self.0)]
#[serde(transparent)]
pub struct EncodedEdgeID(u64);

impl EncodedEdgeID {
    /// Create a new subgraph hyper edge ID by hashing a Hugr wire.
    ///
    /// This is one of the few operations that can be done on arbitrary
    /// `HugrNode`s.
    pub fn new<N: HugrNode>(wire: Wire<N>) -> Self {
        let hash = fxhash::hash64(&wire);
        Self(hash)
    }
}

/// Payload for a pytket barrier metadata that indicates the barrier represents
/// an opaque HUGR subgraph.
///
/// The payload may be encoded inline, embedding the HUGR subgraph as an
/// envelope in the operation's metadata, or be a reference to a subgraph tracked
/// inside a [`EncodedCircuit`][super::super::circuit::EncodedCircuit]
/// structure.
///
/// Inline payloads encode their input and output boundaries that cannot be
/// encoded as pytket qubit/bit registers using [`EncodedEdgeID`]s independent
/// from the hugr. A circuit may mix barriers with both inline and external
/// payloads, as long as there are no edges requiring a [`EncodedEdgeID`]
/// between them.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(tag = "typ")]
pub enum OpaqueSubgraphPayload {
    /// A reference to a subgraph tracked by an `OpaqueSubgraphs` registry
    /// in an [`EncodedCircuit`][super::super::circuit::EncodedCircuit]
    /// structure.
    #[serde(rename = "HugrExternal")]
    External {
        /// The ID of the subgraph in the `OpaqueSubgraphs` registry.
        id: SubgraphId,
        /// Input parameters to the subgraph.
        ///
        /// Pytket may delete parameters specified on a barrier command, so we
        /// need to encode them here instead.
        input_params: Vec<String>,
    },
    /// An inline payload, carrying the encoded envelope for the HUGR subgraph.
    #[serde(rename = "HugrInline")]
    Inline {
        /// A string envelope containing the encoded HUGR subgraph.
        hugr_envelope: String,
        /// Input types of the subgraph.
        ///
        /// Each input is assigned a unique edge identifier, so we can
        /// reconstruct the connections that are not encoded in the pytket
        /// circuit.
        ///
        /// The types can also be inferred from the encoded hugr, but we store
        /// them here for ease of inspection.
        inputs: Vec<(Type, EncodedEdgeID)>,
        /// Output types of the subgraph.
        ///
        /// Each output is assigned a unique edge identifier, so we can reconstruct
        /// the connections that are not encoded in the pytket circuit.
        ///
        /// The types can also be inferred from the encoded hugr or linked
        /// subcircuit, but we store them here for robustness.
        outputs: Vec<(Type, EncodedEdgeID)>,
        /// Input parameters to the subgraph.
        ///
        /// Pytket may delete parameters specified on a barrier command, so we
        /// need to encode them here instead.
        input_params: Vec<String>,
    },
}

impl OpaqueSubgraphPayload {
    /// Create an external payload by referencing a subgraph in the tracked by
    /// an [`EncodedCircuit`][super::super::EncodedCircuit].
    pub fn new_external(
        subgraph_id: SubgraphId,
        input_params: impl IntoIterator<Item = String>,
    ) -> Self {
        Self::External {
            id: subgraph_id,
            input_params: input_params.into_iter().collect(),
        }
    }

    /// Create a new payload for an opaque subgraph in the Hugr.
    ///
    /// Encodes the subgraph into a hugr envelope.
    ///
    /// If the subgraph has non-local edges, they will be ignored. Re-inserting
    /// the subgraph into a Hugr may produce invalid disconnected ports in such
    /// cases.
    ///
    /// # Errors
    ///
    /// Returns an error if a node in the subgraph has children or calls a
    /// global function.
    pub fn new_inline<N: HugrNode>(
        subgraph: &OpaqueSubgraph<N>,
        hugr: &impl HugrView<Node = N>,
        input_params: impl IntoIterator<Item = String>,
    ) -> Result<Self, PytketEncodeError<N>> {
        let signature = subgraph.signature();

        let Some(opaque_hugr) = subgraph
            .is_sibling_subgraph_compatible()
            .then(|| subgraph.extract_subgraph(hugr).ok())
            .flatten()
        else {
            return Err(PytketEncodeOpError::UnsupportedStandaloneSubgraph {
                nodes: subgraph.nodes().iter().cloned().collect_vec(),
            }
            .into());
        };

        let mut inputs = Vec::with_capacity(subgraph.incoming_ports().len());
        for (inp_node, inp_port) in subgraph.incoming_ports() {
            let input_wire = Wire::from_connected_port(*inp_node, *inp_port, hugr);
            let edge_id = EncodedEdgeID::new(input_wire);
            inputs.push(edge_id);
        }

        let outputs = subgraph
            .outgoing_ports()
            .iter()
            .map(|(n, p)| EncodedEdgeID::new(Wire::new(*n, *p)));

        let hugr_envelope = Package::from_hugr(opaque_hugr)
            .store_str(EnvelopeConfig::text())
            .unwrap();

        Ok(Self::Inline {
            hugr_envelope,
            inputs: signature.input().iter().cloned().zip(inputs).collect(),
            outputs: signature.output().iter().cloned().zip(outputs).collect(),
            input_params: input_params.into_iter().collect(),
        })
    }

    /// Load a payload encoded in a json string.
    ///
    /// Updates weak extension references inside the definition after loading.
    pub fn load_str(json: &str, extensions: &ExtensionRegistry) -> Result<Self, PytketDecodeError> {
        let mut payload: Self = serde_json::from_str(json).map_err(|e| {
            PytketDecodeErrorInner::UnsupportedSubgraphInlinePayload {
                source: EnvelopeError::SerdeError { source: e },
            }
            .wrap()
        })?;

        // Resolve the extension ops and types in the inline payload.
        if let Self::Inline {
            inputs, outputs, ..
        } = &mut payload
        {
            let extensions: WeakExtensionRegistry = extensions.into();

            // Resolve the cached input/output types.
            for (ty, _) in inputs.iter_mut().chain(outputs.iter_mut()) {
                resolve_type_extensions(ty, &extensions).map_err(|e| {
                    let registry_load_e =
                        ExtensionRegistryLoadError::ExtensionResolutionError(Box::new(e));
                    let envelope_e = EnvelopeError::ExtensionLoad {
                        source: registry_load_e,
                    };
                    PytketDecodeErrorInner::UnsupportedSubgraphInlinePayload { source: envelope_e }
                        .wrap()
                })?;
            }
        }

        Ok(payload)
    }

    /// Returns `true` if the payload is an inline payload.
    pub fn is_inline(&self) -> bool {
        matches!(self, Self::Inline { .. })
    }

    /// Returns `true` if the payload is an external payload.
    pub fn is_external(&self) -> bool {
        matches!(self, Self::External { .. })
    }

    /// Parse the contents of an [`OpaqueSubgraphPayload::External`] from a
    /// string payload.
    ///
    /// Returns the subgraph ID and the list of input parameter expressions, if
    /// the payload is a valid [`OpaqueSubgraphPayload::External`].
    ///
    /// Returns `None` if the payload is [`OpaqueSubgraphPayload::Inline`] or
    /// not an [`OpaqueSubgraphPayload`].
    ///
    /// This method is more efficient than calling [Self::load_str], as it
    /// requires no allocations.
    pub fn parse_external_payload(payload: &str) -> Option<(SubgraphId, Vec<String>)> {
        #[derive(serde::Deserialize)]
        #[serde(rename = "HugrExternal")]
        #[serde(tag = "typ")]
        struct PartialPayload {
            pub id: SubgraphId,
            pub input_params: Vec<String>,
        }

        // Deserialize the payload if it is External, avoiding a full copy to memory
        // for the other variant.
        serde_json::from_str::<PartialPayload>(payload)
            .ok()
            .map(|payload| (payload.id, payload.input_params))
    }

    /// Returns `true` if a string encodes an [`OpaqueSubgraphPayload`].
    ///
    /// This method is more efficient than calling [Self::load_str], as it
    /// requires no allocations.
    pub fn is_valid_payload(payload: &str) -> bool {
        #[derive(serde::Deserialize)]
        #[serde(tag = "typ")]
        enum PartialPayload {
            HugrExternal {},
            HugrInline {},
        }

        // Deserialize the payload if it is External, avoiding a full copy to memory
        // for the other variant.
        serde_json::from_str::<PartialPayload>(payload).is_ok()
    }
}
