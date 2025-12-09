//! Subgraphs of unsupported (non-encodable) nodes in the hugr, and their
//! encoding as barrier metadata in pytket circuits.

mod payload;
mod subgraph;

pub use subgraph::OpaqueSubgraph;

pub use payload::{EncodedEdgeID, OpaqueSubgraphPayload};

#[expect(deprecated)]
pub use payload::OPGROUP_OPAQUE_HUGR;

use std::collections::BTreeMap;
use std::ops::Index;

use crate::serialize::pytket::PytketEncodeError;
use hugr::core::HugrNode;
use hugr::HugrView;

/// The ID of an [`OpaqueSubgraph`] registered in an `OpaqueSubgraphs` tracker.
#[derive(Debug, derive_more::Display, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[display("{tracker_id}.{local_id}")]
pub struct SubgraphId {
    /// The unique ID of the [`OpaqueSubgraphs`] instance that generated this ID.
    tracker_id: usize,
    /// A locally unique ID in the [`OpaqueSubgraphs`] instance.
    local_id: usize,
}

impl SubgraphId {
    /// Returns a unique parameter name for the `i`-th output parameter of the subgraph.
    pub(crate) fn output_parameter(&self, i: usize) -> String {
        format!(
            "p{tracker}_{local}_out{i}",
            tracker = self.tracker_id,
            local = self.local_id,
        )
    }
}

impl serde::Serialize for SubgraphId {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        (&self.tracker_id, &self.local_id).serialize(s)
    }
}

impl<'de> serde::Deserialize<'de> for SubgraphId {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let (tracker_id, local_id) = serde::Deserialize::deserialize(d)?;
        Ok(Self {
            tracker_id,
            local_id,
        })
    }
}

/// A set of subgraphs a HUGR that have been marked as _unsupported_ during a
/// pytket encoding.
#[derive(Debug, Clone)]
pub(super) struct OpaqueSubgraphs<N> {
    /// A unique ID for this instance of [`OpaqueSubgraphs`].
    ///
    /// New subgraphs encoded by this instance will have a globally unique ID
    /// composed of this index and a sequential ID.
    ///
    /// Note that the IDs in `opaque_subgraphs` may include instances with
    /// different tracker IDs if `merge` has been called.
    id: usize,
    /// Next local ID to assign to a new subgraph.
    next_local_id: usize,
    /// Sets of subgraphs in the HUGR that have been encoded as opaque barriers
    /// in the pytket circuit.
    ///
    /// Subcircuits are identified in the barrier metadata by their ID. See [`SubgraphId`].
    opaque_subgraphs: BTreeMap<SubgraphId, OpaqueSubgraph<N>>,
}

impl<N: HugrNode> OpaqueSubgraphs<N> {
    /// Create a new [`OpaqueSubgraphs`].
    ///
    /// # Arguments
    ///
    /// - `id`: A unique ID for this instance of [`OpaqueSubgraphs`], used
    ///   to generate new globally unique subgraph IDs.
    pub fn new(id: usize) -> Self {
        Self {
            id,
            next_local_id: 0,
            opaque_subgraphs: BTreeMap::new(),
        }
    }

    /// Register a new opaque subgraph in the Hugr.
    ///
    /// Returns and ID that can be used to identify the subgraph in the pytket circuit.
    pub fn register_opaque_subgraph(&mut self, subgraph: OpaqueSubgraph<N>) -> SubgraphId {
        let id = SubgraphId {
            local_id: self.next_local_id,
            tracker_id: self.id,
        };
        self.opaque_subgraphs.insert(id, subgraph);
        self.next_local_id += 1;
        id
    }

    /// Returns the opaque subgraph with the given ID.
    ///
    /// # Panics
    ///
    /// Panics if the ID is invalid.
    pub fn get(&self, id: SubgraphId) -> Option<&OpaqueSubgraph<N>> {
        self.opaque_subgraphs.get(&id)
    }

    /// Returns `true` if the opaque subgraph with the given ID exists.
    pub fn contains(&self, id: SubgraphId) -> bool {
        self.opaque_subgraphs.contains_key(&id)
    }

    /// Returns an iterator over the IDs of the opaque subgraphs.
    pub fn ids(&self) -> impl Iterator<Item = SubgraphId> + '_ {
        self.opaque_subgraphs.keys().copied()
    }

    /// Merge another [`OpaqueSubgraphs`] into this one.
    pub fn merge(&mut self, other: Self) {
        self.opaque_subgraphs.extend(other.opaque_subgraphs);
    }

    /// If the pytket command is a barrier operation encoding an opaque subgraph, replace its [`OpaqueSubgraphPayload::External`] pointer
    /// if present with a [`OpaqueSubgraphPayload::Inline`] payload.
    ///
    /// Ignores barriers whose data payload cannot be decoded into an [`OpaqueSubgraphPayload`].
    pub(super) fn inline_if_payload(
        &self,
        command: &mut tket_json_rs::circuit_json::Command,
        hugr: &impl HugrView<Node = N>,
    ) -> Result<(), PytketEncodeError<N>> {
        if command.op.op_type != tket_json_rs::OpType::Barrier {
            return Ok(());
        }
        let Some(payload) = command.op.data.take() else {
            return Ok(());
        };

        let Some((subgraph_id, input_arguments)) =
            OpaqueSubgraphPayload::parse_external_payload(&payload)
        else {
            // Not an External Payload, nothing to do.
            return Ok(());
        };
        if !self.contains(subgraph_id) {
            return Err(PytketEncodeError::custom(format!("Barrier operation with external subgraph payload points to an unknown subgraph: {subgraph_id}")));
        }

        let payload = OpaqueSubgraphPayload::new_inline(&self[subgraph_id], hugr, input_arguments)?;
        command.op.data = Some(serde_json::to_string(&payload).unwrap());

        Ok(())
    }
}

impl<N: HugrNode> Index<SubgraphId> for OpaqueSubgraphs<N> {
    type Output = OpaqueSubgraph<N>;

    fn index(&self, index: SubgraphId) -> &Self::Output {
        self.get(index)
            .unwrap_or_else(|| panic!("Invalid subgraph ID: {index}"))
    }
}

impl<N> Default for OpaqueSubgraphs<N> {
    fn default() -> Self {
        Self {
            id: 0,
            next_local_id: 0,
            opaque_subgraphs: BTreeMap::new(),
        }
    }
}
