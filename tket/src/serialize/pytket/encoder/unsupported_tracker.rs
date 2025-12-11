//! Tracking of subgraphs of unsupported nodes in the hugr.

use std::collections::{BTreeSet, HashMap};

use hugr::HugrView;
use hugr::core::HugrNode;
use petgraph::unionfind::UnionFind;

use crate::Circuit;
use crate::serialize::pytket::PytketEncodeError;
use crate::serialize::pytket::opaque::OpaqueSubgraph;

/// A structure for tracking nodes in the hugr that cannot be encoded as TKET1
/// operations.
///
/// The nodes are accumulated in connected components of the hugr. When they
/// cannot be grown further, each component is encoded as a single TKET1 barrier
/// containing the unsupported operations as metadata.
#[derive(Debug, Clone)]
pub struct UnsupportedTracker<N> {
    /// Unsupported nodes in the hugr.
    ///
    /// Stores the index of each node in [`Self::components`].
    ///
    /// Once a node has been extracted, it is removed from this map.
    nodes: HashMap<N, UnsupportedNode>,
    /// A UnionFind structure for tracking connected components of `Self::nodes`.
    components: UnionFind<usize>,
}

/// The connected component ID of a node.
///
/// Multiple IDs may be merged together via the union-find structure in
/// [`UnsupportedTracker::components`].
type ComponentId = usize;

#[derive(Debug, Clone, Copy, Default)]
struct UnsupportedNode {
    /// The index of the node in [`UnsupportedTracker::components`].
    component: ComponentId,
}

impl<N: HugrNode> UnsupportedTracker<N> {
    /// Create a new [`UnsupportedTracker`].
    pub fn new(_circ: &Circuit<impl HugrView>) -> Self {
        Self {
            nodes: HashMap::new(),
            components: UnionFind::new_empty(),
        }
    }

    /// Returns `true` if the node is tracked as unsupported.
    pub fn is_unsupported(&self, node: N) -> bool {
        self.nodes.contains_key(&node)
    }

    /// Record an unsupported node in the hugr.
    pub fn record_node(&mut self, node: N, circ: &Circuit<impl HugrView<Node = N>>) {
        let node_data = UnsupportedNode {
            component: self.components.new_set(),
        };
        self.nodes.insert(node, node_data);

        // Take the union of the component with any currently tracked incoming
        // neighbour.
        for neighbour in circ.hugr().input_neighbours(node) {
            if let Some(neigh_data) = self.nodes.get(&neighbour) {
                self.components
                    .union(neigh_data.component, node_data.component);
            }
        }
    }

    /// Returns the connected component of a node, and marks its elements as
    /// extracted.
    ///
    /// Once a component has been extracted, no new nodes can be added to it and
    /// calling [`UnsupportedTracker::record_node`] will use a new component
    /// instead.
    pub fn extract_component(
        &mut self,
        node: N,
        hugr: &impl HugrView<Node = N>,
    ) -> Result<OpaqueSubgraph<N>, PytketEncodeError<N>> {
        let node_data = self.nodes.remove(&node).unwrap();
        let component = node_data.component;
        let representative = self.components.find_mut(component);

        // Compute the nodes in the component, and mark them as extracted.
        //
        // TODO: Implement efficient iteration over the nodes in a component on petgraph,
        // and use it here. For now we just traverse all unextracted nodes.
        let mut nodes = BTreeSet::new();
        nodes.insert(node);
        for (&n, data) in &self.nodes {
            if self.components.find_mut(data.component) == representative {
                nodes.insert(n);
            }
        }
        for n in &nodes {
            self.nodes.remove(n);
        }

        OpaqueSubgraph::try_from_nodes(nodes, hugr)
    }

    /// Returns an iterator over the unextracted nodes in the tracker.
    pub fn iter(&self) -> impl Iterator<Item = N> + '_ {
        self.nodes.keys().copied()
    }

    /// Returns `true` if there are no unextracted components in the tracker.
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }
}

impl<N> Default for UnsupportedTracker<N> {
    fn default() -> Self {
        Self {
            nodes: HashMap::new(),
            components: UnionFind::new_empty(),
        }
    }
}
