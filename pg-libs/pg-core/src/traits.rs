use crate::PauliGraph;

/// A transformation pass over a Pauli graph.
pub trait PGPass {
    /// Produces a transformed Pauli graph from the provided input graph.
    fn transform(&self, pg: &PauliGraph) -> PauliGraph;
}
