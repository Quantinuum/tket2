//! Subgraph representation using intervals on resource paths.

use core::panic;
use std::cmp::Ordering;

use derive_more::derive::{Display, Error};
use hugr::{Direction, HugrView, IncomingPort, OutgoingPort, Port, core::HugrNode};
use itertools::{Either, Itertools};

use crate::resource::{Position, ResourceId, ResourceScope};

/// A non-empty interval on a resource path.
///
/// An interval is a subpath on a resource path: it starts and ends at specific
/// nodes on the resource path and contains all nodes and edges between the
/// start and the end nodes.
///
/// ### Interval validity
///
/// [`Interval`]s store the node IDs and port offsets of their start and end
/// points, and will become invalid (undefined behaviour) if used on a Hugr
/// graph where those nodes or ports have been modified. Changes to the Hugr
/// graph outside the interval are safe. Changes within the interval are also
/// safe as long as they do not modify the start and end nodes and both nodes
/// are still on the same resource path after the modification.
/// Intervals are independent of the ResourceScope they are used with, and in
/// particular are independent of the choice of node positions and resource IDs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Interval<N> {
    /// An interval containing a single node.
    ///
    /// We identify the resource path this interval is on by providing a port on
    /// that path.
    /// There may be both an incoming and an outgoing port associated with the
    /// resource path. In that case, either port can be used.
    Singleton {
        /// A node and port on the resource path.
        start_or_end: (N, Port),
    },

    /// An interval path of length > 1, starting from an outgoing port to an
    /// incoming port.
    ///
    /// The `end` is always in the future of `start` and on the same resource
    /// path.
    Span {
        /// The start of the subpath: an outgoing port on the resource path.
        start: (N, OutgoingPort),
        /// The end of the subpath: an incoming port on the resource path.
        end: (N, IncomingPort),
    },
}

impl<N: HugrNode> Interval<N> {
    /// Create an interval that spans a single node, on the given resource path.
    ///
    /// If `node` is not on `resource_id`, then `None` is returned.
    pub fn new_singleton(
        resource_id: ResourceId,
        node: N,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Option<Self> {
        let in_port = scope.get_port(node, resource_id, Direction::Incoming);
        let out_port = scope.get_port(node, resource_id, Direction::Outgoing);
        let port = in_port.or(out_port)?;

        Some(Self::Singleton {
            start_or_end: (node, port),
        })
    }

    /// Create an interval that spans from an outgoing port to an incoming port.
    ///
    /// This will panic if the extrema do not correspond to exactly one outgoing
    /// and one incoming port.
    ///
    /// The two extrema nodes must be different, otherwise use
    /// [`Interval::new_singleton`].
    fn new_span(extrema: [(N, Port); 2]) -> Self {
        let mut start = None;
        let mut end = None;
        for (node, port) in extrema {
            match port.as_directed() {
                Either::Left(incoming) => {
                    if end.replace((node, incoming)).is_some() {
                        panic!("multiple incoming ports in span extrema");
                    }
                }
                Either::Right(outgoing) => {
                    if start.replace((node, outgoing)).is_some() {
                        panic!("multiple outgoing ports in span extrema");
                    }
                }
            }
        }

        debug_assert!(
            start.map(|s| s.0) != end.map(|e| e.0),
            "start and end nodes must differ"
        );

        Self::Span {
            start: start.expect("exactly one outgoing port in span extrema"),
            end: end.expect("exactly one incoming port in span extrema"),
        }
    }

    /// Create an interval for a range of nodes.
    ///
    /// This will panic if
    ///  - either node is not on the resource path `resource_id`
    ///  - or the start node is after the end node on the resource path.
    ///
    /// Use [`Interval::try_new`] instead for creating an interval with error
    /// handling.
    pub fn new(
        resource_id: ResourceId,
        start_node: N,
        end_node: N,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Self {
        Self::try_new(resource_id, start_node, end_node, scope).unwrap()
    }

    /// Create an interval for a range of nodes.
    pub fn try_new(
        resource_id: ResourceId,
        start_node: N,
        end_node: N,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<Self, InvalidInterval<N>> {
        let start_pos = scope
            .get_position(start_node)
            .ok_or(InvalidInterval::NotOnResourcePath(start_node))?;
        let end_pos = scope
            .get_position(end_node)
            .ok_or(InvalidInterval::NotOnResourcePath(end_node))?;

        if start_pos > end_pos {
            return Err(InvalidInterval::StartAfterEnd(
                start_node,
                end_node,
                resource_id,
            ));
        }

        if start_node == end_node {
            Interval::new_singleton(resource_id, start_node, scope)
                .ok_or(InvalidInterval::NotOnResourcePath(start_node))
        } else {
            let start_port = scope
                .get_port(start_node, resource_id, Direction::Outgoing)
                .ok_or(InvalidInterval::NotOnResourcePath(start_node))?;
            let end_port = scope
                .get_port(end_node, resource_id, Direction::Incoming)
                .ok_or(InvalidInterval::NotOnResourcePath(end_node))?;
            Ok(Self::new_span([
                (start_node, start_port),
                (end_node, end_port),
            ]))
        }
    }

    /// Get the resource ID of the interval.
    pub fn resource_id(&self, scope: &ResourceScope<impl HugrView<Node = N>>) -> ResourceId {
        let (node, port) = self.any_port();

        scope
            .get_resource_id(node, port)
            .expect("interval port is a resource port in scope")
    }

    /// The first port on the resource path after the interval.
    ///
    /// This may be None if the interval ends at the end of the resource path.
    #[inline]
    pub fn outgoing_boundary_port(
        &self,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Option<(N, OutgoingPort)> {
        self.boundary_port(Direction::Outgoing, scope)
            .map(|(n, p)| (n, p.as_outgoing().expect("outgoing port")))
    }

    /// The last port on the resource path before the interval.
    ///
    /// This may be None if the interval starts at the beginning of the resource
    /// path.
    #[inline]
    pub fn incoming_boundary_port(
        &self,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Option<(N, IncomingPort)> {
        self.boundary_port(Direction::Incoming, scope)
            .map(|(n, p)| (n, p.as_incoming().expect("incoming port")))
    }

    /// The port on the resource path before the beginning or after the end of
    /// the interval.
    ///
    /// This may be None if the interval start/end matches the resource path
    /// start/end.
    pub fn boundary_port(
        &self,
        direction: Direction,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Option<(N, Port)> {
        let (node, port) = match (*self, direction) {
            (Interval::Singleton { start_or_end }, _) => start_or_end,
            (Interval::Span { start, .. }, Direction::Incoming) => (start.0, start.1.into()),
            (Interval::Span { end, .. }, Direction::Outgoing) => (end.0, end.1.into()),
        };
        let resource_id = scope
            .get_resource_id(node, port)
            .expect("interval port is a resource port in scope");
        scope
            .get_port(node, resource_id, direction)
            .map(|port| (node, port))
    }

    /// Get the start node of the interval.
    pub fn start_node(&self) -> N {
        match *self {
            Interval::Singleton {
                start_or_end: (node, _),
            } => node,
            Interval::Span {
                start: (start_node, _),
                ..
            } => start_node,
        }
    }

    /// Get the end node of the interval.
    pub fn end_node(&self) -> N {
        match *self {
            Interval::Singleton {
                start_or_end: (node, _),
            } => node,
            Interval::Span {
                end: (end_node, _), ..
            } => end_node,
        }
    }

    /// Extend the interval to include the given node.
    ///
    /// Return the direction the interval was extended in, that is:
    ///  - if `node` was just before the interval, return `Direction::Incoming`
    ///  - if `node` was just after the interval, return `Direction::Outgoing`
    ///  - if `node` was already in the interval, return `None`
    ///
    /// If `node` is not contiguous with the interval, or if `node` is not
    /// on the interval's resource path, return an error.
    pub fn try_extend(
        &mut self,
        node: N,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Result<Option<Direction>, InvalidInterval<N>> {
        let Some(pos) = scope.get_position(node) else {
            return Err(InvalidInterval::NotOnResourcePath(node));
        };

        match self.position_in_interval(pos, scope) {
            // pos is already within the interval
            Ordering::Equal => {
                return Ok(None);
            }
            // pos is before the interval
            Ordering::Less => {
                let (in_node, in_port) = self
                    .incoming_boundary_port(scope)
                    .ok_or(InvalidInterval::NotContiguous(node))?;
                let (prev_node, _) = scope
                    .hugr()
                    .single_linked_output(in_node, in_port)
                    .ok_or(InvalidInterval::NotContiguous(node))?;
                if prev_node != node {
                    return Err(InvalidInterval::NotContiguous(node));
                }
            }
            // pos is after the interval
            Ordering::Greater => {
                let (out_node, out_port) = self
                    .outgoing_boundary_port(scope)
                    .ok_or(InvalidInterval::NotContiguous(node))?;
                let (next_node, _) = scope
                    .hugr()
                    .single_linked_input(out_node, out_port)
                    .ok_or(InvalidInterval::NotContiguous(node))?;
                if next_node != node {
                    return Err(InvalidInterval::NotContiguous(node));
                }
            }
        };

        Ok(self.add_node_unchecked(node, scope))
    }

    /// Include the given node in the interval.
    ///
    /// Does not check if the node is contiguous with the interval. Use
    /// [`Interval::try_extend`] instead for a safe way to extend an interval.
    ///
    /// Return the direction the interval was extended in.
    pub(crate) fn add_node_unchecked(
        &mut self,
        node: N,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Option<Direction> {
        let pos = scope
            .get_position(node)
            .expect("node must be on resource path");
        let resource_id = self.resource_id(scope);

        let extension_dir = match self.position_in_interval(pos, scope) {
            Ordering::Less => Direction::Incoming,
            Ordering::Greater => Direction::Outgoing,
            Ordering::Equal => {
                // Node is already within the interval
                return None;
            }
        };

        let new_extrema_node = node;
        let new_extrema_port = scope
            .get_port(new_extrema_node, resource_id, extension_dir.reverse())
            .expect("node is on interval resource path");
        let existing_extrema = match (*self, extension_dir) {
            (Interval::Span { end: (n, p), .. }, Direction::Incoming) => (n, p.into()),
            (Interval::Span { start: (n, p), .. }, Direction::Outgoing) => (n, p.into()),
            (Interval::Singleton { start_or_end }, dir) => {
                ensure_direction_resource_port(start_or_end, dir, scope)
                    // start_or_end cannot be the start/end of the resource path,
                    // as we know there is `node` beyond it.
                    .expect("not a resource path end")
            }
        };

        debug_assert!(new_extrema_node != existing_extrema.0,);

        *self = Self::new_span([(new_extrema_node, new_extrema_port), existing_extrema]);

        Some(extension_dir)
    }

    /// Whether `pos` is smaller, larger or within the interval.
    ///
    /// [Ordering] is used to express the relative position of `pos` with
    /// respect to the interval:
    /// - [`Ordering::Less`] if `pos` is before the interval,
    /// - [`Ordering::Greater`] if `pos` is after the interval, and
    /// - [`Ordering::Equal`] if `pos` is within the interval.
    fn position_in_interval(
        &self,
        pos: Position,
        scope: &ResourceScope<impl HugrView<Node = N>>,
    ) -> Ordering {
        if pos < self.start_pos(scope) {
            Ordering::Less
        } else if pos > self.end_pos(scope) {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }

    /// Get a port (unspecified) on the resource path of the interval.
    fn any_port(&self) -> (N, Port) {
        match *self {
            Interval::Singleton {
                start_or_end: node_port,
            } => node_port,
            Interval::Span {
                start: (out_node, out_port),
                ..
            } => (out_node, out_port.into()),
        }
    }

    #[inline]
    fn start_pos(&self, scope: &ResourceScope<impl HugrView<Node = N>>) -> Position {
        let start_node = self.start_node();
        scope
            .get_position(start_node)
            .expect("valid interval start node")
    }

    #[inline]
    fn end_pos(&self, scope: &ResourceScope<impl HugrView<Node = N>>) -> Position {
        let end_node = self.end_node();
        scope
            .get_position(end_node)
            .expect("valid interval end node")
    }
}

/// Make sure `port` is in the given `dir` direction.
///
/// If it is not, get the opposite port on the same resource path.
fn ensure_direction_resource_port<N: HugrNode>(
    (node, port): (N, Port),
    dir: Direction,
    scope: &ResourceScope<impl HugrView<Node = N>>,
) -> Option<(N, Port)> {
    if dir == port.direction() {
        Some((node, port))
    } else {
        let resource_id = scope
            .get_resource_id(node, port)
            .expect("interval port is a resource port in scope");
        let port = scope.get_port(node, resource_id, dir)?;
        Some((node, port))
    }
}

/// Errors that can occur when extending an interval.
#[derive(Debug, Clone, PartialEq, Display, Error)]
pub enum InvalidInterval<N> {
    /// The node is not contiguous with the interval.
    #[display("node {_0:?} is not contiguous with the interval")]
    NotContiguous(N),
    /// The node is not on the interval's resource path.
    #[display("node {_0:?} is not on the interval's resource path")]
    NotOnResourcePath(N),
    /// The start node is after the end node.
    #[display("start node {_0:?} is after end node {_1:?} on resource path {_2:?}")]
    StartAfterEnd(N, N, ResourceId),
}

impl<H: HugrView> ResourceScope<H> {
    /// Get the nodes in an interval.
    pub fn nodes_in_interval(
        &self,
        interval: Interval<H::Node>,
    ) -> impl Iterator<Item = H::Node> + '_ {
        let start_node = interval.start_node();
        let end_node = interval.end_node();
        let resource_id = interval.resource_id(self);
        self.resource_path_iter(resource_id, start_node, Direction::Outgoing)
            .take_while_inclusive(move |&node| node != end_node)
    }
}

#[cfg(test)]
mod tests {
    use super::{ResourceScope, *};
    use std::ops::RangeInclusive;

    use crate::{Circuit, resource::tests::cx_circuit};

    use itertools::Itertools;
    use rstest::{fixture, rstest};

    #[test]
    fn test_nodes_in_interval() {
        let circ = cx_circuit(5);
        let subgraph = Circuit::from(&circ).subgraph().unwrap();
        let cx_nodes = subgraph.nodes().to_owned();
        let scope = ResourceScope::new(&circ, subgraph);

        assert_eq!(cx_nodes.len(), 5);

        let pos_interval = 1usize..4;

        for resource_id in [0, 1].map(ResourceId::new) {
            let interval = Interval::new(
                resource_id,
                cx_nodes[pos_interval.start],
                cx_nodes[pos_interval.end - 1],
                &scope,
            );

            assert_eq!(
                Interval::new(
                    resource_id,
                    cx_nodes[pos_interval.start],
                    cx_nodes[pos_interval.end - 1],
                    &scope
                ),
                interval
            );

            assert_eq!(
                scope.nodes_in_interval(interval).collect_vec(),
                cx_nodes[pos_interval.clone()]
            );
        }
    }

    #[fixture]
    fn cx_circuit_scope() -> ResourceScope {
        let circ = cx_circuit(5);
        ResourceScope::from_circuit(Circuit::from(circ))
    }

    #[rstest]
    #[case::extend_left(
        1,
        Some(Direction::Incoming),
        1..=3,
    )]
    #[case::extend_right(
        4,
        Some(Direction::Outgoing),
        2..=4,
    )]
    #[case::node_already_in_interval_start(
        2,
        None,
        2..=3,
    )]
    #[case::node_already_in_interval_end(
        3,
        None,
        2..=3,
    )]
    fn test_try_extend_success(
        cx_circuit_scope: ResourceScope,
        #[case] node_to_extend: usize,
        #[case] expected_direction: Option<Direction>,
        #[case] expected_range: RangeInclusive<usize>,
    ) {
        let cx_nodes = cx_circuit_scope.nodes();

        // Create an interval from nodes 2 to 3 (middle of circuit)
        let mut interval = Interval::new(
            ResourceId::new(0),
            cx_nodes[2],
            cx_nodes[3],
            &cx_circuit_scope,
        );

        // Apply the test case
        let result = interval
            .try_extend(cx_nodes[node_to_extend], &cx_circuit_scope)
            .unwrap();

        assert_eq!(result, expected_direction);
        assert_eq!(interval.start_node(), cx_nodes[*expected_range.start()]);
        assert_eq!(interval.end_node(), cx_nodes[*expected_range.end()]);
    }

    #[rstest]
    #[case::extend_left(
        1,
        Some(Direction::Incoming),
        1..=2,
    )]
    #[case::extend_right(
        3,
        Some(Direction::Outgoing),
        2..=3,
    )]
    #[case::node_already_in_interval_start(
        2,
        None,
        2..=2,
    )]
    fn test_try_extend_singleton(
        cx_circuit_scope: ResourceScope,

        #[case] node_to_extend: usize,
        #[case] expected_direction: Option<Direction>,
        #[case] expected_range: RangeInclusive<usize>,
    ) {
        let cx_nodes = cx_circuit_scope.nodes();

        // Create a singleton interval
        let mut interval = Interval::new(
            ResourceId::new(0),
            cx_nodes[2],
            cx_nodes[2],
            &cx_circuit_scope,
        );

        assert_eq!(
            interval,
            Interval::new_singleton(ResourceId::new(0), cx_nodes[2], &cx_circuit_scope).unwrap()
        );

        // Apply the test case
        let result = interval
            .try_extend(cx_nodes[node_to_extend], &cx_circuit_scope)
            .unwrap();

        assert_eq!(result, expected_direction);
        assert_eq!(interval.start_node(), cx_nodes[*expected_range.start()]);
        assert_eq!(interval.end_node(), cx_nodes[*expected_range.end()]);
    }

    #[rstest]
    fn test_try_extend_error(cx_circuit_scope: ResourceScope) {
        let cx_nodes = cx_circuit_scope.nodes();

        // Create an interval from nodes 2 to 3 (middle of circuit)
        let mut interval = Interval::new(
            ResourceId::new(0),
            cx_nodes[2],
            cx_nodes[3],
            &cx_circuit_scope,
        );

        let result = interval
            .try_extend(cx_nodes[0], &cx_circuit_scope)
            .unwrap_err();

        assert_eq!(result, InvalidInterval::NotContiguous(cx_nodes[0]));
        assert_eq!(interval.start_node(), cx_nodes[2]);
        assert_eq!(interval.end_node(), cx_nodes[3]);
    }

    #[rstest]
    fn interval_is_scope_independent(cx_circuit_scope: ResourceScope) {
        let other_scope = {
            let mut tmp_scope = cx_circuit_scope.clone();
            tmp_scope.map_positions(|pos| pos.increment());
            tmp_scope
        };
        let scope = cx_circuit_scope;
        let cx_nodes = scope.nodes();

        let interval = Interval::new(ResourceId::new(0), cx_nodes[2], cx_nodes[4], &scope);

        assert_eq!(
            scope.nodes_in_interval(interval).collect_vec(),
            other_scope.nodes_in_interval(interval).collect_vec()
        );

        assert_eq!(
            interval.incoming_boundary_port(&scope),
            interval.incoming_boundary_port(&other_scope)
        );

        assert_eq!(
            interval.outgoing_boundary_port(&scope),
            interval.outgoing_boundary_port(&other_scope)
        );
    }
}
