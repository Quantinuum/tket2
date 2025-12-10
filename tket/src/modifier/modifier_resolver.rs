//! Try to delete modifier by applying the modifier to each component.
//!
//! The entry point of this module is [`resolve_modifier_with_entrypoints`]
//! which takes a hugraph and a list of entry points.
//! Modifier resolver visits all the nodes reachable from the entry points.
//!
//! The main struct [`ModifierResolver`] holds the state during the process,
//! and implements the core logic. `corresp_map` holds
//! the main information during the process, which is a map from wires
//! in the graph being modified to wires in the new graph being constructed.
//!
//! A modifier is assumed to be applied to a loaded function
//! and called directly exactly once by another modifier or
//! an `IndirectedCall` node.
//! That is, the following structure is assumed:
//! ```text
//! LoadFunction -> Modifier* -> IndirectedCall
//! ```
//! Any other structure is not supported at this point, such as:
//! ```text
//! LoadFunction -> Modifier -> IndirectedCall
//!                 |
//!                 +-> Modifier -> IndirectedCall.
//! ```
//! The resolver finds the last modifier in a chain of modifiers,
//! and starts resolving the function loaded by the `LoadFunction` node,
//! which is done in
//! `apply_modifier_chain_to_loaded_fn`.
//!
//! While resolving modifiers, we hold the original hugr `h` and the node to be modified `n`,
//! and a builder `new_dfg` to construct the new graph.
//! The correspondence map (`corresp_map`) keeps the correspondence
//! from wires in `h` to wires in `new_dfg`.
//! See `modify_op`, which is the main function that modifies each node.
//!
//! During the resolution, when a node with some data flow included (such as a function) is encountered,
//! the function `modify_dfg_body`
//! is called.
//! This function modifies the I/O nodes and then calls
//! `modify_dfg_children`
//! to visit all other children nodes.
//! When dagger is applied, the order of nodes to be processed is reversed,
//! since the control qubits are passed in the reverse order.
//! After visiting all children, `modify_dfg_body` calls
//! [`connect_all`](ModifierResolver::connect_all) to connect all wires that are registered
//! in the correspondence map.
//!
//! Importantly, when dagger is applied, not only the order of nodes is reversed,
//! the direction of wires that includes any qubits is also reversed.
//! Let us explain this with an example.
//! Suppose we have a graph like below:
//! ```text
//! In(0) -------> [Rx] -------> [S] -------> Out(0)
//!                 ^
//!                 |
//!   angle(π) ----+
//! ```
//! The resulting graph after applying dagger should be:
//! ```text
//! In(0) -------> [Sdg] -------> [Rx] -------> Out(0)
//!                                ^
//!                                |
//! angle(π) ------- [fneg] ------+
//! ```
//! Looking at on the edge between `Rx` and `S` in `h`,
//! one can see that the direction of the edge is reversed in the new graph.
//! In other words, the incoming port of `S` is mapped to the outgoing port of `Sdg`,
//! and the outgoing port of `Rx` is mapped to the incoming port of `Rx`.
//! On the other hand, when looking at the edge between `angle(π)` and `Rx`,
//! the outgoing port of `angle(π)` is not changed in the new graph,
//! but the incoming port of `Rx` is mapped to the incoming port of `fneg` that reverses the angle.
//! Therefore, the correspondence map should contain:
//! ```text
//! (S, In(0))          -> (Sdg, Out(0))
//! (Rx, Out(0))        -> (Rx, In(0))
//! (angle(π), Out(0)) -> (angle(π), Out(0))
//! (Rx, In(1))         -> (fneg, In(1))
//! ```
//! From this correspondence map, we can see that the direction of wires in the new graph
//! can be completely mixed up.
//! The logic of registering such correspondence is implemented in a function such as
//! `wire_node_inout`.
//! Also, the correspondence of I/O wires should be changed accordingly, depending on whether
//! it includes qubits or not.
//! We also should not forget to connect `fneg` to `Rx` in the new graph, whose edge/wires has
//! no correspondence in the original graph.
//!
//! ## Not supported/TODO cases
//! - Power: Power modifier is not supported at this point.
//! - Non-trivial CFGs: We cannot support dagger for complicated CFGs
//!   since it is not clear at all whether we should reverse the control flow or not.
//!   Currently, when any non-trivial cfg with more than one block is encountered during
//!   the resolution, an error is returned.
//! - Branching in modifier chain: As noted above, we assume that a modifier is
//!   chained linearly.
//! - StateOrder edge: Currently, the modified function does not contain StateOrder edges
//!   in any case.
//!   This won't be manageable if dagger is applied, but if not, it should be handled in the future.
//! - User defined extension ops: There is no way to infer modified unknown extension ops.
//!   We currently try to insert the original optype without any modification,
//!   but this could result in an unexpected error.
use itertools::{Either, Itertools};
use std::{
    collections::{HashMap, VecDeque},
    iter, mem,
};

pub mod array_modify;
pub mod call_modify;
pub mod dfg_modify;
pub mod global_phase_modify;
pub mod tket_op_modify;

use super::{CombinedModifier, ModifierFlags};
use crate::passes::unpack_container::TypeUnpacker;
use crate::{extension::global_phase::GlobalPhase, modifier::Modifier, TketOp};
use global_phase_modify::delete_phase;

use hugr::{
    builder::{BuildError, CFGBuilder, Container, Dataflow, SubContainer},
    core::HugrNode,
    extension::{prelude::qb_t, simple_op::MakeExtensionOp},
    hugr::hugrmut::HugrMut,
    ops::{Const, OpType, CFG},
    std_extensions::collections::array::array_type,
    type_row,
    types::{EdgeKind, FuncTypeBase, Signature, Type},
    HugrView, IncomingPort, Node, OutgoingPort, Port, PortIndex, Wire,
};

/// A wire of eigher direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct DirWire<N = Node>(N, Port);

impl<N: HugrNode> std::fmt::Display for DirWire<N> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let dir = match self.1.as_directed() {
            Either::Left(_) => "In",
            Either::Right(_) => "Out",
        };
        write!(f, "DirWire({}, {}({}))", self.0, dir, self.1.index())
    }
}

impl<N> DirWire<N> {
    #[allow(dead_code)]
    pub fn new(node: N, port: Port) -> Self {
        DirWire(node, port)
    }

    pub fn reverse(self) -> Self {
        let index = self.1.index();
        let port = match self.1.as_directed() {
            Either::Left(_in) => OutgoingPort::from(index).into(),
            Either::Right(_out) => IncomingPort::from(index).into(),
        };
        DirWire::new(self.0, port)
    }
}

impl<N: HugrNode> From<Wire<N>> for DirWire<N> {
    fn from(wire: Wire<N>) -> Self {
        DirWire(wire.node(), wire.source().into())
    }
}
impl<N: HugrNode> From<(N, OutgoingPort)> for DirWire<N> {
    fn from((node, port): (N, OutgoingPort)) -> Self {
        DirWire(node, port.into())
    }
}
impl<N: HugrNode> From<(N, IncomingPort)> for DirWire<N> {
    fn from((node, port): (N, IncomingPort)) -> Self {
        DirWire(node, port.into())
    }
}
impl<N: HugrNode> TryFrom<DirWire<N>> for Wire<N> {
    type Error = hugr::hugr::HugrError;

    fn try_from(value: DirWire<N>) -> Result<Self, Self::Error> {
        let out_port = value.1.as_outgoing()?;
        Ok(Wire::new(value.0, out_port))
    }
}
impl<N: HugrNode> TryFrom<DirWire<N>> for (N, IncomingPort) {
    type Error = hugr::hugr::HugrError;

    fn try_from(value: DirWire<N>) -> Result<Self, Self::Error> {
        let in_port = value.1.as_incoming()?;
        Ok((value.0, in_port))
    }
}

fn connect<N>(
    new_dfg: &mut impl Container,
    w1: &DirWire<Node>,
    w2: &DirWire<Node>,
) -> Result<(), ModifierResolverErrors<N>> {
    let (n_o, p_o, n_i, p_i) = match (w1.1.as_directed(), w2.1.as_directed()) {
        (Either::Right(p_o), Either::Left(p_i)) => (w1.0, p_o, w2.0, p_i),
        (Either::Left(p_i), Either::Right(p_o)) => (w2.0, p_o, w1.0, p_i),
        _ => {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Cannot connect the wires with the same direction: {} -> {}",
                w1, w2
            )))
        }
    };
    new_dfg.hugr_mut().connect(n_o, p_o, n_i, p_i);
    Ok(())
}

/// Connect a wire to a node by its number, returning the other side of the connection.
fn connect_by_num(
    new_dfg: &mut impl Dataflow,
    dw: &DirWire<Node>,
    node: Node,
    num: usize,
) -> DirWire<Node> {
    let dw_node = dw.0;
    match dw.1.as_directed() {
        Either::Left(incoming) => {
            new_dfg.hugr_mut().connect(node, num, dw_node, incoming);
            (node, IncomingPort::from(num)).into()
        }
        Either::Right(outgoing) => {
            new_dfg.hugr_mut().connect(dw_node, outgoing, node, num);
            (node, OutgoingPort::from(num)).into()
        }
    }
}

trait PortExt {
    fn shift(self, offset: usize) -> Self;
}
impl PortExt for Port {
    fn shift(self, offset: usize) -> Self {
        Port::new(self.direction(), self.index() + offset)
    }
}
impl PortExt for IncomingPort {
    fn shift(self, offset: usize) -> Self {
        IncomingPort::from(self.index() + offset)
    }
}
impl PortExt for OutgoingPort {
    fn shift(self, offset: usize) -> Self {
        OutgoingPort::from(self.index() + offset)
    }
}
impl<N> PortExt for DirWire<N> {
    fn shift(self, offset: usize) -> Self {
        DirWire(self.0, self.1.shift(offset))
    }
}

/// A vector of ports for each node.
/// The `if_rev` vector is used to swap the wires if the dagger is applied.
pub struct PortVector<N = Node> {
    incoming: Vec<DirWire<N>>,
    outgoing: Vec<DirWire<N>>,
}
impl<N: HugrNode> PortVector<N> {
    fn from_single_node(
        n: N,
        inputs: impl Iterator<Item = usize>,
        outputs: impl Iterator<Item = usize>,
    ) -> Self {
        let incoming = inputs.map(|p| (n, IncomingPort::from(p)).into()).collect();
        let outgoing = outputs.map(|p| (n, OutgoingPort::from(p)).into()).collect();
        PortVector { incoming, outgoing }
    }
    fn port_vector_rev(
        n: N,
        inputs: impl Iterator<Item = usize>,
        outputs: impl Iterator<Item = usize>,
        iter: impl Iterator<Item = usize>,
    ) -> Self {
        let iter = iter.collect::<Vec<_>>();
        let incoming = inputs
            .map(|p| {
                if iter.contains(&p) {
                    (n, OutgoingPort::from(p)).into()
                } else {
                    (n, IncomingPort::from(p)).into()
                }
            })
            .collect();
        let outgoing = outputs
            .map(|p| {
                if iter.contains(&p) {
                    (n, IncomingPort::from(p)).into()
                } else {
                    (n, OutgoingPort::from(p)).into()
                }
            })
            .collect();
        PortVector { incoming, outgoing }
    }
}

/// A container for modifier resolving.
/// This struct holds the state during the modifier resolution process.
pub struct ModifierResolver<N = Node> {
    /// Current accumulated modifiers.
    modifiers: CombinedModifier,
    /// A map from old wire to new wires.
    /// The keys are old wires, and the values are new wires.
    /// As noted at the head of this module, especially when dagger is applied,
    /// an incoming wire may correspond to an outgoing wire and vice versa.
    corresp_map: HashMap<DirWire<N>, Vec<DirWire>>,
    /// The current control outgoing wires
    controls: Vec<Wire>,
    /// The worklist of nodes to be processed.
    /// This is needed to avoid modifying a node that is generated during the process.
    worklist: VecDeque<N>,
    /// A map of static edges to be added after insertion of subgraph.
    call_map: HashMap<N, (Node, IncomingPort)>,
    // TODO:
    // Should keep track of the collection of modifiers that are applied to the same function.
    // This will prevent the duplicated generation of Controlled-functions.
    // Some HashMap should be held here so that we remember such information.
    // ```
    // _modified_functions: HashMap<N, (CombinedModifier, Node)>,
    // ```
    qubit_finder: TypeUnpacker,
}

impl<N> ModifierResolver<N> {
    /// Create a new modifier resolver.
    pub fn new() -> Self {
        ModifierResolver {
            modifiers: CombinedModifier::default(),
            corresp_map: HashMap::default(),
            controls: Vec::default(),
            worklist: VecDeque::default(),
            call_map: HashMap::default(),
            qubit_finder: TypeUnpacker::for_qubits(),
        }
    }
}

impl<N> Default for ModifierResolver<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Error that can occur when resolving modifiers.
#[derive(Debug, derive_more::Error, derive_more::Display)]
pub enum ModifierError<N = Node> {
    /// The node is not a modifier
    #[display("Node to modify {_0} expected to be a modifier but actually {_1}")]
    NotModifier(N, OpType),
    /// No caller of this modified function exists.
    #[display("No caller of the modified function exists for node {_0}")]
    #[error(ignore)]
    NoCaller(N),
    /// No target of this modifer exists.
    #[display("No caller of the modified function exists for node {_0}")]
    #[error(ignore)]
    NoTarget(N),
    /// Not the first modifier in a chain.
    #[display("Node {_0} is not the first modifier in a chain. It is called by {_0}")]
    NotInitialModifier(N, OpType),
    /// The modifier cannot be applied to the node.
    #[display("Modifier cannot be applied to the node {_0} of type {_1}")]
    ModifierNotApplicable(N, OpType),
}

impl<N> ModifierError<N> {
    fn node(self) -> N {
        match self {
            ModifierError::NotModifier(n, _)
            | ModifierError::NoCaller(n)
            | ModifierError::NoTarget(n)
            | ModifierError::NotInitialModifier(n, _)
            | ModifierError::ModifierNotApplicable(n, _) => n,
        }
    }
}

/// Possible errors that can occur during the modifier resolution process.
#[derive(Debug, derive_more::Display, derive_more::Error, derive_more::From)]
pub enum ModifierResolverErrors<N = Node> {
    /// Cannot modify the node.
    #[display("{_0}")]
    #[from]
    ModifierError(ModifierError<N>),
    /// Error during the DFG build process.
    #[display("{_0}")]
    #[from]
    BuildError(BuildError),
    /// Error that is caused by a bug in this resolver which should be unreachable.
    #[display("Unreachable error: {msg}")]
    Unreachable {
        /// The message of the unreachable error.
        msg: String,
    },
    /// Modifier applied to a node that cannot be modified.
    #[display("Modifier {node} applied to the node {msg} cannot be modified")]
    UnResolvable {
        /// The node that cannot be modified.
        node: N,
        /// The message of the unresolvable error.
        msg: String,
        /// The operation type that cannot be modified.
        optype: OpType,
    },
    /// The node cannot be modified.
    #[display("Modification by {_0:?} is not defined for the node {_1}")]
    Unimplemented(Modifier, OpType),
}

impl<N> ModifierResolverErrors<N> {
    /// Create an unreachable error.
    pub fn unreachable(msg: impl Into<String>) -> Self {
        Self::Unreachable { msg: msg.into() }
    }

    /// Create an unresolvable error.
    pub fn unresolvable(node: N, msg: impl Into<String>, optype: OpType) -> Self {
        Self::UnResolvable {
            node,
            msg: msg.into(),
            optype,
        }
    }
}

// Utility functions for ModifierResolver
impl<N: HugrNode> ModifierResolver<N> {
    fn modifiers_mut(&mut self) -> &mut CombinedModifier {
        &mut self.modifiers
    }
    fn modifiers(&self) -> &CombinedModifier {
        &self.modifiers
    }
    fn control_num(&self) -> usize {
        self.modifiers.control
    }
    fn controls(&mut self) -> &mut Vec<Wire> {
        &mut self.controls
    }
    fn controls_ref(&self) -> &Vec<Wire> {
        &self.controls
    }
    fn worklist(&mut self) -> &mut VecDeque<N> {
        &mut self.worklist
    }
    fn corresp_map(&mut self) -> &mut HashMap<DirWire<N>, Vec<DirWire>> {
        &mut self.corresp_map
    }
    fn call_map(&mut self) -> &mut HashMap<N, (Node, IncomingPort)> {
        &mut self.call_map
    }

    fn with_worklist<T>(&mut self, worklist: VecDeque<N>, f: impl FnOnce(&mut Self) -> T) -> T {
        let worklist = mem::replace(self.worklist(), worklist);
        let r = f(self);
        *self.worklist() = worklist;
        r
    }
    fn with_modifiers<T>(
        &mut self,
        modifiers: CombinedModifier,
        f: impl FnOnce(&mut Self) -> T,
    ) -> T {
        let modifiers = mem::replace(self.modifiers_mut(), modifiers);
        let r = f(self);
        *self.modifiers_mut() = modifiers;
        r
    }
    fn with_ancilla<T>(
        &mut self,
        wire: &mut Wire<Node>,
        ancilla: &mut Vec<Wire<Node>>,
        f: impl FnOnce(&mut Self, &mut Vec<Wire<Node>>) -> T,
    ) -> T {
        ancilla.push(*wire);
        let r = f(self, ancilla);
        *wire = ancilla.pop().unwrap();
        r
    }

    fn pop_control(&mut self) -> Option<Wire<Node>> {
        if let Some(c) = self.controls().pop() {
            self.modifiers.control -= 1;
            Some(c)
        } else {
            None
        }
    }

    fn push_control(&mut self, c: Wire<Node>) {
        self.controls().push(c);
        self.modifiers.control += 1;
    }

    /// Register a correspondence from old to new wire.
    fn map_insert(
        &mut self,
        old: DirWire<N>,
        new: DirWire,
    ) -> Result<(), ModifierResolverErrors<N>> {
        self.corresp_map()
            .insert(old, vec![new])
            .map_or(Ok(()), |former| {
                // If the old wire is already registered, raise an error.
                Err(ModifierResolverErrors::unreachable(format!(
                    "Wire already registered for node {}. Former [{},...], Latter {}.",
                    old.0, former[0], new
                )))
            })
    }

    /// Remember that old wire has no correspondence.
    /// This adds an entry with empty vector if not already present.
    /// Note that this does not overwrite existing entry.
    fn map_insert_none(&mut self, old: DirWire<N>) -> Result<(), ModifierResolverErrors<N>> {
        self.corresp_map().entry(old).or_default();
        Ok(())
    }

    fn map_get(&self, key: &DirWire<N>) -> Result<&Vec<DirWire>, ModifierResolverErrors<N>> {
        self.corresp_map
            .get(key)
            .ok_or(ModifierResolverErrors::unreachable(format!(
                "No correspondence for the wire: {}",
                key
            )))
    }

    fn forget_node(
        &mut self,
        h: &impl HugrView<Node = N>,
        n: N,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // If a node has not registered correspondence, register None for all its ports.
        for port in h.all_node_ports(n) {
            let dw = DirWire(n, port);
            self.map_insert_none(dw)?;
        }
        Ok(())
    }

    /// This function adds a node to the builder, that does not affected by the modifiers.
    fn add_node_no_modification(
        &mut self,
        h: &impl HugrMut<Node = N>,
        old_n: N,
        op: impl Into<OpType>,
        new_dfg: &mut impl Container,
    ) -> Result<Node, ModifierResolverErrors<N>> {
        let node = new_dfg.add_child_node(op);
        for port in h.all_node_ports(old_n) {
            self.map_insert(DirWire(old_n, port), DirWire(node, port))?;
        }
        Ok(node)
    }

    fn port_vector_dagger(
        &self,
        n: Node,
        inputs: impl Iterator<Item = usize>,
        outputs: impl Iterator<Item = usize>,
        iter: impl Iterator<Item = usize>,
    ) -> PortVector<Node> {
        if self.modifiers.dagger {
            PortVector::port_vector_rev(n, inputs, outputs, iter)
        } else {
            PortVector::from_single_node(n, inputs, outputs)
        }
    }

    fn add_edge_from_pv(
        &mut self,
        h: &impl HugrMut<Node = N>,
        n: N,
        pv: PortVector<Node>,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let PortVector { incoming, outgoing } = pv;
        for (old_in, new) in (0..h.num_inputs(n)).map(IncomingPort::from).zip(incoming) {
            self.map_insert((n, old_in).into(), new)?
        }
        for (old_out, new) in (0..h.num_outputs(n)).map(OutgoingPort::from).zip(outgoing) {
            self.map_insert((n, old_out).into(), new)?
        }
        Ok(())
    }

    /// Add a node to the builder, plugging the control qubits to the first n-inputs and outputs.
    fn add_node_control(&mut self, new_dfg: &mut impl Container, op: impl Into<OpType>) -> Node {
        let node = new_dfg.add_child_node(op);
        for (i, ctrl) in self.controls().iter_mut().enumerate() {
            new_dfg
                .hugr_mut()
                .connect(ctrl.node(), ctrl.source(), node, i);
            *ctrl = Wire::new(node, i);
        }
        node
    }

    /// connects all the wires in the builder.
    pub fn connect_all(
        &mut self,
        h: &impl HugrView<Node = N>,
        new_dfg: &mut impl Container,
        parent: N,
    ) -> Result<(), ModifierResolverErrors<N>> {
        for out_node in h.children(parent) {
            for out_port in h.node_outputs(out_node) {
                if let Some(EdgeKind::StateOrder) = h.get_optype(out_node).port_kind(out_port) {
                    // TODO: Currently, we just ignore StateOrder edges.
                    // This might be OK when the dagger is applied since StateOrder is not managable then.
                    // However, if not, we should preserve the StateOrder edges.
                    // This could be done in two ways:
                    // 1. Register StateOrder edges to `corresp_map` as well as data edges.
                    // 2. Use another `HashMap` to keep track of StateOrder edges.
                    continue;
                }
                for (in_node, in_port) in h.linked_inputs(out_node, out_port) {
                    for a in self.map_get(&(in_node, in_port).into())? {
                        for b in self.map_get(&(out_node, out_port).into())? {
                            connect(new_dfg, a, b)?
                        }
                    }
                }
            }
        }
        // FIXME: StateOrder is not preserved here.
        Ok(())
    }
}

impl<N: HugrNode> ModifierResolver<N> {
    // FIXME: Shouldn't we check that there is a caller of the modified function?
    // We don't want to modify a function that is loaded and modified but never called.
    // When more than one modifier is chained, after the last modifier is resolved,
    // we delete the last modifier node, but the previous modifiers are not deleted.
    // If the second last modifier was only called by the last modifier, that will not be called anymore.
    fn verify(&self, h: &impl HugrView<Node = N>, n: N) -> Result<(), ModifierError<N>> {
        // Check if the node is a modifier, modifying an operation.
        let optype = h.get_optype(n);
        if Modifier::from_optype(optype).is_none() {
            return Err(ModifierError::NotModifier(n, optype.clone()));
        }
        // Check if this is the first modifier in a chain.
        if let Ok((caller, _)) = h.linked_inputs(n, 0).exactly_one() {
            let optype = h.get_optype(caller);
            if Modifier::from_optype(optype).is_some() {
                return Err(ModifierError::NotInitialModifier(caller, optype.clone()));
            }
        } else {
            return Err(ModifierError::NoCaller(n));
        }
        Ok(())
    }

    /// Apply the resolver the current node `n`.
    /// It first checks if the node is a modifier and can be applied.
    /// If not, it returns an [`ModifierError`].
    /// If yes, it applies the modifier to the loaded function,
    fn try_rewrite(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // Verify that the rewrite can be applied.
        self.verify(h, n)?;

        // the ports that takes inputs from the modified function.
        let modified_fn_loader: Vec<(_, Vec<_>)> = h
            .node_outputs(n)
            .map(|p| (p, h.linked_inputs(n, p).collect()))
            .collect();

        // Modify the chain of modifiers.
        // Make sure that the modifiers are initially empty.
        let modifiers = CombinedModifier::default();
        let new_load = self.with_modifiers(modifiers, |this| {
            this.apply_modifier_chain_to_loaded_fn(h, n)
        })?;

        // Connect the modified function to the inputs
        for (out_port, inputs) in modified_fn_loader {
            for (recv, recv_port) in inputs {
                h.disconnect(recv, recv_port);
                h.connect(new_load, out_port, recv, recv_port);
            }
        }

        Ok(())
    }

    /// Takes a signature and modifies it according to the combined modifier.
    /// flatten = true means that control qubits are represented as individual wires,
    /// while false means that they are packed to some arrays.
    /// This false mode is used for function definitions,
    pub fn modify_signature(&self, signature: &mut Signature, flatten: bool) {
        let FuncTypeBase { input, output } = signature;

        if flatten {
            let n = self.control_num();
            input.to_mut().splice(0..0, iter::repeat_n(qb_t(), n));
            output.to_mut().splice(0..0, iter::repeat_n(qb_t(), n));
        } else {
            for ctrls in &self.modifiers.accum_ctrl {
                let n = *ctrls as u64;
                input.to_mut().insert(0, array_type(n, qb_t()));
                output.to_mut().insert(0, array_type(n, qb_t()));
            }
        }
    }

    // We take arbitral topological order of the circuit so that we can plug the control qubits
    // and pass around them in that order. This might not be ideal, as it may produce an inefficient order.
    fn modify_op(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let optype = &h.get_optype(n).clone();
        match optype {
            // Skip input/output nodes: it should be handled by its parent as it sets control qubits.
            OpType::Input(_) | OpType::Output(_) => {}

            // CFG
            OpType::CFG(cfg) => self.modify_cfg(h, n, cfg, new_dfg)?,

            // DFGs
            OpType::DFG(dfg) => self.modify_dfg(h, n, dfg, new_dfg)?,
            OpType::TailLoop(tail_loop) => self.modify_tail_loop(h, n, tail_loop, new_dfg)?,
            OpType::Conditional(conditional) => {
                self.modify_conditional(h, n, conditional, new_dfg)?
            }

            // Function calls
            OpType::Call(_) => self.modify_call(h, n, optype, new_dfg)?,
            OpType::CallIndirect(indir_call) => {
                self.modify_indirect_call(h, n, indir_call, new_dfg)?
            }
            OpType::LoadFunction(load) => self.modify_load_function(h, n, load, new_dfg)?,

            // Operations
            OpType::ExtensionOp(_) => {
                self.modify_extension_op(h, n, optype, new_dfg)?;
            }
            OpType::Const(constant) => {
                self.modify_constant(n, constant, new_dfg)?;
            }
            OpType::LoadConstant(_) | OpType::OpaqueOp(_) | OpType::Tag(_) => {
                self.add_node_no_modification(h, n, optype.clone(), new_dfg)?;
            }

            OpType::FuncDefn(_) | OpType::FuncDecl(_) | OpType::Module(_) => {
                return Err(ModifierResolverErrors::unreachable(format!(
                    "Invalid node found inside modified function (OpType = {})",
                    optype.clone()
                )))
            }
            OpType::Case(_) => {
                return Err(ModifierResolverErrors::unreachable(
                    "Case cannot be directly modified.".to_string(),
                ))
            }

            // Not resolvable
            OpType::AliasDecl(_)
            | OpType::AliasDefn(_)
            | OpType::ExitBlock(_)
            | OpType::DataflowBlock(_) => {
                return Err(ModifierResolverErrors::unresolvable(
                    n,
                    "Unmodifiable node found".to_string(),
                    optype.clone(),
                ));
            }
            _ => {
                // Q. Maybe we should just ignore unknown operations?
                return Err(ModifierResolverErrors::unresolvable(
                    n,
                    "Unknown operation".to_string(),
                    optype.clone(),
                ));
            }
        }
        Ok(())
    }

    /// This function registers the correspondence of the data-flow ports of the old node to the new node.
    /// If the dagger is not applied, the ports are mapped directly.
    /// If the dagger is applied, the quantum input/output ports are swapped.
    /// Inputs:
    /// * `n`: the old node
    /// * `node`: the new node
    /// * `inputs`/`outputs`: the types of the input/output ports of the old node
    /// * `input_offset`/`output_offset`: the offset of the ports of the old and new node
    ///   - e.g., for IndirectCall, the first input port is the loaded function, which we want to ignore here.
    ///     So the `input_offset` is 1.
    /// * `new_offset`: the offset of the ports of the new node, especially the number of control qubits.
    ///
    /// The order of the resulting ports is determined as follows:
    /// - Every ports are devided into quantum ports and non-quantum ports.
    /// - Until the first quantum port is reached, the non-quantum ports are wired in order.
    /// - When a quantum port is reached for both inputs and outputs,
    ///   if the dagger is applied, the quantum input is wired to the output,
    ///   and the quantum output is wired to the input until they reaches the next non-quantum port.
    /// - This is repeated until all ports are wired.
    ///
    /// For example, if the input types are `[qubit, int, qubit, qubit, int]` and
    /// the output types are `[qubit, array[qubit, _]]`,
    /// and the dagger is applied, the wiring is as follows:
    /// - input: [out0:qubit, in1:int, out1:array[qubit, _], in4:int]
    /// - output: [in0:qubit, in2:qubit, in3:qubit]
    ///
    /// FIXME: This reverses everything that can contain qubits, which might not be intended in general.
    /// TODO: Handle state order edges.
    fn wire_node_inout<'a>(
        &mut self,
        n: N,
        node: Node,
        (inputs, outputs): (
            impl Iterator<Item = &'a Type>,
            impl Iterator<Item = &'a Type>,
        ),
        (input_offset, output_offset, new_offset): (usize, usize, usize),
    ) -> Result<(), ModifierResolverErrors<N>> {
        self.wire_inout(
            (n, n),
            (node, node),
            (inputs, outputs),
            (input_offset, output_offset, new_offset),
        )
    }

    fn wire_inout<'a>(
        &mut self,
        (old_in, old_out): (N, N),
        (new_in, new_out): (Node, Node),
        (mut inputs, mut outputs): (
            impl Iterator<Item = &'a Type>,
            impl Iterator<Item = &'a Type>,
        ),
        (input_offset, output_offset, new_offset): (usize, usize, usize),
    ) -> Result<(), ModifierResolverErrors<N>> {
        let mut old_in_wire = (old_in, IncomingPort::from(input_offset)).into();
        let mut old_out_wire = (old_out, OutgoingPort::from(output_offset)).into();
        let mut new_in_wire = (new_in, IncomingPort::from(input_offset + new_offset)).into();
        let mut new_out_wire = (new_out, OutgoingPort::from(output_offset + new_offset)).into();
        let mut in_ty = inputs.next();
        let mut out_ty = outputs.next();

        loop {
            // Wire inputs until the first quantum type
            while let Some(ty) = in_ty {
                if self.qubit_finder.contains_element_type(ty) {
                    break;
                }
                self.map_insert(old_in_wire, new_in_wire)?;
                old_in_wire = old_in_wire.shift(1);
                new_in_wire = new_in_wire.shift(1);
                in_ty = inputs.next();
            }

            // Wire outputs until the first quantum type
            while let Some(ty) = out_ty {
                if self.qubit_finder.contains_element_type(ty) {
                    break;
                }
                self.map_insert(old_out_wire, new_out_wire)?;
                old_out_wire = old_out_wire.shift(1);
                new_out_wire = new_out_wire.shift(1);
                out_ty = outputs.next();
            }

            // If both are quantum types, wire them in the opposite direction until the next non-quantum type
            while let Some(ty) = in_ty {
                if !self.qubit_finder.contains_element_type(ty) {
                    break;
                }
                let new_in = if !self.modifiers.dagger {
                    let new_in = new_in_wire;
                    new_in_wire = new_in_wire.shift(1);
                    new_in
                } else {
                    let new_in = new_out_wire;
                    new_out_wire = new_out_wire.shift(1);
                    new_in
                };
                self.map_insert(old_in_wire, new_in)?;
                old_in_wire = old_in_wire.shift(1);
                in_ty = inputs.next();
            }
            while let Some(ty) = out_ty {
                if !self.qubit_finder.contains_element_type(ty) {
                    break;
                }
                let new_out = if !self.modifiers.dagger {
                    let new_out = new_out_wire;
                    new_out_wire = new_out_wire.shift(1);
                    new_out
                } else {
                    let new_out = new_in_wire;
                    new_in_wire = new_in_wire.shift(1);
                    new_out
                };
                self.map_insert(old_out_wire, new_out)?;
                old_out_wire = old_out_wire.shift(1);
                out_ty = outputs.next();
            }

            // Break if ended
            if in_ty.is_none() && out_ty.is_none() {
                break;
            }
        }

        Ok(())
    }

    // WIP
    fn _wire_others(
        &mut self,
        n: N,
        n_optype: &OpType,
        node: Node,
        node_optype: &OpType,
    ) -> Result<(), ModifierResolverErrors<N>> {
        if let (Some(old), Some(new)) =
            (n_optype.other_input_port(), node_optype.other_input_port())
        {
            self.map_insert((n, old).into(), (node, new).into())?;
        }
        if let (Some(old), Some(new)) = (
            n_optype.other_output_port(),
            node_optype.other_output_port(),
        ) {
            self.map_insert((n, old).into(), (node, new).into())?;
        }
        Ok(())
    }

    fn modify_constant(
        &mut self,
        n: N,
        constant: &Const,
        new_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let output = new_dfg.add_child_node(constant.clone());
        self.map_insert(Wire::new(n, 0).into(), Wire::new(output, 0).into())
    }

    /// Copy the dataflow operation to the new function.
    /// These are the operations that are not modified by the modifier.
    fn modify_dataflow_op(
        &mut self,
        h: &impl HugrMut<Node = N>,
        n: N,
        optype: &OpType,
        new_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let node = new_dfg.add_child_node(optype.clone());
        let signature = h.signature(n).unwrap();
        let inputs = signature.input.iter();
        let outputs = signature.output.iter();
        self.wire_node_inout(n, node, (inputs, outputs), (0, 0, 0))?;
        Ok(())
    }

    fn modify_extension_op(
        &mut self,
        h: &impl HugrMut<Node = N>,
        n: N,
        optype: &OpType,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        if self.controls().len() != self.control_num() {
            return Err(ModifierResolverErrors::unreachable(
                "Control qubits are not set correctly.".to_string(),
            ));
        }

        if let Some(op) = TketOp::from_optype(optype) {
            let pv = self.modify_tket_op(n, op, new_dfg, &mut vec![])?;
            self.add_edge_from_pv(h, n, pv)
        } else if GlobalPhase::from_optype(optype).is_some() {
            let inputs = self.modify_global_phase(n, new_dfg, &mut vec![])?;
            self.corresp_map().insert(
                (n, IncomingPort::from(0)).into(),
                inputs.into_iter().map(Into::into).collect(),
            );
            Ok(())
        } else if Modifier::from_optype(optype).is_some() {
            // TODO: check if this is ok.
            self.forget_node(h, n)
        } else if self.modify_array_op(h, n, optype, new_dfg)?
            || self.try_array_convert(h, n, optype, new_dfg)?
        {
            Ok(())
        } else {
            // Some other Hugr extension operation.
            // Here, we do not know what is the modified version.
            // We try to place the original operation.
            self.modify_dataflow_op(h, n, optype, new_dfg)
        }
    }

    /// This modifier expects that the CFG contains only one block.
    /// If not, it returns an error.
    fn modify_cfg(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        n: N,
        cfg: &CFG,
        new_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // Check if the CFG contains only one block.
        let children: Vec<N> = h
            .children(n)
            .filter(|child| h.get_optype(*child).is_dataflow_block())
            .collect();
        if children.len() != 1 {
            return Err(ModifierResolverErrors::unresolvable(
                n,
                "CFG with more than one node found.".to_string(),
                cfg.clone().into(),
            ));
        }
        let old_bb = children[0];

        let mut signature = cfg.signature.clone();
        self.modify_signature(&mut signature, true);
        let mut new_cfg = CFGBuilder::new(signature.clone())?;
        let mut new_bb = new_cfg.entry_builder([type_row![]], signature.output.clone())?;
        self.modify_dfg_body(h, old_bb, &mut new_bb)?;
        let bb_id = new_bb.finish_sub_container()?;
        new_cfg.branch(&bb_id, 0, &new_cfg.exit_block())?;

        let new = self.insert_sub_dfg(new_dfg, new_cfg)?;

        // connect the controls and register the IOs
        for (i, c) in self.controls().iter_mut().enumerate() {
            new_dfg.hugr_mut().connect(c.node(), c.source(), new, i);
            *c = Wire::new(new, i);
        }
        let offset = self.control_num();
        self.wire_node_inout(
            n,
            new,
            (signature.input.iter(), signature.output.iter()),
            (0, 0, offset),
        )?;
        // self.wire_others(n, cfg.into(), new, new_dfg.hugr().get_optype(new))?;
        // TODO: handle other ports
        Ok(())
    }
}

/// Resolve modifiers in a circuit by applying them to each entry point.
//
// Shouldn't we use a worklist of nodes?
// As we may want to change the order of resolving modifiers
// but might want to rollback if the second last one is called in a different path,
// this may be needed.
pub fn resolve_modifier_with_entrypoints(
    h: &mut impl HugrMut<Node = Node>,
    entry_points: impl IntoIterator<Item = Node>,
) -> Result<(), ModifierResolverErrors<Node>> {
    use ModifierResolverErrors::*;

    let entry_points: VecDeque<_> = entry_points.into_iter().collect();

    let mut resolver = ModifierResolver::new();
    let mut worklist = entry_points.clone();
    let mut visited = vec![];
    while let Some(node) = worklist.pop_front() {
        if !h.contains_node(node) || visited.contains(&node) {
            continue;
        }
        worklist.extend(h.children(node).filter(|n| !visited.contains(n)));
        worklist.extend(h.all_neighbours(node).filter(|n| !visited.contains(n)));
        visited.push(node);
        if let Err(e) = resolver.try_rewrite(h, node) {
            // ModifierError means this node is skippable.
            // Otherwise, return the error.
            if !matches!(e, ModifierError(_)) {
                return Err(e);
            }
        }
    }

    // TODO:
    // This might be insufficient as a cleanup since the resolution procedure might
    // generate nodes that are not reachable from the entry points.
    // If more thorough cleanup is needed, we should run dead code elimination.
    let mut deletelist = entry_points.clone();
    let mut visited = vec![];
    while let Some(node) = deletelist.pop_front() {
        deletelist.extend(h.children(node).filter(|n| !visited.contains(n)));
        deletelist.extend(h.all_neighbours(node).filter(|n| !visited.contains(n)));
        visited.push(node);
        if h.contains_node(node) {
            let optype = h.get_optype(node);
            if Modifier::from_optype(optype).is_some() {
                let mut l = vec![node];
                while let Some(n) = l.pop() {
                    l.extend(h.output_neighbours(n));
                    h.remove_node(n);
                }
            }
        }
    }
    // Alternatively, we can just remove all the modifiers in the graph.
    // let entry_points = vec![h.module_root()];
    // for entry_point in entry_points.clone() {
    //     let descendants = h.descendants(entry_point).collect::<Vec<_>>();
    //     for node in descendants {
    //         if !h.contains_node(node) {
    //             continue;
    //         }
    //         let optype = h.get_optype(node);
    //         if Modifier::from_optype(optype).is_some() {
    //             let mut l = vec![node];
    //             while let Some(n) = l.pop() {
    //                 l.extend(h.output_neighbours(n));
    //                 h.remove_node(n);
    //             }
    //         }
    //     }
    // }

    // TODO: This as well.
    // Ad hoc cleanup procedure.
    delete_phase(h, [h.module_root()])?;

    h.validate()
        .map_err(|e| ModifierResolverErrors::BuildError(e.into()))?;

    Ok(())
}

// Definitions of helpers for tests
#[cfg(test)]
mod tests {
    use cool_asserts::assert_matches;
    use hugr::{
        builder::{DataflowSubContainer, HugrBuilder, ModuleBuilder},
        ops::{handle::FuncID, CallIndirect, ExtensionOp},
        std_extensions::collections::array::ArrayOpBuilder,
        types::Term,
        Hugr,
    };

    use crate::{
        extension::modifier::{CONTROL_OP_ID, DAGGER_OP_ID, MODIFIER_EXTENSION},
        TketOp,
    };

    use super::*;

    pub(crate) trait SetUnitary {
        fn set_unitary(&mut self);
    }
    impl<T: Container> SetUnitary for T {
        fn set_unitary(&mut self) {
            self.set_metadata("unitary", 7);
        }
    }

    pub(crate) fn test_modifier_resolver(
        t_num: usize,
        c_num: u64,
        foo: impl FnOnce(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        dagger: bool,
    ) {
        let mut module = ModuleBuilder::new();
        let call_sig = Signature::new_endo(
            [array_type(c_num, qb_t())]
                .into_iter()
                .chain(iter::repeat_n(qb_t(), t_num))
                .collect::<Vec<_>>(),
        );
        let main_sig = Signature::new(
            type_row![],
            vec![array_type(c_num, qb_t())]
                .into_iter()
                .chain(iter::repeat_n(qb_t(), t_num))
                .collect::<Vec<_>>(),
        );

        let dagger_op: ExtensionOp = {
            MODIFIER_EXTENSION
                .instantiate_extension_op(
                    &DAGGER_OP_ID,
                    [
                        iter::repeat_n(qb_t().into(), t_num)
                            .collect::<Vec<_>>()
                            .into(),
                        vec![].into(),
                    ],
                )
                .unwrap()
        };

        let control_op: ExtensionOp = {
            MODIFIER_EXTENSION
                .instantiate_extension_op(
                    &CONTROL_OP_ID,
                    [
                        Term::BoundedNat(c_num),
                        iter::repeat_n(qb_t().into(), t_num)
                            .collect::<Vec<_>>()
                            .into(),
                        vec![].into(),
                    ],
                )
                .unwrap()
        };

        let foo = foo(&mut module, t_num);

        let _main = {
            let mut func = module.define_function("main", main_sig).unwrap();
            let mut call = func.load_func(&foo, &[]).unwrap();
            if dagger {
                call = func
                    .add_dataflow_op(dagger_op, vec![call])
                    .unwrap()
                    .out_wire(0);
            }
            call = func
                .add_dataflow_op(control_op, vec![call])
                .unwrap()
                .out_wire(0);

            let mut controls = Vec::new();
            for _ in 0..c_num {
                controls.push(
                    func.add_dataflow_op(TketOp::QAlloc, vec![])
                        .unwrap()
                        .out_wire(0),
                );
            }

            let mut targ = Vec::new();
            for _ in 0..t_num {
                targ.push(
                    func.add_dataflow_op(TketOp::QAlloc, vec![])
                        .unwrap()
                        .out_wire(0),
                )
            }

            let control_arr = func.add_new_array(qb_t(), controls).unwrap();
            let fn_outs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: call_sig,
                    },
                    [call, control_arr].into_iter().chain(targ),
                )
                .unwrap()
                .outputs();

            func.finish_with_outputs(fn_outs).unwrap()
        };

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();

        assert_matches!(h.validate(), Ok(()));
    }
}
