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
//! After resolution, original function nodes that have been replaced by solved
//! modified versions may be removed if they are no longer needed and the pass
//! scope allows removing them. Nodes whose interface must be preserved by the
//! scope are kept.
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
//! ModifierResolver::connect_all to connect all wires that are registered
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
use fxhash::FxHashSet;
use itertools::{Either, Itertools};
use std::{
    collections::{HashMap, HashSet, VecDeque},
    iter, mem,
};

pub mod array_modify;
pub mod call_modify;
pub mod dfg_modify;
pub mod global_phase_modify;
pub mod tket_op_modify;

use super::{CombinedModifier, ModifierFlags};
use crate::passes::utils::unpack_container::TypeUnpacker;
use crate::passes::{InScope, PassScope};
use crate::{TketOp, extension::global_phase::GlobalPhase, modifier::Modifier};
use global_phase_modify::delete_phase;

use hugr::{
    HugrView, IncomingPort, Node, OutgoingPort, Port, PortIndex, Wire,
    builder::{BuildError, CFGBuilder, Container, Dataflow, SubContainer},
    core::HugrNode,
    extension::{prelude::qb_t, simple_op::MakeExtensionOp},
    hugr::hugrmut::HugrMut,
    ops::{CFG, Const, OpType},
    std_extensions::collections::array::array_type,
    types::{EdgeKind, FuncTypeBase, Signature, Term, Type, TypeRow},
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
    /// Create a new DirWire.
    fn new(node: N, port: Port) -> Self {
        DirWire(node, port)
    }

    /// Reverse the direction of the wire.
    pub(crate) fn reverse(self) -> Self {
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
            )));
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
    /// Static edges to be added after insertion of a subgraph.
    /// Multiple calls can reference the same function node, so each source
    /// maps to every copied static input that must be reconnected.
    call_map: HashMap<N, Vec<(Node, IncomingPort)>>,
    // TODO:
    // Should keep track of the collection of modifiers that are applied to the same function.
    // This will prevent the duplicated generation of Controlled-functions.
    // Some HashMap should be held here so that we remember such information.
    // ```
    // _modified_functions: HashMap<N, (CombinedModifier, Node)>,
    // ```
    /// Original functions for which the resolver generated modified replacements.
    modified_functions: HashSet<N>,
    /// Function input ports that must receive already-modified function values
    /// when calling the function currently being rewritten.
    dynamic_input_modifiers: Vec<(usize, CombinedModifier)>,
    /// Function input ports of the function currently being rewritten whose
    /// value types must be changed to match their required modifiers.
    active_function_input_modifiers: Vec<(usize, CombinedModifier)>,
    /// Requirements for function-valued inputs of generated modified functions.
    function_input_modifiers: HashMap<N, Vec<(usize, CombinedModifier)>>,
    qubit_finder: TypeUnpacker,
}

impl<N> ModifierResolver<N> {
    /// Create a new modifier resolver.
    fn new() -> Self {
        ModifierResolver {
            modifiers: CombinedModifier::default(),
            corresp_map: HashMap::default(),
            controls: Vec::default(),
            worklist: VecDeque::default(),
            call_map: HashMap::default(),
            modified_functions: HashSet::default(),
            dynamic_input_modifiers: Vec::default(),
            active_function_input_modifiers: Vec::default(),
            function_input_modifiers: HashMap::default(),
            qubit_finder: TypeUnpacker::for_qubits(),
        }
    }
}

impl<N> Default for ModifierResolver<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Errors that can occur when tracing and validating a chain of modifiers and its target.
#[derive(Debug, derive_more::Error, derive_more::Display)]
#[non_exhaustive]
pub enum ModifierError<N = Node> {
    /// The node is not a modifier
    #[display("Node to modify {_0} expected to be a modifier but actually {_1}")]
    NotModifier(N, OpType),
    /// No caller of this modified function exists.
    #[display("No caller of the modified function exists for node {_0}")]
    #[error(ignore)]
    NoCaller(N),
    /// No target of this modifier exists.
    #[display("The modifier node {_0} chain has no target")]
    #[error(ignore)]
    NoTarget(N),
    /// Not the first modifier in a chain.
    #[display("Node {_0} of type {_1} is not the first modifier in a chain.")]
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
#[non_exhaustive]
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
    /// The power modifier is not supported.
    #[display("Found power modifier in node: {node}. Power modifier is not supported yet.")]
    PowerModifierNotSupported {
        /// The `power` node
        node: N,
    },
}

impl<N> ModifierResolverErrors<N> {
    /// Create an unreachable error.
    fn unreachable(msg: impl Into<String>) -> Self {
        Self::Unreachable { msg: msg.into() }
    }

    /// Create an unresolvable error.
    fn unresolvable(node: N, msg: impl Into<String>, optype: OpType) -> Self {
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
    fn call_map(&mut self) -> &mut HashMap<N, Vec<(Node, IncomingPort)>> {
        &mut self.call_map
    }

    fn call_map_insert(&mut self, source: N, target: (Node, IncomingPort)) {
        self.call_map().entry(source).or_default().push(target);
    }

    fn dynamic_input_modifiers(&mut self) -> &mut Vec<(usize, CombinedModifier)> {
        &mut self.dynamic_input_modifiers
    }

    fn active_function_input_modifiers(&mut self) -> &mut Vec<(usize, CombinedModifier)> {
        &mut self.active_function_input_modifiers
    }

    fn function_input_modifiers(&self, func: N) -> &[(usize, CombinedModifier)] {
        self.function_input_modifiers
            .get(&func)
            .map(Vec::as_slice)
            .unwrap_or_default()
    }

    fn trace_modifier_chain_with(
        &self,
        h: &impl HugrMut<Node = N>,
        n: N,
        port: OutgoingPort,
        mut modifiers: CombinedModifier,
    ) -> Result<(N, OutgoingPort, CombinedModifier), ModifierResolverErrors<N>> {
        let mut current = n;
        let mut current_port = port;
        loop {
            let optype = h.get_optype(current);
            if Modifier::from_optype(optype).is_none() {
                break;
            }

            modifiers.push(optype.as_extension_op().unwrap(), current)?;
            let next = h
                .single_linked_output(current, 0)
                .ok_or(ModifierError::NoTarget(n))?;
            current = next.0;
            current_port = next.1;
        }
        Ok((current, current_port, modifiers))
    }

    /// Find function inputs that callers must provide in already-modified form.
    ///
    /// This is a pre-rewrite scan over `func`. It follows indirect calls and
    /// direct calls to higher-order helpers to discover requirements like
    /// "input 1 is called under `control`, so callers must pass the controlled
    /// version of that function value". The returned indices refer to the
    /// original top-level function signature, which makes them safer to store
    /// than requirements discovered later inside nested CFG/conditional/loop
    /// bodies where input numbering is local to the container.
    fn higher_order_input_modifiers(
        &self,
        h: &impl HugrMut<Node = N>,
        func: N,
    ) -> Result<Vec<(usize, CombinedModifier)>, ModifierResolverErrors<N>> {
        let mut visiting = HashSet::new();
        self.higher_order_input_modifiers_inner(h, func, &mut visiting)
    }

    fn higher_order_input_modifiers_inner(
        &self,
        h: &impl HugrMut<Node = N>,
        func: N,
        visiting: &mut HashSet<N>,
    ) -> Result<Vec<(usize, CombinedModifier)>, ModifierResolverErrors<N>> {
        if !visiting.insert(func) {
            return Ok(Vec::new());
        }

        let OpType::FuncDefn(func_defn) = h.get_optype(func) else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Cannot inspect higher-order input modifiers for non-function node: {}",
                h.get_optype(func)
            )));
        };
        let function_inputs = func_defn.signature().body().input.clone();
        let function_input_indices = function_inputs
            .iter()
            .enumerate()
            .filter_map(|(index, ty)| matches!(**ty, Term::FunctionType(_)).then_some(index))
            .collect::<HashSet<_>>();
        let mut quantum_function_input_indices = HashSet::new();
        for (index, ty) in function_inputs.iter().enumerate() {
            if self.function_type_has_quantum_data(ty)? {
                quantum_function_input_indices.insert(index);
            }
        }

        let mut requirements = Vec::new();
        for node in h.descendants(func) {
            match h.get_optype(node) {
                OpType::CallIndirect(call) => {
                    if !self.signature_has_quantum_data(&call.signature) {
                        continue;
                    }
                    // A modified indirect call through a function input cannot
                    // be solved inside this function body. The generated
                    // function must instead require that input to already have
                    // the corresponding modified function type.
                    let source = h.single_linked_output(node, 0).ok_or_else(|| {
                        ModifierResolverErrors::unreachable(
                            "CallIndirect function input has no source.".to_string(),
                        )
                    })?;
                    let (target, target_port, modifiers) = self.trace_modifier_chain_with(
                        h,
                        source.0,
                        source.1,
                        self.modifiers().clone(),
                    )?;
                    if matches!(h.get_optype(target), OpType::Input(_)) {
                        requirements.push((target_port.index(), modifiers));
                    }
                }
                OpType::Call(call) => {
                    let Some((callee, _)) =
                        h.single_linked_output(node, call.called_function_port())
                    else {
                        continue;
                    };
                    if !matches!(h.get_optype(callee), OpType::FuncDefn(_)) {
                        continue;
                    }

                    // A direct call to a higher-order function can force one of
                    // this function's own inputs to be pre-modified. This is the
                    // recursive case for wrappers such as f -> g(f) -> h(f).
                    for (callee_input, modifiers) in
                        self.higher_order_input_modifiers_inner(h, callee, visiting)?
                    {
                        let source = h.single_linked_output(node, callee_input).ok_or_else(|| {
                            ModifierResolverErrors::unreachable(format!(
                                "Call input {callee_input} has no source while propagating higher-order modifiers."
                            ))
                        })?;
                        let (target, target_port, modifiers) =
                            self.trace_modifier_chain_with(h, source.0, source.1, modifiers)?;
                        if matches!(h.get_optype(target), OpType::Input(_)) {
                            requirements.push((target_port.index(), modifiers));
                        }
                    }
                }
                _ => {}
            }
        }
        visiting.remove(&func);

        requirements.retain(|(input, _)| function_input_indices.contains(input));
        if !requirements.is_empty() {
            requirements.extend(
                quantum_function_input_indices
                    .iter()
                    .copied()
                    .map(|input| (input, self.modifiers().clone())),
            );
        }

        Ok(requirements.into_iter().unique().collect())
    }

    fn modified_function_input_type(&self, ty: &Type) -> Result<Type, ModifierResolverErrors<N>> {
        let Term::FunctionType(func_ty) = &**ty else {
            return Err(ModifierResolverErrors::unreachable(format!(
                "Higher-order modifier requirement found for a non-function input: {ty:?}"
            )));
        };
        let mut signature = Signature::try_from((**func_ty).clone()).map_err(BuildError::from)?;
        self.modify_signature(&mut signature, false);
        Ok(Type::new_function(signature))
    }

    fn signature_has_quantum_data(&self, signature: &Signature) -> bool {
        signature
            .input
            .iter()
            .chain(signature.output.iter())
            .any(|ty| self.qubit_finder.contains_element_type(ty))
    }

    fn function_type_has_quantum_data(&self, ty: &Type) -> Result<bool, ModifierResolverErrors<N>> {
        let Term::FunctionType(func_ty) = &**ty else {
            return Ok(false);
        };
        let signature = Signature::try_from((**func_ty).clone()).map_err(BuildError::from)?;
        Ok(self.signature_has_quantum_data(&signature))
    }

    /// Rewrite function-valued inputs that must be supplied already modified.
    ///
    /// `active_function_input_modifiers` stores requirements using the original
    /// function input indices. `offset` accounts for control arrays inserted
    /// before those inputs in a modified function signature. This helper is
    /// intentionally strict: if a recorded requirement does not point to a
    /// function-typed input, the resolver state is inconsistent and we report an
    /// unreachable error.
    fn modify_higher_order_input_types(
        &mut self,
        input: &mut hugr::types::TypeRow,
        offset: usize,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let modifiers = self.active_function_input_modifiers().clone();
        for (input_index, modifier) in modifiers {
            let saved_modifiers = mem::replace(self.modifiers_mut(), modifier);
            let index = input_index + offset;
            let Some(input_ty) = input.get(index).cloned() else {
                *self.modifiers_mut() = saved_modifiers;
                return Err(ModifierResolverErrors::unreachable(format!(
                    "Higher-order modifier requirement refers to missing input {index}"
                )));
            };
            let Term::FunctionType(_) = &*input_ty else {
                *self.modifiers_mut() = saved_modifiers;
                return Err(ModifierResolverErrors::unreachable(format!(
                    "Higher-order modifier requirement found for a non-function input: {input_ty:?}"
                )));
            };
            let input_ty = input[index].clone();
            let modified_input_ty = self.modified_function_input_type(&input_ty);
            *self.modifiers_mut() = saved_modifiers;
            input.to_mut()[index] = modified_input_ty?;
        }
        Ok(())
    }

    /// Rewrite higher-order function types nested inside a sum value.
    ///
    /// Container boundaries such as `Conditional`, `TailLoop`, `CFG`, and `Tag`
    /// can carry function values inside sum variants. When one of those
    /// function values will be called under the active modifier, every variant
    /// row that carries it must expose the modified function type as well. If
    /// `ty` is not a sum, there is nothing to rewrite.
    fn modify_higher_order_sum_type_if_present(
        &mut self,
        ty: &mut Type,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let Some(sum) = ty.as_sum() else {
            return Ok(());
        };
        let variants = sum
            .variants()
            .cloned()
            .map(TypeRow::try_from)
            .collect::<Result<Vec<_>, _>>()
            .map_err(|e| {
                ModifierResolverErrors::unreachable(format!(
                    "Higher-order modifier rewrite found an open sum variant row: {e}"
                ))
            })?;

        let mut variants = variants;
        for row in &mut variants {
            self.modify_carried_higher_order_types_if_present(row)?;
        }
        *ty = Type::new_sum(variants);
        Ok(())
    }

    fn modify_carried_higher_order_types_if_present(
        &mut self,
        row: &mut TypeRow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        if self.active_function_input_modifiers().is_empty() {
            return Ok(());
        }

        for ty in row.to_mut() {
            match &**ty {
                Term::FunctionType(_) if self.function_type_has_quantum_data(ty)? => {
                    let modified_ty = self.modified_function_input_type(ty)?;
                    *ty = modified_ty;
                }
                Term::SumType(_) => self.modify_higher_order_sum_type_if_present(ty)?,
                _ => {}
            }
        }
        Ok(())
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
        match self.corresp_map().entry(old) {
            std::collections::hash_map::Entry::Vacant(entry) => {
                entry.insert(vec![new]);
                Ok(())
            }
            // Empty entry means that the old wire has no correspondence, so we can insert the new wire.
            std::collections::hash_map::Entry::Occupied(mut entry) if entry.get().is_empty() => {
                entry.insert(vec![new]);
                Ok(())
            }
            // If the old wire is already registered, raise an error.
            std::collections::hash_map::Entry::Occupied(entry) => {
                let former = entry.get();
                Err(ModifierResolverErrors::unreachable(format!(
                    "Wire already registered for node {}. Former [{},...], Latter {}.",
                    old.0, former[0], new
                )))
            }
        }
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
    fn connect_all(
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
                    for w1 in self.map_get(&(in_node, in_port).into())? {
                        for w2 in self.map_get(&(out_node, out_port).into())? {
                            connect(new_dfg, w1, w2)?
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
        let Ok((caller, _)) = h.linked_inputs(n, 0).exactly_one() else {
            return Err(ModifierError::NoCaller(n));
        };
        let optype = h.get_optype(caller);
        if Modifier::from_optype(optype).is_some() {
            return Err(ModifierError::NotInitialModifier(caller, optype.clone()));
        }
        Ok(())
    }

    /// Apply the resolver the current node `n`.
    /// It first checks if the node is a modifier and can be applied.
    /// If not, it returns an [`ModifierError`].
    /// If yes, it applies the modifier to the loaded function,
    fn try_rewrite(
        &mut self,
        hugr: &mut impl HugrMut<Node = N>,
        modifier_node: N,
    ) -> Result<(), ModifierResolverErrors<N>> {
        // Verify that the rewrite can be applied.
        self.verify(hugr, modifier_node)?;

        // The ports that takes inputs from the modified function to the IndirectCall node.
        let modified_fn_loader: Vec<(_, Vec<_>)> = hugr
            .node_outputs(modifier_node)
            .map(|p| (p, hugr.linked_inputs(modifier_node, p).collect()))
            .collect();

        // Modify the chain of modifiers.
        // Make sure that the modifiers are initially empty.
        let modifiers = CombinedModifier::default();
        let new_load = self.with_modifiers(modifiers, |this| {
            this.apply_modifier_chain_to_loaded_fn(hugr, modifier_node)
        })?;

        // Connect the modified function to the inputs
        for (out_port, inputs) in modified_fn_loader {
            for (recv, recv_port) in inputs {
                hugr.disconnect(recv, recv_port);
                hugr.connect(new_load, out_port, recv, recv_port);
            }
        }
        Ok(())
    }

    /// Modifies a function signature to account for control qubits added by modifiers.
    ///
    /// # Arguments
    /// * `signature` - The function signature to modify
    /// * `flatten` - If true, control qubits are represented as individual `Qubit` types,
    ///   if false, control qubits are packed into arrays (used for function definitions).
    fn modify_signature(&self, signature: &mut Signature, flatten: bool) {
        let FuncTypeBase { input, output } = signature;

        if flatten {
            // Flattened mode: represent each control qubit as an individual Qubit type
            let n = self.control_num();
            input.to_mut().splice(0..0, iter::repeat_n(qb_t(), n));
            output.to_mut().splice(0..0, iter::repeat_n(qb_t(), n));
        } else {
            // Non-flattened mode: pack control qubits into arrays (used for function definitions)
            // Build array types for each control group: each element in accum_ctrl specifies
            // how many qubits should be grouped together in a single array
            let control_types = self
                .modifiers
                .accum_ctrl
                .iter()
                .map(|ctrls| array_type(*ctrls as u64, qb_t()))
                .collect::<Vec<_>>();

            // Insert the control array types at the beginning of the input signature
            // splice(0..0, ...) inserts elements at position 0 without removing anything
            input.to_mut().splice(0..0, control_types.iter().cloned());

            // Insert the same control array types at the beginning of the output signature
            output.to_mut().splice(0..0, control_types);
        }
    }

    // We take arbitral topological order of the circuit so that we can plug the control qubits
    // and pass around them in that order. This might not be ideal, as it may produce an inefficient order.
    fn modify_op(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        target_node: N,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let optype = &h.get_optype(target_node).clone();
        match optype {
            // Skip input/output nodes: it should be handled by its parent as it sets control qubits.
            OpType::Input(_) | OpType::Output(_) => {}
            // CFG
            OpType::CFG(cfg) => self.modify_cfg(h, target_node, cfg, new_dfg)?,
            // DFGs
            OpType::DFG(dfg) => self.modify_dfg(h, target_node, dfg, new_dfg)?,
            // TailLoop
            OpType::TailLoop(tail_loop) => {
                self.modify_tail_loop(h, target_node, tail_loop, new_dfg)?
            }
            // Conditional
            OpType::Conditional(conditional) => {
                self.modify_conditional(h, target_node, conditional, new_dfg)?
            }
            // Function calls
            OpType::Call(_) => self.modify_call(h, target_node, optype, new_dfg)?,
            // Indirect call
            OpType::CallIndirect(indir_call) => {
                self.modify_indirect_call(h, target_node, indir_call, new_dfg)?
            }
            // Load function
            OpType::LoadFunction(load) => {
                self.modify_load_function(h, target_node, load, new_dfg)?
            }
            // Operations
            OpType::ExtensionOp(_) => {
                self.modify_extension_op(h, target_node, optype, new_dfg)?;
            }
            // Constants
            OpType::Const(constant) => {
                self.modify_constant(target_node, constant, new_dfg)?;
            }
            // Load constant
            OpType::LoadConstant(_) | OpType::OpaqueOp(_) => {
                self.add_node_no_modification(h, target_node, optype.clone(), new_dfg)?;
            }
            OpType::Tag(tag) => {
                let mut tag = tag.clone();
                for variant in &mut tag.variants {
                    // Tag stores the full sum variant rows in its own optype.
                    // When a branch returns a function value that has been
                    // resolved under a modifier, the Tag output sum must use
                    // the same modified function type as the surrounding
                    // Conditional/CFG edge.
                    self.modify_carried_higher_order_types_if_present(variant)?;
                }
                self.add_node_no_modification(h, target_node, tag, new_dfg)?;
            }

            // Invalid nodes
            OpType::FuncDefn(_) | OpType::FuncDecl(_) | OpType::Module(_) => {
                return Err(ModifierResolverErrors::unreachable(format!(
                    "Invalid node found inside modified function (OpType = {})",
                    optype.clone()
                )));
            }
            OpType::Case(_) => {
                return Err(ModifierResolverErrors::unreachable(
                    "Case cannot be directly modified.".to_string(),
                ));
            }

            // Not resolvable
            OpType::AliasDecl(_)
            | OpType::AliasDefn(_)
            | OpType::ExitBlock(_)
            | OpType::DataflowBlock(_) => {
                return Err(ModifierResolverErrors::unresolvable(
                    target_node,
                    "Unmodifiable node found".to_string(),
                    optype.clone(),
                ));
            }
            _ => {
                // Q. Maybe we should just ignore unknown operations?
                return Err(ModifierResolverErrors::unresolvable(
                    target_node,
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
    /// * `old_node`: the old node
    /// * `new_node`: the new node
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
        old_node: N,
        new_node: Node,
        (inputs, outputs): (
            impl Iterator<Item = &'a Type>,
            impl Iterator<Item = &'a Type>,
        ),
        (input_offset, output_offset, new_offset): (usize, usize, usize),
    ) -> Result<(), ModifierResolverErrors<N>> {
        self.wire_inout(
            (old_node, old_node),
            (new_node, new_node),
            (inputs, outputs),
            (input_offset, output_offset, new_offset),
            &HashSet::new(),
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
        skip_inputs: &HashSet<usize>,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let mut old_in_wire: DirWire<N> = (old_in, IncomingPort::from(input_offset)).into();
        let mut old_out_wire: DirWire<N> = (old_out, OutgoingPort::from(output_offset)).into();
        let mut new_in_wire: DirWire =
            (new_in, IncomingPort::from(input_offset + new_offset)).into();
        let mut new_out_wire: DirWire =
            (new_out, OutgoingPort::from(output_offset + new_offset)).into();
        let mut in_ty = inputs.next();
        let mut out_ty = outputs.next();

        loop {
            // Wire inputs until the first quantum type
            while let Some(ty) = in_ty {
                if self.qubit_finder.contains_element_type(ty) {
                    break;
                }
                if skip_inputs.contains(&old_in_wire.1.index()) {
                    self.map_insert_none(old_in_wire)?;
                } else {
                    self.map_insert(old_in_wire, new_in_wire)?;
                }
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

            // If both are quantum types, wire them in the opposite direction (if dagger is applied)
            // until the next non-quantum type
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
                if skip_inputs.contains(&old_in_wire.1.index()) {
                    self.map_insert_none(old_in_wire)?;
                } else {
                    self.map_insert(old_in_wire, new_in)?;
                }
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
        op_node: N,
        optype: &OpType,
        new_dfg: &mut impl Dataflow,
    ) -> Result<(), ModifierResolverErrors<N>> {
        if self.controls().len() != self.control_num() {
            return Err(ModifierResolverErrors::unreachable(
                "Control qubits are not set correctly.".to_string(),
            ));
        }

        if let Some(tket_op) = TketOp::from_optype(optype) {
            let pv = self.modify_tket_op(op_node, tket_op, new_dfg, &mut vec![])?;
            self.add_edge_from_pv(h, op_node, pv)
        } else if GlobalPhase::from_optype(optype).is_some() {
            let inputs = self.modify_global_phase(op_node, new_dfg, &mut vec![])?;
            self.corresp_map().insert(
                (op_node, IncomingPort::from(0)).into(),
                inputs.into_iter().map(Into::into).collect(),
            );
            Ok(())
        } else if Modifier::from_optype(optype).is_some() {
            // TODO: check if this is ok.
            self.forget_node(h, op_node)
        } else if self.modify_array_op(h, op_node, optype, new_dfg)?
            || self.try_array_convert(h, op_node, optype, new_dfg)?
        {
            Ok(())
        } else {
            // Some other Hugr extension operation.
            // Here, we do not know what is the modified version.
            // We try to place the original operation.
            // TODO: Revisit whether unknown extension operations should return
            // an explicit error instead of falling back to the original operation.
            self.modify_dataflow_op(h, op_node, optype, new_dfg)
        }
    }

    /// Returns a row with modifier controls in the layout expected by a CFG edge.
    fn cfg_control_types(&self, mut row: hugr::types::TypeRow) -> hugr::types::TypeRow {
        let control_num = self.control_num();
        if control_num == 0 {
            return row;
        }

        let types = row.to_mut();
        types.reserve(control_num);
        types.extend(iter::repeat_n(qb_t(), control_num));
        row
    }

    /// Modifies a CFG. Dagger is supported for single node CFGs only.
    fn modify_cfg(
        &mut self,
        h: &mut impl HugrMut<Node = N>,
        cfg_node: N,
        cfg: &CFG,
        new_dfg: &mut impl Container,
    ) -> Result<(), ModifierResolverErrors<N>> {
        let children: Vec<N> = h
            .children(cfg_node)
            .filter(|child| h.get_optype(*child).is_dataflow_block())
            .collect();
        // NOTE: Up to now we support dagger only on CFG with a single node. We may relax this restriction in the future.
        if children.len() != 1 && self.modifiers().dagger {
            return Err(ModifierResolverErrors::unresolvable(
                cfg_node,
                "CFG with more than one node cannot be daggered.".to_string(),
                cfg.clone().into(),
            ));
        }

        // CFGs always thread controls as carried values after block data.
        let mut cfg_input = cfg.signature.input.clone();
        self.modify_carried_higher_order_types_if_present(&mut cfg_input)?;
        let mut cfg_output = cfg.signature.output.clone();
        self.modify_carried_higher_order_types_if_present(&mut cfg_output)?;
        let signature = Signature::new(
            self.cfg_control_types(cfg_input),
            self.cfg_control_types(cfg_output),
        );
        let mut new_cfg = CFGBuilder::new(signature)?;
        let mut bb_map = HashMap::new();

        // Rebuild each basic block with modified body and adjusted block IO.
        for (i, old_bb) in children.iter().copied().enumerate() {
            let OpType::DataflowBlock(old_block) = h.get_optype(old_bb).clone() else {
                return Err(ModifierResolverErrors::unreachable(
                    "Non-basic-block node found while modifying CFG.".to_string(),
                ));
            };
            let mut input = old_block.inputs.clone();
            self.modify_carried_higher_order_types_if_present(&mut input)?;
            let input = self.cfg_control_types(input);
            let mut other_outputs = old_block.other_outputs.clone();
            self.modify_carried_higher_order_types_if_present(&mut other_outputs)?;
            let other_outputs = self.cfg_control_types(other_outputs);
            let mut sum_rows = old_block.sum_rows.clone();
            for row in sum_rows.iter_mut() {
                self.modify_carried_higher_order_types_if_present(row)?;
            }
            let mut new_bb = if i == 0 {
                new_cfg.entry_builder(sum_rows, other_outputs)?
            } else {
                new_cfg.block_builder(input, sum_rows, other_outputs)?
            };
            self.modify_dfg_body(h, old_bb, &mut new_bb)?;
            let new_bb_id = new_bb.finish_sub_container()?;
            bb_map.insert(old_bb, new_bb_id);
        }

        // Recreate the original CFG branch graph over the rebuilt blocks.
        for old_bb in children.iter().copied() {
            let OpType::DataflowBlock(old_block) = h.get_optype(old_bb) else {
                return Err(ModifierResolverErrors::unreachable(
                    "Non-basic-block node found while connecting CFG branches.".to_string(),
                ));
            };
            let new_bb = bb_map.get(&old_bb).ok_or_else(|| {
                ModifierResolverErrors::unreachable("Missing modified basic block.".to_string())
            })?;
            for branch in 0..old_block.sum_rows.len() {
                let (successor, _) = h
                    .linked_inputs(old_bb, OutgoingPort::from(branch))
                    .exactly_one()
                    .map_err(|_| {
                        ModifierResolverErrors::unreachable(format!(
                            "Expected one successor for CFG block branch {branch}."
                        ))
                    })?;
                let new_successor = if let Some(successor) = bb_map.get(&successor) {
                    *successor
                } else if matches!(h.get_optype(successor), OpType::ExitBlock(_)) {
                    new_cfg.exit_block()
                } else {
                    return Err(ModifierResolverErrors::unreachable(
                        "CFG branch successor is neither a basic block nor the exit block."
                            .to_string(),
                    ));
                };
                new_cfg.branch(new_bb, branch, &new_successor)?;
            }
        }

        let new_node = self.insert_sub_dfg(new_dfg, new_cfg)?;

        self.wire_node_inout(
            cfg_node,
            new_node,
            (cfg.signature.input.iter(), cfg.signature.output.iter()),
            (0, 0, 0),
        )?;

        // Expose the controls after the CFG boundary data.
        let input_offset = cfg.signature.input.len();
        let output_offset = cfg.signature.output.len();
        for (i, c) in self.controls().iter_mut().enumerate() {
            new_dfg
                .hugr_mut()
                .connect(c.node(), c.source(), new_node, input_offset + i);
            *c = Wire::new(new_node, OutgoingPort::from(output_offset + i));
        }

        Ok(())
    }
}

/// Returns the direct child of the module root that contains `node`.
///
/// If `node` is not contained under the module root, returns `None`.
fn module_child_containing<N: HugrNode>(h: &impl HugrView<Node = N>, node: N) -> Option<N> {
    let mut child = node;
    while let Some(parent) = h.get_parent(child) {
        if parent == h.module_root() {
            return Some(child);
        }
        child = parent;
    }
    None
}

/// Returns whether `func` has any static target outside `candidates`.
///
/// Functions without readable static targets are treated as used outside the
/// candidate set, so they are preserved.
fn has_static_use_outside_candidates<N: HugrNode>(
    h: &impl HugrView<Node = N>,
    func: N,
    candidates: &HashSet<N>,
) -> bool {
    let Some(mut targets) = h.static_targets(func) else {
        return true;
    };
    // Return true if:
    // - any static target is outside the candidate set, or
    // - any static target is not contained under the module root
    targets.any(|(target, _)| {
        module_child_containing(h, target)
            .is_none_or(|target_owner| !candidates.contains(&target_owner))
    })
}

/// Returns static dependencies of `func` that are also in `candidates`.
fn candidate_static_dependencies<N: HugrNode>(
    h: &impl HugrView<Node = N>,
    func: N,
    candidates: &HashSet<N>,
) -> Vec<N> {
    h.descendants(func)
        .filter_map(|node| h.static_source(node))
        .filter(|target| candidates.contains(target))
        .collect_vec()
}

/// Removes generated modified functions that are no longer reachable.
///
/// A candidate is kept if it is the entrypoint's containing function, is not
/// removable under `scope`, is used from outside the candidate set, or is a
/// static dependency of another kept candidate.
fn remove_unused_modified_functions<N: HugrNode>(
    h: &mut impl HugrMut<Node = N>,
    modified_functions: &HashSet<N>,
    scope: &PassScope,
) {
    let mut candidates = modified_functions
        .iter()
        .copied()
        .filter(|func| {
            h.contains_node(*func)
                && h.get_optype(*func).as_func_defn().is_some()
                && scope.in_scope(h, *func) == InScope::Yes
        })
        .collect::<HashSet<_>>();

    // Removing the function containing the entrypoint would leave an invalid HUGR.
    if let Some(entrypoint_owner) = module_child_containing(h, h.entrypoint()) {
        candidates.remove(&entrypoint_owner);
    }

    let mut live = candidates
        .iter()
        .copied()
        .filter(|func| has_static_use_outside_candidates(h, *func, &candidates))
        .collect::<HashSet<_>>();
    let mut worklist = live.iter().copied().collect::<VecDeque<_>>();

    while let Some(func) = worklist.pop_front() {
        for dependency in candidate_static_dependencies(h, func, &candidates) {
            if live.insert(dependency) {
                worklist.push_back(dependency);
            }
        }
    }

    let unused = candidates.difference(&live).copied().collect_vec();

    for func in unused {
        if h.contains_node(func) {
            h.remove_subtree(func);
        }
    }
}

/// Resolve modifiers in a circuit by applying them to each entry point.
///
/// When resolution creates modified replacements for loaded functions, the
/// original solved function nodes are removed if they are no longer reachable
/// from the entrypoint, from nodes whose interface is preserved by the default
/// pass scope, or from other preserved modified functions.
///
/// Use [`resolve_modifier_with_entrypoints_and_scope`] to make cleanup follow a
/// specific [`PassScope`].
//
// Shouldn't we use a worklist of nodes?
// As we may want to change the order of resolving modifiers
// but might want to rollback if the second last one is called in a different path,
// this may be needed.
pub fn resolve_modifier_with_entrypoints(
    h: &mut impl HugrMut<Node = Node>,
    entry_points: impl IntoIterator<Item = Node>,
) -> Result<(), ModifierResolverErrors<Node>> {
    resolve_modifier_with_entrypoints_and_scope(h, entry_points, &PassScope::default())
}

/// Resolve modifiers in a circuit by applying them to each entry point.
///
/// Cleanup of solved original function nodes respects `scope`: a function is
/// only removed when it is no longer needed and [`PassScope::in_scope`] says the
/// function may be modified freely.
pub fn resolve_modifier_with_entrypoints_and_scope(
    h: &mut impl HugrMut<Node = Node>,
    entry_points: impl IntoIterator<Item = Node>,
    scope: &PassScope,
) -> Result<(), ModifierResolverErrors<Node>> {
    use ModifierResolverErrors::*;

    // Collect entry points into a deque so they can be cloned for later cleanup passes.
    let entry_points: VecDeque<_> = entry_points.into_iter().collect();

    // Walk all nodes reachable from the entry points (children and neighbours)
    // and attempt to rewrite each modifier node it encounters.
    let mut resolver = ModifierResolver::new();
    let mut worklist = entry_points.clone();
    let mut visited = FxHashSet::default();

    while let Some(node) = worklist.pop_front() {
        // Skip nodes that have been removed during previous rewrites or already visited.
        if !h.contains_node(node) || visited.contains(&node) {
            continue;
        }
        // `modify_fn` leaves the original function in the module and records it in
        // `modified_functions` after generating the replacement. From this point on,
        // the original body is stale: walking into it again would resolve modifier
        // chains that have already been accounted for in the replacement function.
        if module_child_containing(h, node)
            .is_some_and(|owner| resolver.modified_functions.contains(&owner))
        {
            visited.insert(node);
            continue;
        }
        // Expand the frontier: enqueue children and dataflow neighbours not yet visited.
        worklist.extend(h.children(node).filter(|n| !visited.contains(n)));
        worklist.extend(h.all_neighbours(node).filter(|n| !visited.contains(n)));
        visited.insert(node);
        if let Err(e) = resolver.try_rewrite(h, node) {
            // ModifierError means this node is not a modifier (or is not the first
            // in its chain) and can safely be skipped.
            // Any other error is a genuine failure and must be propagated.
            if !matches!(e, ModifierError(_)) {
                return Err(e);
            }
        }
    }

    // After all rewrites, some modifier nodes may still remain in the graph
    // (e.g. intermediate nodes in a chain whose last modifier was the one rewritten).
    // Walk the same reachable set again and delete any surviving modifier nodes,
    // together with every downstream node that consumes their output.
    // TODO:
    // This might be insufficient as a cleanup since the resolution procedure might
    // generate nodes that are not reachable from the entry points.
    // If more thorough cleanup is needed, we should run dead code elimination.
    let mut deletelist = entry_points.clone();
    let mut visited = FxHashSet::default();
    while let Some(node) = deletelist.pop_front() {
        // Keep the cleanup pass out of stale original function bodies too. Their
        // modifier nodes may still be present, but removing them after the
        // replacement has been built can invalidate the untouched original HUGR
        // structure and is unnecessary for the solved entrypoint.
        if module_child_containing(h, node)
            .is_some_and(|owner| resolver.modified_functions.contains(&owner))
        {
            visited.insert(node);
            continue;
        }
        deletelist.extend(h.children(node).filter(|n| !visited.contains(n)));
        deletelist.extend(h.all_neighbours(node).filter(|n| !visited.contains(n)));
        visited.insert(node);
        if h.contains_node(node) {
            let optype = h.get_optype(node);
            if Modifier::from_optype(optype).is_some() {
                // Remove the modifier node and all nodes reachable through its
                // output edges (i.e. nodes that would become disconnected).
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
    // Ad hoc cleanup procedure: remove any dangling global-phase nodes that
    // were produced or left behind by the resolution passes above.
    delete_phase(h, entry_points)?;

    // Remove only original functions for which this resolver generated modified
    // replacements, and only when no remaining non-obsolete function uses them.
    remove_unused_modified_functions(h, &resolver.modified_functions, scope);

    h.validate()
        .map_err(|e| ModifierResolverErrors::BuildError(e.into()))?;

    Ok(())
}

// Definitions of helpers for tests
#[cfg(test)]
mod tests {

    use std::{fs, io::BufReader, path::Path};

    use cool_asserts::assert_matches;
    use hugr::{
        Hugr,
        builder::{DataflowSubContainer, HugrBuilder, ModuleBuilder},
        ops::{
            CallIndirect, ExtensionOp,
            handle::{FuncID, NodeHandle},
        },
        std_extensions::collections::array::ArrayOpBuilder,
        type_row,
        types::Term,
    };

    use hugr_core::Visibility;

    use crate::{
        TketOp,
        extension::modifier::{CONTROL_OP_ID, DAGGER_OP_ID, MODIFIER_EXTENSION},
        metadata,
        passes::composable::Preserve,
    };

    use super::*;

    pub(crate) trait SetUnitary {
        fn set_unitary(&mut self);
    }
    impl<T: Container> SetUnitary for T {
        fn set_unitary(&mut self) {
            let node = self.container_node();
            self.hugr_mut()
                .set_metadata::<metadata::UnitaryFlags>(node, 7);
        }
    }

    /// Helper that builds a test hugr with a modifier chain and runs the resolver on it.
    ///
    /// The graph it constructs looks like:
    /// ```text
    /// LoadFunction(foo) -> [Dagger?] -> Control -> CallIndirect
    /// ```
    /// where `foo` is supplied by the caller.
    ///
    /// Parameters:
    /// * `target_num`  – number of plain qubit (target) arguments that `foo` accepts.
    /// * `ctrl_num`  – number of control qubits to wrap around `foo`.
    /// * `foo`  – closure that inserts the function-under-test into the module and
    ///   returns its `FuncID`.
    /// * `dagger`  – if `true`, a `Dagger` modifier is inserted before the `Control`
    ///   modifier, so the full chain is `Dagger → Control`.
    pub(crate) fn test_modifier_resolver(
        target_num: usize,
        ctrl_num: u64,
        foo: impl FnOnce(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        dagger: bool,
    ) {
        let _ = resolved_modifier_test_hugr(target_num, ctrl_num, foo, dagger);
    }

    pub(crate) fn resolved_modifier_test_hugr(
        target_num: usize,
        ctrl_num: u64,
        foo: impl FnOnce(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        dagger: bool,
    ) -> Hugr {
        let (mut h, foo_node) = modifier_test_hugr(target_num, ctrl_num, foo, dagger);

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();

        // We check that the original function node has been removed in the resolved hugr
        assert!(!h.contains_node(foo_node));

        // We also check that there is no modifier node in the resolved hugr.
        assert!(
            h.nodes()
                .all(|node| Modifier::from_optype(h.get_optype(node)).is_none())
        );

        // The resolved hugr must still be structurally valid.
        assert_matches!(h.validate(), Ok(()));

        h
    }

    pub(crate) fn modifier_test_hugr(
        target_num: usize,
        ctrl_num: u64,
        foo: impl FnOnce(&mut ModuleBuilder<Hugr>, usize) -> FuncID<true>,
        dagger: bool,
    ) -> (Hugr, Node) {
        // --- Build the module ---
        let mut module = ModuleBuilder::new();

        // Signature used by the CallIndirect node:
        // inputs/outputs are [array<qubit, ctrl_num>, qubit × target_num] (endomorphic).
        let call_sig = Signature::new_endo(
            [array_type(ctrl_num, qb_t())]
                .into_iter()
                .chain(iter::repeat_n(qb_t(), target_num))
                .collect::<Vec<_>>(),
        );

        // Signature of the "main" function that drives the test:
        // no inputs, outputs are [array<qubit, ctrl_num>, qubit × target_num].
        let main_sig = Signature::new(
            type_row![],
            vec![array_type(ctrl_num, qb_t())]
                .into_iter()
                .chain(iter::repeat_n(qb_t(), target_num))
                .collect::<Vec<_>>(),
        );

        // Dagger modifier parameterised by the target qubit types.
        let dagger_op: ExtensionOp = {
            MODIFIER_EXTENSION
                .instantiate_extension_op(
                    &DAGGER_OP_ID,
                    [
                        iter::repeat_n(qb_t().into(), target_num)
                            .collect::<Vec<_>>()
                            .into(),
                        vec![].into(),
                    ],
                )
                .unwrap()
        };

        // Control modifier parameterised by c_num control qubits and the target qubit types.
        let control_op: ExtensionOp = {
            MODIFIER_EXTENSION
                .instantiate_extension_op(
                    &CONTROL_OP_ID,
                    [
                        Term::BoundedNat(ctrl_num),
                        iter::repeat_n(qb_t().into(), target_num)
                            .collect::<Vec<_>>()
                            .into(),
                        vec![].into(),
                    ],
                )
                .unwrap()
        };

        // Let the caller insert the function-under-test into the module.
        let foo = foo(&mut module, target_num);
        let foo_node = foo.node();

        // Build the "main" function body ---
        let _main = {
            let mut func = module.define_function("main", main_sig).unwrap();

            // Load the function value; this is the wire that will be passed through modifiers.
            let mut call = func.load_func(&foo, &[]).unwrap();

            if dagger {
                // Wrap with Dagger before Control.
                call = func
                    .add_dataflow_op(dagger_op, vec![call])
                    .unwrap()
                    .out_wire(0);
            }

            // Wrap the (possibly daggered) function reference with the Control modifier.
            call = func
                .add_dataflow_op(control_op, vec![call])
                .unwrap()
                .out_wire(0);

            // Allocate ctrl_num fresh qubits to serve as control qubits.
            let mut controls = Vec::new();
            for _ in 0..ctrl_num {
                controls.push(
                    func.add_dataflow_op(TketOp::QAlloc, vec![])
                        .unwrap()
                        .out_wire(0),
                );
            }

            // Allocate target_num fresh qubits to serve as target qubits.
            let mut targ = Vec::new();
            for _ in 0..target_num {
                targ.push(
                    func.add_dataflow_op(TketOp::QAlloc, vec![])
                        .unwrap()
                        .out_wire(0),
                )
            }

            // Pack the control qubits into an array, then call the modified function
            // indirectly with [modified_fn, control_arr, targ...].
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

        // Run the resolver and validate
        let h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));
        (h, foo_node)
    }

    #[test]
    /// Test that a LoadFunction node that is shared between a modifier and a direct call is not removed during resolution.
    fn shared_loaded_function_is_not_removed() {
        let mut module = ModuleBuilder::new();

        let foo_sig = Signature::new_endo(vec![qb_t()]);
        let foo = {
            let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
            func.set_unitary();
            let mut inputs: Vec<Wire> = func.input_wires().collect();
            inputs[0] = func
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs(inputs).unwrap()
        };
        let foo_node = foo.node();

        let ctrl_num = 1;
        let controlled_sig = Signature::new_endo(vec![array_type(ctrl_num, qb_t()), qb_t()]);
        let main_sig = Signature::new(
            type_row![],
            vec![array_type(ctrl_num, qb_t()), qb_t(), qb_t()],
        );
        let control_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(ctrl_num),
                    vec![qb_t().into()].into(),
                    vec![].into(),
                ],
            )
            .unwrap();

        let shared_load_node = {
            let mut func = module.define_function("main", main_sig).unwrap();
            let loaded = func.load_func(foo.handle(), &[]).unwrap();
            let shared_load_node = loaded.node();

            let modified_fn = func
                .add_dataflow_op(control_op, vec![loaded])
                .unwrap()
                .out_wire(0);

            let control = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let controlled_target = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let direct_target = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let control_arr = func.add_new_array(qb_t(), [control]).unwrap();

            let [control_arr, controlled_target] = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig,
                    },
                    [modified_fn, control_arr, controlled_target],
                )
                .unwrap()
                .outputs_arr();

            let direct_target = func
                .add_dataflow_op(CallIndirect { signature: foo_sig }, [loaded, direct_target])
                .unwrap()
                .out_wire(0);

            func.finish_with_outputs([control_arr, controlled_target, direct_target])
                .unwrap();
            shared_load_node
        };

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();

        // Check that the shared load and original function are still present after resolution.
        assert!(h.contains_node(shared_load_node));
        assert!(h.contains_node(foo_node));
        assert_matches!(h.validate(), Ok(()));
    }

    #[test]
    /// Test that an unmodified function that is not used by any remaining modifier is preserved after resolution.
    fn unused_unmodified_function_is_preserved() {
        let mut module = ModuleBuilder::new();

        // `foo` is loaded through a modifier in `main`, so resolving the modifier
        // should create a replacement function and leave the original `foo` unused.
        let foo_sig = Signature::new_endo(vec![qb_t()]);
        let foo = {
            let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
            func.set_unitary();
            let mut inputs: Vec<Wire> = func.input_wires().collect();
            inputs[0] = func
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs(inputs).unwrap()
        };
        let foo_node = foo.node();

        // This function is unused before and after resolution, but it was not
        // modified by the resolver and so must be preserved by this cleanup.
        let unused = {
            let func = module
                .define_function("unused", Signature::new_endo(vec![qb_t()]))
                .unwrap();
            let inputs = func.input_wires();
            func.finish_with_outputs(inputs).unwrap()
        };
        let unused_node = unused.node();

        let ctrl_num = 1;
        let controlled_sig = Signature::new_endo(vec![array_type(ctrl_num, qb_t()), qb_t()]);
        let main_sig = Signature::new(type_row![], vec![array_type(ctrl_num, qb_t()), qb_t()]);
        let control_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(ctrl_num),
                    vec![qb_t().into()].into(),
                    vec![].into(),
                ],
            )
            .unwrap();

        {
            let mut func = module.define_function("main", main_sig).unwrap();
            // Build `LoadFunction(foo) -> Control -> CallIndirect`.
            let loaded = func.load_func(foo.handle(), &[]).unwrap();
            let modified_fn = func
                .add_dataflow_op(control_op, vec![loaded])
                .unwrap()
                .out_wire(0);
            let control = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let target = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let control_arr = func.add_new_array(qb_t(), [control]).unwrap();
            let outputs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig,
                    },
                    [modified_fn, control_arr, target],
                )
                .unwrap()
                .outputs();
            func.finish_with_outputs(outputs).unwrap();
        }

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();

        // Only the original function that was actually replaced is removed.
        assert!(!h.contains_node(foo_node));
        assert!(h.contains_node(unused_node));
        assert_matches!(h.validate(), Ok(()));
    }

    #[test]
    /// Test that a public function used through a modifier is preserved after resolution.
    fn modified_public_function_is_not_removed_after_passes() {
        let mut module = ModuleBuilder::new();

        let foo_sig = Signature::new_endo(vec![qb_t()]);
        let foo = {
            let mut func = module
                .define_function_vis("foo", foo_sig, Visibility::Public)
                .unwrap();

            func.set_unitary();
            let mut inputs: Vec<Wire> = func.input_wires().collect();
            inputs[0] = func
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs(inputs).unwrap()
        };
        let foo_node = foo.node();

        let ctrl_num = 1;
        let controlled_sig = Signature::new_endo(vec![array_type(ctrl_num, qb_t()), qb_t()]);
        let main_sig = Signature::new(type_row![], vec![array_type(ctrl_num, qb_t()), qb_t()]);
        let control_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(ctrl_num),
                    vec![qb_t().into()].into(),
                    vec![].into(),
                ],
            )
            .unwrap();

        {
            let mut func = module.define_function("main", main_sig).unwrap();
            let loaded = func.load_func(foo.handle(), &[]).unwrap();
            let modified_fn = func
                .add_dataflow_op(control_op, vec![loaded])
                .unwrap()
                .out_wire(0);
            let control = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let target = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let control_arr = func.add_new_array(qb_t(), [control]).unwrap();
            let outputs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig,
                    },
                    [modified_fn, control_arr, target],
                )
                .unwrap()
                .outputs();
            func.finish_with_outputs(outputs).unwrap();
        }

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();

        assert!(h.contains_node(foo_node));
        assert_matches!(h.validate(), Ok(()));
    }

    #[test]
    /// Test that public modified functions may be removed when the scope permits it.
    fn modified_public_function_is_removed_when_not_preserved_by_scope() {
        let mut module = ModuleBuilder::new();

        let foo_sig = Signature::new_endo(vec![qb_t()]);
        let foo = {
            let mut func = module
                .define_function_vis("foo", foo_sig, Visibility::Public)
                .unwrap();
            func.set_unitary();
            let mut inputs: Vec<Wire> = func.input_wires().collect();
            inputs[0] = func
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs(inputs).unwrap()
        };
        let foo_node = foo.node();

        let ctrl_num = 1;
        let controlled_sig = Signature::new_endo(vec![array_type(ctrl_num, qb_t()), qb_t()]);
        let main_sig = Signature::new(type_row![], vec![array_type(ctrl_num, qb_t()), qb_t()]);
        let control_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(ctrl_num),
                    vec![qb_t().into()].into(),
                    vec![].into(),
                ],
            )
            .unwrap();

        let main_node = {
            let mut func = module.define_function("main", main_sig).unwrap();
            let loaded = func.load_func(foo.handle(), &[]).unwrap();
            let modified_fn = func
                .add_dataflow_op(control_op, vec![loaded])
                .unwrap()
                .out_wire(0);
            let control = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let target = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let control_arr = func.add_new_array(qb_t(), [control]).unwrap();
            let outputs = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig,
                    },
                    [modified_fn, control_arr, target],
                )
                .unwrap()
                .outputs();
            func.finish_with_outputs(outputs).unwrap().node()
        };

        let mut h = module.finish_hugr().unwrap();
        h.set_entrypoint(main_node);
        assert_matches!(h.validate(), Ok(()));

        let scope = PassScope::Global(Preserve::Entrypoint);
        let root = scope.root(&h).unwrap();
        resolve_modifier_with_entrypoints_and_scope(&mut h, [root], &scope).unwrap();

        assert!(!h.contains_node(foo_node));
        assert_matches!(h.validate(), Ok(()));
    }

    #[test]
    /// Test that a still used function is not removed
    fn modified_dependency_is_preserved_when_original_caller_is_live() {
        let mut module = ModuleBuilder::new();

        // `foo` is a dependency of `bar`. Resolving the modified call to `bar`
        // also creates a modified copy of `foo` for the replacement `bar`.
        let foo_sig = Signature::new_endo(vec![qb_t()]);
        let foo = {
            let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
            func.set_unitary();
            let mut inputs: Vec<Wire> = func.input_wires().collect();
            inputs[0] = func
                .add_dataflow_op(TketOp::X, vec![inputs[0]])
                .unwrap()
                .out_wire(0);
            func.finish_with_outputs(inputs).unwrap()
        };
        let foo_node = foo.node();

        // `bar` is used both through a modifier and by a plain direct call in
        // `main`, so the original `bar` must remain live after resolution.
        let bar = {
            let mut func = module.define_function("bar", foo_sig.clone()).unwrap();
            func.set_unitary();
            let call = func.call(foo.handle(), &[], func.input_wires()).unwrap();
            func.finish_with_outputs(call.outputs()).unwrap()
        };
        let bar_node = bar.node();

        let ctrl_num = 1;
        let controlled_sig = Signature::new_endo(vec![array_type(ctrl_num, qb_t()), qb_t()]);
        let main_sig = Signature::new(
            type_row![],
            vec![array_type(ctrl_num, qb_t()), qb_t(), qb_t()],
        );
        let control_op: ExtensionOp = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(ctrl_num),
                    vec![qb_t().into()].into(),
                    vec![].into(),
                ],
            )
            .unwrap();

        {
            let mut func = module.define_function("main", main_sig).unwrap();
            // One branch uses a controlled indirect call to `bar`; the other
            // branch calls the original `bar` directly.
            let loaded = func.load_func(bar.handle(), &[]).unwrap();
            let modified_fn = func
                .add_dataflow_op(control_op, vec![loaded])
                .unwrap()
                .out_wire(0);

            let control = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let controlled_target = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let direct_target = func
                .add_dataflow_op(TketOp::QAlloc, vec![])
                .unwrap()
                .out_wire(0);
            let control_arr = func.add_new_array(qb_t(), [control]).unwrap();

            let [control_arr, controlled_target] = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig,
                    },
                    [modified_fn, control_arr, controlled_target],
                )
                .unwrap()
                .outputs_arr();
            let direct_target = func
                .call(bar.handle(), &[], [direct_target])
                .unwrap()
                .out_wire(0);

            func.finish_with_outputs([control_arr, controlled_target, direct_target])
                .unwrap();
        }

        let mut h = module.finish_hugr().unwrap();
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(&mut h, [entrypoint]).unwrap();

        // Keeping original `bar` also requires keeping its original dependency `foo`.
        assert!(h.contains_node(bar_node));
        assert!(h.contains_node(foo_node));
        assert_matches!(h.validate(), Ok(()));
    }

    fn load_guppy_example(file: impl AsRef<Path>) -> std::io::Result<Hugr> {
        let reader = fs::File::open(file)?;
        let reader = BufReader::new(reader);
        Ok(Hugr::load(reader, None).unwrap())
    }

    /// Resolve modifiers in `h`
    fn test_resolve(h: &mut Hugr) {
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        resolve_modifier_with_entrypoints(h, [entrypoint]).unwrap();

        assert!(
            h.nodes()
                .all(|node| Modifier::from_optype(h.get_optype(node)).is_none())
        );
        assert_matches!(h.validate(), Ok(()));
    }

    /// Run the pass on hugrs generated by guppy and modifier examples.
    #[rstest::rstest]
    #[case::even_dagger("../test_files/modifier_examples/even_dagger.hugr")]
    #[case::higher_order_recursive("../test_files/modifier_examples/higher_order_recursive.hugr")]
    #[case::higher_order_classical("../test_files/modifier_examples/higher_order_classical.hugr")]
    #[case::higher_order_function_w_loops(
        "../test_files/modifier_examples/higher_order_function_w_loops.hugr"
    )]
    #[case::higher_order_function_w_arrays(
        "../test_files/modifier_examples/higher_order_function_w_arrays.hugr"
    )]
    #[case::multiple_functions_in_ctrl_dagger(
        "../test_files/modifier_examples/multiple_functions_in_ctrl_dagger.hugr"
    )]
    #[case::guppy_modifiers("../test_files/guppy_examples/modifiers.hugr")]
    #[case::assign_in_dagger("../test_files/modifier_examples/assign_in_dagger.hugr")]
    #[case::classical_array_op("../test_files/modifier_examples/classical_array_op.hugr")]
    #[case::classical_function1("../test_files/modifier_examples/classical_function1.hugr")]
    #[case::classical_function2("../test_files/modifier_examples/classical_function2.hugr")]
    #[case::classical_function3("../test_files/modifier_examples/classical_function3.hugr")]
    #[case::ctrl_on_cfg("../test_files/modifier_examples/ctrl_on_cfg.hugr")]
    #[case::multiple_gates2_in_ctrl("../test_files/modifier_examples/multiple_gates2_in_ctrl.hugr")]
    #[case::subscript_in_ctrl("../test_files/modifier_examples/subscript_in_ctrl.hugr")]
    #[case::subscript_in_dagger("../test_files/modifier_examples/subscript_in_dagger.hugr")]
    #[case::subscript_as_controller("../test_files/modifier_examples/subscript_as_controller.hugr")]
    #[case::complex_modifier_stress("../test_files/modifier_examples/complex_modifier_stress.hugr")]
    #[case::ctrl_array_controller("../test_files/modifier_examples/ctrl_array_controller.hugr")]
    #[case::call1_in_ctrl("../test_files/modifier_examples/call1_in_ctrl.hugr")]
    #[case::call2_in_ctrl("../test_files/modifier_examples/call2_in_ctrl.hugr")]
    #[case::multiple_gates1_in_ctrl("../test_files/modifier_examples/multiple_gates1_in_ctrl.hugr")]
    #[case::gate_in_ctrl("../test_files/modifier_examples/gate_in_ctrl.hugr")]
    #[case::call_in_dagger("../test_files/modifier_examples/call_in_dagger.hugr")]
    #[case::multiple_functions_in_dagger(
        "../test_files/modifier_examples/multiple_functions_in_dagger.hugr"
    )]
    #[case::multiple_gates1_in_dagger(
        "../test_files/modifier_examples/multiple_gates1_in_dagger.hugr"
    )]
    #[case::multiple_gates2_in_dagger(
        "../test_files/modifier_examples/multiple_gates2_in_dagger.hugr"
    )]
    #[case::multiple_gates3_in_dagger(
        "../test_files/modifier_examples/multiple_gates3_in_dagger.hugr"
    )]
    #[case::double_modifier("../test_files/modifier_examples/double_modifier.hugr")]
    #[case::modify_array("../test_files/modifier_examples/modify_array.hugr")]
    #[case::multiple_dagger("../test_files/modifier_examples/multiple_dagger.hugr")]
    #[case::nested_ctrl_dagger1("../test_files/modifier_examples/nested_ctrl_dagger1.hugr")]
    #[case::nested_multiple_ctrl1("../test_files/modifier_examples/nested_multiple_ctrl1.hugr")]
    #[case::swap_in_dagger("../test_files/modifier_examples/swap_in_dagger.hugr")]
    #[case::subscript_in_dagger_ctrl(
        "../test_files/modifier_examples/subscript_in_dagger_ctrl.hugr"
    )]
    #[cfg_attr(miri, ignore)] // Opening files is not supported in (isolated) miri
    fn test_examples(#[case] example: &str) {
        let mut h = load_guppy_example(example).unwrap();
        test_resolve(&mut h);
    }

    #[test]
    #[cfg_attr(miri, ignore)] // Opening files is not supported in (isolated) miri
    fn test_power_modifier_error() {
        let mut h = load_guppy_example("../test_files/guppy_examples/use_of_power.hugr").unwrap();
        assert_matches!(h.validate(), Ok(()));

        let entrypoint = h.entrypoint();
        let result = resolve_modifier_with_entrypoints(&mut h, [entrypoint]);
        assert_matches!(
            result,
            Err(ModifierResolverErrors::PowerModifierNotSupported { node: _ })
        );
    }
}
