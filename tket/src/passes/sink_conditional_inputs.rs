//! Sink branch-local producer nodes into HUGR conditionals.
//!
//! This pass looks for dataflow producer nodes outside a [`Conditional`] whose
//! outputs are only threaded through the conditional's shared `other_inputs`,
//! and are used by at most one case. When the producer's operands are already
//! available in every case that needs them, the pass rebuilds the conditional
//! so the producer runs inside that single case instead. If no case uses a
//! shared input, the pass drops that input from the conditional interface and
//! leaves any newly dead producer cleanup to later dead-code elimination.
//!
//! The transformation is inspired by the branch-localization optimization
//! discussed in the RVSDG literature, where expressions used in only one
//! alternative are moved into that alternative to expose further
//! simplifications.
//!
//! The current implementation is intentionally conservative:
//! - It only rewrites plain HUGR [`Conditional`] nodes.
//! - It only sinks leaf dataflow producers with no linked non-value ports.
//! - It only rebuilds case bodies made of leaf dataflow nodes with no linked
//!   non-value ports.
//! - A producer's external operands must already be available as
//!   non-branch-local `other_inputs` of the conditional.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use hugr_core::builder::{
    BuildError, ConditionalBuilder, Dataflow, DataflowSubContainer, HugrBuilder,
};
use hugr_core::hugr::hugrmut::HugrMut;
use hugr_core::hugr::internal::PortgraphNodeMap;
use hugr_core::ops::{Conditional, Const, OpTrait, OpType};
use hugr_core::types::EdgeKind;
use hugr_core::{Hugr, HugrView, IncomingPort, Node, OutgoingPort, PortIndex, Wire};
use itertools::Itertools;
use petgraph::visit::{Topo, Walker};
use thiserror::Error;

use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};

/// A pass that sinks branch-local producer nodes into the relevant
/// [`Conditional`] case.
#[derive(Clone, Debug, Default)]
pub struct SinkConditionalInputsPass {
    scope: PassScope,
}

/// Result type for [`SinkConditionalInputsPass`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SinkConditionalInputsResult {
    /// Number of rewrites applied.
    pub rewrites_applied: usize,
}

/// Errors produced by [`SinkConditionalInputsPass`].
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum SinkConditionalInputsError {
    /// Building a replacement HUGR failed.
    #[error("failed to build replacement conditional: {0}")]
    Build(#[from] BuildError),
    /// The rebuilt conditional did not validate.
    #[error("rebuilt conditional was invalid: {0}")]
    InvalidReplacement(#[from] hugr_core::hugr::ValidationError<Node>),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CaseUsage {
    /// No case reads this conditional `other_input`.
    Unused,
    /// More than one case reads this conditional `other_input`.
    Shared,
    /// Exactly one case reads this conditional `other_input`.
    Local(usize),
}

/// A sinkable node together with the conditional inputs it would eliminate.
///
#[derive(Clone, Debug)]
struct CandidateNode {
    case_index: usize,
    removed_other_inputs: BTreeMap<usize, OutgoingPort>,
}

/// How a selected candidate should be rebuilt or removed.
#[derive(Clone, Debug)]
enum CandidateKind {
    /// A plain value-only dataflow node rebuilt by cloning the op.
    Plain,
    /// A `LoadConstant` rebuilt from its source `Const`.
    LoadConst { konst: Const },
}

/// A complete rewrite plan for one conditional.
///
/// The plan records which shared `other_inputs` are kept, which input edges are
/// removed, and which external nodes are sunk into each case body.
#[derive(Clone, Debug)]
struct ConditionalRewritePlan {
    conditional: Conditional,
    conditional_node: Node,
    case_nodes: Vec<Node>,
    kept_other_inputs: Vec<usize>,
    selected_nodes_by_case: BTreeMap<usize, Vec<Node>>,
    removed_other_inputs: BTreeMap<usize, (Node, OutgoingPort)>,
    selected_nodes: Vec<Node>,
}

impl SinkConditionalInputsPass {
    /// Try to build a rewrite plan for one conditional node.
    ///
    /// Planning is intentionally conservative. We only return a plan when the
    /// conditional body can be rebuilt with plain dataflow nodes and when every
    /// selected producer can be moved into exactly one case.
    fn plan_rewrite<H: HugrView<Node = Node>>(
        &self,
        hugr: &H,
        conditional_node: Node,
    ) -> Option<ConditionalRewritePlan> {
        let conditional: Conditional = hugr.get_optype(conditional_node).clone().try_into().ok()?;
        let parent = hugr.get_parent(conditional_node)?;
        if !is_plain_conditional(hugr, conditional_node) {
            return None;
        }

        let case_nodes = hugr.children(conditional_node).collect::<Vec<_>>();
        if case_nodes.len() != conditional.sum_rows.len() {
            return None;
        }
        if !case_nodes
            .iter()
            .all(|&case_node| case_body_is_rebuildable(hugr, case_node))
        {
            return None;
        }

        let usage = (0..conditional.other_inputs.len())
            .map(|other_input| {
                classify_other_input_usage(hugr, &conditional, &case_nodes, other_input)
            })
            .collect::<Vec<_>>();

        let shared_sources = usage
            .iter()
            .enumerate()
            .filter(|(_, usage)| matches!(usage, CaseUsage::Shared))
            .filter_map(|(other_input, _)| other_input_source(hugr, conditional_node, other_input))
            .collect::<BTreeSet<_>>();

        let mut removed_other_inputs = (0..conditional.other_inputs.len())
            .filter(|&other_input| matches!(usage[other_input], CaseUsage::Unused))
            .filter_map(|other_input| {
                other_input_source(hugr, conditional_node, other_input)
                    .map(|source| (other_input, source))
            })
            .collect::<BTreeMap<_, _>>();

        let mut candidate_nodes = BTreeMap::<Node, CandidateNode>::new();
        for other_input in 0..conditional.other_inputs.len() {
            let Some(case_index) = removable_case(usage[other_input]) else {
                continue;
            };
            let Some((source_node, source_port)) =
                other_input_source(hugr, conditional_node, other_input)
            else {
                continue;
            };
            if hugr.get_parent(source_node) != Some(parent) {
                continue;
            }
            let Some(candidate) =
                analyze_seed_candidate(hugr, conditional_node, case_index, source_node, &usage)
            else {
                continue;
            };
            let entry = candidate_nodes
                .entry(source_node)
                .or_insert(candidate.clone());
            if entry.case_index != case_index {
                continue;
            }
            entry.removed_other_inputs.insert(other_input, source_port);
        }

        discover_upstream_candidates(hugr, conditional_node, &usage, &mut candidate_nodes);

        let mut selected_nodes_by_case = BTreeMap::<usize, Vec<Node>>::new();
        let mut selected_nodes = BTreeSet::new();
        loop {
            let next = candidate_nodes.iter().find_map(|(&node, candidate)| {
                (!selected_nodes.contains(&node)
                    && candidate_inputs_are_available(
                        hugr,
                        node,
                        candidate.case_index,
                        &candidate_nodes,
                        &selected_nodes,
                        &shared_sources,
                    ))
                .then_some((node, candidate.case_index))
            });
            let Some((node, case_index)) = next else {
                break;
            };
            selected_nodes.insert(node);
            selected_nodes_by_case
                .entry(case_index)
                .or_default()
                .push(node);
        }

        removed_other_inputs.extend(selected_nodes.iter().flat_map(|node| {
            candidate_nodes[node]
                .removed_other_inputs
                .iter()
                .map(move |(&other_input, &port)| (other_input, (*node, port)))
        }));

        let kept_other_inputs = (0..conditional.other_inputs.len())
            .filter(|other_input| !removed_other_inputs.contains_key(other_input))
            .collect::<Vec<_>>();

        if removed_other_inputs.is_empty() {
            return None;
        }

        Some(ConditionalRewritePlan {
            conditional,
            conditional_node,
            case_nodes,
            kept_other_inputs,
            selected_nodes_by_case,
            removed_other_inputs,
            selected_nodes: selected_nodes.into_iter().collect(),
        })
    }

    /// Rewrite one region, visiting the region before any nested regions.
    ///
    /// This walk is intentionally region-first. We inspect and possibly rewrite
    /// direct `Conditional` children of `region` before descending into child
    /// regions, so nested conditionals are never collected ahead of their
    /// parent region.
    fn rewrite_region<H: HugrMut<Node = Node>>(
        &self,
        hugr: &mut H,
        region: Node,
        recursive: bool,
    ) -> Result<bool, SinkConditionalInputsError> {
        if matches!(
            hugr.get_optype(region),
            OpType::DFG(_) | OpType::FuncDefn(_) | OpType::Case(_)
        ) {
            let direct_conditionals = hugr
                .children(region)
                .filter(|node| matches!(hugr.get_optype(*node), OpType::Conditional(_)))
                .collect::<Vec<_>>();
            for conditional in direct_conditionals {
                let Some(plan) = self.plan_rewrite(hugr, conditional) else {
                    continue;
                };
                apply_plan(hugr, &plan)?;
                return Ok(true);
            }
        }

        if !recursive {
            return Ok(false);
        }

        let child_regions = hugr
            .children(region)
            .filter(|child| hugr.first_child(*child).is_some())
            .collect::<Vec<_>>();
        for child in child_regions {
            if self.rewrite_region(hugr, child, true)? {
                return Ok(true);
            }
        }

        Ok(false)
    }
}

impl WithScope for SinkConditionalInputsPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for SinkConditionalInputsPass {
    type Error = SinkConditionalInputsError;
    type Result = SinkConditionalInputsResult;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let mut rewrites_applied = 0;
        let Some(root) = self.scope.root(hugr) else {
            return Ok(SinkConditionalInputsResult { rewrites_applied });
        };

        loop {
            if self.rewrite_region(hugr, root, self.scope.recursive())? {
                rewrites_applied += 1;
                continue;
            }
            break;
        }

        Ok(SinkConditionalInputsResult { rewrites_applied })
    }
}

/// Classify how one conditional `other_input` is used across the cases.
fn classify_other_input_usage<H: HugrView<Node = Node>>(
    hugr: &H,
    conditional: &Conditional,
    case_nodes: &[Node],
    other_input: usize,
) -> CaseUsage {
    let cases = case_nodes
        .iter()
        .enumerate()
        .filter_map(|(case_index, &case_node)| {
            let [input, _] = hugr.get_io(case_node)?;
            let port = OutgoingPort::from(conditional.sum_rows[case_index].len() + other_input);
            hugr.linked_inputs(input, port).next().map(|_| case_index)
        })
        .collect::<BTreeSet<_>>();
    match cases.len() {
        0 => CaseUsage::Unused,
        1 => CaseUsage::Local(*cases.first().expect("single case")),
        _ => CaseUsage::Shared,
    }
}

/// Return the external source feeding a conditional `other_input`.
fn other_input_source<H: HugrView<Node = Node>>(
    hugr: &H,
    conditional_node: Node,
    other_input: usize,
) -> Option<(Node, OutgoingPort)> {
    hugr.single_linked_output(conditional_node, IncomingPort::from(other_input + 1))
}

/// Analyze a direct producer of a conditional `other_input`.
///
/// Seed candidates are the first layer of nodes outside the conditional that
/// may be cloned into one case.
fn analyze_seed_candidate<H: HugrView<Node = Node>>(
    hugr: &H,
    conditional_node: Node,
    case_index: usize,
    node: Node,
    usage: &[CaseUsage],
) -> Option<CandidateNode> {
    candidate_kind(hugr, node)?;

    let mut removed_other_inputs = BTreeMap::new();
    for output in hugr.node_outputs(node) {
        let users = hugr.linked_inputs(node, output).collect::<Vec<_>>();
        if users.is_empty() {
            continue;
        }
        for (target, input) in users {
            if target != conditional_node {
                return None;
            }
            let other_input = input.index().checked_sub(1)?;
            match usage.get(other_input)? {
                CaseUsage::Local(local_case) if *local_case == case_index => {
                    removed_other_inputs.insert(other_input, output);
                }
                _ => return None,
            }
        }
    }

    (!removed_other_inputs.is_empty()).then_some(CandidateNode {
        case_index,
        removed_other_inputs,
    })
}

/// Expand the seed set to include upstream chains that stay entirely inside one
/// sinkable slice.
///
/// This lets the pass sink `node1 -> node2 -> conditional` rather than only the
/// last producer directly wired into the conditional.
fn discover_upstream_candidates<H: HugrView<Node = Node>>(
    hugr: &H,
    conditional_node: Node,
    usage: &[CaseUsage],
    candidate_nodes: &mut BTreeMap<Node, CandidateNode>,
) {
    loop {
        let mut additions = Vec::new();
        for (&node, candidate) in candidate_nodes.iter() {
            let signature = hugr
                .get_optype(node)
                .dataflow_signature()
                .expect("candidate nodes are rebuildable dataflow ops");
            for index in 0..signature.input_count() {
                let input = IncomingPort::from(index);
                let Some((source_node, _)) = hugr.single_linked_output(node, input) else {
                    continue;
                };
                if candidate_nodes.contains_key(&source_node) {
                    continue;
                }
                let Some(upstream) = analyze_upstream_candidate(
                    hugr,
                    conditional_node,
                    candidate.case_index,
                    source_node,
                    usage,
                    candidate_nodes,
                ) else {
                    continue;
                };
                additions.push((source_node, upstream));
            }
        }

        if additions.is_empty() {
            break;
        }

        for (node, candidate) in additions {
            candidate_nodes.entry(node).or_insert(candidate);
        }
    }
}

/// Analyze whether an upstream producer can join an existing sinkable chain.
///
/// Every user of `node` must either be:
/// - the conditional itself, through a removable branch-local `other_input`, or
/// - an already-known candidate for the same target case.
fn analyze_upstream_candidate<H: HugrView<Node = Node>>(
    hugr: &H,
    conditional_node: Node,
    case_index: usize,
    node: Node,
    usage: &[CaseUsage],
    candidate_nodes: &BTreeMap<Node, CandidateNode>,
) -> Option<CandidateNode> {
    candidate_kind(hugr, node)?;

    let mut removed_other_inputs = BTreeMap::new();
    let mut has_user = false;
    for output in hugr.node_outputs(node) {
        let users = hugr.linked_inputs(node, output).collect::<Vec<_>>();
        if users.is_empty() {
            continue;
        }
        for (target, input) in users {
            has_user = true;
            if target == conditional_node {
                let other_input = input.index().checked_sub(1)?;
                match usage.get(other_input)? {
                    CaseUsage::Local(local_case) if *local_case == case_index => {
                        removed_other_inputs.insert(other_input, output);
                    }
                    _ => return None,
                }
                continue;
            }

            let target_candidate = candidate_nodes.get(&target)?;
            if target_candidate.case_index != case_index {
                return None;
            }
        }
    }

    has_user.then_some(CandidateNode {
        case_index,
        removed_other_inputs,
    })
}

/// Check whether a candidate's inputs are already available inside its target
/// case.
///
/// Shared conditional inputs are always available; branch-local inputs must be
/// provided by candidates that have already been selected earlier in the same
/// topological expansion.
fn candidate_inputs_are_available<H: HugrView<Node = Node>>(
    hugr: &H,
    node: Node,
    case_index: usize,
    candidate_nodes: &BTreeMap<Node, CandidateNode>,
    selected_nodes: &BTreeSet<Node>,
    shared_sources: &BTreeSet<(Node, OutgoingPort)>,
) -> bool {
    let signature = hugr
        .get_optype(node)
        .dataflow_signature()
        .expect("candidate nodes are rebuildable dataflow ops");
    for index in 0..signature.input_count() {
        let input = IncomingPort::from(index);
        let Some((source_node, source_port)) = hugr.single_linked_output(node, input) else {
            return false;
        };
        if shared_sources.contains(&(source_node, source_port)) {
            continue;
        }
        if !selected_nodes.contains(&source_node) {
            return false;
        }
        let Some(source_candidate) = candidate_nodes.get(&source_node) else {
            return false;
        };
        if source_candidate.case_index != case_index {
            return false;
        }
    }
    true
}

/// Return the unique branch-local case for one usage summary.
fn removable_case(usage: CaseUsage) -> Option<usize> {
    match usage {
        CaseUsage::Local(case_index) => Some(case_index),
        CaseUsage::Unused | CaseUsage::Shared => None,
    }
}

/// Return how a node can be rebuilt by this pass.
///
/// In addition to plain value-only leaf nodes, we also accept `LoadConstant`
/// as a special case. Its static const edge is preserved by rebuilding it with
/// `add_load_const`, so it does not need to satisfy the generic "no linked
/// non-value ports" rule.
fn candidate_kind<H: HugrView<Node = Node>>(hugr: &H, node: Node) -> Option<CandidateKind> {
    if matches!(hugr.get_optype(node), OpType::Input(_) | OpType::Output(_)) {
        return None;
    }

    if is_plain_leaf_dataflow_node(hugr, node) {
        return Some(CandidateKind::Plain);
    }

    let OpType::LoadConstant(_) = hugr.get_optype(node) else {
        return None;
    };
    let const_node = hugr.input_neighbours(node).exactly_one().ok()?;
    let konst: Const = hugr.get_optype(const_node).clone().try_into().ok()?;
    let (value_port, _) = hugr.out_value_types(node).exactly_one().ok()?;
    if hugr
        .node_outputs(node)
        .filter(|port| *port != value_port)
        .any(|port| hugr.linked_inputs(node, port).next().is_some())
    {
        return None;
    }

    Some(CandidateKind::LoadConst { konst })
}

/// Build a replacement conditional containing the rewritten case bodies.
fn build_replacement_conditional<H: HugrView<Node = Node>>(
    hugr: &H,
    plan: &ConditionalRewritePlan,
) -> Result<Hugr, SinkConditionalInputsError> {
    let kept_other_inputs = plan
        .kept_other_inputs
        .iter()
        .map(|&other_input| plan.conditional.other_inputs[other_input].clone())
        .collect::<Vec<_>>();

    let mut conditional_builder = ConditionalBuilder::new(
        plan.conditional.sum_rows.clone(),
        kept_other_inputs,
        plan.conditional.outputs.clone(),
    )?;

    let kept_positions = plan
        .kept_other_inputs
        .iter()
        .enumerate()
        .map(|(new_index, &old_index)| (old_index, new_index))
        .collect::<HashMap<_, _>>();

    for (case_index, &old_case_node) in plan.case_nodes.iter().enumerate() {
        let case_builder = conditional_builder.case_builder(case_index)?;
        rebuild_case_body(
            hugr,
            plan,
            old_case_node,
            case_index,
            &kept_positions,
            case_builder,
        )?;
    }

    Ok(conditional_builder.finish_hugr()?)
}

/// Rebuild one case body with the selected branch-local nodes inlined.
///
/// The rebuilt case preserves the original dataflow order, while inserting the
/// sunk producers ahead of the original nodes that now consume them.
fn rebuild_case_body<H, B>(
    hugr: &H,
    plan: &ConditionalRewritePlan,
    old_case_node: Node,
    case_index: usize,
    kept_positions: &HashMap<usize, usize>,
    mut case_builder: B,
) -> Result<B::ContainerHandle, SinkConditionalInputsError>
where
    H: HugrView<Node = Node>,
    B: Dataflow + DataflowSubContainer,
{
    let [old_input, old_output] = hugr.get_io(old_case_node).expect("case has IO");
    let case_inputs = case_builder.input_wires().collect::<Vec<_>>();
    let sum_len = plan.conditional.sum_rows[case_index].len();

    let mut old_case_inputs = HashMap::<OutgoingPort, Wire>::new();
    for (index, wire) in case_inputs.iter().take(sum_len).copied().enumerate() {
        old_case_inputs.insert(OutgoingPort::from(index), wire);
    }

    for (&old_other_input, &new_position) in kept_positions {
        old_case_inputs.insert(
            OutgoingPort::from(sum_len + old_other_input),
            case_inputs[sum_len + new_position],
        );
    }

    let source_wires = plan
        .kept_other_inputs
        .iter()
        .filter_map(|&old_other_input| {
            let new_position = kept_positions[&old_other_input];
            let (source_node, source_port) =
                other_input_source(hugr, plan.conditional_node, old_other_input)?;
            Some((
                (source_node, source_port),
                case_inputs[sum_len + new_position],
            ))
        })
        .collect::<HashMap<_, _>>();

    let mut built_outputs = HashMap::<(Node, OutgoingPort), Wire>::new();
    for &node in plan
        .selected_nodes_by_case
        .get(&case_index)
        .map(Vec::as_slice)
        .unwrap_or(&[])
    {
        let outputs = rebuild_node(hugr, node, &source_wires, &built_outputs, &mut case_builder)?;
        for (index, wire) in outputs.into_iter().enumerate() {
            built_outputs.insert((node, OutgoingPort::from(index)), wire);
        }
    }

    for (&other_input, &(node, output)) in &plan.removed_other_inputs {
        if plan
            .selected_nodes_by_case
            .get(&case_index)
            .is_some_and(|nodes| nodes.contains(&node))
        {
            let wire = built_outputs[&(node, output)];
            old_case_inputs.insert(OutgoingPort::from(sum_len + other_input), wire);
        }
    }

    let old_case_input_wires = old_case_inputs
        .iter()
        .map(|(&port, &wire)| ((old_input, port), wire))
        .collect::<HashMap<_, _>>();

    let topo_order = case_internal_nodes_in_topological_order(hugr, old_case_node);
    for node in topo_order {
        let outputs = rebuild_node(
            hugr,
            node,
            &old_case_input_wires,
            &built_outputs,
            &mut case_builder,
        )?;
        for (index, wire) in outputs.into_iter().enumerate() {
            built_outputs.insert((node, OutgoingPort::from(index)), wire);
        }
    }

    let outputs = hugr
        .in_value_types(old_output)
        .map(|(input, _)| {
            let (source_node, source_port) = hugr
                .single_linked_output(old_output, input)
                .expect("case output linked");
            if source_node == old_input {
                old_case_inputs[&source_port]
            } else {
                built_outputs[&(source_node, source_port)]
            }
        })
        .collect::<Vec<_>>();

    Ok(case_builder.finish_with_outputs(outputs)?)
}

/// Apply a previously planned rewrite to the in-place HUGR.
///
/// This inserts the rebuilt conditional, reconnects the surviving inputs and
/// outputs, then removes the old conditional and any sunk outer producer nodes.
fn apply_plan<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    plan: &ConditionalRewritePlan,
) -> Result<(), SinkConditionalInputsError> {
    let parent = hugr
        .get_parent(plan.conditional_node)
        .expect("planned conditional still has a parent");
    let replacement = build_replacement_conditional(hugr, plan)?;
    let inserted = hugr.insert_hugr(parent, replacement);
    let new_conditional = inserted.inserted_entrypoint;

    let mut new_input_ports = std::iter::once(IncomingPort::from(0))
        .chain((0..plan.kept_other_inputs.len()).map(|index| IncomingPort::from(index + 1)));
    let mut old_input_ports = std::iter::once(IncomingPort::from(0)).chain(
        plan.kept_other_inputs
            .iter()
            .copied()
            .map(|index| IncomingPort::from(index + 1)),
    );
    for (new_port, old_port) in new_input_ports.by_ref().zip(old_input_ports.by_ref()) {
        let (source_node, source_port) = hugr
            .single_linked_output(plan.conditional_node, old_port)
            .expect("old conditional input is linked");
        hugr.connect(source_node, source_port, new_conditional, new_port);
    }

    for output_index in 0..plan.conditional.outputs.len() {
        let old_port = OutgoingPort::from(output_index);
        let users = hugr
            .linked_inputs(plan.conditional_node, old_port)
            .collect_vec();
        for (target, input) in users {
            hugr.connect(new_conditional, old_port, target, input);
        }
    }

    hugr.remove_subtree(plan.conditional_node);
    for &node in plan.selected_nodes.iter().rev() {
        hugr.remove_node(node);
    }

    Ok(())
}

/// Rebuild one supported node into a case body.
///
/// `available_inputs` maps already-available source outputs to wires in the new
/// case. `built_outputs` provides wires for previously rebuilt internal nodes.
fn rebuild_node<H, B>(
    hugr: &H,
    node: Node,
    available_inputs: &HashMap<(Node, OutgoingPort), Wire>,
    built_outputs: &HashMap<(Node, OutgoingPort), Wire>,
    case_builder: &mut B,
) -> Result<Vec<Wire>, SinkConditionalInputsError>
where
    H: HugrView<Node = Node>,
    B: Dataflow + DataflowSubContainer,
{
    match candidate_kind(hugr, node).expect("node was checked to be rebuildable") {
        CandidateKind::Plain => {
            let signature = hugr
                .get_optype(node)
                .dataflow_signature()
                .expect("plain node has dataflow signature");
            let inputs = (0..signature.input_count())
                .map(|index| {
                    let input = IncomingPort::from(index);
                    let (source_node, source_port) = hugr
                        .single_linked_output(node, input)
                        .expect("rebuilt node input linked");
                    available_inputs
                        .get(&(source_node, source_port))
                        .copied()
                        .or_else(|| built_outputs.get(&(source_node, source_port)).copied())
                        .expect("rebuilt node inputs were planned to be available")
                })
                .collect::<Vec<_>>();
            Ok(case_builder
                .add_dataflow_op(hugr.get_optype(node).clone(), inputs)?
                .outputs()
                .collect::<Vec<_>>())
        }
        CandidateKind::LoadConst { konst } => Ok(vec![case_builder.add_load_const(konst)]),
    }
}

/// Return the internal nodes of a case body in deterministic topological order.
fn case_internal_nodes_in_topological_order<H: HugrView<Node = Node>>(
    hugr: &H,
    case_node: Node,
) -> Vec<Node> {
    let [input, output] = hugr.get_io(case_node).expect("case has IO");
    let (region_graph, node_map) = hugr.region_portgraph(case_node);
    Topo::new(&region_graph)
        .iter(&region_graph)
        .map(|node| node_map.from_portgraph(node))
        .filter(|node| {
            *node != input
                && *node != output
                && hugr.get_parent(*node) == Some(case_node)
                && !hugr.get_optype(*node).is_const()
        })
        .collect()
}

/// Check whether `node` is a plain conditional suitable for rebuilding.
fn is_plain_conditional<H: HugrView<Node = Node>>(hugr: &H, node: Node) -> bool {
    let op = hugr.get_optype(node);
    !has_linked_non_value_ports(hugr, node)
        && matches!(op, OpType::Conditional(_))
        && hugr
            .children(node)
            .all(|case_node| matches!(hugr.get_optype(case_node), OpType::Case(_)))
}

/// Check whether a case body contains only rebuildable plain dataflow nodes.
fn case_body_is_rebuildable<H: HugrView<Node = Node>>(hugr: &H, case_node: Node) -> bool {
    hugr.get_io(case_node).is_some()
        && hugr
            .children(case_node)
            .skip(2)
            .all(|node| hugr.get_optype(node).is_const() || candidate_kind(hugr, node).is_some())
}

/// Check whether a node is a leaf dataflow op with no linked non-value ports.
fn is_plain_leaf_dataflow_node<H: HugrView<Node = Node>>(hugr: &H, node: Node) -> bool {
    let op = hugr.get_optype(node);
    !op.is_container()
        && op.dataflow_signature().is_some()
        && !has_linked_non_value_ports(hugr, node)
}

/// Return whether any linked port on `node` is non-value.
///
/// The pass currently refuses to rebuild control, order, or other non-value
/// wiring, so any linked non-value port makes the node ineligible.
fn has_linked_non_value_ports<H: HugrView<Node = Node>>(hugr: &H, node: Node) -> bool {
    let has_input = hugr.node_inputs(node).any(|port| {
        let Some(kind) = hugr.get_optype(node).port_kind(port) else {
            return false;
        };
        !matches!(kind, EdgeKind::Value(_)) && hugr.linked_outputs(node, port).next().is_some()
    });
    let has_output = hugr.node_outputs(node).any(|port| {
        let Some(kind) = hugr.get_optype(node).port_kind(port) else {
            return false;
        };
        !matches!(kind, EdgeKind::Value(_)) && hugr.linked_inputs(node, port).next().is_some()
    });
    has_input || has_output
}

#[cfg(test)]
mod test {
    use std::fs;
    use std::io::BufReader;
    use std::path::Path;

    use hugr_core::builder::{DFGBuilder, Dataflow, DataflowHugr, SubContainer};
    use hugr_core::extension::prelude::usize_t;
    use hugr_core::extension::prelude::{ConstUsize, Noop};
    use hugr_core::ops::{Conditional, Value};
    use hugr_core::types::Signature;
    use hugr_core::{HugrView, type_row};
    use rstest::rstest;

    use super::*;
    use crate::control::structuralize::StructuralizationStrategy;
    use crate::passes::StructuralizeCfgsPass;

    fn build_branch_local_noop_hugr(reuse_noop_outside: bool) -> Hugr {
        let mut builder = DFGBuilder::new(Signature::new(
            vec![usize_t(), usize_t()],
            vec![usize_t(), usize_t()],
        ))
        .unwrap();
        let [x, y] = builder.input_wires_arr();
        let [x2] = builder
            .add_dataflow_op(Noop::new(usize_t()), [x])
            .unwrap()
            .outputs_arr();
        let pred = builder.add_load_value(Value::true_val());

        let mut conditional = builder
            .conditional_builder(
                ([type_row![], type_row![]], pred),
                [(usize_t(), x), (usize_t(), x2), (usize_t(), y)],
                vec![usize_t(), usize_t()].into(),
            )
            .unwrap();

        let true_case = conditional.case_builder(0).unwrap();
        let inputs = true_case.input_wires().collect::<Vec<_>>();
        true_case
            .finish_with_outputs([inputs[1], inputs[0]])
            .unwrap();

        let false_case = conditional.case_builder(1).unwrap();
        let inputs = false_case.input_wires().collect::<Vec<_>>();
        false_case
            .finish_with_outputs([inputs[2], inputs[0]])
            .unwrap();

        let conditional = conditional.finish_sub_container().unwrap();
        let mut outputs = conditional.outputs().collect::<Vec<_>>();
        if reuse_noop_outside {
            outputs[1] = x2;
        }

        builder.finish_hugr_with_outputs(outputs).unwrap()
    }

    fn build_branch_local_noop_chain_hugr() -> Hugr {
        let mut builder = DFGBuilder::new(Signature::new(
            vec![usize_t(), usize_t()],
            vec![usize_t(), usize_t()],
        ))
        .unwrap();
        let [x, y] = builder.input_wires_arr();
        let [x1] = builder
            .add_dataflow_op(Noop::new(usize_t()), [x])
            .unwrap()
            .outputs_arr();
        let [x2] = builder
            .add_dataflow_op(Noop::new(usize_t()), [x1])
            .unwrap()
            .outputs_arr();
        let pred = builder.add_load_value(Value::true_val());

        let mut conditional = builder
            .conditional_builder(
                ([type_row![], type_row![]], pred),
                [(usize_t(), x), (usize_t(), x2), (usize_t(), y)],
                vec![usize_t(), usize_t()].into(),
            )
            .unwrap();

        let true_case = conditional.case_builder(0).unwrap();
        let inputs = true_case.input_wires().collect::<Vec<_>>();
        true_case
            .finish_with_outputs([inputs[1], inputs[0]])
            .unwrap();

        let false_case = conditional.case_builder(1).unwrap();
        let inputs = false_case.input_wires().collect::<Vec<_>>();
        false_case
            .finish_with_outputs([inputs[2], inputs[0]])
            .unwrap();

        let conditional = conditional.finish_sub_container().unwrap();
        let outputs = conditional.outputs().collect::<Vec<_>>();
        builder.finish_hugr_with_outputs(outputs).unwrap()
    }

    fn build_unused_load_constant_inputs_hugr() -> Hugr {
        let mut builder =
            DFGBuilder::new(Signature::new(vec![], vec![usize_t(), usize_t()])).unwrap();
        let pred = builder.add_load_value(Value::true_val());
        let x = builder.add_load_value(Value::true_val());
        let y = builder.add_load_value(Value::unit());
        let z = builder.add_load_value(Value::false_val());

        let mut conditional = builder
            .conditional_builder(
                ([type_row![], type_row![]], pred),
                [
                    (Value::true_val().get_type().clone(), x),
                    (Value::unit().get_type().clone(), y),
                    (Value::false_val().get_type().clone(), z),
                ],
                vec![usize_t(), usize_t()].into(),
            )
            .unwrap();

        let mut true_case = conditional.case_builder(0).unwrap();
        let out0 = true_case.add_load_value(ConstUsize::new(1));
        let out1 = true_case.add_load_value(ConstUsize::new(2));
        true_case.finish_with_outputs([out0, out1]).unwrap();

        let mut false_case = conditional.case_builder(1).unwrap();
        let out0 = false_case.add_load_value(ConstUsize::new(3));
        let out1 = false_case.add_load_value(ConstUsize::new(4));
        false_case.finish_with_outputs([out0, out1]).unwrap();

        let conditional = conditional.finish_sub_container().unwrap();
        let outputs = conditional.outputs().collect::<Vec<_>>();
        builder.finish_hugr_with_outputs(outputs).unwrap()
    }

    fn build_unused_conditional_input_with_external_use_hugr() -> Hugr {
        let mut builder =
            DFGBuilder::new(Signature::new(vec![usize_t()], vec![usize_t(), usize_t()])).unwrap();
        let [x] = builder.input_wires_arr();
        let [x2] = builder
            .add_dataflow_op(Noop::new(usize_t()), [x])
            .unwrap()
            .outputs_arr();
        let pred = builder.add_load_value(Value::true_val());

        let mut conditional = builder
            .conditional_builder(
                ([type_row![], type_row![]], pred),
                [(usize_t(), x2)],
                vec![usize_t()].into(),
            )
            .unwrap();

        let mut true_case = conditional.case_builder(0).unwrap();
        let one = true_case.add_load_value(ConstUsize::new(1));
        true_case.finish_with_outputs([one]).unwrap();

        let mut false_case = conditional.case_builder(1).unwrap();
        let two = false_case.add_load_value(ConstUsize::new(2));
        false_case.finish_with_outputs([two]).unwrap();

        let conditional = conditional.finish_sub_container().unwrap();
        let [branch_result] = conditional.outputs_arr();
        builder
            .finish_hugr_with_outputs([branch_result, x2])
            .unwrap()
    }

    fn first_conditional(hugr: &Hugr) -> (Node, Conditional) {
        let node = hugr
            .entry_descendants()
            .find(|node| matches!(hugr.get_optype(*node), OpType::Conditional(_)))
            .expect("conditional exists");
        let conditional: Conditional = hugr.get_optype(node).clone().try_into().unwrap();
        (node, conditional)
    }

    fn load_hugr_fixture(file_name: &str) -> Hugr {
        let file = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../test_files/guppy_examples")
            .join(file_name);
        let reader = BufReader::new(fs::File::open(file).unwrap());
        Hugr::load(reader, None).unwrap()
    }

    fn load_guppy_example(name: &str) -> Hugr {
        load_hugr_fixture(&format!("{name}.hugr"))
    }

    #[test]
    fn sinks_branch_local_noop() {
        let mut hugr = build_branch_local_noop_hugr(false);
        let result = SinkConditionalInputsPass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 1);
        hugr.validate().unwrap();

        let (conditional_node, conditional) = first_conditional(&hugr);
        assert_eq!(conditional.other_inputs.len(), 2);

        let cases = hugr.children(conditional_node).collect::<Vec<_>>();
        let case_noop_counts = cases
            .iter()
            .map(|&case_node| {
                hugr.children(case_node)
                    .skip(2)
                    .filter(|node| hugr.get_optype(*node).cast::<Noop>().is_some())
                    .count()
            })
            .collect::<Vec<_>>();
        assert_eq!(case_noop_counts.iter().sum::<usize>(), 1);

        let outer_noops = hugr
            .get_parent(conditional_node)
            .into_iter()
            .flat_map(|parent| hugr.children(parent))
            .filter(|node| hugr.get_optype(*node).cast::<Noop>().is_some())
            .count();
        assert_eq!(outer_noops, 0);
        assert!(cases.iter().any(|&case_node| {
            hugr.children(case_node)
                .skip(2)
                .any(|node| hugr.get_optype(node).cast::<Noop>().is_some())
        }));
    }

    #[test]
    fn keeps_conditional_input_when_producer_has_external_use() {
        let mut hugr = build_branch_local_noop_hugr(true);
        let result = SinkConditionalInputsPass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 0);
        hugr.validate().unwrap();

        let (conditional_node, conditional) = first_conditional(&hugr);
        assert_eq!(conditional.other_inputs.len(), 3);

        let parent = hugr.get_parent(conditional_node).unwrap();
        assert!(
            hugr.children(parent)
                .any(|node| hugr.get_optype(node).cast::<Noop>().is_some())
        );
    }

    #[test]
    fn removes_unused_branch_local_input() {
        let mut builder =
            DFGBuilder::new(Signature::new(vec![usize_t()], vec![usize_t()])).unwrap();
        let [x] = builder.input_wires_arr();
        let [x2] = builder
            .add_dataflow_op(Noop::new(usize_t()), [x])
            .unwrap()
            .outputs_arr();
        let pred = builder.add_load_value(Value::true_val());

        let mut conditional = builder
            .conditional_builder(
                ([type_row![], type_row![]], pred),
                [(usize_t(), x), (usize_t(), x2)],
                vec![usize_t()].into(),
            )
            .unwrap();

        let true_case = conditional.case_builder(0).unwrap();
        let inputs = true_case.input_wires().collect::<Vec<_>>();
        true_case.finish_with_outputs([inputs[0]]).unwrap();

        let false_case = conditional.case_builder(1).unwrap();
        let inputs = false_case.input_wires().collect::<Vec<_>>();
        false_case.finish_with_outputs([inputs[0]]).unwrap();

        let conditional = conditional.finish_sub_container().unwrap();
        let outputs = conditional.outputs().collect::<Vec<_>>();
        let mut hugr = builder.finish_hugr_with_outputs(outputs).unwrap();

        let result = SinkConditionalInputsPass::default().run(&mut hugr).unwrap();
        assert_eq!(result.rewrites_applied, 1);
        hugr.validate().unwrap();

        let (conditional_node, conditional) = first_conditional(&hugr);
        assert_eq!(conditional.other_inputs.len(), 1);

        let parent = hugr.get_parent(conditional_node).unwrap();
        assert!(
            hugr.children(parent)
                .any(|node| hugr.get_optype(node).cast::<Noop>().is_some())
        );
    }

    #[test]
    fn sinks_branch_local_noop_chain() {
        let mut hugr = build_branch_local_noop_chain_hugr();
        let result = SinkConditionalInputsPass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 1);
        hugr.validate().unwrap();

        let (conditional_node, conditional) = first_conditional(&hugr);
        assert_eq!(conditional.other_inputs.len(), 2);

        let cases = hugr.children(conditional_node).collect::<Vec<_>>();
        let case_noop_counts = cases
            .iter()
            .map(|&case_node| {
                hugr.children(case_node)
                    .skip(2)
                    .filter(|node| hugr.get_optype(*node).cast::<Noop>().is_some())
                    .count()
            })
            .collect::<Vec<_>>();
        assert_eq!(case_noop_counts.iter().sum::<usize>(), 2);

        let parent = hugr.get_parent(conditional_node).unwrap();
        assert_eq!(
            hugr.children(parent)
                .filter(|node| hugr.get_optype(*node).cast::<Noop>().is_some())
                .count(),
            0
        );
    }

    #[test]
    fn removes_unused_load_constant_inputs() {
        let mut hugr = build_unused_load_constant_inputs_hugr();
        let result = SinkConditionalInputsPass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 1);
        hugr.validate().unwrap();

        let (conditional_node, conditional) = first_conditional(&hugr);
        assert_eq!(conditional.other_inputs.len(), 0);

        let parent = hugr.get_parent(conditional_node).unwrap();
        let direct_load_constants = hugr
            .children(parent)
            .filter(|node| matches!(hugr.get_optype(*node), OpType::LoadConstant(_)))
            .count();
        assert_eq!(direct_load_constants, 4);
    }

    #[test]
    fn removes_unused_conditional_input_with_external_use() {
        let mut hugr = build_unused_conditional_input_with_external_use_hugr();
        let result = SinkConditionalInputsPass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 1);
        hugr.validate().unwrap();

        let (conditional_node, conditional) = first_conditional(&hugr);
        assert_eq!(conditional.other_inputs.len(), 0);

        let parent = hugr.get_parent(conditional_node).unwrap();
        let [_input, output] = hugr.get_io(parent).unwrap();
        let external_value_source = hugr
            .single_linked_output(output, IncomingPort::from(1))
            .expect("second root output remains connected");
        assert_ne!(external_value_source.0, conditional_node);
    }

    #[rstest]
    #[case::rvsdg(StructuralizationStrategy::Rvsdg)]
    #[case::relooper(StructuralizationStrategy::Relooper)]
    fn runs_after_structuralize_shortcircuit(#[case] strategy: StructuralizationStrategy) {
        let mut hugr = load_guppy_example("shortcircuit");
        StructuralizeCfgsPass::default()
            .with_strategy(strategy)
            .run(&mut hugr)
            .unwrap();
        SinkConditionalInputsPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
    }

    #[test]
    fn runs_on_shortcircuit_fixture() {
        let mut hugr = load_guppy_example("shortcircuit");

        SinkConditionalInputsPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
    }
}
