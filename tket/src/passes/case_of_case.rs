//! Fuse direct sibling conditional chains with a case-of-case rewrite.
//!
//! This pass looks for two direct sibling [`Conditional`] nodes in the same
//! region where one output of the producer feeds the selector input of the
//! consumer. When every producer case constructs a known selector variant for
//! that output, the pass pushes the consumer branch into the producer cases and
//! immediately selects the now-known consumer case.
//!
//! This rewrite is inspired by the classic case-of-case transformation used in
//! functional-language compilers, and by the closely related jump-threading
//! family of CFG optimizations. See:
//! - Simon Peyton Jones et al., GHC Core simplifier notes on case-of-case in
//!   [`GHC.Core`](https://ghc.gitlab.haskell.org/ghc/doc/libraries/ghc-9.15-inplace/src/GHC.Core.html)
//! - LLVM's [`JumpThreadingPass`](https://llvm.org/doxygen/classllvm_1_1JumpThreadingPass.html)
//!
//! The current implementation is intentionally conservative:
//! - It only rewrites direct sibling `Conditional A -> Conditional B` patterns.
//! - The fused producer output must have a single use: consumer input `0`.
//! - Producer and consumer case bodies must be rebuildable from plain leaf
//!   dataflow nodes plus `LoadConstant`.
//! - The user can bound duplication with `max_duplicated_nodes`.

use std::collections::HashMap;

use hugr_core::builder::{
    BuildError, ConditionalBuilder, Dataflow, DataflowSubContainer, HugrBuilder,
};
use hugr_core::hugr::hugrmut::HugrMut;
use hugr_core::hugr::internal::PortgraphNodeMap;
use hugr_core::ops::{Conditional, Const, OpTag, OpTrait, OpType, Tag};
use hugr_core::types::EdgeKind;
use hugr_core::{Hugr, HugrView, IncomingPort, Node, OutgoingPort, PortIndex, Wire};
use itertools::Itertools;
use petgraph::visit::{Topo, Walker};
use thiserror::Error;

use crate::passes::composable::WithScope;
use crate::passes::{ComposablePass, PassScope};

/// Pass that fuses direct sibling conditional chains.
#[derive(Clone, Debug)]
pub struct CaseOfCasePass {
    scope: PassScope,
    max_duplicated_nodes: usize,
}

impl Default for CaseOfCasePass {
    fn default() -> Self {
        Self {
            scope: PassScope::default(),
            max_duplicated_nodes: 32,
        }
    }
}

/// Result type for [`CaseOfCasePass`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CaseOfCaseResult {
    /// Number of rewrites applied.
    pub rewrites_applied: usize,
}

/// Errors produced by [`CaseOfCasePass`].
#[derive(Debug, Error)]
#[non_exhaustive]
pub enum CaseOfCaseError {
    /// Building a replacement conditional failed.
    #[error("failed to build replacement conditional: {0}")]
    Build(#[from] BuildError),
    /// The rebuilt conditional did not validate.
    #[error("rebuilt conditional was invalid: {0}")]
    InvalidReplacement(#[from] hugr_core::hugr::ValidationError<Node>),
}

/// How one consumer shared input is sourced after fusion.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ConsumerOtherInputSource {
    /// The input still comes from outside the fused conditional.
    External {
        old_index: usize,
        fused_index: usize,
    },
    /// The input is satisfied by one surviving producer output.
    ProducerOutput { output_index: usize },
}

/// Selected consumer case for one producer case.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ProducerCaseSelection {
    consumer_case_index: usize,
    selector_tag_node: Node,
}

/// Complete rewrite plan for one direct producer/consumer pair.
#[derive(Clone, Debug)]
struct CaseOfCasePlan {
    producer: Conditional,
    producer_node: Node,
    producer_case_nodes: Vec<Node>,
    consumer: Conditional,
    consumer_node: Node,
    consumer_case_nodes: Vec<Node>,
    surviving_producer_outputs: Vec<usize>,
    external_consumer_other_inputs: Vec<usize>,
    consumer_other_inputs: Vec<ConsumerOtherInputSource>,
    selections: Vec<ProducerCaseSelection>,
}

/// How a rebuildable case node is reconstructed.
#[derive(Clone, Debug)]
enum RebuildNodeKind {
    /// A plain value-only leaf dataflow node rebuilt by cloning the op.
    Plain,
    /// A `LoadConstant` rebuilt from its source `Const`.
    LoadConst { konst: Const },
}

impl CaseOfCasePass {
    /// Set the duplication budget used by the matcher.
    pub fn with_max_duplicated_nodes(mut self, max_duplicated_nodes: usize) -> Self {
        self.max_duplicated_nodes = max_duplicated_nodes;
        self
    }

    /// Try to plan a direct sibling case-of-case rewrite rooted at `consumer_node`.
    ///
    /// Matching is conservative and only succeeds for one producer/consumer
    /// pair that can be rebuilt entirely from supported case-body nodes.
    fn plan_rewrite<H: HugrView<Node = Node>>(
        &self,
        hugr: &H,
        consumer_node: Node,
    ) -> Option<CaseOfCasePlan> {
        let consumer: Conditional = hugr.get_optype(consumer_node).clone().try_into().ok()?;
        let parent = hugr.get_parent(consumer_node)?;
        if !is_plain_conditional(hugr, consumer_node) {
            return None;
        }

        let (producer_node, fused_output_port) =
            hugr.single_linked_output(consumer_node, IncomingPort::from(0))?;
        if hugr.get_parent(producer_node) != Some(parent) {
            return None;
        }
        let fused_output_index = fused_output_port.index();
        let producer: Conditional = hugr.get_optype(producer_node).clone().try_into().ok()?;
        if !is_plain_conditional(hugr, producer_node) {
            return None;
        }

        let producer_output_users = hugr
            .linked_inputs(producer_node, fused_output_port)
            .collect::<Vec<_>>();
        if producer_output_users != vec![(consumer_node, IncomingPort::from(0))] {
            return None;
        }

        let producer_case_nodes = hugr.children(producer_node).collect::<Vec<_>>();
        if producer_case_nodes.len() != producer.sum_rows.len()
            || !producer_case_nodes
                .iter()
                .all(|&case_node| case_body_is_rebuildable(hugr, case_node))
        {
            return None;
        }

        let consumer_case_nodes = hugr.children(consumer_node).collect::<Vec<_>>();
        if consumer_case_nodes.len() != consumer.sum_rows.len()
            || !consumer_case_nodes
                .iter()
                .all(|&case_node| case_body_is_rebuildable(hugr, case_node))
        {
            return None;
        }

        let surviving_producer_outputs = (0..producer.outputs.len())
            .filter(|&index| index != fused_output_index)
            .collect::<Vec<_>>();

        let mut external_consumer_other_inputs = Vec::new();
        let mut consumer_other_inputs = Vec::with_capacity(consumer.other_inputs.len());
        for old_index in 0..consumer.other_inputs.len() {
            let (source_node, source_port) = other_input_source(hugr, consumer_node, old_index)
                .expect("conditional input linked");
            if source_node == producer_node {
                if source_port.index() == fused_output_index {
                    return None;
                }
                consumer_other_inputs.push(ConsumerOtherInputSource::ProducerOutput {
                    output_index: source_port.index(),
                });
            } else {
                let fused_index = external_consumer_other_inputs.len();
                external_consumer_other_inputs.push(old_index);
                consumer_other_inputs.push(ConsumerOtherInputSource::External {
                    old_index,
                    fused_index,
                });
            }
        }

        let mut duplication_cost = 0usize;
        let mut selections = Vec::with_capacity(producer_case_nodes.len());
        for &producer_case_node in &producer_case_nodes {
            let selection = producer_case_selection(
                hugr,
                producer_case_node,
                fused_output_index,
                consumer_case_nodes.len(),
            )?;
            duplication_cost += case_internal_nodes_in_topological_order(
                hugr,
                consumer_case_nodes[selection.consumer_case_index],
            )
            .len();
            selections.push(selection);
        }
        if duplication_cost > self.max_duplicated_nodes {
            return None;
        }

        Some(CaseOfCasePlan {
            producer,
            producer_node,
            producer_case_nodes,
            consumer,
            consumer_node,
            consumer_case_nodes,
            surviving_producer_outputs,
            external_consumer_other_inputs,
            consumer_other_inputs,
            selections,
        })
    }

    /// Rewrite one region before visiting any nested child regions.
    ///
    /// The pass processes direct sibling opportunities in the current region to
    /// a local fixpoint before recursing, so newly exposed sibling chains are
    /// handled in-region first.
    fn rewrite_region<H: HugrMut<Node = Node>>(
        &self,
        hugr: &mut H,
        region: Node,
        recursive: bool,
    ) -> Result<bool, CaseOfCaseError> {
        if OpTag::DataflowParent.is_superset(hugr.get_optype(region).tag()) {
            let direct_conditionals = hugr
                .children(region)
                .filter(|node| matches!(hugr.get_optype(*node), OpType::Conditional(_)))
                .collect::<Vec<_>>();
            for consumer_node in direct_conditionals {
                let Some(plan) = self.plan_rewrite(hugr, consumer_node) else {
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

impl WithScope for CaseOfCasePass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for CaseOfCasePass {
    type Error = CaseOfCaseError;
    type Result = CaseOfCaseResult;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let Some(root) = self.scope.root(hugr) else {
            return Ok(CaseOfCaseResult::default());
        };

        let mut rewrites_applied = 0;
        while self.rewrite_region(hugr, root, self.scope.recursive())? {
            rewrites_applied += 1;
        }

        Ok(CaseOfCaseResult { rewrites_applied })
    }
}

/// Return how one consumer shared input is currently sourced.
fn other_input_source<H: HugrView<Node = Node>>(
    hugr: &H,
    conditional_node: Node,
    other_input: usize,
) -> Option<(Node, OutgoingPort)> {
    hugr.single_linked_output(conditional_node, IncomingPort::from(other_input + 1))
}

/// Return the selected consumer case for one producer case.
///
/// The fused producer output must come directly from a case-local `Tag`.
fn producer_case_selection<H: HugrView<Node = Node>>(
    hugr: &H,
    producer_case_node: Node,
    fused_output_index: usize,
    consumer_case_count: usize,
) -> Option<ProducerCaseSelection> {
    let [_input, output] = hugr.get_io(producer_case_node)?;
    let (selector_node, _) =
        hugr.single_linked_output(output, IncomingPort::from(fused_output_index))?;
    if hugr.get_parent(selector_node) != Some(producer_case_node) {
        return None;
    }
    let tag: Tag = hugr.get_optype(selector_node).clone().try_into().ok()?;
    (tag.tag < consumer_case_count).then_some(ProducerCaseSelection {
        consumer_case_index: tag.tag,
        selector_tag_node: selector_node,
    })
}

/// Build the fused replacement conditional described by `plan`.
fn build_replacement_conditional<H: HugrView<Node = Node>>(
    hugr: &H,
    plan: &CaseOfCasePlan,
) -> Result<Hugr, CaseOfCaseError> {
    let fused_other_inputs = plan
        .producer
        .other_inputs
        .iter()
        .cloned()
        .chain(
            plan.external_consumer_other_inputs
                .iter()
                .map(|&old_index| plan.consumer.other_inputs[old_index].clone()),
        )
        .collect::<Vec<_>>();
    let fused_outputs = plan
        .surviving_producer_outputs
        .iter()
        .map(|&index| plan.producer.outputs[index].clone())
        .chain(plan.consumer.outputs.iter().cloned())
        .collect::<Vec<_>>();

    let mut conditional_builder = ConditionalBuilder::new(
        plan.producer.sum_rows.clone(),
        fused_other_inputs,
        fused_outputs,
    )?;

    for (producer_case_index, &producer_case_node) in plan.producer_case_nodes.iter().enumerate() {
        let case_builder = conditional_builder.case_builder(producer_case_index)?;
        rebuild_fused_case_body(
            hugr,
            plan,
            producer_case_index,
            producer_case_node,
            case_builder,
        )?;
    }

    Ok(conditional_builder.finish_hugr()?)
}

/// Rebuild one fused producer case.
fn rebuild_fused_case_body<H, B>(
    hugr: &H,
    plan: &CaseOfCasePlan,
    producer_case_index: usize,
    producer_case_node: Node,
    mut case_builder: B,
) -> Result<B::ContainerHandle, CaseOfCaseError>
where
    H: HugrView<Node = Node>,
    B: Dataflow + DataflowSubContainer,
{
    let [producer_input, producer_output] = hugr.get_io(producer_case_node).expect("case has IO");
    let case_inputs = case_builder.input_wires().collect::<Vec<_>>();
    let producer_sum_len = plan.producer.sum_rows[producer_case_index].len();

    let mut producer_case_inputs = HashMap::<OutgoingPort, Wire>::new();
    for (index, wire) in case_inputs
        .iter()
        .take(producer_sum_len)
        .copied()
        .enumerate()
    {
        producer_case_inputs.insert(OutgoingPort::from(index), wire);
    }
    for (index, wire) in case_inputs
        .iter()
        .skip(producer_sum_len)
        .take(plan.producer.other_inputs.len())
        .copied()
        .enumerate()
    {
        producer_case_inputs.insert(OutgoingPort::from(producer_sum_len + index), wire);
    }

    let producer_case_input_wires = producer_case_inputs
        .iter()
        .map(|(&port, &wire)| ((producer_input, port), wire))
        .collect::<HashMap<_, _>>();
    let mut producer_built_outputs = HashMap::<(Node, OutgoingPort), Wire>::new();
    for node in case_internal_nodes_in_topological_order(hugr, producer_case_node) {
        let outputs = rebuild_node(
            hugr,
            node,
            &producer_case_input_wires,
            &producer_built_outputs,
            &mut case_builder,
        )?;
        for (index, wire) in outputs.into_iter().enumerate() {
            producer_built_outputs.insert((node, OutgoingPort::from(index)), wire);
        }
    }

    let producer_outputs = (0..plan.producer.outputs.len())
        .map(|output_index| {
            let input = IncomingPort::from(output_index);
            let (source_node, source_port) = hugr
                .single_linked_output(producer_output, input)
                .expect("case output linked");
            if source_node == producer_input {
                producer_case_inputs[&source_port]
            } else {
                producer_built_outputs[&(source_node, source_port)]
            }
        })
        .collect::<Vec<_>>();

    let selection = plan.selections[producer_case_index];
    let tag_payloads = tag_payload_wires(
        hugr,
        selection.selector_tag_node,
        producer_input,
        &producer_case_inputs,
        &producer_built_outputs,
    );

    let consumer_case_node = plan.consumer_case_nodes[selection.consumer_case_index];
    let [consumer_input, consumer_output] = hugr.get_io(consumer_case_node).expect("case has IO");
    let consumer_sum_len = plan.consumer.sum_rows[selection.consumer_case_index].len();

    let mut consumer_case_inputs = HashMap::<OutgoingPort, Wire>::new();
    for (index, wire) in tag_payloads.into_iter().enumerate() {
        consumer_case_inputs.insert(OutgoingPort::from(index), wire);
    }
    for (old_index, binding) in plan.consumer_other_inputs.iter().enumerate() {
        let wire = match binding {
            ConsumerOtherInputSource::External { fused_index, .. } => {
                case_inputs[producer_sum_len + plan.producer.other_inputs.len() + fused_index]
            }
            ConsumerOtherInputSource::ProducerOutput { output_index } => {
                producer_outputs[*output_index]
            }
        };
        consumer_case_inputs.insert(OutgoingPort::from(consumer_sum_len + old_index), wire);
    }

    let consumer_case_input_wires = consumer_case_inputs
        .iter()
        .map(|(&port, &wire)| ((consumer_input, port), wire))
        .collect::<HashMap<_, _>>();
    let mut consumer_built_outputs = HashMap::<(Node, OutgoingPort), Wire>::new();
    for node in case_internal_nodes_in_topological_order(hugr, consumer_case_node) {
        let outputs = rebuild_node(
            hugr,
            node,
            &consumer_case_input_wires,
            &consumer_built_outputs,
            &mut case_builder,
        )?;
        for (index, wire) in outputs.into_iter().enumerate() {
            consumer_built_outputs.insert((node, OutgoingPort::from(index)), wire);
        }
    }

    let consumer_outputs = (0..plan.consumer.outputs.len())
        .map(|output_index| {
            let input = IncomingPort::from(output_index);
            let (source_node, source_port) = hugr
                .single_linked_output(consumer_output, input)
                .expect("case output linked");
            if source_node == consumer_input {
                consumer_case_inputs[&source_port]
            } else {
                consumer_built_outputs[&(source_node, source_port)]
            }
        })
        .collect::<Vec<_>>();

    let outputs = plan
        .surviving_producer_outputs
        .iter()
        .map(|&index| producer_outputs[index])
        .chain(consumer_outputs)
        .collect::<Vec<_>>();
    Ok(case_builder.finish_with_outputs(outputs)?)
}

/// Return the payload wires feeding one rebuilt `Tag`.
fn tag_payload_wires<H: HugrView<Node = Node>>(
    hugr: &H,
    tag_node: Node,
    producer_input: Node,
    producer_case_inputs: &HashMap<OutgoingPort, Wire>,
    producer_built_outputs: &HashMap<(Node, OutgoingPort), Wire>,
) -> Vec<Wire> {
    let tag: Tag = hugr
        .get_optype(tag_node)
        .clone()
        .try_into()
        .expect("tag node");
    (0..tag.variants[tag.tag].len())
        .map(|index| {
            let input = IncomingPort::from(index);
            let (source_node, source_port) = hugr
                .single_linked_output(tag_node, input)
                .expect("tag payload linked");
            if source_node == producer_input {
                producer_case_inputs[&source_port]
            } else {
                producer_built_outputs[&(source_node, source_port)]
            }
        })
        .collect()
}

/// Apply a previously planned rewrite in-place.
fn apply_plan<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    plan: &CaseOfCasePlan,
) -> Result<(), CaseOfCaseError> {
    let parent = hugr
        .get_parent(plan.producer_node)
        .expect("planned producer still has a parent");
    let replacement = build_replacement_conditional(hugr, plan)?;
    let inserted = hugr.insert_hugr(parent, replacement);
    let new_conditional = inserted.inserted_entrypoint;

    let mut new_input_port = 0usize;
    let old_producer_selector = hugr
        .single_linked_output(plan.producer_node, IncomingPort::from(0))
        .expect("producer selector linked");
    hugr.connect(
        old_producer_selector.0,
        old_producer_selector.1,
        new_conditional,
        IncomingPort::from(new_input_port),
    );
    new_input_port += 1;

    for old_index in 0..plan.producer.other_inputs.len() {
        let (source_node, source_port) =
            other_input_source(hugr, plan.producer_node, old_index).expect("producer input linked");
        hugr.connect(
            source_node,
            source_port,
            new_conditional,
            IncomingPort::from(new_input_port),
        );
        new_input_port += 1;
    }
    for &old_index in &plan.external_consumer_other_inputs {
        let (source_node, source_port) =
            other_input_source(hugr, plan.consumer_node, old_index).expect("consumer input linked");
        hugr.connect(
            source_node,
            source_port,
            new_conditional,
            IncomingPort::from(new_input_port),
        );
        new_input_port += 1;
    }

    for (new_index, &old_index) in plan.surviving_producer_outputs.iter().enumerate() {
        reconnect_users_skipping(
            hugr,
            plan.producer_node,
            OutgoingPort::from(old_index),
            new_conditional,
            OutgoingPort::from(new_index),
            Some(plan.consumer_node),
        );
    }
    let consumer_output_offset = plan.surviving_producer_outputs.len();
    for old_index in 0..plan.consumer.outputs.len() {
        reconnect_users_skipping(
            hugr,
            plan.consumer_node,
            OutgoingPort::from(old_index),
            new_conditional,
            OutgoingPort::from(consumer_output_offset + old_index),
            None,
        );
    }

    hugr.remove_subtree(plan.consumer_node);
    hugr.remove_subtree(plan.producer_node);
    Ok(())
}

/// Reconnect all users of one old output to the replacement output.
fn reconnect_users_skipping<H: HugrMut<Node = Node>>(
    hugr: &mut H,
    old_node: Node,
    old_port: OutgoingPort,
    new_node: Node,
    new_port: OutgoingPort,
    skip_target: Option<Node>,
) {
    let users = hugr.linked_inputs(old_node, old_port).collect::<Vec<_>>();
    for (target, input) in users {
        if skip_target == Some(target) {
            continue;
        }
        hugr.connect(new_node, new_port, target, input);
    }
}

/// Rebuild one supported leaf node.
fn rebuild_node<H, B>(
    hugr: &H,
    node: Node,
    available_inputs: &HashMap<(Node, OutgoingPort), Wire>,
    built_outputs: &HashMap<(Node, OutgoingPort), Wire>,
    case_builder: &mut B,
) -> Result<Vec<Wire>, CaseOfCaseError>
where
    H: HugrView<Node = Node>,
    B: Dataflow + DataflowSubContainer,
{
    match rebuild_node_kind(hugr, node).expect("node checked to be rebuildable") {
        RebuildNodeKind::Plain => {
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
                        .expect("rebuilt inputs were planned to be available")
                })
                .collect::<Vec<_>>();
            Ok(case_builder
                .add_dataflow_op(hugr.get_optype(node).clone(), inputs)?
                .outputs()
                .collect::<Vec<_>>())
        }
        RebuildNodeKind::LoadConst { konst } => Ok(vec![case_builder.add_load_const(konst)]),
    }
}

/// Return the internal nodes of one case body in deterministic topological order.
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

/// Return whether `node` is a plain conditional suitable for rebuilding.
fn is_plain_conditional<H: HugrView<Node = Node>>(hugr: &H, node: Node) -> bool {
    !has_linked_non_value_ports(hugr, node)
        && matches!(hugr.get_optype(node), OpType::Conditional(_))
        && hugr
            .children(node)
            .all(|case_node| matches!(hugr.get_optype(case_node), OpType::Case(_)))
}

/// Return whether one case body contains only rebuildable plain leaf nodes.
fn case_body_is_rebuildable<H: HugrView<Node = Node>>(hugr: &H, case_node: Node) -> bool {
    hugr.get_io(case_node).is_some()
        && hugr
            .children(case_node)
            .skip(2)
            .all(|node| hugr.get_optype(node).is_const() || rebuild_node_kind(hugr, node).is_some())
}

/// Return how one rebuildable leaf node should be reconstructed.
fn rebuild_node_kind<H: HugrView<Node = Node>>(hugr: &H, node: Node) -> Option<RebuildNodeKind> {
    if matches!(hugr.get_optype(node), OpType::Input(_) | OpType::Output(_)) {
        return None;
    }
    if is_plain_leaf_dataflow_node(hugr, node) {
        return Some(RebuildNodeKind::Plain);
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

    Some(RebuildNodeKind::LoadConst { konst })
}

/// Return whether one node is a value-only leaf dataflow op.
fn is_plain_leaf_dataflow_node<H: HugrView<Node = Node>>(hugr: &H, node: Node) -> bool {
    let op = hugr.get_optype(node);
    !op.is_container()
        && op.dataflow_signature().is_some()
        && !has_linked_non_value_ports(hugr, node)
}

/// Return whether any linked port on `node` is non-value.
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
    use hugr_core::builder::{DFGBuilder, Dataflow, DataflowHugr, SubContainer};
    use hugr_core::extension::prelude::usize_t;
    use hugr_core::extension::prelude::{ConstUsize, Noop};
    use hugr_core::ops::{Conditional, Value};
    use hugr_core::types::{Signature, Type, TypeRow};
    use hugr_core::{HugrView, type_row};

    use super::*;

    fn first_conditional(hugr: &Hugr) -> (Node, Conditional) {
        let node = hugr
            .entry_descendants()
            .find(|node| matches!(hugr.get_optype(*node), OpType::Conditional(_)))
            .expect("conditional exists");
        let conditional: Conditional = hugr.get_optype(node).clone().try_into().unwrap();
        (node, conditional)
    }

    fn conditional_count(hugr: &Hugr) -> usize {
        hugr.entry_descendants()
            .filter(|node| matches!(hugr.get_optype(*node), OpType::Conditional(_)))
            .count()
    }

    fn two_way_sum_rows() -> Vec<TypeRow> {
        vec![type_row![], type_row![]]
    }

    fn build_simple_case_of_case_hugr() -> Hugr {
        let mut builder = DFGBuilder::new(Signature::new(vec![], vec![usize_t()])).unwrap();
        let pred = builder.add_load_value(Value::true_val());
        let mut producer = builder
            .conditional_builder(
                (two_way_sum_rows(), pred),
                [],
                vec![Type::new_sum(two_way_sum_rows())].into(),
            )
            .unwrap();

        for case_index in 0..2 {
            let mut case = producer.case_builder(case_index).unwrap();
            let tag = case
                .add_dataflow_op(Tag::new(case_index, two_way_sum_rows()), [])
                .unwrap()
                .out_wire(0);
            case.finish_with_outputs([tag]).unwrap();
        }

        let producer = producer.finish_sub_container().unwrap();
        let [selector] = producer.outputs_arr();

        let mut consumer = builder
            .conditional_builder((two_way_sum_rows(), selector), [], vec![usize_t()].into())
            .unwrap();
        let mut case0 = consumer.case_builder(0).unwrap();
        let out0 = case0.add_load_value(ConstUsize::new(1));
        case0.finish_with_outputs([out0]).unwrap();
        let mut case1 = consumer.case_builder(1).unwrap();
        let out1 = case1.add_load_value(ConstUsize::new(2));
        case1.finish_with_outputs([out1]).unwrap();
        let consumer = consumer.finish_sub_container().unwrap();

        builder
            .finish_hugr_with_outputs(consumer.outputs().collect::<Vec<_>>())
            .unwrap()
    }

    fn build_enum_case_of_case_hugr() -> Hugr {
        let rows = vec![
            [usize_t()].into(),
            type_row![],
            [usize_t(), usize_t()].into(),
        ];
        let sum_ty = Type::new_sum(rows.clone());
        let mut builder = DFGBuilder::new(Signature::new(
            vec![sum_ty.clone(), usize_t(), usize_t()],
            vec![usize_t()],
        ))
        .unwrap();
        let inputs = builder.input_wires().collect::<Vec<_>>();
        let pred = inputs[0];
        let x = inputs[1];
        let y = inputs[2];
        let mut producer = builder
            .conditional_builder(
                (rows.clone(), pred),
                [(usize_t(), x), (usize_t(), y)],
                vec![Type::new_sum(rows.clone())].into(),
            )
            .unwrap();

        let mut case0 = producer.case_builder(0).unwrap();
        let inputs = case0.input_wires().collect::<Vec<_>>();
        let tag = case0
            .add_dataflow_op(Tag::new(0, rows.clone()), [inputs[1]])
            .unwrap()
            .out_wire(0);
        case0.finish_with_outputs([tag]).unwrap();

        let mut case1 = producer.case_builder(1).unwrap();
        let tag = case1
            .add_dataflow_op(Tag::new(1, rows.clone()), [])
            .unwrap()
            .out_wire(0);
        case1.finish_with_outputs([tag]).unwrap();

        let mut case2 = producer.case_builder(2).unwrap();
        let inputs = case2.input_wires().collect::<Vec<_>>();
        let tag = case2
            .add_dataflow_op(Tag::new(2, rows.clone()), [inputs[2], inputs[3]])
            .unwrap()
            .out_wire(0);
        case2.finish_with_outputs([tag]).unwrap();

        let producer = producer.finish_sub_container().unwrap();
        let [selector] = producer.outputs_arr();

        let mut consumer = builder
            .conditional_builder((rows.clone(), selector), [], vec![usize_t()].into())
            .unwrap();
        for (case_index, value) in [10, 20, 30].into_iter().enumerate() {
            let mut case = consumer.case_builder(case_index).unwrap();
            let out = case.add_load_value(ConstUsize::new(value));
            case.finish_with_outputs([out]).unwrap();
        }
        let consumer = consumer.finish_sub_container().unwrap();

        builder
            .finish_hugr_with_outputs(consumer.outputs().collect::<Vec<_>>())
            .unwrap()
    }

    fn build_multi_output_case_of_case_hugr() -> Hugr {
        let mut builder = DFGBuilder::new(Signature::new(
            vec![usize_t(), usize_t()],
            vec![usize_t(), usize_t()],
        ))
        .unwrap();
        let [x, y] = builder.input_wires_arr();
        let pred = builder.add_load_value(Value::true_val());
        let mut producer = builder
            .conditional_builder(
                (two_way_sum_rows(), pred),
                [(usize_t(), x)],
                vec![Type::new_sum(two_way_sum_rows()), usize_t()].into(),
            )
            .unwrap();

        let mut case0 = producer.case_builder(0).unwrap();
        let inputs = case0.input_wires().collect::<Vec<_>>();
        let selector = case0
            .add_dataflow_op(Tag::new(0, two_way_sum_rows()), [])
            .unwrap()
            .out_wire(0);
        case0.finish_with_outputs([selector, inputs[0]]).unwrap();

        let mut case1 = producer.case_builder(1).unwrap();
        let inputs = case1.input_wires().collect::<Vec<_>>();
        let selector = case1
            .add_dataflow_op(Tag::new(1, two_way_sum_rows()), [])
            .unwrap()
            .out_wire(0);
        case1.finish_with_outputs([selector, inputs[0]]).unwrap();

        let producer = producer.finish_sub_container().unwrap();
        let [selector, shared] = producer.outputs_arr();

        let mut consumer = builder
            .conditional_builder(
                (two_way_sum_rows(), selector),
                [(usize_t(), shared), (usize_t(), y)],
                vec![usize_t()].into(),
            )
            .unwrap();
        for case_index in 0..2 {
            let case = consumer.case_builder(case_index).unwrap();
            let inputs = case.input_wires().collect::<Vec<_>>();
            case.finish_with_outputs([inputs[1 - case_index]]).unwrap();
        }
        let consumer = consumer.finish_sub_container().unwrap();
        let [result] = consumer.outputs_arr();

        builder.finish_hugr_with_outputs([shared, result]).unwrap()
    }

    fn build_case_chain_hugr() -> Hugr {
        let mut builder = DFGBuilder::new(Signature::new(vec![], vec![usize_t()])).unwrap();
        let pred = builder.add_load_value(Value::true_val());
        let mut a = builder
            .conditional_builder(
                (two_way_sum_rows(), pred),
                [],
                vec![Type::new_sum(two_way_sum_rows())].into(),
            )
            .unwrap();
        for case_index in 0..2 {
            let mut case = a.case_builder(case_index).unwrap();
            let tag = case
                .add_dataflow_op(Tag::new(case_index, two_way_sum_rows()), [])
                .unwrap()
                .out_wire(0);
            case.finish_with_outputs([tag]).unwrap();
        }
        let a = a.finish_sub_container().unwrap();
        let [a_selector] = a.outputs_arr();

        let mut b = builder
            .conditional_builder(
                (two_way_sum_rows(), a_selector),
                [],
                vec![Type::new_sum(two_way_sum_rows())].into(),
            )
            .unwrap();
        let mut case0 = b.case_builder(0).unwrap();
        let tag0 = case0
            .add_dataflow_op(Tag::new(1, two_way_sum_rows()), [])
            .unwrap()
            .out_wire(0);
        case0.finish_with_outputs([tag0]).unwrap();
        let mut case1 = b.case_builder(1).unwrap();
        let tag1 = case1
            .add_dataflow_op(Tag::new(0, two_way_sum_rows()), [])
            .unwrap()
            .out_wire(0);
        case1.finish_with_outputs([tag1]).unwrap();
        let b = b.finish_sub_container().unwrap();
        let [b_selector] = b.outputs_arr();

        let mut c = builder
            .conditional_builder((two_way_sum_rows(), b_selector), [], vec![usize_t()].into())
            .unwrap();
        let mut case0 = c.case_builder(0).unwrap();
        let out0 = case0.add_load_value(ConstUsize::new(7));
        case0.finish_with_outputs([out0]).unwrap();
        let mut case1 = c.case_builder(1).unwrap();
        let out1 = case1.add_load_value(ConstUsize::new(9));
        case1.finish_with_outputs([out1]).unwrap();
        let c = c.finish_sub_container().unwrap();

        builder
            .finish_hugr_with_outputs(c.outputs().collect::<Vec<_>>())
            .unwrap()
    }

    fn build_external_use_case_of_case_hugr() -> Hugr {
        let mut builder = DFGBuilder::new(Signature::new(
            vec![],
            vec![Type::new_sum(two_way_sum_rows()), usize_t()],
        ))
        .unwrap();
        let pred = builder.add_load_value(Value::true_val());
        let mut producer = builder
            .conditional_builder(
                (two_way_sum_rows(), pred),
                [],
                vec![Type::new_sum(two_way_sum_rows())].into(),
            )
            .unwrap();
        for case_index in 0..2 {
            let mut case = producer.case_builder(case_index).unwrap();
            let tag = case
                .add_dataflow_op(Tag::new(case_index, two_way_sum_rows()), [])
                .unwrap()
                .out_wire(0);
            case.finish_with_outputs([tag]).unwrap();
        }
        let producer = producer.finish_sub_container().unwrap();
        let [selector] = producer.outputs_arr();

        let mut consumer = builder
            .conditional_builder((two_way_sum_rows(), selector), [], vec![usize_t()].into())
            .unwrap();
        let mut case0 = consumer.case_builder(0).unwrap();
        let out0 = case0.add_load_value(ConstUsize::new(1));
        case0.finish_with_outputs([out0]).unwrap();
        let mut case1 = consumer.case_builder(1).unwrap();
        let out1 = case1.add_load_value(ConstUsize::new(2));
        case1.finish_with_outputs([out1]).unwrap();
        let consumer = consumer.finish_sub_container().unwrap();

        builder
            .finish_hugr_with_outputs([selector, consumer.out_wire(0)])
            .unwrap()
    }

    fn build_indirect_selector_case_of_case_hugr() -> Hugr {
        let sum_ty = Type::new_sum(two_way_sum_rows());
        let mut builder = DFGBuilder::new(Signature::new(vec![], vec![usize_t()])).unwrap();
        let pred = builder.add_load_value(Value::true_val());
        let mut producer = builder
            .conditional_builder((two_way_sum_rows(), pred), [], vec![sum_ty.clone()].into())
            .unwrap();
        for case_index in 0..2 {
            let mut case = producer.case_builder(case_index).unwrap();
            let tag = case
                .add_dataflow_op(Tag::new(case_index, two_way_sum_rows()), [])
                .unwrap()
                .out_wire(0);
            let passthrough = case
                .add_dataflow_op(Noop::new(sum_ty.clone()), [tag])
                .unwrap()
                .out_wire(0);
            case.finish_with_outputs([passthrough]).unwrap();
        }
        let producer = producer.finish_sub_container().unwrap();
        let [selector] = producer.outputs_arr();

        let mut consumer = builder
            .conditional_builder((two_way_sum_rows(), selector), [], vec![usize_t()].into())
            .unwrap();
        let mut case0 = consumer.case_builder(0).unwrap();
        let out0 = case0.add_load_value(ConstUsize::new(1));
        case0.finish_with_outputs([out0]).unwrap();
        let mut case1 = consumer.case_builder(1).unwrap();
        let out1 = case1.add_load_value(ConstUsize::new(2));
        case1.finish_with_outputs([out1]).unwrap();
        let consumer = consumer.finish_sub_container().unwrap();

        builder
            .finish_hugr_with_outputs(consumer.outputs().collect::<Vec<_>>())
            .unwrap()
    }

    fn build_unrebuildable_consumer_case_hugr() -> Hugr {
        let mut builder = DFGBuilder::new(Signature::new(vec![], vec![usize_t()])).unwrap();
        let pred = builder.add_load_value(Value::true_val());
        let mut producer = builder
            .conditional_builder(
                (two_way_sum_rows(), pred),
                [],
                vec![Type::new_sum(two_way_sum_rows())].into(),
            )
            .unwrap();
        for case_index in 0..2 {
            let mut case = producer.case_builder(case_index).unwrap();
            let tag = case
                .add_dataflow_op(Tag::new(case_index, two_way_sum_rows()), [])
                .unwrap()
                .out_wire(0);
            case.finish_with_outputs([tag]).unwrap();
        }
        let producer = producer.finish_sub_container().unwrap();
        let [selector] = producer.outputs_arr();

        let mut consumer = builder
            .conditional_builder((two_way_sum_rows(), selector), [], vec![usize_t()].into())
            .unwrap();
        let mut case0 = consumer.case_builder(0).unwrap();
        let nested_pred = case0.add_load_value(Value::true_val());
        let mut nested = case0
            .conditional_builder(
                (two_way_sum_rows(), nested_pred),
                [],
                vec![usize_t()].into(),
            )
            .unwrap();
        let mut nested_case0 = nested.case_builder(0).unwrap();
        let nested_out0 = nested_case0.add_load_value(ConstUsize::new(1));
        nested_case0.finish_with_outputs([nested_out0]).unwrap();
        let mut nested_case1 = nested.case_builder(1).unwrap();
        let nested_out1 = nested_case1.add_load_value(ConstUsize::new(2));
        nested_case1.finish_with_outputs([nested_out1]).unwrap();
        let nested = nested.finish_sub_container().unwrap();
        case0
            .finish_with_outputs(nested.outputs().collect::<Vec<_>>())
            .unwrap();

        let mut case1 = consumer.case_builder(1).unwrap();
        let out1 = case1.add_load_value(ConstUsize::new(3));
        case1.finish_with_outputs([out1]).unwrap();
        let consumer = consumer.finish_sub_container().unwrap();

        builder
            .finish_hugr_with_outputs(consumer.outputs().collect::<Vec<_>>())
            .unwrap()
    }

    fn build_over_budget_case_of_case_hugr() -> Hugr {
        let mut builder =
            DFGBuilder::new(Signature::new(vec![usize_t()], vec![usize_t()])).unwrap();
        let [x] = builder.input_wires_arr();
        let pred = builder.add_load_value(Value::true_val());
        let mut producer = builder
            .conditional_builder(
                (two_way_sum_rows(), pred),
                [],
                vec![Type::new_sum(two_way_sum_rows())].into(),
            )
            .unwrap();
        for case_index in 0..2 {
            let mut case = producer.case_builder(case_index).unwrap();
            let tag = case
                .add_dataflow_op(Tag::new(case_index, two_way_sum_rows()), [])
                .unwrap()
                .out_wire(0);
            case.finish_with_outputs([tag]).unwrap();
        }
        let producer = producer.finish_sub_container().unwrap();
        let [selector] = producer.outputs_arr();

        let mut consumer = builder
            .conditional_builder(
                (two_way_sum_rows(), selector),
                [(usize_t(), x)],
                vec![usize_t()].into(),
            )
            .unwrap();
        for case_index in 0..2 {
            let mut case = consumer.case_builder(case_index).unwrap();
            let inputs = case.input_wires().collect::<Vec<_>>();
            let n1 = case
                .add_dataflow_op(Noop::new(usize_t()), [inputs[0]])
                .unwrap()
                .out_wire(0);
            let n2 = case
                .add_dataflow_op(Noop::new(usize_t()), [n1])
                .unwrap()
                .out_wire(0);
            let n3 = case
                .add_dataflow_op(Noop::new(usize_t()), [n2])
                .unwrap()
                .out_wire(0);
            case.finish_with_outputs([n3]).unwrap();
        }
        let consumer = consumer.finish_sub_container().unwrap();

        builder
            .finish_hugr_with_outputs(consumer.outputs().collect::<Vec<_>>())
            .unwrap()
    }

    #[test]
    fn rewrites_simple_case_of_case() {
        let mut hugr = build_simple_case_of_case_hugr();
        let result = CaseOfCasePass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 1);
        hugr.validate().unwrap();
        assert_eq!(conditional_count(&hugr), 1);

        let (_, conditional) = first_conditional(&hugr);
        assert_eq!(conditional.other_inputs.len(), 0);
        assert_eq!(conditional.outputs.len(), 1);
    }

    #[test]
    fn rewrites_enum_case_of_case() {
        let mut hugr = build_enum_case_of_case_hugr();
        let result = CaseOfCasePass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 1);
        hugr.validate().unwrap();
        assert_eq!(conditional_count(&hugr), 1);
    }

    #[test]
    fn preserves_producer_extra_outputs() {
        let mut hugr = build_multi_output_case_of_case_hugr();
        let result = CaseOfCasePass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 1);
        hugr.validate().unwrap();
        assert_eq!(conditional_count(&hugr), 1);

        let (_, conditional) = first_conditional(&hugr);
        assert_eq!(conditional.other_inputs.len(), 2);
        assert_eq!(conditional.outputs.len(), 2);
    }

    #[test]
    fn rewrites_case_chain_before_recursing() {
        let mut hugr = build_case_chain_hugr();
        let result = CaseOfCasePass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 2);
        hugr.validate().unwrap();
        assert_eq!(conditional_count(&hugr), 1);
    }

    #[test]
    fn skips_when_fused_output_has_external_use() {
        let mut hugr = build_external_use_case_of_case_hugr();
        let result = CaseOfCasePass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 0);
        hugr.validate().unwrap();
        assert_eq!(conditional_count(&hugr), 2);
    }

    #[test]
    fn skips_when_selector_is_not_direct_tag() {
        let mut hugr = build_indirect_selector_case_of_case_hugr();
        let result = CaseOfCasePass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 0);
        hugr.validate().unwrap();
        assert_eq!(conditional_count(&hugr), 2);
    }

    #[test]
    fn skips_when_consumer_case_is_not_rebuildable() {
        let mut hugr = build_unrebuildable_consumer_case_hugr();
        let result = CaseOfCasePass::default().run(&mut hugr).unwrap();

        assert_eq!(result.rewrites_applied, 0);
        hugr.validate().unwrap();
        assert_eq!(conditional_count(&hugr), 3);
    }

    #[test]
    fn skips_when_duplication_cost_exceeds_budget() {
        let mut hugr = build_over_budget_case_of_case_hugr();
        let result = CaseOfCasePass::default()
            .with_max_duplicated_nodes(1)
            .run(&mut hugr)
            .unwrap();

        assert_eq!(result.rewrites_applied, 0);
        hugr.validate().unwrap();
        assert_eq!(conditional_count(&hugr), 2);
    }

    #[test]
    fn is_deterministic() {
        let mut a = build_case_chain_hugr();
        let mut b = build_case_chain_hugr();

        CaseOfCasePass::default().run(&mut a).unwrap();
        CaseOfCasePass::default().run(&mut b).unwrap();

        assert_eq!(a.mermaid_string(), b.mermaid_string());
    }
}
