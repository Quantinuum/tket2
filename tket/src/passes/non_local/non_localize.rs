//! Convert redundant local forwarding edges back into non-local edges.

use std::collections::{BTreeMap, BTreeSet};
use std::convert::Infallible;

use hugr_core::{
    HugrView, IncomingPort, OutgoingPort, PortIndex, Wire,
    builder::{ConditionalBuilder, Dataflow, DataflowSubContainer, HugrBuilder},
    core::HugrNode,
    hugr::hugrmut::HugrMut,
    ops::{OpType, Tag, TailLoop},
    types::{EdgeKind, Type, TypeRow},
};
use itertools::Itertools as _;
use petgraph::algo::dominators;

use crate::passes::composable::{InScope, WithScope};
use crate::passes::{ComposablePass, PassScope};

/// Replaces local forwarding through container inputs with equivalent non-local edges.
///
/// Only copyable values are considered. The pass handles ordinary DFG inputs,
/// Conditional `other_inputs`, invariant [`TailLoop::just_inputs`], and values
/// threaded through CFG basic-block branch payloads by [`super::LocalizeEdges`].
/// Container interfaces protected by [`PassScope`] are unchanged.
///
/// CFG rewrites restore Ext edges from outside the CFG or Dom edges from a
/// strictly dominating basic block. Every incoming path must resolve to the
/// same source. TailLoop and CFG control payloads are changed only when their
/// producer exactly matches the structural forwarding layer emitted by
/// [`super::LocalizeEdges`].
///
/// This is a conservative compression pass, not a global edge minimizer. It
/// leaves TailLoop `rest` values, pre-existing unused inputs, unresolved cyclic
/// or conflicting provenance, and unrecognized control layers unchanged.
#[derive(Clone, Debug, Default, Hash)]
pub struct NonLocalizeEdges {
    scope: PassScope,
}

impl WithScope for NonLocalizeEdges {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut> ComposablePass<H> for NonLocalizeEdges {
    type Error = Infallible;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let regions = self.scope.regions(hugr).collect_vec();
        for node in regions {
            if !hugr.contains_node(node) || self.scope.in_scope(hugr, node) != InScope::Yes {
                continue;
            }
            match hugr.get_optype(node) {
                OpType::DFG(_) => nonlocalize_dfg(hugr, node),
                OpType::Conditional(_) => nonlocalize_conditional(hugr, node),
                OpType::TailLoop(_) => nonlocalize_tail_loop(hugr, node),
                OpType::CFG(_) => nonlocalize_cfg(hugr, node),
                _ => {}
            }
        }
        Ok(())
    }
}

impl NonLocalizeEdges {
    /// Returns the configured pass scope.
    #[must_use]
    pub fn scope(&self) -> &PassScope {
        &self.scope
    }
}

type Sources<N> = Vec<Vec<(N, OutgoingPort)>>;
type Targets<N> = Vec<Vec<(N, IncomingPort)>>;

struct IncomingLinks<N> {
    values: Sources<N>,
    other: Vec<(N, OutgoingPort)>,
}

struct OutgoingLinks<N> {
    values: Targets<N>,
    other: Vec<(N, IncomingPort)>,
}

fn take_incoming_links<H: HugrMut>(hugr: &mut H, node: H::Node) -> IncomingLinks<H::Node> {
    let values = hugr
        .in_value_types(node)
        .map(|(port, _)| hugr.linked_outputs(node, port).collect())
        .collect();
    let other = hugr
        .get_optype(node)
        .other_input_port()
        .into_iter()
        .flat_map(|port| hugr.linked_outputs(node, port))
        .collect();
    let ports = hugr.node_inputs(node).collect_vec();
    for port in ports {
        hugr.disconnect(node, port);
    }
    IncomingLinks { values, other }
}

fn restore_incoming_links<H: HugrMut>(
    hugr: &mut H,
    node: H::Node,
    links: IncomingLinks<H::Node>,
    keep: &[usize],
) {
    let op = hugr.get_optype(node);
    let (incoming, outgoing) = (op.input_count(), op.output_count());
    hugr.set_num_ports(node, incoming, outgoing);
    for (new_index, &old_index) in keep.iter().enumerate() {
        for &(source, source_port) in &links.values[old_index] {
            hugr.connect(source, source_port, node, new_index);
        }
    }
    if let Some(port) = hugr.get_optype(node).other_input_port() {
        for (source, source_port) in links.other {
            hugr.connect(source, source_port, node, port);
        }
    } else {
        debug_assert!(links.other.is_empty());
    }
}

fn take_outgoing_links<H: HugrMut>(hugr: &mut H, node: H::Node) -> OutgoingLinks<H::Node> {
    let values = hugr
        .out_value_types(node)
        .map(|(port, _)| hugr.linked_inputs(node, port).collect())
        .collect();
    let other = hugr
        .get_optype(node)
        .other_output_port()
        .into_iter()
        .flat_map(|port| hugr.linked_inputs(node, port))
        .collect();
    let ports = hugr.node_outputs(node).collect_vec();
    for port in ports {
        hugr.disconnect(node, port);
    }
    OutgoingLinks { values, other }
}

fn restore_outgoing_links<H: HugrMut>(
    hugr: &mut H,
    node: H::Node,
    links: OutgoingLinks<H::Node>,
    keep: &[usize],
) {
    let op = hugr.get_optype(node);
    let (incoming, outgoing) = (op.input_count(), op.output_count());
    hugr.set_num_ports(node, incoming, outgoing);
    for (new_index, &old_index) in keep.iter().enumerate() {
        for &(target, target_port) in &links.values[old_index] {
            hugr.connect(node, new_index, target, target_port);
        }
    }
    if let Some(port) = hugr.get_optype(node).other_output_port() {
        for (target, target_port) in links.other {
            hugr.connect(node, port, target, target_port);
        }
    } else {
        debug_assert!(links.other.is_empty());
    }
}

fn retained(row: &TypeRow, keep: &[usize]) -> TypeRow {
    keep.iter().map(|&index| row[index].clone()).collect()
}

fn wire_type<H: HugrView>(hugr: &H, wire: Wire<H::Node>) -> Option<Type> {
    match hugr.get_optype(wire.node()).port_kind(wire.source())? {
        EdgeKind::Value(ty) => Some(ty),
        _ => None,
    }
}

fn copyable_source<H: HugrView>(
    hugr: &H,
    node: H::Node,
    port: IncomingPort,
) -> Option<Wire<H::Node>> {
    let (source, source_port) = hugr.single_linked_output(node, port)?;
    let wire = Wire::new(source, source_port);
    wire_type(hugr, wire)?.copyable().then_some(wire)
}

#[derive(Clone)]
struct Rewrite<N: HugrNode> {
    source: Wire<N>,
    consumers: Vec<(N, IncomingPort)>,
}

fn plan_rewrite<H: HugrView>(
    hugr: &H,
    input: H::Node,
    index: usize,
    source: Wire<H::Node>,
    skip: Option<H::Node>,
) -> Rewrite<H::Node> {
    let consumers = hugr
        .linked_inputs(input, index)
        .filter(|(node, _)| Some(*node) != skip)
        .collect_vec();
    Rewrite { source, consumers }
}

fn apply_rewrite<H: HugrMut>(hugr: &mut H, rewrite: &Rewrite<H::Node>) {
    for &(target, target_port) in &rewrite.consumers {
        hugr.disconnect(target, target_port);
        hugr.connect(
            rewrite.source.node(),
            rewrite.source.source(),
            target,
            target_port,
        );
    }
}

fn compact_input_node<H: HugrMut>(hugr: &mut H, input: H::Node, keep: &[usize]) {
    let links = take_outgoing_links(hugr, input);
    let OpType::Input(op) = hugr.optype_mut(input) else {
        unreachable!("dataflow parent must have an Input child")
    };
    op.types = retained(&op.types, keep);
    restore_outgoing_links(hugr, input, links, keep);
}

fn nonlocalize_dfg<H: HugrMut>(hugr: &mut H, node: H::Node) {
    let [input, _] = hugr.get_io(node).expect("valid DFG");
    let input_count = hugr
        .get_optype(node)
        .as_dfg()
        .expect("checked by caller")
        .signature
        .input_count();
    let rewrites = (0..input_count)
        .filter_map(|index| {
            let source = copyable_source(hugr, node, index.into())?;
            let rewrite = plan_rewrite(hugr, input, index, source, None);
            (!rewrite.consumers.is_empty()).then_some((index, rewrite))
        })
        .collect_vec();
    if rewrites.is_empty() {
        return;
    }
    let remove: BTreeSet<_> = rewrites.iter().map(|(index, _)| *index).collect();
    for (_, rewrite) in &rewrites {
        apply_rewrite(hugr, rewrite);
    }
    let keep = (0..input_count)
        .filter(|index| !remove.contains(index))
        .collect_vec();
    let links = take_incoming_links(hugr, node);
    let OpType::DFG(op) = hugr.optype_mut(node) else {
        unreachable!()
    };
    op.signature.input = retained(&op.signature.input, &keep);
    restore_incoming_links(hugr, node, links, &keep);
    compact_input_node(hugr, input, &keep);
}

fn nonlocalize_conditional<H: HugrMut>(hugr: &mut H, node: H::Node) {
    let (sum_rows, other_count) = {
        let op = hugr
            .get_optype(node)
            .as_conditional()
            .expect("checked by caller");
        (op.sum_rows.clone(), op.other_inputs.len())
    };
    let cases = hugr.children(node).collect_vec();
    let rewrites = (0..other_count)
        .filter_map(|other_index| {
            let source = copyable_source(hugr, node, (other_index + 1).into())?;
            let consumers = cases
                .iter()
                .enumerate()
                .flat_map(|(case_index, &case)| {
                    let [input, _] = hugr.get_io(case).expect("valid Case");
                    hugr.linked_inputs(input, sum_rows[case_index].len() + other_index)
                })
                .collect_vec();
            (!consumers.is_empty()).then_some((other_index, Rewrite { source, consumers }))
        })
        .collect_vec();
    if rewrites.is_empty() {
        return;
    }
    let remove: BTreeSet<_> = rewrites
        .iter()
        .map(|(other_index, _)| *other_index)
        .collect();
    for (_, rewrite) in &rewrites {
        apply_rewrite(hugr, rewrite);
    }

    let keep_other = (0..other_count)
        .filter(|index| !remove.contains(index))
        .collect_vec();
    let keep_node = std::iter::once(0)
        .chain(keep_other.iter().map(|index| index + 1))
        .collect_vec();
    let links = take_incoming_links(hugr, node);
    let OpType::Conditional(op) = hugr.optype_mut(node) else {
        unreachable!()
    };
    op.other_inputs = retained(&op.other_inputs, &keep_other);
    restore_incoming_links(hugr, node, links, &keep_node);

    for (case_index, case) in cases.into_iter().enumerate() {
        let [input, _] = hugr.get_io(case).expect("valid Case");
        let prefix = sum_rows[case_index].len();
        let keep = (0..prefix)
            .chain(keep_other.iter().map(|index| prefix + index))
            .collect_vec();
        let OpType::Case(op) = hugr.optype_mut(case) else {
            unreachable!()
        };
        op.signature.input = retained(&op.signature.input, &keep);
        compact_input_node(hugr, input, &keep);
    }
}

#[derive(Clone)]
struct ControlLayer<N: HugrNode> {
    wrapper: N,
    output: N,
    old_source: Wire<N>,
    old_rows: Vec<TypeRow>,
    added_sources: Vec<Vec<Wire<N>>>,
}

fn control_layer<H: HugrView>(
    hugr: &H,
    parent: H::Node,
    current_rows: &[TypeRow],
) -> Option<ControlLayer<H::Node>> {
    let [_, output] = hugr.get_io(parent)?;
    let (wrapper, wrapper_port) = hugr.single_linked_output(output, 0)?;
    if wrapper_port.index() != 0
        || hugr.get_parent(wrapper) != Some(parent)
        || hugr.linked_inputs(wrapper, 0).collect_vec().as_slice()
            != [(output, IncomingPort::from(0))]
    {
        return None;
    }
    let wrapper_op = hugr.get_optype(wrapper);
    let conditional = wrapper_op.as_conditional()?;
    if conditional.sum_rows.len() != current_rows.len()
        || conditional.outputs.as_ref() != [Type::new_sum(current_rows.to_vec())]
    {
        return None;
    }
    if wrapper_op
        .other_output_port()
        .is_some_and(|port| hugr.linked_inputs(wrapper, port).next().is_some())
        || wrapper_op
            .other_input_port()
            .is_some_and(|port| hugr.linked_outputs(wrapper, port).next().is_some())
    {
        return None;
    }

    let old_rows = conditional.sum_rows.clone();
    let cases = hugr.children(wrapper).collect_vec();
    if cases.len() != current_rows.len()
        || cases.iter().any(|case| !hugr.get_optype(*case).is_case())
    {
        return None;
    }
    let (old_source_node, old_source_port) = hugr.single_linked_output(wrapper, 0)?;
    if hugr.get_parent(old_source_node) != Some(parent) {
        return None;
    }
    let old_source = Wire::new(old_source_node, old_source_port);
    let mut added_sources = Vec::with_capacity(current_rows.len());
    for (variant, &case) in cases.iter().enumerate() {
        let old_len = old_rows[variant].len();
        let added = current_rows[variant].len().checked_sub(old_len)?;
        if current_rows[variant][added..] != old_rows[variant][..] {
            return None;
        }
        let case_op = hugr.get_optype(case).as_case()?;
        let expected_case_inputs = old_rows[variant]
            .iter()
            .chain(conditional.other_inputs.iter())
            .cloned()
            .collect::<TypeRow>();
        if case_op.signature.input != expected_case_inputs
            || case_op.signature.output.as_ref() != [Type::new_sum(current_rows.to_vec())]
        {
            return None;
        }
        let [case_input, case_output] = hugr.get_io(case)?;
        let (tag, tag_port) = hugr.single_linked_output(case_output, 0)?;
        if tag_port.index() != 0
            || hugr.get_parent(tag) != Some(case)
            || hugr
                .children(case)
                .filter(|child| {
                    !hugr.get_optype(*child).is_input() && !hugr.get_optype(*child).is_output()
                })
                .collect_vec()
                .as_slice()
                != [tag]
            || hugr.linked_inputs(tag, 0).collect_vec().as_slice()
                != [(case_output, IncomingPort::from(0))]
        {
            return None;
        }
        let OpType::Tag(Tag {
            tag: tag_variant,
            variants,
            ..
        }) = hugr.get_optype(tag)
        else {
            return None;
        };
        if *tag_variant != variant || variants != current_rows {
            return None;
        }
        let mut sources = Vec::with_capacity(added);
        for field in 0..current_rows[variant].len() {
            let (field_source, field_port) = hugr.single_linked_output(tag, field)?;
            if field_source != case_input {
                return None;
            }
            if field < added {
                if field_port.index() < old_len {
                    return None;
                }
                let other_index = field_port.index() - old_len;
                let (source, source_port) = hugr.single_linked_output(wrapper, other_index + 1)?;
                sources.push(Wire::new(source, source_port));
            } else if field_port.index() != field - added {
                return None;
            }
        }
        added_sources.push(sources);
    }
    Some(ControlLayer {
        wrapper,
        output,
        old_source,
        old_rows,
        added_sources,
    })
}

fn bypass_control_layer<H: HugrMut>(hugr: &mut H, layer: &ControlLayer<H::Node>) {
    hugr.disconnect(layer.output, IncomingPort::from(0));
    hugr.connect(
        layer.old_source.node(),
        layer.old_source.source(),
        layer.output,
        0,
    );
    let OpType::Output(output) = hugr.optype_mut(layer.output) else {
        unreachable!()
    };
    output.types.to_mut()[0] = Type::new_sum(layer.old_rows.clone());
    hugr.remove_subtree(layer.wrapper);
}

fn project_control<H: HugrMut>(
    hugr: &mut H,
    output: H::Node,
    remove: &[BTreeSet<usize>],
) -> Vec<TypeRow> {
    let Some(EdgeKind::Value(old_type)) = hugr.get_optype(output).port_kind(IncomingPort::from(0))
    else {
        unreachable!()
    };
    let old_rows: Vec<TypeRow> = old_type
        .as_sum()
        .expect("control output must be a Sum")
        .variants()
        .map(|row| row.clone().try_into().expect("concrete control row"))
        .collect_vec();
    let new_rows = old_rows
        .iter()
        .zip_eq(remove)
        .map(|(row, removed)| {
            row.iter()
                .enumerate()
                .filter(|(index, _)| !removed.contains(index))
                .map(|(_, ty)| ty.clone())
                .collect()
        })
        .collect_vec();
    if old_rows == new_rows {
        return new_rows;
    }

    let new_type = Type::new_sum(new_rows.clone());
    let mut builder = ConditionalBuilder::new(old_rows.clone(), [], [new_type.clone()])
        .expect("valid control projection");
    for (variant, (row, removed)) in old_rows.iter().zip_eq(remove).enumerate() {
        let mut case = builder.case_builder(variant).expect("valid variant");
        let args = case
            .input_wires()
            .enumerate()
            .filter(|(index, _)| !removed.contains(index))
            .map(|(_, wire)| wire)
            .collect_vec();
        let tagged = case
            .add_dataflow_op(Tag::new(variant, new_rows.clone()), args)
            .expect("valid projected Tag")
            .outputs();
        case.finish_with_outputs(tagged)
            .expect("valid projection case");
        debug_assert_eq!(row.len() - removed.len(), new_rows[variant].len());
    }
    let projection = builder.finish_hugr().expect("valid control projection");
    let parent = hugr.get_parent(output).expect("Output has a parent");
    let projection = hugr.insert_hugr(parent, projection).inserted_entrypoint;
    let (source, source_port) = hugr
        .single_linked_output(output, 0)
        .expect("control output is connected");
    hugr.connect(source, source_port, projection, 0);
    hugr.disconnect(output, IncomingPort::from(0));
    hugr.connect(projection, 0, output, 0);
    let OpType::Output(output_op) = hugr.optype_mut(output) else {
        unreachable!()
    };
    output_op.types.to_mut()[0] = new_type;
    new_rows
}

fn nonlocalize_tail_loop<H: HugrMut>(hugr: &mut H, node: H::Node) {
    let (just_inputs, just_outputs, rest) = {
        let op = hugr
            .get_optype(node)
            .as_tail_loop()
            .expect("checked by caller");
        (
            op.just_inputs.clone(),
            op.just_outputs.clone(),
            op.rest.clone(),
        )
    };
    let current_rows = vec![just_inputs.clone(), just_outputs.clone()];
    let Some(layer) = control_layer(hugr, node, &current_rows) else {
        return;
    };
    if layer.old_rows[TailLoop::BREAK_TAG] != just_outputs {
        return;
    }
    let remove_count = layer.added_sources[TailLoop::CONTINUE_TAG].len();
    if remove_count == 0
        || remove_count + layer.old_rows[TailLoop::CONTINUE_TAG].len() != just_inputs.len()
        || !layer.added_sources[TailLoop::BREAK_TAG].is_empty()
    {
        return;
    }
    let [input, _] = hugr.get_io(node).expect("valid TailLoop");
    let mut rewrites = Vec::with_capacity(remove_count);
    for index in 0..remove_count {
        if layer.added_sources[TailLoop::CONTINUE_TAG][index] != Wire::new(input, index) {
            return;
        }
        let Some(source) = copyable_source(hugr, node, index.into()) else {
            return;
        };
        let rewrite = plan_rewrite(hugr, input, index, source, Some(layer.wrapper));
        if rewrite.consumers.is_empty() {
            return;
        }
        rewrites.push(rewrite);
    }
    for rewrite in &rewrites {
        apply_rewrite(hugr, rewrite);
    }
    bypass_control_layer(hugr, &layer);

    let old_input_count = just_inputs.len() + rest.len();
    let keep = (remove_count..old_input_count).collect_vec();
    let links = take_incoming_links(hugr, node);
    let OpType::TailLoop(op) = hugr.optype_mut(node) else {
        unreachable!()
    };
    op.just_inputs = layer.old_rows[TailLoop::CONTINUE_TAG].clone();
    restore_incoming_links(hugr, node, links, &keep);
    compact_input_node(hugr, input, &keep);
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Provenance<N: HugrNode> {
    Unknown,
    Source(Wire<N>),
    Conflict,
}

impl<N: HugrNode> Provenance<N> {
    fn merge(self, other: Self) -> Self {
        match (self, other) {
            (Self::Conflict, _) | (_, Self::Conflict) => Self::Conflict,
            (Self::Unknown, value) | (value, Self::Unknown) => value,
            (Self::Source(left), Self::Source(right)) if left == right => Self::Source(left),
            (Self::Source(_), Self::Source(_)) => Self::Conflict,
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Contributor<N: HugrNode> {
    Source(Wire<N>),
    Dependency((N, usize)),
    Conflict,
}

fn cfg_child_containing<H: HugrView>(hugr: &H, cfg: H::Node, node: H::Node) -> Option<H::Node> {
    let mut child = node;
    while let Some(parent) = hugr.get_parent(child) {
        if parent == cfg {
            return Some(child);
        }
        child = parent;
    }
    None
}

fn nonlocalize_cfg<H: HugrMut>(hugr: &mut H, cfg: H::Node) {
    let blocks = hugr
        .children(cfg)
        .filter(|node| hugr.get_optype(*node).is_dataflow_block())
        .collect_vec();
    let Some(&entry) = blocks.first() else {
        return;
    };
    let inputs = blocks
        .iter()
        .map(|&block| (block, hugr.get_io(block).expect("valid DFB")[0]))
        .collect::<BTreeMap<_, _>>();
    let layers = blocks
        .iter()
        .filter_map(|&block| {
            let rows = &hugr
                .get_optype(block)
                .as_dataflow_block()
                .expect("DFB")
                .sum_rows;
            control_layer(hugr, block, rows).map(|layer| (block, layer))
        })
        .collect::<BTreeMap<_, _>>();

    let mut incoming = BTreeMap::<H::Node, Vec<(H::Node, usize)>>::new();
    let mut successors = BTreeMap::<H::Node, Vec<H::Node>>::new();
    for &block in &blocks {
        let succs = hugr.output_neighbours(block).collect_vec();
        for (variant, &succ) in succs.iter().enumerate() {
            if inputs.contains_key(&succ) {
                incoming.entry(succ).or_default().push((block, variant));
            }
        }
        successors.insert(block, succs);
    }

    let cfg_sources = hugr
        .in_value_types(cfg)
        .map(|(port, _)| copyable_source(hugr, cfg, port))
        .collect_vec();
    let candidate_counts = blocks
        .iter()
        .map(|&block| {
            let preds = incoming.get(&block).map(Vec::as_slice).unwrap_or_default();
            let forwarded = preds
                .iter()
                .map(|(pred, variant)| {
                    layers
                        .get(pred)
                        .map(|layer| layer.added_sources[*variant].len())
                        .unwrap_or(0)
                })
                .min();
            let count = if block == entry {
                forwarded.map_or(cfg_sources.len(), |count| count.min(cfg_sources.len()))
            } else {
                forwarded.unwrap_or(0)
            };
            (block, count)
        })
        .collect::<BTreeMap<_, _>>();

    let mut equations = BTreeMap::<(H::Node, usize), Vec<Contributor<H::Node>>>::new();
    for (&block, &count) in &candidate_counts {
        for index in 0..count {
            let mut contributors = Vec::new();
            if block == entry {
                contributors.push(
                    cfg_sources
                        .get(index)
                        .copied()
                        .flatten()
                        .map(Contributor::Source)
                        .unwrap_or(Contributor::Conflict),
                );
            }
            for &(pred, variant) in incoming.get(&block).into_iter().flatten() {
                let Some(source) = layers
                    .get(&pred)
                    .and_then(|layer| layer.added_sources[variant].get(index))
                    .copied()
                else {
                    contributors.push(Contributor::Conflict);
                    continue;
                };
                let dependency = inputs.iter().find_map(|(&source_block, &input)| {
                    (input == source.node()).then_some((source_block, source.source().index()))
                });
                contributors
                    .push(dependency.map_or(Contributor::Source(source), Contributor::Dependency));
            }
            if contributors.is_empty() {
                contributors.push(Contributor::Conflict);
            }
            equations.insert((block, index), contributors);
        }
    }

    // Provenance moves monotonically from unknown to one source or conflict.
    // Conflicts are permanent, so cyclic forwarding converges without choosing
    // an arbitrary predecessor source.
    let mut provenance = equations
        .keys()
        .map(|&key| (key, Provenance::Unknown))
        .collect::<BTreeMap<_, _>>();
    loop {
        let mut changed = false;
        for (&key, contributors) in &equations {
            let mut next = provenance[&key];
            for contributor in contributors {
                let value = match contributor {
                    Contributor::Source(source) => Provenance::Source(*source),
                    Contributor::Dependency(dependency) => provenance
                        .get(dependency)
                        .copied()
                        .unwrap_or(Provenance::Conflict),
                    Contributor::Conflict => Provenance::Conflict,
                };
                next = next.merge(value);
            }
            if next != provenance[&key] {
                provenance.insert(key, next);
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }

    let (remove, rewrites) = {
        let scheduling = hugr.scheduling_graph(cfg);
        let dominator_tree =
            dominators::simple_fast(scheduling.petgraph(), scheduling.node_to_pg(entry));
        let mut remove = BTreeMap::<H::Node, BTreeSet<usize>>::new();
        let mut rewrites = BTreeMap::new();
        for (&key @ (block, index), contributors) in &equations {
            let Provenance::Source(source) = provenance[&key] else {
                continue;
            };
            let complete = contributors.iter().all(|contributor| match contributor {
                Contributor::Source(other) => *other == source,
                Contributor::Dependency(dependency) => {
                    provenance.get(dependency) == Some(&Provenance::Source(source))
                }
                Contributor::Conflict => false,
            });
            if !complete || !wire_type(hugr, source).is_some_and(|ty| ty.copyable()) {
                continue;
            }
            let source_is_legal = match cfg_child_containing(hugr, cfg, source.node()) {
                None => true,
                Some(source_block) if source_block == block => false,
                Some(source_block) if inputs.contains_key(&source_block) => dominator_tree
                    .dominators(scheduling.node_to_pg(block))
                    .is_some_and(|dominators| {
                        dominators
                            .into_iter()
                            .any(|dominator| dominator == scheduling.node_to_pg(source_block))
                    }),
                Some(_) => false,
            };
            if source_is_legal {
                let rewrite = plan_rewrite(hugr, inputs[&block], index, source, None);
                if !rewrite.consumers.is_empty() {
                    remove.entry(block).or_default().insert(index);
                    rewrites.insert(key, rewrite);
                }
            }
        }
        (remove, rewrites)
    };
    if rewrites.is_empty() {
        return;
    }
    for rewrite in rewrites.values() {
        apply_rewrite(hugr, rewrite);
    }

    for &block in &blocks {
        let Some(layer) = layers.get(&block) else {
            continue;
        };
        let succs = &successors[&block];
        let removed_by_variant = succs
            .iter()
            .enumerate()
            .map(|(variant, successor)| {
                remove
                    .get(successor)
                    .into_iter()
                    .flat_map(BTreeSet::iter)
                    .copied()
                    .filter(|index| *index < layer.added_sources[variant].len())
                    .collect::<BTreeSet<_>>()
            })
            .collect_vec();
        if removed_by_variant.iter().all(BTreeSet::is_empty) {
            continue;
        }
        let removes_whole_layer = removed_by_variant
            .iter()
            .zip_eq(&layer.added_sources)
            .all(|(removed, added)| removed.iter().copied().eq(0..added.len()));
        let new_rows = if removes_whole_layer {
            bypass_control_layer(hugr, layer);
            layer.old_rows.clone()
        } else {
            project_control(hugr, layer.output, &removed_by_variant)
        };
        let OpType::DataflowBlock(op) = hugr.optype_mut(block) else {
            unreachable!()
        };
        op.sum_rows = new_rows;
    }

    for &block in &blocks {
        let Some(removed) = remove.get(&block) else {
            continue;
        };
        let input_count = hugr
            .get_optype(block)
            .as_dataflow_block()
            .expect("DFB")
            .inputs
            .len();
        let keep = (0..input_count)
            .filter(|index| !removed.contains(index))
            .collect_vec();
        let OpType::DataflowBlock(op) = hugr.optype_mut(block) else {
            unreachable!()
        };
        op.inputs = retained(&op.inputs, &keep);
        compact_input_node(hugr, inputs[&block], &keep);
    }

    let Some(entry_removed) = remove.get(&entry) else {
        return;
    };
    let cfg_input_count = hugr
        .get_optype(cfg)
        .as_cfg()
        .expect("checked by caller")
        .signature
        .input_count();
    let keep = (0..cfg_input_count)
        .filter(|index| !entry_removed.contains(index))
        .collect_vec();
    let links = take_incoming_links(hugr, cfg);
    let OpType::CFG(op) = hugr.optype_mut(cfg) else {
        unreachable!()
    };
    op.signature.input = retained(&op.signature.input, &keep);
    restore_incoming_links(hugr, cfg, links, &keep);
}

#[cfg(test)]
mod tests {
    use hugr_core::{
        HugrView, IncomingPort, OutgoingPort,
        builder::{DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, SubContainer},
        extension::prelude::{Noop, bool_t, qb_t},
        hugr::hugrmut::HugrMut,
        ops::{OpType, Tag, TailLoop, Value, handle::NodeHandle},
        type_row,
        types::{Signature, Type},
    };

    use crate::passes::composable::WithScope;
    use crate::passes::non_local::{LocalizeEdges, remove_nonlocal_edges};
    use crate::passes::{ComposablePass, PassScope};

    use super::NonLocalizeEdges;

    #[test]
    fn nonlocalize_dfg_removes_forwarding_input() {
        let (mut hugr, source, target) = {
            let mut outer = DFGBuilder::new(Signature::new_endo([bool_t()])).unwrap();
            let [source] = outer.input_wires_arr();
            let (result, target) = {
                let mut inner = outer.dfg_builder_endo([(bool_t(), source)]).unwrap();
                let [local] = inner.input_wires_arr();
                let target = inner.add_dataflow_op(Noop::new(bool_t()), [local]).unwrap();
                let target_node = target.node();
                let [result] = inner
                    .finish_with_outputs(target.outputs())
                    .unwrap()
                    .outputs_arr();
                (result, target_node)
            };
            let hugr = outer.finish_hugr_with_outputs([result]).unwrap();
            (hugr, source, target)
        };
        let nested = hugr
            .entry_descendants()
            .find(|node| *node != hugr.entrypoint() && hugr.get_optype(*node).is_dfg())
            .unwrap();

        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();

        assert_eq!(
            hugr.single_linked_output(target, 0),
            Some((source.node(), source.source()))
        );
        assert!(
            hugr.get_optype(nested)
                .as_dfg()
                .unwrap()
                .signature
                .input
                .is_empty()
        );
        assert!(
            LocalizeEdges::default()
                .with_scope(PassScope::EntrypointRecursive)
                .check_no_nonlocal_edges(&hugr)
                .is_err()
        );

        let once = hugr.clone();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        assert_eq!(hugr, once);
    }

    #[test]
    fn nonlocalize_preserves_scoped_interface() {
        let mut hugr = {
            let mut outer = DFGBuilder::new(Signature::new_endo([bool_t()])).unwrap();
            let [source] = outer.input_wires_arr();
            let [result] = {
                let mut inner = outer.dfg_builder_endo([(bool_t(), source)]).unwrap();
                let [local] = inner.input_wires_arr();
                let target = inner.add_dataflow_op(Noop::new(bool_t()), [local]).unwrap();
                inner
                    .finish_with_outputs(target.outputs())
                    .unwrap()
                    .outputs_arr()
            };
            outer.finish_hugr_with_outputs([result]).unwrap()
        };
        let nested = hugr
            .entry_descendants()
            .find(|node| *node != hugr.entrypoint() && hugr.get_optype(*node).is_dfg())
            .unwrap();
        hugr.set_entrypoint(nested);
        let before = hugr.clone();
        let pass = NonLocalizeEdges::default().with_scope(PassScope::EntrypointRecursive);
        assert_eq!(pass.scope(), &PassScope::EntrypointRecursive);
        pass.run(&mut hugr).unwrap();
        assert_eq!(hugr, before);
    }

    #[test]
    fn nonlocalize_compacts_middle_input_and_rewires_all_consumers() {
        let (mut hugr, source, targets) = {
            let mut outer =
                DFGBuilder::new(Signature::new_endo([qb_t(), bool_t(), qb_t()])).unwrap();
            let [q0, source, q1] = outer.input_wires_arr();
            let (outputs, targets): ([hugr_core::Wire; 3], _) = {
                let mut inner = outer
                    .dfg_builder_endo([(qb_t(), q0), (bool_t(), source), (qb_t(), q1)])
                    .unwrap();
                let [q0, local, q1] = inner.input_wires_arr();
                let first = inner.add_dataflow_op(Noop::new(bool_t()), [local]).unwrap();
                let second = inner.add_dataflow_op(Noop::new(bool_t()), [local]).unwrap();
                let targets = [first.node(), second.node()];
                (
                    inner
                        .finish_with_outputs([q0, first.out_wire(0), q1])
                        .unwrap()
                        .outputs_arr(),
                    targets,
                )
            };
            (
                outer.finish_hugr_with_outputs(outputs).unwrap(),
                source,
                targets,
            )
        };
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();

        for target in targets {
            assert_eq!(
                hugr.single_linked_output(target, 0),
                Some((source.node(), source.source()))
            );
        }
        let nested = hugr
            .entry_descendants()
            .find(|node| *node != hugr.entrypoint() && hugr.get_optype(*node).is_dfg())
            .unwrap();
        assert_eq!(
            hugr.get_optype(nested)
                .as_dfg()
                .unwrap()
                .signature
                .input
                .as_ref(),
            [qb_t(), qb_t()]
        );
    }

    #[test]
    fn nonlocalize_preserves_preexisting_unused_input() {
        let mut hugr = {
            let mut outer = DFGBuilder::new(Signature::new_endo([bool_t()])).unwrap();
            let [source] = outer.input_wires_arr();
            let [result] = {
                let mut inner = outer
                    .dfg_builder(Signature::new([bool_t()], [bool_t()]), [source])
                    .unwrap();
                let result = inner.add_load_value(Value::true_val());
                inner.finish_with_outputs([result]).unwrap().outputs_arr()
            };
            outer.finish_hugr_with_outputs([result]).unwrap()
        };
        let before = hugr.clone();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        assert_eq!(hugr, before);
    }

    #[test]
    fn nonlocalize_preserves_linear_input() {
        let mut hugr = {
            let mut outer = DFGBuilder::new(Signature::new_endo([qb_t()])).unwrap();
            let [source] = outer.input_wires_arr();
            let [result] = {
                let inner = outer.dfg_builder_endo([(qb_t(), source)]).unwrap();
                let [local] = inner.input_wires_arr();
                inner.finish_with_outputs([local]).unwrap().outputs_arr()
            };
            outer.finish_hugr_with_outputs([result]).unwrap()
        };
        let before = hugr.clone();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        assert_eq!(hugr, before);
    }

    #[test]
    fn nonlocalize_conditional_removes_other_input() {
        let (mut hugr, source, targets) = {
            let mut outer = DFGBuilder::new(Signature::new(
                [Type::new_unit_sum(2), bool_t()],
                [bool_t()],
            ))
            .unwrap();
            let [selector, source] = outer.input_wires_arr();
            let (result, targets) = {
                let mut conditional = outer
                    .conditional_builder((vec![type_row![]; 2], selector), [], [bool_t()].into())
                    .unwrap();
                let mut targets = Vec::new();
                for variant in 0..2 {
                    let mut case = conditional.case_builder(variant).unwrap();
                    let target = case.add_dataflow_op(Noop::new(bool_t()), [source]).unwrap();
                    targets.push(target.node());
                    case.finish_with_outputs(target.outputs()).unwrap();
                }
                (
                    conditional.finish_sub_container().unwrap().out_wire(0),
                    targets,
                )
            };
            (
                outer.finish_hugr_with_outputs([result]).unwrap(),
                source,
                targets,
            )
        };
        remove_nonlocal_edges(&mut hugr).unwrap();
        hugr.validate().unwrap();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();

        for target in targets {
            assert_eq!(
                hugr.single_linked_output(target, 0),
                Some((source.node(), source.source()))
            );
        }
        let conditional = hugr
            .entry_descendants()
            .find(|node| hugr.get_optype(*node).is_conditional())
            .unwrap();
        assert!(
            hugr.get_optype(conditional)
                .as_conditional()
                .unwrap()
                .other_inputs
                .is_empty()
        );
    }

    #[test]
    fn nonlocalize_tail_loop_updates_control_signature() {
        let (mut hugr, source, target) = {
            let (just_ty, source_ty, rest_ty) = (Type::UNIT, bool_t(), Type::new_unit_sum(3));
            let mut outer = DFGBuilder::new(Signature::new_endo([
                just_ty.clone(),
                source_ty.clone(),
                rest_ty.clone(),
            ]))
            .unwrap();
            let [just, source, rest] = outer.input_wires_arr();
            let (loop_outputs, target): ([hugr_core::Wire; 2], _) = {
                let mut loop_builder = outer
                    .tail_loop_builder(
                        [(just_ty.clone(), just)],
                        [(rest_ty.clone(), rest)],
                        vec![source_ty.clone()].into(),
                    )
                    .unwrap();
                let [_just, rest] = loop_builder.input_wires_arr();
                let target = loop_builder
                    .add_dataflow_op(
                        Tag::new(
                            TailLoop::BREAK_TAG,
                            vec![vec![just_ty.clone()].into(), vec![source_ty].into()],
                        ),
                        [source],
                    )
                    .unwrap();
                let target_node = target.node();
                (
                    loop_builder
                        .finish_with_outputs(target.out_wire(0), [rest])
                        .unwrap()
                        .outputs_arr(),
                    target_node,
                )
            };
            (
                outer
                    .finish_hugr_with_outputs([just, loop_outputs[0], loop_outputs[1]])
                    .unwrap(),
                source,
                target,
            )
        };
        remove_nonlocal_edges(&mut hugr).unwrap();
        hugr.validate().unwrap();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();

        assert_eq!(
            hugr.single_linked_output(target, 0),
            Some((source.node(), source.source()))
        );
        let tail_loop = hugr
            .entry_descendants()
            .find(|node| hugr.get_optype(*node).is_tail_loop())
            .unwrap();
        let OpType::TailLoop(op) = hugr.get_optype(tail_loop) else {
            unreachable!()
        };
        assert_eq!(op.just_inputs.as_ref(), [Type::UNIT]);
        assert_eq!(op.rest.as_ref(), [Type::new_unit_sum(3)]);

        let once = hugr.clone();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        assert_eq!(hugr, once);
    }

    #[test]
    fn nonlocalize_rejects_inexact_control_layers() {
        let mut hugr = {
            let (just_ty, source_ty, rest_ty) = (bool_t(), bool_t(), Type::new_unit_sum(3));
            let mut outer = DFGBuilder::new(Signature::new_endo([
                just_ty.clone(),
                source_ty.clone(),
                rest_ty.clone(),
            ]))
            .unwrap();
            let [just, source, rest] = outer.input_wires_arr();
            let loop_outputs: [hugr_core::Wire; 2] = {
                let mut loop_builder = outer
                    .tail_loop_builder(
                        [(just_ty.clone(), just)],
                        [(rest_ty.clone(), rest)],
                        vec![source_ty.clone()].into(),
                    )
                    .unwrap();
                let [_just, rest] = loop_builder.input_wires_arr();
                let control = loop_builder
                    .add_dataflow_op(
                        Tag::new(
                            TailLoop::BREAK_TAG,
                            vec![vec![just_ty].into(), vec![source_ty].into()],
                        ),
                        [source],
                    )
                    .unwrap();
                loop_builder
                    .finish_with_outputs(control.out_wire(0), [rest])
                    .unwrap()
                    .outputs_arr()
            };
            outer
                .finish_hugr_with_outputs([just, loop_outputs[0], loop_outputs[1]])
                .unwrap()
        };
        remove_nonlocal_edges(&mut hugr).unwrap();
        hugr.validate().unwrap();

        let tail_loop = hugr
            .entry_descendants()
            .find(|node| hugr.get_optype(*node).is_tail_loop())
            .unwrap();
        let op = hugr.get_optype(tail_loop).as_tail_loop().unwrap();
        let rows = vec![op.just_inputs.clone(), op.just_outputs.clone()];
        let layer = super::control_layer(&hugr, tail_loop, &rows).unwrap();
        let mut transformed = hugr.clone();
        let continue_case = transformed
            .children(layer.wrapper)
            .nth(TailLoop::CONTINUE_TAG)
            .unwrap();
        let [case_input, case_output] = transformed.get_io(continue_case).unwrap();
        let (tag, _) = transformed.single_linked_output(case_output, 0).unwrap();
        transformed.disconnect(tag, IncomingPort::from(1));
        transformed.connect(case_input, 1, tag, 1);
        transformed.validate().unwrap();
        NonLocalizeEdges::default().run(&mut transformed).unwrap();
        transformed.validate().unwrap();
        assert!(transformed.contains_node(layer.wrapper));
        assert_eq!(
            transformed.single_linked_output(layer.output, 0),
            Some((layer.wrapper, OutgoingPort::from(0)))
        );

        let extra = hugr.add_node_with_parent(tail_loop, Noop::new(Type::new_sum(rows)));
        hugr.connect(layer.wrapper, 0, extra, 0);
        hugr.validate().unwrap();

        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        assert!(hugr.contains_node(layer.wrapper));
        assert_eq!(
            hugr.single_linked_output(layer.output, 0),
            Some((layer.wrapper, OutgoingPort::from(0)))
        );
        assert!(
            hugr.linked_inputs(layer.wrapper, 0)
                .any(|(target, _)| target == extra)
        );
    }

    #[test]
    fn nonlocalize_cfg_projects_partially_removed_payload() {
        let (mut hugr, block, retained_target) = {
            let mut outer =
                DFGBuilder::new(Signature::new([bool_t(), bool_t(), Type::UNIT], [bool_t()]))
                    .unwrap();
            let [first_source, second_source, unit] = outer.input_wires_arr();
            let mut cfg = outer
                .cfg_builder([(Type::UNIT, unit)], vec![bool_t()].into())
                .unwrap();
            let entry = {
                let mut entry = cfg.entry_builder([type_row![]], type_row![]).unwrap();
                let control = entry.add_load_value(Value::unit_sum(0, 1).unwrap());
                entry.finish_with_outputs(control, []).unwrap()
            };
            let exit = cfg.exit_block();
            let (block, first_target, second_target) = {
                let mut block = cfg
                    .block_builder(type_row![], [type_row![]], vec![bool_t()].into())
                    .unwrap();
                let first_target = block
                    .add_dataflow_op(Noop::new(bool_t()), [first_source])
                    .unwrap();
                let second_target = block
                    .add_dataflow_op(Noop::new(bool_t()), [second_source])
                    .unwrap();
                let control = block.add_load_value(Value::unit_sum(0, 1).unwrap());
                (
                    block
                        .finish_with_outputs(control, [first_target.out_wire(0)])
                        .unwrap(),
                    first_target.node(),
                    second_target.node(),
                )
            };
            cfg.branch(&entry, 0, &block).unwrap();
            cfg.branch(&block, 0, &exit).unwrap();
            let [output] = cfg.finish_sub_container().unwrap().outputs_arr();
            (
                outer.finish_hugr_with_outputs([output]).unwrap(),
                block.node(),
                (first_target, second_target),
            )
        };
        remove_nonlocal_edges(&mut hugr).unwrap();
        hugr.validate().unwrap();
        hugr.remove_subtree(retained_target.1);
        hugr.validate().unwrap();

        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        let [block_input, _] = hugr.get_io(block).unwrap();
        assert_eq!(
            hugr.get_optype(block)
                .as_dataflow_block()
                .unwrap()
                .inputs
                .as_ref(),
            [bool_t()]
        );
        assert_ne!(
            hugr.single_linked_output(retained_target.0, 0),
            Some((block_input, OutgoingPort::from(0)))
        );

        let once = hugr.clone();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        assert_eq!(hugr, once);
    }

    #[test]
    fn nonlocalize_cfg_rejects_conflicting_loop_provenance() {
        let (mut hugr, block, target, alternate) = {
            let mut outer =
                DFGBuilder::new(Signature::new([bool_t(), Type::UNIT], [bool_t()])).unwrap();
            let [ext_source, unit] = outer.input_wires_arr();
            let mut cfg = outer
                .cfg_builder([(Type::UNIT, unit)], vec![bool_t()].into())
                .unwrap();
            let entry = {
                let mut entry = cfg.entry_builder([type_row![]], type_row![]).unwrap();
                let control = entry.add_load_value(Value::unit_sum(0, 1).unwrap());
                entry.finish_with_outputs(control, []).unwrap()
            };
            let exit = cfg.exit_block();
            let (block, target, alternate) = {
                let mut block = cfg
                    .block_builder(
                        type_row![],
                        [vec![Type::UNIT].into(), vec![bool_t()].into()],
                        type_row![],
                    )
                    .unwrap();
                let target = block
                    .add_dataflow_op(Noop::new(bool_t()), [ext_source])
                    .unwrap();
                let alternate = block.add_load_value(Value::false_val());
                let control = block
                    .add_dataflow_op(
                        Tag::new(1, vec![vec![Type::UNIT].into(), vec![bool_t()].into()]),
                        [target.out_wire(0)],
                    )
                    .unwrap();
                (
                    block.finish_with_outputs(control.out_wire(0), []).unwrap(),
                    target.node(),
                    alternate,
                )
            };
            cfg.branch(&entry, 0, &block).unwrap();
            cfg.branch(&block, 0, &entry).unwrap();
            cfg.branch(&block, 1, &exit).unwrap();
            let [output] = cfg.finish_sub_container().unwrap().outputs_arr();
            (
                outer.finish_hugr_with_outputs([output]).unwrap(),
                block.node(),
                target,
                alternate,
            )
        };
        remove_nonlocal_edges(&mut hugr).unwrap();
        hugr.validate().unwrap();

        let rows = hugr
            .get_optype(block)
            .as_dataflow_block()
            .unwrap()
            .sum_rows
            .clone();
        let layer = super::control_layer(&hugr, block, &rows).unwrap();
        hugr.disconnect(layer.wrapper, IncomingPort::from(1));
        hugr.connect(alternate.node(), alternate.source(), layer.wrapper, 1);
        hugr.validate().unwrap();

        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
        let [block_input, _] = hugr.get_io(block).unwrap();
        assert_eq!(
            hugr.single_linked_output(target, 0),
            Some((block_input, OutgoingPort::from(0)))
        );
        let cfg = hugr
            .entry_descendants()
            .find(|node| hugr.get_optype(*node).is_cfg())
            .unwrap();
        assert_eq!(
            hugr.get_optype(cfg)
                .as_cfg()
                .unwrap()
                .signature
                .input
                .as_ref(),
            [bool_t(), Type::UNIT]
        );
    }

    #[test]
    fn nonlocalize_cfg_restores_ext_and_dom_edges() {
        let (mut hugr, ext_source, dom_source, targets) = {
            let mut outer =
                DFGBuilder::new(Signature::new([bool_t(), Type::UNIT], [bool_t(), bool_t()]))
                    .unwrap();
            let [ext_source, unit] = outer.input_wires_arr();
            let mut cfg = outer
                .cfg_builder([(Type::UNIT, unit)], vec![bool_t(), bool_t()].into())
                .unwrap();
            let (entry, dom_source) = {
                let mut entry = cfg.entry_builder([type_row![]], type_row![]).unwrap();
                let dom_source = entry.add_load_value(Value::true_val());
                let control = entry.add_load_value(Value::unit_sum(0, 1).unwrap());
                (entry.finish_with_outputs(control, []).unwrap(), dom_source)
            };
            let exit = cfg.exit_block();
            let (block, targets) = {
                let mut block = cfg
                    .block_builder(type_row![], [type_row![]], [bool_t(), bool_t()].into())
                    .unwrap();
                let ext_target = block
                    .add_dataflow_op(Noop::new(bool_t()), [ext_source])
                    .unwrap();
                let dom_target = block
                    .add_dataflow_op(Noop::new(bool_t()), [dom_source])
                    .unwrap();
                let targets = [ext_target.node(), dom_target.node()];
                let control = block.add_load_value(Value::unit_sum(0, 1).unwrap());
                (
                    block
                        .finish_with_outputs(
                            control,
                            [ext_target.out_wire(0), dom_target.out_wire(0)],
                        )
                        .unwrap(),
                    targets,
                )
            };
            cfg.branch(&entry, 0, &block).unwrap();
            cfg.branch(&block, 0, &exit).unwrap();
            let outputs: [hugr_core::Wire; 2] = cfg.finish_sub_container().unwrap().outputs_arr();
            (
                outer.finish_hugr_with_outputs(outputs).unwrap(),
                ext_source,
                dom_source,
                targets,
            )
        };
        remove_nonlocal_edges(&mut hugr).unwrap();
        hugr.validate().unwrap();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();

        assert_eq!(
            hugr.single_linked_output(targets[0], 0),
            Some((ext_source.node(), ext_source.source()))
        );
        assert_eq!(
            hugr.single_linked_output(targets[1], 0),
            Some((dom_source.node(), dom_source.source()))
        );
        let cfg = hugr
            .entry_descendants()
            .find(|node| hugr.get_optype(*node).is_cfg())
            .unwrap();
        assert_eq!(
            hugr.get_optype(cfg)
                .as_cfg()
                .unwrap()
                .signature
                .input
                .as_ref(),
            [Type::UNIT]
        );

        let once = hugr.clone();
        NonLocalizeEdges::default().run(&mut hugr).unwrap();
        assert_eq!(hugr, once);
    }
}
