//! Remove cycles from a [DomTreeWithBackedges]

use hugr::HugrView;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{CFG, DataflowBlock, ExitBlock, TailLoop};
use hugr::types::{Signature, Type, TypeRow};
use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};

use super::gating_path::{DomTreeNode, DomTreeWithBackedges, GatingPath};

/// Used as [DomTreeNode::loop_], records information about a CFG within a TailLoop within
/// the [DomTreeNode::node]
pub struct InnerTailLoop<N>(pub Box<DomTreeNode<N, InnerTailLoop<N>>>);

/// Turn any loops in the input into loops inside [TailLoop] nodes (containing an inner
/// CFG with the loop blocks) inside the original header block.
pub fn nest_loop<H: HugrMut>(
    dtn: DomTreeWithBackedges<H::Node>,
    hugr: &mut H,
) -> DomTreeNode<H::Node, InnerTailLoop<H::Node>> {
    let Some(backedges) = dtn.loop_.as_ref() else {
        return DomTreeNode {
            node: dtn.node,
            children: dtn
                .children
                .into_iter()
                .map(|(cp, cn)| (cp, nest_loop(cn, hugr)))
                .collect(),
            exit_edges: dtn.exit_edges,
            loop_: None,
        };
    };
    let loop_blocks = loop_blocks(hugr, dtn.node, &backedges);
    let (loop_dtn, out_of_loop, exit_targets) = dtn.detach(hugr, &loop_blocks);
    let loop_dtn = loop_dtn.unwrap(); // header is in loop!!
    let exit_targets = BTreeMap::from_iter(exit_targets);

    let exit_type_rows = exit_targets
        .keys()
        .map(|n| {
            hugr.get_optype(*n)
                .as_dataflow_block()
                .unwrap()
                .inputs
                .clone()
        })
        .collect::<Vec<_>>();
    let exit_tags: HashMap<H::Node, usize> = exit_targets
        .keys()
        .enumerate()
        .map(|(i, k)| (*k, i))
        .collect();

    // TODO add BBs doing Tags for each exit_target, positioned according to LCA (as vec)
    // Any "exit edges" representing jumps to said tags also need recomputing.
    // And we need to position tag_exit (could do according to loop_dtn.exit_edges, if properly computed)
    // and tag_continue (could do according to loop_dtn.loop_ backedges) in loop_dtn's Dom Tree,
    // along with the ExitBlock.

    let [inner_cfg, loop_node, tag_continue, tag_exit] =
        make_inner_cfg(hugr, loop_dtn.node, exit_type_rows);
    for &n in &loop_blocks {
        hugr.set_parent(n, inner_cfg);
        for succ_port in hugr.node_outputs(n).collect::<Vec<_>>() {
            let (succ, _) = hugr.single_linked_input(n, succ_port).unwrap();
            if succ == loop_dtn.node {
                //loop backedge
                hugr.disconnect(n, succ_port);
                hugr.connect(n, succ_port, tag_continue, 0);
            } else if !loop_blocks.contains(&succ) {
                hugr.disconnect(n, succ_port);
                let tag = *exit_tags.get(&succ).unwrap();
                // TODO insert tag? Or tag block? exit_targets.values() does give LCA of tag block
                hugr.connect(n, succ_port, tag_exit, 0);
            }
        }
    }

    for (pred_n, outport) in hugr.all_linked_outputs(loop_dtn.node).collect::<Vec<_>>() {
        assert_eq!(
            hugr.single_linked_input(pred_n, outport),
            Some((loop_dtn.node, 0.into()))
        );
        hugr.disconnect(pred_n, outport);
        assert!(!loop_blocks.contains(&pred_n)); // loop predecessors disconnected above
        hugr.connect(pred_n, outport, loop_node, 0);
    }

    DomTreeNode {
        node: loop_node,
        children: out_of_loop
            .into_iter()
            .map(|ch| {
                let exit_num = *exit_tags.get(&ch.node).unwrap();
                (
                    GatingPath::Always(loop_node, exit_num.into()),
                    nest_loop(ch, hugr),
                )
            })
            .collect(),
        // TODO no recompute here, from child exit_edges and own outgoing edges.
        // (Any edges exitting the inner loop have been turned into outports of the containing
        // block regardless of target)
        exit_edges: dtn.exit_edges,
        loop_: Some(InnerTailLoop(Box::new(DomTreeNode {
            node: loop_dtn.node,
            children: loop_dtn
                .children
                .into_iter()
                .map(|(cp, cn)| (cp, nest_loop(cn, hugr)))
                .collect(),
            exit_edges: None, // every node in inner CFG is dominated by the header, including the ExitBlock
            loop_: None,
        }))),
    }
}

fn loop_blocks<H: HugrView>(
    hugr: &H,
    loop_header: H::Node,
    backedges: &GatingPath<H::Node>,
) -> HashSet<H::Node> {
    let mut blocks = HashSet::new();
    let mut queue = VecDeque::from_iter(backedges.leaves(hugr).into_iter().map(|lp| lp.src.0));
    while let Some(n) = queue.pop_front() {
        if n == loop_header || !blocks.insert(n) {
            continue;
        }
        queue.extend(hugr.input_neighbours(n));
    }
    blocks
}

fn make_inner_cfg<H: HugrMut>(
    hugr: &mut H,
    old_loop_node: H::Node,
    exit_type_rows: Vec<TypeRow>,
) -> [H::Node; 4] {
    let header_inputs = hugr
        .get_optype(old_loop_node)
        .as_dataflow_block()
        .unwrap()
        .inputs
        .clone();
    // TODO(?) If exit_type_rows of length one, then no need for this, hmmm.
    let exit_sum_row = TypeRow::from([Type::new_sum(exit_type_rows.clone())]);

    let outer_cfg = hugr.get_parent(old_loop_node).unwrap();
    let loop_node = hugr.add_node_with_parent(
        outer_cfg,
        DataflowBlock {
            inputs: header_inputs.clone(),
            other_outputs: TypeRow::new(),
            sum_rows: exit_type_rows.clone(),
        },
    );
    let tail_loop = hugr.add_node_with_parent(
        loop_node,
        TailLoop {
            just_inputs: header_inputs.clone(),
            just_outputs: exit_sum_row.clone(),
            rest: TypeRow::new(),
        },
    ); // TODO and wire up (loop_node needs input/output first)
    let exit_or_continue_row =
        TypeRow::from([Type::new_sum([header_inputs.clone(), exit_sum_row.clone()])]);
    let inner_cfg = hugr.add_node_with_parent(
        tail_loop,
        CFG {
            signature: Signature::new(header_inputs.clone(), exit_or_continue_row.clone()),
        },
    ); // TODO and wire up (tail_loop needs input/output first)
    hugr.set_parent(old_loop_node, inner_cfg);
    let exit_block = hugr.add_node_with_parent(
        inner_cfg,
        ExitBlock {
            cfg_outputs: exit_or_continue_row.clone(),
        },
    );

    let tag_continue = hugr.add_node_with_parent(
        inner_cfg,
        DataflowBlock {
            inputs: header_inputs.clone(),
            other_outputs: TypeRow::new(),
            sum_rows: vec![exit_or_continue_row.clone()],
        },
    );
    // TODO add and wire up tag_continue's children
    let tag_exit = hugr.add_node_with_parent(
        inner_cfg,
        DataflowBlock {
            inputs: exit_sum_row,
            other_outputs: TypeRow::new(),
            sum_rows: vec![exit_or_continue_row.clone()],
        },
    );
    // TODO add and wire up tag_exit's children
    hugr.connect(tag_continue, 0, exit_block, 0);
    hugr.connect(tag_exit, 0, exit_block, 0);

    [inner_cfg, loop_node, tag_continue, tag_exit]
}
