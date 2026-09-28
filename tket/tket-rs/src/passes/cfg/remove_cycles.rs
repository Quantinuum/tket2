//! Remove cycles from a [DomTreeWithBackedges]

use hugr::HugrView;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{CFG, DataflowBlock, ExitBlock, TailLoop};
use hugr::types::{Signature, Type, TypeRow};
use std::collections::{HashSet, VecDeque};

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
    let Some(backedges) = dtn.loop_ else {
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
    let mut in_loop_children = Vec::new();
    let mut out_of_loop_children = Vec::new();
    for (cp, cn) in dtn.children {
        let (in_loop, out_of_loop) = cn.detach(hugr, &loop_blocks);
        if let Some(in_loop) = in_loop {
            in_loop_children.push((cp, in_loop));
        }
        out_of_loop_children.extend(out_of_loop);
    }

    let exit_type_rows = out_of_loop_children
        .iter()
        .map(|n| {
            hugr.get_optype(n.node)
                .as_dataflow_block()
                .unwrap()
                .inputs
                .clone()
        })
        .collect::<Vec<_>>();

    let [inner_cfg, loop_node, tag_continue, tag_exit] =
        make_inner_cfg(hugr, dtn.node, exit_type_rows);
    for &n in &loop_blocks {
        hugr.set_parent(n, inner_cfg);
        for succ_port in hugr.node_outputs(n).collect::<Vec<_>>() {
            let (succ, _) = hugr.single_linked_input(n, succ_port).unwrap();
            if succ == dtn.node {
                //loop backedge
                hugr.disconnect(n, succ_port);
                hugr.connect(n, succ_port, tag_continue, 0);
            } else if !loop_blocks.contains(&succ) {
                hugr.disconnect(n, succ_port);
                hugr.connect(n, succ_port, tag_exit, 0);
            }
        }
    }

    for (pred_n, outport) in hugr.all_linked_outputs(dtn.node).collect::<Vec<_>>() {
        assert_eq!(
            hugr.single_linked_input(pred_n, outport),
            Some((dtn.node, 0.into()))
        );
        hugr.disconnect(pred_n, outport);
        assert!(!loop_blocks.contains(&pred_n)); // loop predecessors disconnected above
        hugr.connect(pred_n, outport, loop_node, 0);
    }

    DomTreeNode {
        node: loop_node,
        // TODO we need also to include edges that exitted both the loop and the header's dom tree.
        // these do not produce "children" here but instead add to exit_edges.
        children: out_of_loop_children
            .into_iter()
            .enumerate()
            .map(|(exit_num, n)| {
                (
                    GatingPath::Always(loop_node, exit_num.into()),
                    nest_loop(n, hugr),
                )
            })
            .collect(),
        // TODO no recompute here. Old exit edges could include edges directly from the loop,
        // these are now edges from the new header (loop_node)
        exit_edges: dtn.exit_edges,
        loop_: Some(InnerTailLoop(Box::new(DomTreeNode {
            node: dtn.node,
            children: in_loop_children
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
