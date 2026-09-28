//! Remove cycles from a [DomTreeWithBackedges]

use hugr::HugrView;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{CFG, DataflowBlock, ExitBlock, Input, OpTrait, OpType, Output, Tag, TailLoop};
use hugr::types::{Signature, Type, TypeRow};
use itertools::Itertools;
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

    let [loop_node, inner_cfg, tag_continue, tag_exit] =
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

    let mut dtn = DomTreeNode::<H::Node, InnerTailLoop<H::Node>>::new(
        loop_node,
        out_of_loop
            .into_iter()
            .map(|ch| nest_loop(ch, hugr))
            .collect(),
        |_, lp| {
            assert_ne!(lp.tgt, loop_node);
            false
        },
        hugr,
    );

    dtn.loop_ = Some(InnerTailLoop(Box::new(DomTreeNode {
        node: loop_dtn.node,
        children: loop_dtn
            .children
            .into_iter()
            .map(|(cp, cn)| (cp, nest_loop(cn, hugr)))
            .collect(),
        exit_edges: None, // every node in inner CFG is dominated by the header, including the ExitBlock
        loop_: None,
    })));
    dtn
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

/// Makes, inside the parent of `old_loop_node` (parent should be a CFG),
/// a new basic block, containing a TailLoop, containing a CFG,
/// into two `old_loop_node` is moved (becoming the entry block).
///
/// Return an array, in outside-to-in order, of:
/// * the new basic block
/// * the new CFG,
/// * the `continue` and `exit` tagging nodes (which jump to the CFG's ExitBlock)
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
    let [loop_node, l_in, l_out] = create_with_io(
        hugr,
        outer_cfg,
        DataflowBlock {
            inputs: header_inputs.clone(),
            other_outputs: TypeRow::new(),
            sum_rows: exit_type_rows.clone(),
        },
    );
    let [tail_loop, t_in, t_out] = create_with_io(
        hugr,
        loop_node,
        TailLoop {
            just_inputs: header_inputs.clone(),
            just_outputs: exit_sum_row.clone(),
            rest: TypeRow::new(),
        },
    );
    wire_all(hugr, l_in, tail_loop);
    wire_all(hugr, tail_loop, l_out);
    let exit_or_continue_rows = BTreeMap::from([
        (TailLoop::CONTINUE_TAG, header_inputs.clone()),
        (TailLoop::BREAK_TAG, exit_sum_row.clone()),
    ])
    .into_values()
    .collect_array::<2>()
    .unwrap();
    let exit_or_continue_row = TypeRow::from([Type::new_sum(exit_or_continue_rows.clone())]);
    let inner_cfg = hugr.add_node_with_parent(
        tail_loop,
        CFG {
            signature: Signature::new(header_inputs.clone(), exit_or_continue_row.clone()),
        },
    );
    wire_all(hugr, t_in, inner_cfg);
    wire_all(hugr, inner_cfg, t_out);
    hugr.set_parent(old_loop_node, inner_cfg);
    let exit_block = hugr.add_node_with_parent(
        inner_cfg,
        ExitBlock {
            cfg_outputs: exit_or_continue_row.clone(),
        },
    );

    let [continue_block, c_i, c_o] = create_with_io(
        hugr,
        inner_cfg,
        DataflowBlock {
            inputs: header_inputs.clone(),
            other_outputs: TypeRow::new(),
            sum_rows: vec![exit_or_continue_row.clone()],
        },
    );
    let tag_continue = hugr.add_node_with_parent(
        continue_block,
        Tag::new(TailLoop::CONTINUE_TAG, exit_or_continue_rows.to_vec()),
    );
    wire_all(hugr, c_i, tag_continue);
    wire_all(hugr, tag_continue, c_o);
    let [break_block, b_i, b_o] = create_with_io(
        hugr,
        inner_cfg,
        DataflowBlock {
            inputs: exit_sum_row,
            other_outputs: TypeRow::new(),
            sum_rows: vec![exit_or_continue_row.clone()],
        },
    );
    let tag_exit = hugr.add_node_with_parent(
        exit_block,
        Tag::new(TailLoop::BREAK_TAG, exit_or_continue_rows.to_vec()),
    );
    wire_all(hugr, b_i, tag_exit);
    wire_all(hugr, tag_exit, b_o);

    hugr.connect(continue_block, 0, exit_block, 0);
    hugr.connect(break_block, 0, exit_block, 0);

    [loop_node, inner_cfg, continue_block, break_block]
}

fn create_with_io<H: HugrMut>(
    h: &mut H,
    parent: H::Node,
    op: impl OpTrait + Into<OpType>,
) -> [H::Node; 3] {
    let Signature { input, output } = op.dataflow_signature().unwrap().into_owned();
    let op = op.into();

    let n = h.add_node_with_parent(parent, op);
    let i = h.add_node_with_parent(n, Input { types: input });
    let o = h.add_node_with_parent(n, Output { types: output });
    return [n, i, o];
}

fn wire_all<H: HugrMut>(h: &mut H, src_node: H::Node, tgt_node: H::Node) {
    let outports = h.node_outputs(src_node).collect::<Vec<_>>();
    let inports = h.node_inputs(tgt_node).collect::<Vec<_>>();
    for (outport, inport) in outports.iter().zip_eq(inports.iter()) {
        h.connect(src_node, *outport, tgt_node, *inport);
    }
}
