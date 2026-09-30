//! Remove cycles from a [DomTreeWithBackedges]

use hugr::HugrView;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{CFG, DataflowBlock, ExitBlock, Input, OpParent, OpType, Output, Tag, TailLoop};
use hugr::types::{Signature, Type, TypeRow};
use itertools::Itertools;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};

use crate::passes::cfg::gating_path::leaves;

use super::gating_path::{DomTreeNode, DomTreeWithBackedges, GatingPath};

/// Used as [DomTreeNode::loop_], records information about a CFG within a TailLoop within
/// the [DomTreeNode::node]
pub struct InnerTailLoop<N> {
    pub inside_loop: Box<DomTreeNode<N, InnerTailLoop<N>>>,
    /// Not added to `inside_loop` even though should be
    extra_blocks: Vec<N>,
}

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
    // While we could (more efficiently) extract the list of post-loop blocks during the detach operation,
    // it's more helpful to have the complete list available now so we can put Tags into place during detach.
    let post_loop_blocks = loop_blocks
        .iter()
        .flat_map(|n| hugr.output_neighbours(*n))
        .filter(|n| !loop_blocks.contains(n))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .enumerate()
        .map(|(i, n)| (n, i))
        .collect::<HashMap<_, _>>();

    let break_rows = post_loop_blocks
        .keys()
        .map(|n| {
            TypeRow::from_iter(
                hugr.get_optype(*n)
                    .value_input_types()
                    .map(|(_, t)| t.clone()),
            )
        })
        .collect::<Vec<_>>();

    let [loop_node, inner_cfg, continue_bb, break_bb, exit_block] =
        make_inner_cfg(hugr, dtn.node, break_rows.clone());
    for block in &loop_blocks {
        hugr.set_parent(*block, inner_cfg);
    }

    // For each control-flow edge that exits the loop, make a new BB that exits the loop with a value
    // tagged to indicate which post-loop block to go to, and retarget the edge.
    let break_row = TypeRow::from([Type::new_sum(break_rows.clone())]);
    let break_blocks = post_loop_blocks
        .iter()
        .map(|(&n, &tag)| {
            let inp_row = TypeRow::from_iter(
                hugr.get_optype(n)
                    .value_input_types()
                    .map(|(_, t)| t.clone()),
            );
            let [bb, i, o] = create_with_io(
                hugr,
                inner_cfg,
                DataflowBlock {
                    inputs: inp_row,
                    other_outputs: TypeRow::new(),
                    sum_rows: vec![break_row.clone()],
                },
            );
            let which_break = hugr.add_node_with_parent(bb, Tag::new(tag, break_rows.clone()));
            wire_all(hugr, i, which_break);
            let bb_predicate = hugr.add_node_with_parent(bb, Tag::new(0, vec![break_row.clone()]));
            hugr.connect(which_break, 0, bb_predicate, 0);
            hugr.connect(bb_predicate, 0, o, 0);
            hugr.connect(bb, 0, break_bb, 0);
            // Disconnect the original control-flow edge from the loop to the post-loop block.
            for (n, p) in hugr.linked_outputs(n, 0).collect::<Vec<_>>() {
                hugr.disconnect(n, p);
                hugr.connect(n, p, bb, 0);
            }
            (n, bb)
        })
        .collect::<HashMap<_, _>>();

    // Any edges that exit the original subtree necessarily exit the loop (as entirely
    // contained within subtree), so the corresponding break-blocks will not be added by detach
    let break_blocks_exitting_subtree = leaves(&dtn.exit_edges, hugr)
        .into_iter()
        .map(|lp| break_blocks[&lp.tgt])
        .collect::<Vec<_>>();

    // now build the dominator tree for inside the loop. Its exit-edges will include all control-flow edges to:
    //   break_bb (i.e. all edges from detached subtree's individual break_block's)
    //   any break_blocks for nodes outside the subtree
    let (loop_dtn, out_loop_children) = dtn.detach(hugr, &loop_blocks, &break_blocks);
    let loop_dtn = loop_dtn.unwrap(); // header is in loop!!
    assert!(loop_dtn.loop_.is_some()); // detach has detailed assertion

    // disconnect backedges, reconnect to continue_bb
    for (n, p) in hugr.linked_outputs(loop_dtn.node, 0).collect::<Vec<_>>() {
        hugr.disconnect(n, p);
        hugr.connect(n, p, continue_bb, 0);
    }

    let mut dtn = DomTreeNode::<H::Node, InnerTailLoop<H::Node>>::new(
        loop_node,
        out_loop_children
            .into_iter()
            .map(|ch| nest_loop(ch, hugr))
            .collect(),
        |_, lp| {
            assert_ne!(lp.tgt, loop_node);
            false
        },
        hugr,
    );

    dtn.loop_ = Some(InnerTailLoop {
        inside_loop: Box::new(DomTreeNode {
            node: loop_dtn.node,
            children: loop_dtn
                .children
                .into_iter()
                .map(|(cp, cn)| (cp, nest_loop(cn, hugr)))
                .collect(),
            exit_edges: None, // every node in inner CFG is dominated by the header, including the ExitBlock
            loop_: None,
        }),
        extra_blocks: break_blocks_exitting_subtree
            .into_iter()
            .chain([break_bb, continue_bb, exit_block])
            .collect(),
    });
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
        if !blocks.insert(n) || n == loop_header {
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
/// * the new (containing) [DataflowBlock]
/// * the new [CFG],
/// * the `continue` and `exit` tagging nodes - both of which jump to:
/// * the new CFG's [ExitBlock]
fn make_inner_cfg<H: HugrMut>(
    hugr: &mut H,
    old_loop_node: H::Node,
    exit_type_rows: Vec<TypeRow>,
) -> [H::Node; 5] {
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

    [
        loop_node,
        inner_cfg,
        continue_block,
        break_block,
        exit_block,
    ]
}

fn create_with_io<H: HugrMut>(h: &mut H, parent: H::Node, op: impl Into<OpType>) -> [H::Node; 3] {
    let op = op.into();
    let Signature { input, output } = op.inner_function_type().unwrap().into_owned();

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

#[cfg(test)]
mod test {
    use hugr::{Hugr, HugrView, envelope::EnvelopeConfig};
    use itertools::Itertools;
    use rstest::rstest;
    use std::{fs::File, io::BufReader, path::Path};

    use super::nest_loop;
    use crate::passes::cfg::gating_path::DomTreeWithBackedges;

    #[rstest]
    #[case("/Users/alanlawrence/repos/tierkreis/doubler.hugr")]
    #[case("/Users/alanlawrence/repos/tierkreis/doubler_indirect_minopt.hugr")]
    #[case("/Users/alanlawrence/repos/tierkreis/shortcircuit_if.hugr")]
    fn non_loop(#[case] fname: impl AsRef<Path>) {
        let reader = BufReader::new(File::open(fname).unwrap());
        let backup = Hugr::load(reader, None).unwrap();
        let mut h = backup.clone();
        let cfgs = h
            .nodes()
            .filter(|n| h.get_optype(*n).is_cfg())
            .collect_vec();
        for n in cfgs {
            let dtn = DomTreeWithBackedges::new_for_cfg(&h, n);
            nest_loop(dtn, &mut h);
        }
        assert_eq!(h, backup); // Did nothing
    }

    #[rstest]
    #[case("/Users/alanlawrence/repos/tierkreis/tierkreis_loop.hugr")]
    fn tierkreis_loop(#[case] fname: impl AsRef<Path>) {
        let outfile = fname.as_ref().with_added_extension("nested");
        let reader = BufReader::new(File::open(fname).unwrap());
        let backup = Hugr::load(reader, None).unwrap();
        eprintln!("{}", backup.mermaid_string());
        let mut h = backup.clone();
        let cfgs = h
            .nodes()
            .filter(|n| h.get_optype(*n).is_cfg())
            .collect_vec();
        eprintln!("Found {} cfgs", cfgs.len());
        for n in cfgs {
            eprintln!("Processing cfg node {}", n);
            let dtn = DomTreeWithBackedges::new_for_cfg(&h, n);
            nest_loop(dtn, &mut h);
        }
        let f = File::create(outfile).unwrap();
        h.store(f, EnvelopeConfig::default()).unwrap();
    }
}
