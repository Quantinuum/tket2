//! Quick pass for removing redundant tuple pack/unpack operations.

use core::panic;

use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr};
use hugr::extension::prelude::{MakeTuple, TupleOpDef};
use hugr::extension::simple_op::MakeExtensionOp;
use hugr::hugr::views::SiblingSubgraph;
use hugr::ops::{OpTrait, OpType};
use hugr::types::Type;
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::Circuit;
use crate::circuit::Command;
use crate::rewrite::CircuitRewrite;

/// Find tuple pack operations followed by tuple unpack operations
/// and generate rewrites to remove them.
#[deprecated(since = "0.15.1", note = "Use hugr::algorithms::UntuplePass instead")]
pub fn find_tuple_unpack_rewrites(
    circ: &Circuit<impl HugrView<Node = Node>>,
) -> impl Iterator<Item = CircuitRewrite> + '_ {
    circ.commands().filter_map(|cmd| make_rewrite(circ, cmd))
}

/// Returns true if the given optype is a MakeTuple operation.
///
/// Boilerplate required due to https://github.com/quantinuum/hugr/issues/1496
fn is_make_tuple(optype: &OpType) -> bool {
    optype.to_string() == format!("prelude.{}", TupleOpDef::MakeTuple.op_id())
}

/// Returns true if the given optype is an UnpackTuple operation.
///
/// Boilerplate required due to https://github.com/quantinuum/hugr/issues/1496
fn is_unpack_tuple(optype: &OpType) -> bool {
    optype.to_string() == format!("prelude.{}", TupleOpDef::UnpackTuple.op_id())
}

fn make_rewrite<T: HugrView<Node = Node>>(
    circ: &Circuit<T>,
    cmd: Command<T>,
) -> Option<CircuitRewrite> {
    let cmd_optype = cmd.optype();
    let tuple_node = cmd.node();
    if !is_make_tuple(cmd_optype) {
        return None;
    }
    let tuple_types = cmd_optype
        .dataflow_signature()
        .unwrap()
        .input_types()
        .to_vec();

    // Make tuple should have a single output
    let Ok((_, wire)) = cmd.output_wires().exactly_one() else {
        panic!("MakeTuple at node {tuple_node} should have a single output wire.");
    };

    // See if it is followed by a tuple unpack
    let links = circ
        .hugr()
        .linked_inputs(tuple_node, wire.source())
        .map(|(neigh, _)| neigh)
        .collect_vec();

    if links.is_empty() {
        return None;
    }

    let unpack_nodes = links
        .iter()
        .filter(|&&neigh| is_unpack_tuple(circ.hugr().get_optype(neigh)))
        .copied()
        .collect_vec();

    if unpack_nodes.is_empty() {
        return None;
    }

    // Remove all unpack operations, but only remove the pack operation if all neighbours are unpacks.
    let num_other_outputs = links.len() - unpack_nodes.len();
    Some(remove_pack_unpack(
        circ,
        &tuple_types,
        tuple_node,
        unpack_nodes,
        num_other_outputs,
    ))
}

/// Returns a rewrite to remove a tuple pack operation that's followed by unpack operations,
/// and `other_tuple_links` other operations.
fn remove_pack_unpack<T: HugrView<Node = Node>>(
    circ: &Circuit<T>,
    tuple_types: &[Type],
    pack_node: Node,
    unpack_nodes: Vec<Node>,
    num_other_outputs: usize,
) -> CircuitRewrite {
    let num_unpack_outputs = tuple_types.len() * unpack_nodes.len();

    let mut nodes = unpack_nodes;
    nodes.push(pack_node);
    let subgraph = SiblingSubgraph::try_from_nodes(nodes, circ.hugr()).expect("is convex");
    let subcirc_signature = subgraph.signature(circ.hugr());

    // The output port order in `Subcircuit::try_from_nodes` is not too well defined.
    // Check that the outputs are in the expected order.
    debug_assert!(
        itertools::equal(
            subcirc_signature.output().iter(),
            tuple_types
                .iter()
                .cycle()
                .take(num_unpack_outputs)
                .chain(itertools::repeat_n(
                    &Type::new_tuple(tuple_types.to_vec()),
                    num_other_outputs
                ))
        ),
        "Unpacked tuple values must come before tupled values"
    );

    let mut replacement = DFGBuilder::new(subcirc_signature).unwrap();
    let mut outputs = Vec::with_capacity(num_unpack_outputs + num_other_outputs);

    // Wire the inputs directly to the unpack outputs
    outputs.extend(replacement.input_wires().cycle().take(num_unpack_outputs));

    // If needed, re-add the tuple pack node and connect its output to the tuple outputs.
    if num_other_outputs > 0 {
        let op = MakeTuple::new(tuple_types.to_vec().into());
        let [tuple] = replacement
            .add_dataflow_op(op, replacement.input_wires())
            .unwrap()
            .outputs_arr();
        outputs.extend(std::iter::repeat_n(tuple, num_other_outputs))
    }

    let replacement = replacement
        .finish_hugr_with_outputs(outputs)
        .unwrap_or_else(|e| {
            panic!("Failed to create replacement for removing tuple pack/unpack operations. {e}")
        });

    subgraph
        .create_simple_replacement(circ.hugr(), replacement)
        .unwrap_or_else(|e| {
            panic!("Failed to create rewrite for removing tuple pack/unpack operations. {e}")
        })
        .into()
}

#[cfg(test)]
mod test {
    use super::*;
    use hugr::extension::prelude::{UnpackTuple, bool_t, qb_t};

    use hugr::types::Signature;
    use rstest::{fixture, rstest};

    /// A simple pack operation followed by an unpack operation.
    ///
    /// These can be removed entirely.
    #[fixture]
    fn simple_pack_unpack() -> Circuit {
        let mut h = DFGBuilder::new(Signature::new_endo(vec![qb_t(), bool_t()])).unwrap();
        let mut inps = h.input_wires();
        let qb1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([qb1, b2]).unwrap();

        let op = UnpackTuple::new(vec![qb_t(), bool_t()].into());
        let [qb1, b2] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        h.finish_hugr_with_outputs([qb1, b2]).unwrap().into()
    }

    /// A pack operation followed by two unpack operations from the same tuple.
    ///
    /// These can be removed entirely.
    #[fixture]
    fn multi_unpack() -> Circuit {
        let mut h = DFGBuilder::new(Signature::new(
            vec![bool_t(), bool_t()],
            vec![bool_t(), bool_t(), bool_t(), bool_t()],
        ))
        .unwrap();
        let mut inps = h.input_wires();
        let b1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([b1, b2]).unwrap();

        let op = UnpackTuple::new(vec![bool_t(), bool_t()].into());
        let [b1, b2] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        let op = UnpackTuple::new(vec![bool_t(), bool_t()].into());
        let [b3, b4] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        h.finish_hugr_with_outputs([b1, b2, b3, b4]).unwrap().into()
    }

    /// A pack operation followed by an unpack operation, where the tuple is also returned.
    ///
    /// The unpack operation can be removed, but the pack operation cannot.
    #[fixture]
    fn partial_unpack() -> Circuit {
        let mut h = DFGBuilder::new(Signature::new(
            vec![bool_t(), bool_t()],
            vec![
                bool_t(),
                bool_t(),
                Type::new_tuple(vec![bool_t(), bool_t()]),
            ],
        ))
        .unwrap();
        let mut inps = h.input_wires();
        let b1 = inps.next().unwrap();
        let b2 = inps.next().unwrap();

        let tuple = h.make_tuple([b1, b2]).unwrap();

        let op = UnpackTuple::new(vec![bool_t(), bool_t()].into());
        let [b1, b2] = h.add_dataflow_op(op, [tuple]).unwrap().outputs_arr();

        h.finish_hugr_with_outputs([b1, b2, tuple]).unwrap().into()
    }

    #[rstest]
    #[case::simple(simple_pack_unpack(), 1, 0)]
    #[case::multi(multi_unpack(), 1, 0)]
    #[case::partial(partial_unpack(), 1, 1)]
    fn test_pack_unpack(
        #[case] mut circ: Circuit,
        #[case] expected_rewrites: usize,
        #[case] remaining_commands: usize,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut num_rewrites = 0;
        loop {
            #[allow(deprecated)]
            let Some(rewrite) = find_tuple_unpack_rewrites(&circ).next() else {
                break;
            };
            num_rewrites += 1;
            rewrite.apply(&mut circ)?;
        }
        assert_eq!(num_rewrites, expected_rewrites);

        assert_eq!(circ.commands().count(), remaining_commands);

        Ok(())
    }
}
