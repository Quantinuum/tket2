//! Internal implementation of `Circuit::extract_dfg`.

use hugr::HugrView;
use hugr::hugr::hugrmut::HugrMut;
use hugr::ops::{DFG, OpTrait, OpType, Output};
use hugr::types::{Signature, SumType, TypeEnum};
use hugr_core::hugr::internal::HugrMutInternals;
use itertools::Itertools;

use crate::{Circuit, CircuitMutError};

/// Internal method used by [`extract_dfg`] to replace the parent node with a DFG node.
#[expect(unused)] // TODO: Remove if things work without it
pub(super) fn rewrite_into_dfg(circ: &mut Circuit) -> Result<(), CircuitMutError> {
    // Replace the parent node with a DFG node, if necessary.
    let old_optype = circ.hugr.get_optype(circ.parent());
    if matches!(old_optype, OpType::DFG(_)) {
        return Ok(());
    }

    // If the region was a cfg with a single successor, unpack the output sum type.
    let signature = circ.circuit_signature().into_owned();
    let signature = match old_optype {
        OpType::DataflowBlock(_) => remove_cfg_empty_output_tuple(circ, signature)?,
        _ => signature,
    };

    circ.hugr.set_num_ports(
        circ.parent(),
        signature.input_count() + 1,
        signature.input_count() + 1,
    );
    circ.hugr.replace_op(circ.parent(), DFG { signature });

    Ok(())
}

/// Remove an empty sum from a cfg's DataflowBlock output node, if possible.
///
/// Bails out if it cannot match the exact pattern, without modifying the
/// circuit.
///
/// TODO: This function is specialized towards the specific functions generated
///     by guppy. We should generalize this to work with non-empty sum types
///     when possible.
fn remove_cfg_empty_output_tuple(
    circ: &mut Circuit,
    signature: Signature,
) -> Result<Signature, CircuitMutError> {
    let sig = signature;
    let input_node = circ.input_node();

    let output_node = circ.output_node();
    let output_op = circ.hugr.get_optype(output_node).clone();

    let output_sig = output_op
        .dataflow_signature()
        .expect("Exit node with no dataflow signature.");

    // Only remove the port if it's an empty sum type.
    if !matches!(
        output_sig.input[0].as_type_enum(),
        TypeEnum::Sum(SumType::Unit { size: 1 })
    ) {
        return Ok(sig);
    }

    // There must be a zero-sized `Tag` operation.
    let Some((tag_node, _)) = circ.hugr.single_linked_output(output_node, 0) else {
        return Ok(sig);
    };

    let tag_op = circ.hugr.get_optype(tag_node);
    if !matches!(tag_op, OpType::Tag(_)) {
        return Ok(sig);
    }

    // Hacky replacement for the nodes.

    // Drop the old nodes
    let hugr = circ.hugr_mut();
    let input_neighs = hugr.all_linked_outputs(output_node).skip(1).collect_vec();

    hugr.remove_node(output_node);
    hugr.remove_node(tag_node);

    // Add a new output node.
    let new_types = output_sig.input[1..].to_vec();
    let new_node = hugr.add_node_after(
        input_node,
        Output {
            types: new_types.clone().into(),
        },
    );

    // Reconnect the outputs.
    for (i, (neigh, port)) in input_neighs.into_iter().enumerate() {
        hugr.connect(neigh, port, new_node, i);
    }

    // Return the updated circuit signature.
    let sig = Signature::new(sig.input, new_types);
    Ok(sig)
}
