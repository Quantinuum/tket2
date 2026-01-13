//! A pass that applies the gridsynth algorithm to all Rz gates in a HUGR.

use std::collections::HashMap;

use crate::TketOp;
use crate::extension::rotation::ConstRotation;
use crate::hugr::HugrView;
use crate::hugr::Node;
use crate::hugr::NodeIndex;
use crate::hugr::hugr::{ValidationError, hugrmut::HugrMut};
use crate::passes::guppy::{NormalizeGuppy, NormalizeGuppyErrors};
use crate::{Hugr, hugr, op_matches};
use hugr::algorithms::ComposablePass;
use hugr::std_extensions::arithmetic::float_types::ConstF64;
use hugr_core::OutgoingPort;
use rsgridsynth::config::config_from_theta_epsilon;
use rsgridsynth::gridsynth::gridsynth_gates;

/// Errors that can occur during the Gridsynth pass due to acting on a hugr that
/// goes beyond the scope of what the pass can optimise. The most likely reasons for this
/// are that the Rz angle is defined at runtime or that the NormalizeGuppy pass is unable
/// to standardise the form of the HUGR enough. Issues may occur when the constant node
/// providing the angle crosses function boundaries or if the control flow is especially
/// complicated
#[derive(derive_more::Error, Debug, derive_more::Display, derive_more::From)]
pub enum GridsynthError {
    /// Error during the NormalizeGuppy pass
    NormalizeGuppyErrors(NormalizeGuppyErrors),
    /// Error during validation of the HUGR
    ValidationError(ValidationError<Node>),
}
/// Garbage collector for cleaning up constant nodes providing angles to the Rz gates and
/// all nodes on the path to them.
struct GarbageCollector {
    references: HashMap<usize, usize>, // key: node index (of Const node containing angle),
    // value: reference counter for that node
    path: HashMap<usize, Vec<Node>>, // key: node index (of Const node containing angle),
                                     // value: the nodes leading up to the constant node and the constant
                                     // node itself
} // CONCERN: I am concerned that this approach may not clean up properly if the Guppy user
// has redundant calls of the constant (eg, has used it to define another constant but then
// not used that constant).

impl GarbageCollector {
    /// Add references to constant node
    fn add_references(&mut self, node: Node, increment: usize) {
        // if reference not in references add it with the default value 1, else increment count
        let count = self.references.entry(node.index()).or_insert(1);
        *count += increment;
    }

    /// Remove reference to a constant node
    fn remove_references(&mut self, node: Node, increment: usize) {
        // reduce reference count
        let count = self.references.get_mut(&node.index()).unwrap();
        *count -= increment;
    }

    /// Infer how many references there are to the angle-containing Const node
    /// given the corresponding `load_const_node`
    fn infer_references_to_angle(
        &mut self,
        hugr: &mut Hugr,
        load_const_node: Node,
        const_node: Node,
    ) {
        let references_collection: Vec<_> = hugr.node_outputs(load_const_node).collect();
        let num_references = references_collection.len();
        // if reference not in references add it with the default value num_references, else do nothing
        self.references
            .entry(const_node.index())
            .or_insert(num_references);
    }

    /// If there are no references remaining to const_node, remove it and the nodes leading to it
    fn collect(&mut self, hugr: &mut Hugr, const_node: Node) {
        let node_index = &const_node.index();
        if self.references[node_index] == 0 {
            let path: Vec<Node> = self.path.get(node_index).unwrap().to_vec();
            for node in path {
                hugr.remove_node(node);
            }
        }
    }
}

/// Find the nodes for the Rz gates.
fn find_rzs(hugr: &mut Hugr) -> Option<Vec<hugr::Node>> {
    let mut rz_nodes: Vec<Node> = Vec::new();
    for node in hugr.nodes() {
        let op_type = hugr.get_optype(node);
        if op_matches(op_type, TketOp::Rz) {
            rz_nodes.push(node);
        }
    }

    // if there are rz_nodes:
    if !(rz_nodes.is_empty()) {
        return Some(rz_nodes);
    }
    None
}

/// Find the output port and node linked to the input specified by `port_idx` for `node`
fn find_single_linked_output_by_index(
    hugr: &mut Hugr,
    node: Node,
    port_idx: usize,
) -> (Node, OutgoingPort) {
    let ports = hugr.node_inputs(node);
    let collected_ports: Vec<_> = ports.collect();

    hugr.single_linked_output(node, collected_ports[port_idx])
        .expect("Not yet set-up to handle cases where there are no previous nodes")
}

/// Find the constant node containing the angle to be inputted to the Rz gate.
/// It is assumed that `hugr` has had the NormalizeGuppy passes applied to it
/// prior to being applied. This function also cleans up behind itself removing
/// everything on the path to the `angle_node` but not the `angle_node` itself,
/// which is still needed.
fn find_angle_node(
    hugr: &mut Hugr,
    rz_node: Node,
    garbage_collector: &mut GarbageCollector,
) -> Node {
    // Find linked ports to the rz port where the angle will be inputted
    // the port offset of the angle is known to be 1 for the rz gate.
    let (mut prev_node, _) = find_single_linked_output_by_index(hugr, rz_node, 1);

    // As all of the NormalizeGuppy passes have been run on the `hugr` before it enters this function,
    // and these passes include constant folding, we can assume that we can follow the 0th ports back
    // to a constant node where the angle is defined.
    let max_iterations = 10;
    let mut ii = 0;
    let mut path = Vec::new(); // The nodes leading up to the angle_node and the angle_node
    // itself
    loop {
        let (current_node, _) = find_single_linked_output_by_index(hugr, prev_node, 0);
        let op_type = hugr.get_optype(current_node);
        path.push(current_node);

        garbage_collector.add_references(current_node, 1);

        if op_type.is_const() {
            let load_const_node = prev_node;
            let angle_node = current_node;
            // Add references to angle node if this has not already been done
            garbage_collector.infer_references_to_angle(hugr, load_const_node, angle_node);
            // Remove one reference to reflect the fact that we are about to use the angle node
            garbage_collector.remove_references(angle_node, 1);
            // Let garbage collector know what nodes led to the angle node
            garbage_collector
                .path
                .entry(angle_node.index())
                .or_insert(path);
            return angle_node;
        }
        if ii >= max_iterations {
            panic!("Angle finding failed");
        }

        prev_node = current_node;
        ii += 1;
    }
}

fn find_angle(hugr: &mut Hugr, rz_node: Node, garbage_collector: &mut GarbageCollector) -> f64 {
    let angle_node = find_angle_node(hugr, rz_node, garbage_collector);
    let op_type = hugr.get_optype(angle_node);
    let angle_const = op_type.as_const().unwrap();
    let angle_val = &angle_const.value;

    // Handling likely angle formats. Panic if angle is not one of the anticipated formats
    let angle = if let Some(rot) = angle_val.get_custom_value::<ConstRotation>() {
        rot.to_radians()
    } else if let Some(fl) = angle_val.get_custom_value::<ConstF64>() {
        let half_turns = fl.value();
        ConstRotation::new(half_turns).unwrap().to_radians()
    } else {
        panic!("Angle not specified as ConstRotation or ConstF64")
    };

    // We now have what we need to know from the angle node and can remove it from the HUGR if
    // no further references remain to it
    garbage_collector.collect(hugr, angle_node);

    angle
}

fn apply_gridsynth(
    hugr: &mut Hugr,
    epsilon: f64,
    rz_node: Node,
    garbage_collector: &mut GarbageCollector,
) -> String {
    let theta = find_angle(hugr, rz_node, garbage_collector);
    let seed = 1234;
    let verbose = false;
    let up_to_phase = false;
    let mut gridsynth_config =
        config_from_theta_epsilon(theta, epsilon, seed, verbose, up_to_phase);
    let gates = gridsynth_gates(&mut gridsynth_config);
    gates.gates
}

/// Add a gridsynth gate to some previous node, which may or may not be a gridsynth gate,
/// and connect
fn add_gate_and_connect(
    hugr: &mut Hugr,
    prev_node: Node,
    op: hugr::ops::OpType,
    output_node: Node,
    qubit_providing_node: Node, // The node providing qubit to Rz gate
    qubit_providing_port: OutgoingPort, // The output port providing qubit to Rz gate
) -> Node {
    let current_node = hugr.add_node_after(output_node, op);
    let ports: Vec<_> = hugr.node_outputs(prev_node).collect();

    // If the previous node was the qubit_providing_node then it could have multiple
    // outputs (eg, if multi-qubit gate and so need to be explicit about port)
    let src_port = if prev_node.index() == qubit_providing_node.index() {
        qubit_providing_port
    } else {
        // the ops generated by gridsynth are all single input single output gates, so
        // it is safe to assume that there is only one output port
        ports[0]
    };
    let ports: Vec<_> = hugr.node_inputs(current_node).collect();
    let dst_port = ports[0];
    hugr.connect(prev_node, src_port, current_node, dst_port);

    current_node
} // TO DO: reduce number of arguments to this function. Six is too many.

fn replace_rz_with_gridsynth_output(
    hugr: &mut Hugr,
    rz_node: Node,
    gates: &str,
) -> Result<(), GridsynthError> {
    // getting node and output port that gave qubit to Rz gate
    let inputs: Vec<_> = hugr.node_inputs(rz_node).collect();
    let input_port = inputs[0];
    let (qubit_providing_node, qubit_providing_port) =
        hugr.single_linked_output(rz_node, input_port).unwrap();
    let mut prev_node = qubit_providing_node;

    // find output port
    let outputs: Vec<_> = hugr.node_outputs(rz_node).collect();
    let output_port = outputs[0];
    let (next_node, dst_port) = hugr.single_linked_input(rz_node, output_port).unwrap();

    // we have now inferred what we need to know from the Rz node we are replacing and can remove it
    hugr.remove_node(rz_node);

    // recursively adding next gate in gates to prev_node
    for gate in gates.chars() {
        if gate == 'H' {
            prev_node = add_gate_and_connect(
                hugr,
                prev_node,
                TketOp::H.into(),
                next_node,
                qubit_providing_node,
                qubit_providing_port,
            );
        } else if gate == 'S' {
            prev_node = add_gate_and_connect(
                hugr,
                prev_node,
                TketOp::S.into(),
                next_node,
                qubit_providing_node,
                qubit_providing_port,
            );
        } else if gate == 'T' {
            prev_node = add_gate_and_connect(
                hugr,
                prev_node,
                TketOp::T.into(),
                next_node,
                qubit_providing_node,
                qubit_providing_port,
            );
        } else if gate == 'X' {
            prev_node = add_gate_and_connect(
                hugr,
                prev_node,
                TketOp::X.into(),
                next_node,
                qubit_providing_node,
                qubit_providing_port,
            );
        } else if gate == 'W' {
            break; // Ignoring global phases for now.
        } else {
            panic!("The gate {gate} is not supported")
        }
    }
    let ports: Vec<_> = hugr.node_outputs(prev_node).collect();
    // Assuming there were no outgoing ports to begin with when deciding port offset
    let src_port = ports[0];
    hugr.connect(prev_node, src_port, next_node, dst_port);
    hugr.validate()?;
    Ok(())
}

/// Replace an Rz gate with the corresponding gates outputted by gridsynth
pub fn apply_gridsynth_pass(hugr: &mut Hugr, epsilon: f64) -> Result<(), GridsynthError> {
    // Running passes to convert HUGR to standard form
    NormalizeGuppy::default()
        .simplify_cfgs(true)
        .remove_tuple_untuple(true)
        .constant_folding(true)
        .remove_dead_funcs(true)
        .inline_dfgs(true)
        .run(hugr)?;

    let rz_nodes = find_rzs(hugr).unwrap();
    let mut garbage_collector = GarbageCollector {
        references: HashMap::new(),
        path: HashMap::new(),
    };
    for node in rz_nodes {
        let gates = apply_gridsynth(hugr, epsilon, node, &mut garbage_collector);
        replace_rz_with_gridsynth_output(hugr, node, &gates)?;
    }
    Ok(())
}

/// Example error.
#[derive(Debug, derive_more::Display, derive_more::Error)]
#[display("Example error: {message}")]
pub struct ExampleError {
    message: String,
}

// The following tests only check if any errors occur because Selene is challenging to access from the rust
// API. However, Selene simulations in Python versions of the HUGRs in these tests and more complicated HUGRS are
// available at https://github.com/Quantinuum/gridsynth_guppy_demo.git
#[cfg(test)]
mod tests {
    use super::*;

    use crate::extension::bool::bool_type;
    use crate::extension::rotation::ConstRotation;
    use crate::hugr::builder::{Container, DFGBuilder, Dataflow, HugrBuilder};
    use crate::hugr::extension::prelude::qb_t;
    use crate::hugr::ops::Value;
    use crate::hugr::types::Signature;
    use hugr::builder::DataflowHugr;

    fn build_rz_only_circ() -> (Hugr, Node) {
        let theta = 0.64;
        let qb_row = vec![qb_t(); 1];
        let mut h = DFGBuilder::new(Signature::new(qb_row.clone(), qb_row)).unwrap();
        let [q_in] = h.input_wires_arr();

        let constant = h.add_constant(Value::extension(
            ConstRotation::from_radians(theta).unwrap(),
        ));
        let loaded_const = h.load_const(&constant);
        let rz = h.add_dataflow_op(TketOp::Rz, [q_in, loaded_const]).unwrap();
        let _ = h.set_outputs(rz.outputs());
        let mut circ = h.finish_hugr().unwrap();
        circ.validate().unwrap_or_else(|e| panic!("{e}"));
        let rz_nodes = find_rzs(&mut circ).unwrap();
        let rz_node = rz_nodes[0];
        (circ, rz_node)
    }

    fn build_non_trivial_circ() -> Hugr {
        // Defining some angles for Rz gates in radians
        let alpha = 0.23;
        let beta = 1.78;
        let inverse_angle = -alpha - beta;

        // Defining builder for circuit
        let qb_row = vec![qb_t(); 1];
        let meas_row = vec![bool_type(); 1];
        let mut builder =
            DFGBuilder::new(Signature::new(qb_row.clone(), meas_row.clone())).unwrap();
        let [q1] = builder.input_wires_arr();

        // Adding constant wires and nodes
        let alpha_const = builder.add_constant(Value::extension(
            ConstRotation::from_radians(alpha).unwrap(),
        ));
        let loaded_alpha = builder.load_const(&alpha_const);
        let beta_const =
            builder.add_constant(Value::extension(ConstRotation::from_radians(beta).unwrap()));
        let loaded_beta = builder.load_const(&beta_const);
        let inverse_const = builder.add_constant(Value::extension(
            ConstRotation::from_radians(inverse_angle).unwrap(),
        ));
        let loaded_inverse = builder.load_const(&inverse_const);

        // Adding gates and measurements
        let had1 = builder.add_dataflow_op(TketOp::H, [q1]).unwrap();
        let [q1] = had1.outputs_arr();
        let rz_alpha = builder
            .add_dataflow_op(TketOp::Rz, [q1, loaded_alpha])
            .unwrap();
        let [q1] = rz_alpha.outputs_arr();
        let rz_beta = builder
            .add_dataflow_op(TketOp::Rz, [q1, loaded_beta])
            .unwrap();
        let [q1] = rz_beta.outputs_arr();
        let rz_inverse = builder
            .add_dataflow_op(TketOp::Rz, [q1, loaded_inverse])
            .unwrap();
        let [q1] = rz_inverse.outputs_arr();
        let had2 = builder.add_dataflow_op(TketOp::H, [q1]).unwrap();
        let [q1] = had2.outputs_arr();
        let meas_res = builder
            .add_dataflow_op(TketOp::MeasureFree, [q1])
            .unwrap()
            .out_wire(0);

        builder
            .finish_hugr_with_outputs([meas_res])
            .unwrap_or_else(|e| panic!("{e}"))
    }

    fn build_non_trivial_circ_2qubits() -> Hugr {
        // Defining some angles for Rz gates in radians
        let alpha = 0.23;
        let beta = 1.78;
        let inverse_angle = -alpha - beta;

        // Defining builder for circuit
        let qb_row = vec![qb_t(); 2];
        let meas_row = vec![bool_type(); 2];
        let mut builder =
            DFGBuilder::new(Signature::new(qb_row.clone(), meas_row.clone())).unwrap();
        let [q1, q2] = builder.input_wires_arr();

        // Adding constant wires and nodes
        let alpha_const = builder.add_constant(Value::extension(
            ConstRotation::from_radians(alpha).unwrap(),
        ));
        let loaded_alpha = builder.load_const(&alpha_const);
        let beta_const =
            builder.add_constant(Value::extension(ConstRotation::from_radians(beta).unwrap()));
        let loaded_beta = builder.load_const(&beta_const);
        let inverse_const = builder.add_constant(Value::extension(
            ConstRotation::from_radians(inverse_angle).unwrap(),
        ));
        let loaded_inverse = builder.load_const(&inverse_const);

        // Adding gates and measurements
        let had1 = builder.add_dataflow_op(TketOp::H, [q1]).unwrap();
        let [q1] = had1.outputs_arr();
        let rz_alpha = builder
            .add_dataflow_op(TketOp::Rz, [q1, loaded_alpha])
            .unwrap();
        let [q1] = rz_alpha.outputs_arr();
        let rz_beta = builder
            .add_dataflow_op(TketOp::Rz, [q1, loaded_beta])
            .unwrap();
        let [q1] = rz_beta.outputs_arr();
        let x = builder.add_dataflow_op(TketOp::X, [q2]).unwrap();
        let [q2] = x.outputs_arr();
        let cx1 = builder.add_dataflow_op(TketOp::CX, [q2, q1]).unwrap();
        let [q2, q1] = cx1.outputs_arr();
        let rz_inverse = builder
            .add_dataflow_op(TketOp::Rz, [q1, loaded_inverse])
            .unwrap();
        let [q1] = rz_inverse.outputs_arr();
        let cx2 = builder.add_dataflow_op(TketOp::CX, [q2, q1]).unwrap();
        let [q2, q1] = cx2.outputs_arr();
        let had2 = builder.add_dataflow_op(TketOp::H, [q1]).unwrap();
        let [q1] = had2.outputs_arr();
        let meas_res1 = builder
            .add_dataflow_op(TketOp::MeasureFree, [q1])
            .unwrap()
            .out_wire(0);
        let meas_res2 = builder
            .add_dataflow_op(TketOp::MeasureFree, [q2])
            .unwrap()
            .out_wire(0);

        builder
            .finish_hugr_with_outputs([meas_res1, meas_res2])
            .unwrap_or_else(|e| panic!("{e}"))
    }

    #[test]
    fn gridsynth_pass_successful() {
        // This test is just to check if a panic occurs
        let (mut circ, _) = build_rz_only_circ();
        let epsilon: f64 = 1e-3;
        apply_gridsynth_pass(&mut circ, epsilon).unwrap();
    }

    #[test]
    fn test_non_trivial_circ_1qubit() {
        // Due to challenge of accessing Selene from rust, this just
        // tests for if errors occur. It would be nice to have a call to
        // Selene here. (See https://github.com/Quantinuum/gridsynth_guppy_demo.git for a Python example
        // of this circuit working)
        let epsilon = 1e-2;
        let mut hugr = build_non_trivial_circ();

        apply_gridsynth_pass(&mut hugr, epsilon).unwrap();
    }

    #[test]
    fn test_non_trivial_circ_2qubits() {
        // Due to challenge of accessing Selene from rust, this just
        // tests for if errors occur. It would be nice to have a call to
        // Selene here. (See https://github.com/Quantinuum/gridsynth_guppy_demo.git for a Python example
        // of this circuit working)
        let epsilon = 1e-2;
        let mut hugr = build_non_trivial_circ_2qubits();

        apply_gridsynth_pass(&mut hugr, epsilon).unwrap();
    }
}
