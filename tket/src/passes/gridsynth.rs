use hugr_core:: OutgoingPort;
use rsgridsynth::config::config_from_theta_epsilon;
use rsgridsynth::gridsynth::gridsynth_gates;
use crate::extension::rotation::ConstRotation;
use crate::{Hugr, hugr, op_matches};
use crate::hugr::builder::{DFGBuilder, Dataflow, HugrBuilder};
use crate::hugr::extension::prelude::{qb_t};
use crate::hugr::HugrView;
use crate::hugr::hugr::hugrmut::HugrMut;
use crate::hugr::{Node, Port};
use crate::hugr::types::Signature;
use crate::TketOp;
use hugr::algorithms::ComposablePass;
use hugr::std_extensions::arithmetic::float_types::ConstF64;
use crate::passes::guppy::NormalizeGuppy;


/// Find the nodes for the Rz gates.
fn find_rzs(hugr: &mut Hugr) -> Option<Vec<hugr::Node>> {
    let mut rz_nodes: Vec<Node> = Vec::new();  
    for node in hugr.nodes() {
        let op_type = hugr.get_optype(node);
        if op_matches(op_type, TketOp::Rz) {
            rz_nodes.push(node);
            // return Some(node);
        }
    }

    // if there are rz_nodes:
    if !(rz_nodes.is_empty()) {
        return Some(rz_nodes);
    }
    // else return None
    None
}


// TO DO: extend this function to find all RZ gates

fn find_linked_incoming_ports(hugr: &mut Hugr, node: Node, port_idx: usize) -> Vec<(Node, Port)> {
    let ports = hugr.node_inputs(node);
    let collected_ports: Vec<_> = ports.collect();
    let linked_ports = hugr.
        linked_ports(node, collected_ports[port_idx]);
    let linked_ports: Vec<(Node, Port)> = linked_ports.collect();
    linked_ports
}

/// find the output port and node linked to the input specified by `port_idz` for `node`
fn find_single_linked_output_by_index(hugr: &mut Hugr, node: Node, port_idx: usize) -> (Node, OutgoingPort) {
    let ports = hugr.node_inputs(node);
    let collected_ports: Vec<_> = ports.collect();
    let prev_node_and_port = hugr.
        single_linked_output(node, collected_ports[port_idx]).unwrap();
    prev_node_and_port
}


/// Find the constant node containing the angle to be inputted to the Rz gate.
/// It is assumed that `hugr` has had the NormalizeGuppy passes applied to it 
/// prior to being applied. This function also cleans up behind itself removing 
/// everything on the path to the `angle_node` but not the `angle_node` itself,
/// which is still needed.
fn find_angle_node(hugr: &mut Hugr, rz_node: Node) -> Node {
    // find linked ports to the rz port where the angle will be inputted
    // the port offset of the angle is known to be 1 for the rz gate.
    let (mut prev_node, _) = find_single_linked_output_by_index(hugr, rz_node, 1);

    // as all of the NormalizeGuppy passes have been run on the `hugr` before it enters this function,
    // and these passes include constant folding, we can assume that we can follow the 0th ports back
    // to a constant node where the angle is defined.
    let max_iterations = 10;
    let mut ii = 0;
    loop {
        let (current_node, _) = find_single_linked_output_by_index(hugr, prev_node, 0);
        let op_type = hugr.get_optype(current_node);
        if op_type.is_const() {
            hugr.remove_node(prev_node);
            let angle_node = current_node;
            return angle_node;
        }
        if ii >= max_iterations {
            panic!("Angle finding failed"); // TO DO: improve error handling
        }

        // Deleting all but nodes on the way to the rz_node but not the rz_node itself
        hugr.remove_node(prev_node);

        prev_node = current_node;
        ii += 1;

    }
}

fn find_angle(hugr: &mut Hugr, rz_node: Node) -> f64 {
    let angle_node = find_angle_node(hugr, rz_node);
    let op_type = hugr.get_optype(angle_node);
    let angle_const = op_type.as_const().unwrap();
    let angle_val = &angle_const.value;

    // handling likely angle formats. Panic if angle is not one of the anticipated formats
    let angle = if let Some(rot) = angle_val.get_custom_value::<ConstRotation>() {
      rot.to_radians()
    } else if let Some(fl) = angle_val.get_custom_value::<ConstF64>() {
      let half_turns = fl.value();
      ConstRotation::new(half_turns).unwrap().to_radians()
    } else {
        panic!("Angle not specified as ConstRotation or ConstF64")
    };
    
    // we now have what we need to know from the angle node and can remove it from the hugr 
    hugr.remove_node(angle_node);

    angle
}

fn apply_gridsynth(hugr: &mut Hugr, epsilon: f64, rz_node: Node) -> String {
    let theta = find_angle(hugr, rz_node);
    // The following parameters could be made user-specifiable. For simplicity, I fix them, for now
    // let epsilon = 1e-1; // very low precision to allow easier visualisation for demo
    let seed = 1234;
    let verbose = false;
    let up_to_phase = false;
    let mut gridsynth_config = config_from_theta_epsilon(theta, epsilon, seed, verbose, up_to_phase);
    let gates = gridsynth_gates(&mut gridsynth_config);
    let gates = gates.gates;
    gates    
}


fn gridsynth_output_to_hugr(gates: &str) -> Hugr {
    let qb_row = vec![qb_t(); 1]; // TO CHECK: will it cause issues to insert new qubit wire?
    let mut h = DFGBuilder::new(Signature::new(qb_row.clone(), qb_row)).unwrap();

    let mut prev_op = h.input(); 
    for gate in gates.chars() {
        if gate == 'H' {
            prev_op = h.add_dataflow_op(TketOp::H, prev_op.outputs()).unwrap();
        }
        else if gate == 'S' {
            prev_op = h.add_dataflow_op(TketOp::S, prev_op.outputs()).unwrap();
        }
        else if gate == 'T' {
            prev_op = h.add_dataflow_op(TketOp::T, prev_op.outputs()).unwrap();
        }
        else if gate == 'W' {
            break; // Ignoring global phases for now.
        }
    }
    h.set_outputs(prev_op.outputs()).unwrap();
    let hugr = h.finish_hugr().unwrap();
    hugr.validate().unwrap_or_else(|e| panic!("{e}"));
    hugr
}


/// get previous node that provided qubit to Rz gate
fn find_qubit_source(hugr: &mut Hugr, rz_node: Node) -> Node {
    let linked_ports = find_linked_incoming_ports(hugr, rz_node, 0);
    let prev_node = linked_ports[0].0;
    prev_node
}

/// Add a gridsynth gate to some previous node, which may or may not be a gridsynth gate, 
/// and connect
fn add_gate_and_connect(hugr: &mut Hugr, prev_node: Node, op: hugr::ops::OpType, output_node: Node) -> Node {
    let current_node =  hugr.add_node_after(output_node, op);
    let ports:  Vec<_> = hugr.node_outputs(prev_node).collect();
    // Assuming there were no outgoing ports to begin with when deciding port offset
    let src_port = ports[0];
    let ports:  Vec<_> = hugr.node_inputs(current_node).collect();
    let dst_port = ports[0];
    hugr.connect(prev_node, src_port, current_node, dst_port);
    let prev_node = current_node;
    prev_node
}


fn replace_rz_with_gridsynth_output(hugr: &mut Hugr, rz_node: Node, gates: &str) {
    // getting node that gave qubit to Rz gate
    let mut prev_node = find_qubit_source(hugr, rz_node);

    // find output port
    let outputs: Vec<_> = hugr.node_outputs(rz_node).collect();
    let output_port = outputs[0];
    let (next_node, _) = hugr
                                    .single_linked_input(rz_node, output_port)
                                    .unwrap();

    // we have now inferred what we need to know from the Rz node we are replacing and can remove it
    hugr.remove_node(rz_node);

    // println!("in panicking function: {}", hugr.mermaid_string());

    // recursively adding next gate in gates to prev_node
    for gate in gates.chars() {
        if gate == 'H' {
            prev_node = add_gate_and_connect(hugr, prev_node, TketOp::H.into(), next_node);
        }
        else if gate == 'S' {
            prev_node = add_gate_and_connect(hugr, prev_node, TketOp::S.into(), next_node);
        }
        else if gate == 'T' {
            prev_node = add_gate_and_connect(hugr, prev_node, TketOp::T.into(), next_node);
        }
        else if gate == 'W' {
            break; // Ignoring global phases for now.
        }
    }
    let ports:  Vec<_> = hugr.node_outputs(prev_node).collect();
    // Assuming there were no outgoing ports to begin with when deciding port offset
    let src_port = ports[0];
    let ports:  Vec<_> = hugr.node_inputs(next_node).collect();
    let dst_port = ports[0];
    hugr.connect(prev_node, src_port, next_node, dst_port);
    // println!("Inside panicking function: {}", hugr.mermaid_string());
    hugr.validate().unwrap_or_else(|e| panic!("{e}"));
} 

/// Replace an Rz gate with the corresponding gates outputted by gridsynth
pub fn apply_gridsynth_pass(hugr: &mut Hugr, epsilon: f64) {
    // Running passes to convert HUGR to standard form
    NormalizeGuppy::default()
        .simplify_cfgs(true)
        .remove_tuple_untuple(true)
        .constant_folding(true)
        .remove_dead_funcs(true)
        .inline_dfgs(true)
        .run(hugr)
        .unwrap();

    let rz_nodes = find_rzs(hugr).unwrap();
    for node in rz_nodes {
        let gates = apply_gridsynth(hugr, epsilon, node);
        replace_rz_with_gridsynth_output(hugr, node, &gates);
    }
} // TO DO: change name of this function to gridsynth



/// Example error.
#[derive(Debug, derive_more::Display, derive_more::Error)]
#[display("Example error: {message}")]
pub struct ExampleError {
    message: String,
}

// Test of example function
#[cfg(test)]
mod tests {
    use std::fs::File;
    use std::io::BufReader;

    use super::*;

    use hugr::NodeIndex;
    use hugr_core::std_extensions::std_reg;
    use crate::hugr::builder::Container;
    use crate::hugr::envelope::read_described_envelope;
    use crate::hugr::ops::Value;
    use crate::extension::rotation::ConstRotation;

    fn build_rz_only_circ() -> (Hugr, Node) {
        let theta = 0.64;
        let qb_row = vec![qb_t(); 1];
        let mut h = DFGBuilder::new(Signature::new(qb_row.clone(), qb_row)).unwrap();
        let [q_in] = h.input_wires_arr();

        let constant = h.add_constant(Value::extension(ConstRotation::from_radians(theta).unwrap()));
        let loaded_const = h.load_const(&constant);
        let rz = h.add_dataflow_op(TketOp::Rz, [q_in, loaded_const]).unwrap();
        let _ = h.set_outputs(rz.outputs());
        let mut circ = h.finish_hugr().unwrap(); //(rz.outputs()).unwrap().into();
        println!("First mermaid string is: {}", circ.mermaid_string());
        circ.validate().unwrap_or_else(|e| panic!("{e}"));
        let rz_nodes = find_rzs(&mut circ).unwrap();
        let rz_node = rz_nodes[0];
        (circ, rz_node)
    }

    fn import_guppy(path: &'static str) -> Hugr {
        let f = File::open(path).unwrap();
        let reader = BufReader::new(f);
        let registry = std_reg();
        let (_, imported_package) = read_described_envelope(reader, &registry).unwrap();
        let imported_hugr = imported_package.modules[0].clone();
        imported_hugr
    }

    fn import_rz_only_guppy_circuit() -> Hugr {
        // TODO: update the following path
        import_guppy("/home/kennycampbell/coding_projects/tket_gridsynth_ext/guppy_rz")
    }

    fn import_2rzs_guppy() -> Hugr {
        import_guppy("/home/kennycampbell/coding_projects/tket_gridsynth_ext/guppy_2Rzs")
    }


    #[test]
    fn gridsynth_pass_successful() {
        // This test is just to check if a panic occurs
        let (mut circ, _) = build_rz_only_circ();
        let epsilon: f64 = 1e-3;
        // let gates = apply_gridsynth(&mut circ, epsilon);
        // println!("{}", &gates);        
        apply_gridsynth_pass(&mut circ, epsilon);
        // println!("{}", circ.mermaid_string());
    }

    #[test]
    fn test_gridsynth_output_to_hugr() {
        let epsilon = 1e-3;
        let (mut circ, rz_node) = build_rz_only_circ();
        let gates = apply_gridsynth(&mut circ, epsilon, rz_node);
        gridsynth_output_to_hugr(&gates);
    }

    #[test]
    fn found_rz_guppy() {
        let imported_hugr = &mut import_rz_only_guppy_circuit();
        NormalizeGuppy::default()
            .simplify_cfgs(true)
            .remove_tuple_untuple(true)
            .constant_folding(true)
            .remove_dead_funcs(true)
            .inline_dfgs(true)
            .run(imported_hugr)
            .unwrap();
        let rz_nodes = find_rzs(imported_hugr).unwrap();
        let rz_node = rz_nodes[0];
        assert_eq!(rz_node.index(), 17)
    }

    #[test]
    fn test_find_angle_node_for_guppy() {
        let imported_hugr = &mut import_rz_only_guppy_circuit();
        NormalizeGuppy::default()
            .simplify_cfgs(true)
            .remove_tuple_untuple(true)
            .constant_folding(true)
            .remove_dead_funcs(true)
            .inline_dfgs(true)
            .run(imported_hugr)
            .unwrap();
        let rz_nodes = find_rzs(imported_hugr).unwrap();
        let rz_node = rz_nodes[0];
        let angle_node = find_angle_node(imported_hugr, rz_node);
        assert_eq!(angle_node.index(), 20);
    }

    #[test]
    fn test_find_angle_for_guppy() {
        let imported_hugr = &mut import_rz_only_guppy_circuit();
        NormalizeGuppy::default()
            .simplify_cfgs(true)
            .remove_tuple_untuple(true)
            .constant_folding(true)
            .remove_dead_funcs(true)
            .inline_dfgs(true)
            .run(imported_hugr)
            .unwrap();
        let rz_nodes = find_rzs(imported_hugr).unwrap();
        let rz_node = rz_nodes[0];
        let angle = find_angle(imported_hugr, rz_node);
        println!("angle is {}", angle);
        println!("hugr is {}", imported_hugr.mermaid_string());
    }

    #[test]
    fn test_with_guppy_hugr() {
        let epsilon = 1e-2;
        let mut imported_hugr = &mut import_rz_only_guppy_circuit();
        NormalizeGuppy::default()
            .simplify_cfgs(true)
            .remove_tuple_untuple(true)
            .constant_folding(true)
            .remove_dead_funcs(true)
            .inline_dfgs(true)
            .run(imported_hugr)
            .unwrap();
        // constant_fold_pass(imported_hugr.as_mut());
        // println!("after {}", imported_hugr.mermaid_string());
        apply_gridsynth_pass(&mut imported_hugr, epsilon);
        println!("after: {}", imported_hugr.mermaid_string());
    }

    #[test]
    fn test_2rzs_guppy_hugr() {
        let epsilon = 1e-2;
        let mut imported_hugr = &mut import_2rzs_guppy();
        println!("before: {}", imported_hugr.mermaid_string());
        NormalizeGuppy::default()
            .simplify_cfgs(true)
            .remove_tuple_untuple(true)
            .constant_folding(true)
            .remove_dead_funcs(true)
            .inline_dfgs(true)
            .run(imported_hugr)
            .unwrap();
        // constant_fold_pass(imported_hugr.as_mut());
        // println!("after {}", imported_hugr.mermaid_string());
        apply_gridsynth_pass(&mut imported_hugr, epsilon);
        println!("after: {}", imported_hugr.mermaid_string());
    }
}
