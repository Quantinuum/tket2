//! Construct guarded tail loops for runtime power modifiers.

use hugr::{
    IncomingPort, Wire,
    core::HugrNode,
    extension::prelude::bool_t,
    hugr::hugrmut::HugrMut,
    ops::{
        CallIndirect, Case, Conditional, Const, Input, LoadConstant, OpType, Output, TailLoop,
        Value,
    },
    std_extensions::arithmetic::{
        int_ops::{self, IntOpDef},
        int_types::{self, ConstInt, int_type},
    },
    type_row,
    types::Signature,
};
use itertools::Itertools;

use super::ResolvedModification;

const EXPONENT_LOG_WIDTH: u8 = 6;

/// Replace a powered indirect call with a conditional containing a tail loop.
pub(super) fn replace_with_power_loop<N: HugrNode>(
    hugr: &mut impl HugrMut<Node = N>,
    indirect_call_node: N,
    resolved: &ResolvedModification<N>,
) {
    let exponent_wires = &resolved.exponent_wires;
    let exponent_wire = multiply_exponents(hugr, indirect_call_node, exponent_wires);

    // NICOLA: COMMENT PR: A.i. suggestion
    // Ensure that the integer extensions are available
    hugr.use_extension(int_types::EXTENSION.clone());
    hugr.use_extension(int_ops::EXTENSION.clone());

    let OpType::CallIndirect(indirect_call_op) = hugr.get_optype(indirect_call_node).clone() else {
        unreachable!("expected CallIndirect");
    };

    // NICOLA: todo: move these variables as below as you can, to leave them closer their first use
    let fun_signature = indirect_call_op.signature.clone();
    // we add the exponent input as last input
    let argument_count = fun_signature.input_count();
    // in the tail loop we pass the function inputs + exponent as inout args
    let loop_inout = fun_signature
        .input
        .extend([&int_type(u64::from(EXPONENT_LOG_WIDTH))]);
    let parent = hugr.get_parent(indirect_call_node).unwrap();

    // Build the external conditional:
    // - if the exponent is > 0 then we run the tail loop representing the powered function
    // - else we skip it
    let is_positive = check_positive(hugr, parent, exponent_wire);
    let conditional = hugr.add_node_after(
        is_positive.node(),
        Conditional {
            sum_rows: vec![type_row![], type_row![]],
            other_inputs: loop_inout.clone(),
            outputs: fun_signature.output.clone(),
        },
    );
    // wire the function inputs to the conditional
    hugr.connect(is_positive.node(), is_positive.source(), conditional, 0);
    for i in 0..argument_count {
        let wire = Wire::from_connected_port(indirect_call_node, IncomingPort::from(i + 1), hugr);
        hugr.connect(wire.node(), wire.source(), conditional, i + 1);
    }
    // wire the exponent
    hugr.connect(
        exponent_wire.node(),
        exponent_wire.source(),
        conditional,
        argument_count + 1,
    );

    let case_signature = Signature::new(loop_inout.clone(), fun_signature.output.clone());

    // branch 0 is the skip case, simply pass inputs to outputs
    let (_, skip_input, skip_output) = add_case(hugr, conditional, &case_signature);
    for i in 0..fun_signature.output_count() {
        hugr.connect(skip_input, i, skip_output, i);
    }

    // branch 1 is the loop case
    let (repeat_case, repeat_input, repeat_output) = add_case(hugr, conditional, &case_signature);
    let loop_node = build_tail_loop(hugr, repeat_case, repeat_input, &indirect_call_op, resolved);
    // connect loop outputs or skip outputs
    for i in 0..fun_signature.output_count() {
        hugr.connect(loop_node, i, repeat_output, i);
    }

    // disconect the indirect call outputs and reconnect to conditional
    for out_port in hugr.node_outputs(indirect_call_node).collect_vec() {
        for (target, target_port) in hugr
            .linked_inputs(indirect_call_node, out_port)
            .collect_vec()
        {
            hugr.disconnect(target, target_port);
            hugr.connect(conditional, out_port, target, target_port);
        }
    }

    hugr.remove_node(indirect_call_node);

    std::fs::write("power_res.mmd", hugr.mermaid_string()).unwrap();
    println!("±±±±±±±±±±±±±±±§§§§§§§§§§§§§§§§§§§±±±±±±±±±±±±±±");
}

/// Combine the exponents of a nonempty chain of power modifiers.
fn multiply_exponents<N: HugrNode>(
    hugr: &mut impl HugrMut<Node = N>,
    indirect_call_node: N,
    exponent_wires: &[Wire<N>],
) -> Wire<N> {
    let mut exponent_wire = exponent_wires[0];
    for wire in &exponent_wires[1..] {
        let multiply = hugr.add_node_before(
            indirect_call_node,
            IntOpDef::imul.with_log_width(EXPONENT_LOG_WIDTH),
        );
        hugr.connect(exponent_wire.node(), exponent_wire.source(), multiply, 0);
        hugr.connect(wire.node(), wire.source(), multiply, 1);
        exponent_wire = Wire::new(multiply, 0);
    }
    exponent_wire
}

/// Add a case in the conditional, given its signature.
fn add_case<N: HugrNode>(
    hugr: &mut impl HugrMut<Node = N>,
    conditional: N,
    signature: &Signature,
) -> (N, N, N) {
    let case = hugr.add_node_with_parent(
        conditional,
        Case {
            signature: signature.clone(),
        },
    );
    let case_input = hugr.add_node_with_parent(
        case,
        Input {
            types: signature.input.clone(),
        },
    );
    let case_output = hugr.add_node_with_parent(
        case,
        Output {
            types: signature.output.clone(),
        },
    );
    (case, case_input, case_output)
}

/// Build the tail loop that corresponds to powering the function call
fn build_tail_loop<N: HugrNode>(
    hugr: &mut impl HugrMut<Node = N>,
    case: N,
    case_input: N,
    indirect_call_op: &CallIndirect,
    resolved: &ResolvedModification<N>,
) -> N {
    let modified_fn = resolved.modified_fn;
    let new_load_op = &resolved.load_op;
    let argument_count = indirect_call_op.signature.input_count();
    // In the tail loop we pass the function inputs + exponent as inout args
    let loop_inout = indirect_call_op
        .signature
        .input
        .extend([&int_type(u64::from(EXPONENT_LOG_WIDTH))]);
    let loop_node = hugr.add_node_with_parent(
        case,
        TailLoop {
            just_inputs: type_row![],
            just_outputs: type_row![],
            rest: loop_inout.clone(),
        },
    );
    for i in 0..=argument_count {
        hugr.connect(case_input, i, loop_node, i);
    }
    let loop_input = hugr.add_node_with_parent(
        loop_node,
        Input {
            types: loop_inout.clone(),
        },
    );
    // the output of the loop is the selector plus inout
    let loop_selector = hugr::types::TypeRow::from(vec![bool_t()]);
    let loop_body_output = loop_selector.extend(loop_inout.iter());
    let loop_output = hugr.add_node_with_parent(
        loop_node,
        Output {
            types: loop_body_output.clone(),
        },
    );

    // load and indirectly call the modified function
    let new_load_node = hugr.add_node_with_parent(loop_node, new_load_op.clone());
    hugr.connect(modified_fn, 0, new_load_node, 0);
    let call = hugr.add_node_after(new_load_node, indirect_call_op.clone());
    hugr.connect(new_load_node, 0, call, 0);
    // all inputs apart from exponent go to call
    for i in 0..argument_count {
        hugr.connect(loop_input, i, call, i + 1);
    }

    // insert the exponent decrement
    let load_one = load_power_integer(hugr, loop_node, 1);
    let decrement = hugr.add_node_after(
        load_one.node(),
        IntOpDef::isub.with_log_width(EXPONENT_LOG_WIDTH),
    );
    hugr.connect(loop_input, argument_count, decrement, 0);
    hugr.connect(load_one.node(), load_one.source(), decrement, 1);

    // compare the exponent with zero
    let should_repeat = check_positive(hugr, loop_node, Wire::new(decrement, 0));

    // the tail loop output takes the compare result followed by the call output + exponent
    hugr.connect(should_repeat.node(), should_repeat.source(), loop_output, 0);
    for i in 0..argument_count {
        hugr.connect(call, i, loop_output, i + 1);
    }
    hugr.connect(decrement, 0, loop_output, argument_count + 1);

    loop_node
}

/// Check whether a signed runtime power exponent is greater than zero.
fn check_positive<N: HugrNode>(
    hugr: &mut impl HugrMut<Node = N>,
    parent: N,
    exponent: Wire<N>,
) -> Wire<N> {
    // Load 0 constant
    let load_zero = load_power_integer(hugr, parent, 0);
    let compare = hugr.add_node_after(
        load_zero.node(),
        IntOpDef::igt_s.with_log_width(EXPONENT_LOG_WIDTH),
    );
    hugr.connect(exponent.node(), exponent.source(), compare, 0);
    hugr.connect(load_zero.node(), load_zero.source(), compare, 1);
    Wire::new(compare, 0)
}

/// Load a signed integer with the width used by runtime power exponents.
fn load_power_integer<N: HugrNode>(
    hugr: &mut impl HugrMut<Node = N>,
    parent: N,
    value: i64,
) -> Wire<N> {
    let constant_node = hugr.add_node_with_parent(
        parent,
        Const::new(Value::extension(
            ConstInt::new_s(EXPONENT_LOG_WIDTH, value).unwrap(),
        )),
    );
    let load_constant_op = LoadConstant {
        datatype: int_type(u64::from(EXPONENT_LOG_WIDTH)),
    };
    let constant_port = load_constant_op.constant_port();
    let loaded_constant = hugr.add_node_after(constant_node, load_constant_op);
    hugr.connect(constant_node, 0, loaded_constant, constant_port);
    Wire::new(loaded_constant, 0)
}
