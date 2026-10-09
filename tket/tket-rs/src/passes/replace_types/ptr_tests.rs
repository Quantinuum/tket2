use super::*;
use hugr_core::builder::{DFGBuilder, DataflowSubContainer, ModuleBuilder, inout_sig};
use hugr_core::extension::prelude::{bool_t, option_type, qb_t, usize_t};
use hugr_core::std_extensions::arithmetic::int_types::int_type;
use hugr_core::std_extensions::collections::array::GenericArrayOpDef;
use hugr_core::std_extensions::collections::{
    array::array_type,
    borrow_array::{BorrowArray, borrow_array_type},
};
use hugr_core::std_extensions::ptr::{PtrOpBuilder, ptr_type};
use hugr_core::types::FuncValueType;

#[test]
fn complete_replacement_preserves_payload_and_traverses_original_types() {
    let target = ptr_type(qb_t());
    let nested = |ty: Type| {
        Type::new_tuple([
            ty.clone(),
            array_type(2, ty.clone()),
            borrow_array_type(3, ty.clone()),
            Type::new_function(FuncValueType::new([ty.clone()], [ty.clone()])),
            ptr_type(ty),
        ])
    };
    let mut ty = nested(qb_t());
    let mut pass = ReplaceTypes::default();
    pass.set_replace_type_with_options(
        qb_t().as_extension().unwrap().clone(),
        target.clone(),
        ReplacementOptions::default(),
    );
    assert!(ty.transform(&pass).unwrap());
    assert_eq!(ty, nested(target));
}

#[test]
fn recursive_setter_keeps_existing_transitive_behavior() {
    let mut pass = ReplaceTypes::default();
    pass.set_replace_type(qb_t().as_extension().unwrap().clone(), ptr_type(usize_t()));
    pass.set_replace_type(usize_t().as_extension().unwrap().clone(), int_type(6));
    let mut ty = qb_t();
    ty.transform(&pass).unwrap();
    assert_eq!(ty, ptr_type(int_type(6)));
}

#[test]
fn original_pointer_map_and_eq_signatures_are_reparameterized_once() {
    let mut mb = ModuleBuilder::new();
    let cb = mb
        .define_function("callback", Signature::new_endo([qb_t()]))
        .unwrap();
    let inputs = cb.input_wires();
    let callback = cb.finish_with_outputs(inputs).unwrap();
    let mut b = mb
        .define_function(
            "main",
            Signature::new(
                [ptr_type(qb_t()), ptr_type(qb_t())],
                [ptr_type(qb_t()), ptr_type(qb_t()), bool_t()],
            ),
        )
        .unwrap();
    let [left, right] = b.input_wires_arr();
    let f = b.load_func(callback.handle(), &[]).unwrap();
    let (left, _) = b.add_map_ptr(left, f, qb_t(), [], []).unwrap();
    let (left, right, equal) = b.add_eq_ptr(left, right, qb_t()).unwrap();
    b.finish_with_outputs([left, right, equal]).unwrap();
    let mut graph = mb.finish_hugr().unwrap();
    let mut pass = ReplaceTypes::default();
    pass.set_replace_type_with_options(
        qb_t().as_extension().unwrap().clone(),
        ptr_type(qb_t()),
        ReplacementOptions::default(),
    );
    assert!(pass.run(&mut graph).unwrap());
    graph.validate().unwrap();
    for op in graph
        .nodes()
        .filter_map(|n| graph.get_optype(n).as_extension_op())
    {
        if op.def().extension_id() == &hugr_core::std_extensions::ptr::EXTENSION_ID {
            assert_eq!(op.args()[0], ptr_type(qb_t()).into());
        }
    }
}

#[rstest::rstest]
#[case(ptr_type(int_type(6)))]
#[case(Type::new_tuple([ptr_type(int_type(6)), usize_t()]))]
fn borrowed_array_get_copies_final_handles_and_returns_original(#[case] target: Type) {
    let arr = borrow_array_type(2, int_type(6));
    let mut b = DFGBuilder::new(inout_sig(
        [arr.clone(), usize_t()],
        [Type::from(option_type([int_type(6)])), arr],
    ))
    .unwrap();
    let inputs = b.input_wires();
    let get = b
        .add_dataflow_op(
            GenericArrayOpDef::<BorrowArray>::get.to_concrete(int_type(6), 2),
            inputs,
        )
        .unwrap();
    let mut graph = b.finish_hugr_with_outputs(get.outputs()).unwrap();
    let mut pass = ReplaceTypes::default();
    pass.set_replace_type_with_options(
        int_type(6).as_extension().unwrap().clone(),
        target.clone(),
        ReplacementOptions::default(),
    );
    handlers::register_linear_array_op_replacements(&mut pass);
    assert!(pass.run(&mut graph).unwrap());
    graph.validate().unwrap();
    let signature = graph.signature(graph.entrypoint()).unwrap();
    assert_eq!(
        signature.output(),
        &TypeRow::from([
            Type::from(option_type([target.clone()])),
            borrow_array_type(2, target)
        ])
    );
    let ops = graph
        .nodes()
        .filter_map(|n| graph.get_optype(n).as_extension_op())
        .collect::<Vec<_>>();
    assert_eq!(
        ops.iter()
            .filter(|op| op.qualified_id() == "ptr.Dup")
            .count(),
        1
    );
    assert!(
        !ops.iter()
            .any(|op| ["ptr.Read", "ptr.New", "ptr.Free"].contains(&op.qualified_id().as_str()))
    );
}

#[rstest::rstest]
#[case(0)]
#[case(2)]
#[case(3)]
fn pointer_linearizer_copies_handles_or_requires_typed_disposal(#[case] outputs: usize) {
    let ty = ptr_type(qb_t());
    let mut b = DFGBuilder::new(inout_sig([ty.clone()], vec![ty.clone(); outputs])).unwrap();
    let template = DelegatingLinearizer::default().copy_discard_op(&ty, outputs);
    if outputs == 0 {
        assert_eq!(template, Err(LinearizeError::NeedCopyDiscard(Box::new(ty))));
        return;
    }
    let template = template.unwrap();
    let inputs = b.input_wires();
    let op = template.add(&mut b, inputs).unwrap();
    let graph = b.finish_hugr_with_outputs(op.outputs()).unwrap();
    graph.validate().unwrap();
    let ops = graph
        .nodes()
        .filter_map(|n| graph.get_optype(n).as_extension_op())
        .collect::<Vec<_>>();
    assert_eq!(ops.len(), outputs - 1);
    assert!(
        ops.iter()
            .all(|op| op.qualified_id() == "ptr.Dup" && op.args() == [qb_t().into()])
    );
}
