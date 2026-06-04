//! Common configurations and helper functions for the [ReplaceTypes] pass.
use hugr::{
    builder::{Container, DFGBuilder, Dataflow, DataflowSubContainer, SubContainer, inout_sig},
    extension::{
        prelude::{option_type, usize_t},
        simple_op::MakeOpDef,
    },
    ops::{ExtensionOp, Tag},
    std_extensions::{
        arithmetic::{conversions::ConvertOpDef, int_ops::IntOpDef, int_types::ConstInt},
        collections::{
            array::{self, ARRAY_CLONE_OP_ID, ARRAY_DISCARD_OP_ID, GenericArrayOpDef, array_type},
            borrow_array::{self, BArrayUnsafeOpDef, BorrowArray, borrow_array_type},
        },
    },
    type_row,
    types::{Term, Type},
};

use crate::{
    extension::guppy::{DROP_OP_NAME, GUPPY_EXTENSION},
    passes::{
        ReplaceTypes,
        replace_types::{Linearizer, NodeTemplate},
    },
};

/// Register any replacements needed to replace array ops that require copyable type
/// bounds with ops that can work linear types.
pub fn replace_array_ops_requiring_copyable_bounds(lowerer: &mut ReplaceTypes) {
    for (array_ext, type_fn) in [
        (
            array::EXTENSION.to_owned(),
            array_type as fn(u64, Type) -> Type,
        ),
        (
            borrow_array::EXTENSION.to_owned(),
            borrow_array_type as fn(u64, Type) -> Type,
        ),
    ] {
        // `Clone` ops get replaced with a DFG that the lineariser can act on.
        lowerer.set_replace_parametrized_op(
            array_ext.get_op(ARRAY_CLONE_OP_ID.as_str()).unwrap(),
            move |args, rt| {
                let [size, elem_ty] = args else {
                    unreachable!()
                };
                let size = size.as_nat().unwrap();
                let elem_ty = elem_ty.as_runtime().unwrap();
                if elem_ty.copyable() {
                    return Ok(None);
                }

                let array_ty = type_fn(size, elem_ty);
                Ok(Some(rt.get_linearizer().copy_discard_op(&array_ty, 2)?))
            },
        );

        // `Discard` ops get replaced with a `Drop` op which can be lowered in the
        // `LowerDropsPass`.
        let drop_op_def = GUPPY_EXTENSION.get_op(DROP_OP_NAME.as_str()).unwrap();
        lowerer.set_replace_parametrized_op(
            array_ext.get_op(ARRAY_DISCARD_OP_ID.as_str()).unwrap(),
            move |args, _| {
                let [size, elem_ty] = args else {
                    unreachable!()
                };
                let size = size.as_nat().unwrap();
                let elem_ty = elem_ty.as_runtime().unwrap();
                if elem_ty.copyable() {
                    return Ok(None);
                }
                let drop_op = ExtensionOp::new(
                    drop_op_def.clone(),
                    vec![type_fn(size, elem_ty.clone()).into()],
                )
                .unwrap();
                Ok(Some(NodeTemplate::SingleOp(drop_op.into())))
            },
        );
    }

    // For borrow arrays, we also replace the `get` op (currently the Guppy compiler
    // doesn't generate `get` ops for standard arrays.) Note we don't need to deal with
    // static arrays as you cannot create static arrays of Measurement values in Guppy.
    fn barray_get_replacement(rt: &ReplaceTypes, size: u64, elem_ty: Type) -> NodeTemplate {
        let array_ty = borrow_array_type(size, elem_ty.clone());
        let opt_el = option_type(vec![elem_ty.clone()]);
        let mut dfb = DFGBuilder::new(inout_sig(
            vec![array_ty.clone(), usize_t()],
            vec![opt_el.clone().into(), array_ty.clone()],
        ))
        .unwrap();
        let [arr_in, idx] = dfb.input_wires_arr();
        let [idx_as_int] = dfb
            .add_dataflow_op(ConvertOpDef::ifromusize.without_log_width(), [idx])
            .unwrap()
            .outputs_arr();
        let bound = dfb.add_load_value(ConstInt::new_u(6, size).unwrap());
        let [is_in_range] = dfb
            .add_dataflow_op(IntOpDef::ilt_u.with_log_width(6), [idx_as_int, bound])
            .unwrap()
            .outputs_arr();
        let mut cb = dfb
            .conditional_builder(
                (vec![type_row![]; 2], is_in_range),
                [(array_ty.clone(), arr_in), (usize_t(), idx)],
                vec![opt_el.clone().into(), array_ty.clone()].into(),
            )
            .unwrap();

        let mut out_of_range = cb.case_builder(0).unwrap();
        let [arr_in, _] = out_of_range.input_wires_arr();
        let [none] = out_of_range
            .add_dataflow_op(
                Tag::new(0, vec![type_row![], vec![elem_ty.clone()].into()]),
                [],
            )
            .unwrap()
            .outputs_arr();
        out_of_range.finish_with_outputs([none, arr_in]).unwrap();

        let mut in_range = cb.case_builder(1).unwrap();
        let [arr_in, idx] = in_range.input_wires_arr();
        let [arr, elem] = in_range
            .add_dataflow_op(
                BArrayUnsafeOpDef::borrow.to_concrete(elem_ty.clone(), size),
                [arr_in, idx],
            )
            .unwrap()
            .outputs_arr();

        let [elem1, elem2] = rt
            .get_linearizer()
            .copy_discard_op(&elem_ty, 2)
            .unwrap()
            .add(&mut in_range, [elem])
            .unwrap()
            .outputs_arr();

        let [arr] = in_range
            .add_dataflow_op(
                BArrayUnsafeOpDef::r#return.to_concrete(elem_ty.clone(), size),
                [arr, idx, elem1],
            )
            .unwrap()
            .outputs_arr();
        let [some] = in_range
            .add_dataflow_op(
                Tag::new(1, vec![type_row![], vec![elem_ty].into()]),
                [elem2],
            )
            .unwrap()
            .outputs_arr();
        in_range.finish_with_outputs([some, arr]).unwrap();

        let outs = cb.finish_sub_container().unwrap().outputs();
        // Do not finish DFG: it contains "invalid" copy_dfg that needs linearizing
        dfb.set_outputs(outs).unwrap();
        let h = std::mem::take(dfb.hugr_mut());
        NodeTemplate::CompoundOp(Box::new(h))
    }

    lowerer.set_replace_parametrized_op(
        borrow_array::EXTENSION
            .get_op(GenericArrayOpDef::<BorrowArray>::get.opdef_id().as_str())
            .unwrap(),
        |args, rt| {
            let [Term::BoundedNat(size), Term::Runtime(elem_ty)] = args else {
                unreachable!()
            };
            if elem_ty.copyable() {
                return Ok(None);
            }
            Ok(Some(barray_get_replacement(rt, *size, elem_ty.clone())))
        },
    );
}
