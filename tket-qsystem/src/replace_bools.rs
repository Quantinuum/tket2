//! Provides a `ReplaceBoolPass` which replaces the tket.bool type and
//! lazifies measure operations.
mod static_array;

use derive_more::{Display, Error, From};
use hugr::algorithms::replace_types::{NodeTemplate, ReplaceTypesError, ReplacementOptions};
use hugr::algorithms::{
    ComposablePass, ReplaceTypes, ensure_no_nonlocal_edges, non_local::FindNonLocalEdgesError,
};
use hugr::builder::{
    BuildHandle, Container, DFGBuilder, Dataflow, DataflowHugr, DataflowSubContainer, SubContainer,
    inout_sig,
};
use hugr::extension::prelude::{bool_t, option_type, qb_t, usize_t};
use hugr::extension::simple_op::{MakeOpDef, MakeRegisteredOp};
use hugr::ops::{ExtensionOp, Tag, Value, handle::ConditionalID};
use hugr::std_extensions::arithmetic::{
    conversions::ConvertOpDef, int_ops::IntOpDef, int_types::ConstInt,
};
use hugr::std_extensions::collections::{
    array::{self, ARRAY_CLONE_OP_ID, ARRAY_DISCARD_OP_ID, GenericArrayOpDef, array_type},
    borrow_array::{self, BArrayUnsafeOpDef, BorrowArray, borrow_array_type},
};
use hugr::std_extensions::logic::LogicOp;
use hugr::types::{SumType, Term, Type};
use hugr::{Hugr, HugrView, Node, Wire, hugr::hugrmut::HugrMut, type_row};
use static_array::{ReplaceStaticArrayBoolPass, ReplaceStaticArrayBoolPassError};
use tket::TketOp;
use tket::extension::{
    bool::{BoolOp, ConstBool, bool_type},
    guppy::{DROP_OP_NAME, GUPPY_EXTENSION},
};

use crate::extension::{
    futures::{FutureOp, FutureOpBuilder, FutureOpDef, future_type},
    qsystem::QSystemOp,
};

#[derive(Error, Debug, Display, From)]
#[non_exhaustive]
/// An error reported from [ReplaceBoolPass].
pub enum ReplaceBoolPassError<N> {
    /// The HUGR was found to contain non-local edges.
    NonLocalEdgesError(FindNonLocalEdgesError<N>),
    /// There was an error while replacing the type.
    ReplacementError(ReplaceTypesError),
    /// There was an error while transforming static arrays containing tket.bool
    /// to static arrays of bool_t.
    ReplaceStaticArrayBoolPassError(ReplaceStaticArrayBoolPassError),
}

/// A HUGR -> HUGR pass which replaces the `tket.bool`, enabling lazifying of measure
/// operations.
///
/// The `tket.bool` type is replaced by a sum type of `bool_t` (the standard
/// HUGR bool type represented by a unit sum) and `future(bool_t)`, with its operations
/// being turned into conditionals that read the future if necessary.
///
/// [TketOp::Measure], [QSystemOp::Measure], and [QSystemOp::MeasureReset] nodes
/// are replaced by [QSystemOp::LazyMeasure] and [QSystemOp::LazyMeasureReset]
/// nodes.
///
/// [TketOp::Measure]: tket::TketOp::Measure
/// [QSystemOp::Measure]: crate::extension::qsystem::QSystemOp::Measure
/// [QSystemOp::MeasureReset]: crate::extension::qsystem::QSystemOp::MeasureReset
/// [QSystemOp::LazyMeasure]: crate::extension::qsystem::QSystemOp::LazyMeasure
/// [QSystemOp::LazyMeasureReset]: crate::extension::qsystem::QSystemOp::LazyMeasureReset
#[derive(Default, Debug, Clone)]
pub struct ReplaceBoolPass;

impl<H: HugrMut<Node = Node>> ComposablePass<H> for ReplaceBoolPass {
    type Error = ReplaceBoolPassError<H::Node>;
    type Result = ();

    fn run(&self, hugr: &mut H) -> Result<(), Self::Error> {
        ensure_no_nonlocal_edges(hugr)?;
        ReplaceStaticArrayBoolPass::default().run(hugr)?;
        let lowerer = lowerer();
        lowerer.run(hugr)?;
        Ok(())
    }
}

/// The type each tket.bool is replaced with.
fn bool_dest() -> Type {
    SumType::new([bool_t(), future_type(bool_t())]).into()
}

fn read_builder(dfb: &mut DFGBuilder<Hugr>, sum_wire: Wire) -> BuildHandle<ConditionalID> {
    let mut cb = dfb
        .conditional_builder(
            ([bool_t().into(), future_type(bool_t()).into()], sum_wire),
            [],
            vec![bool_t()].into(),
        )
        .unwrap();

    // If the input is a bool, we can just return it.
    let case0 = cb.case_builder(0).unwrap();
    let [b] = case0.input_wires_arr();
    case0.finish_with_outputs([b]).unwrap();
    // If the input is a future, we need to read it.
    let mut case1 = cb.case_builder(1).unwrap();
    let [f] = case1.input_wires_arr();
    let [f] = case1.add_read(f, bool_t()).unwrap();
    case1.finish_with_outputs([f]).unwrap();

    cb.finish_sub_container().unwrap()
}

fn read_op_dest() -> NodeTemplate {
    let mut dfb = DFGBuilder::new(inout_sig(vec![bool_dest()], vec![bool_t()])).unwrap();
    let [sum_wire] = dfb.input_wires_arr();
    let cond = read_builder(&mut dfb, sum_wire);
    let h = dfb.finish_hugr_with_outputs(cond.outputs()).unwrap();
    NodeTemplate::CompoundOp(Box::new(h))
}

fn make_opaque_op_dest() -> NodeTemplate {
    let mut dfb = DFGBuilder::new(inout_sig(vec![bool_t()], vec![bool_dest()])).unwrap();
    let [inp] = dfb.input_wires_arr();
    let out = dfb
        .add_dataflow_op(
            Tag::new(0, vec![bool_t().into(), future_type(bool_t()).into()]),
            vec![inp],
        )
        .unwrap();
    let h = dfb.finish_hugr_with_outputs(out.outputs()).unwrap();
    NodeTemplate::CompoundOp(Box::new(h))
}

fn binary_logic_op_dest(op: &BoolOp) -> NodeTemplate {
    let mut dfb =
        DFGBuilder::new(inout_sig(vec![bool_dest(), bool_dest()], vec![bool_dest()])).unwrap();
    let [sum_wire1, sum_wire2] = dfb.input_wires_arr();
    let cond1 = read_builder(&mut dfb, sum_wire1);
    let cond2 = read_builder(&mut dfb, sum_wire2);
    let result = match op {
        BoolOp::eq => dfb
            .add_dataflow_op(LogicOp::Eq, [cond1.out_wire(0), cond2.out_wire(0)])
            .unwrap(),
        BoolOp::and => dfb
            .add_dataflow_op(LogicOp::And, [cond1.out_wire(0), cond2.out_wire(0)])
            .unwrap(),
        BoolOp::or => dfb
            .add_dataflow_op(LogicOp::Or, [cond1.out_wire(0), cond2.out_wire(0)])
            .unwrap(),
        BoolOp::xor => dfb
            .add_dataflow_op(LogicOp::Xor, [cond1.out_wire(0), cond2.out_wire(0)])
            .unwrap(),
        op => panic!("Unknown op name: {op:?}"),
    };
    let out = dfb
        .add_dataflow_op(
            Tag::new(0, vec![bool_t().into(), future_type(bool_t()).into()]),
            vec![result.out_wire(0)],
        )
        .unwrap();

    let h = dfb.finish_hugr_with_outputs(out.outputs()).unwrap();
    NodeTemplate::CompoundOp(Box::new(h))
}

fn not_op_dest() -> NodeTemplate {
    let mut dfb = DFGBuilder::new(inout_sig(vec![bool_dest()], vec![bool_dest()])).unwrap();
    let [sum_wire] = dfb.input_wires_arr();
    let cond = read_builder(&mut dfb, sum_wire);
    let result = dfb
        .add_dataflow_op(LogicOp::Not, [cond.out_wire(0)])
        .unwrap();
    let out = dfb
        .add_dataflow_op(
            Tag::new(0, vec![bool_t().into(), future_type(bool_t()).into()]),
            vec![result.out_wire(0)],
        )
        .unwrap();
    let h = dfb.finish_hugr_with_outputs(out.outputs()).unwrap();
    NodeTemplate::CompoundOp(Box::new(h))
}

fn measure_dest() -> NodeTemplate {
    let lazy_measure = QSystemOp::LazyMeasure.to_extension_op().unwrap();

    let mut dfb = DFGBuilder::new(inout_sig(vec![qb_t()], vec![bool_dest()])).unwrap();
    let [q] = dfb.input_wires_arr();
    let measure = dfb.add_dataflow_op(lazy_measure, vec![q]).unwrap();
    let tagged_output = dfb
        .add_dataflow_op(
            Tag::new(1, vec![bool_t().into(), future_type(bool_t()).into()]),
            vec![measure.out_wire(0)],
        )
        .unwrap();
    let h = dfb
        .finish_hugr_with_outputs(tagged_output.outputs())
        .unwrap();
    NodeTemplate::CompoundOp(Box::new(h))
}

fn measure_reset_dest() -> NodeTemplate {
    let lazy_measure_reset = QSystemOp::LazyMeasureReset.to_extension_op().unwrap();

    let mut dfb = DFGBuilder::new(inout_sig(vec![qb_t()], vec![qb_t(), bool_dest()])).unwrap();
    let [q] = dfb.input_wires_arr();
    let measure = dfb.add_dataflow_op(lazy_measure_reset, vec![q]).unwrap();
    let tagged_output = dfb
        .add_dataflow_op(
            Tag::new(1, vec![bool_t().into(), future_type(bool_t()).into()]),
            vec![measure.out_wire(1)],
        )
        .unwrap();
    let h = dfb
        .finish_hugr_with_outputs(vec![measure.out_wire(0), tagged_output.out_wire(0)])
        .unwrap();
    NodeTemplate::CompoundOp(Box::new(h))
}

fn copy_dfg(ty: Type) -> Hugr {
    let mut dfb = DFGBuilder::new(inout_sig(ty.clone(), vec![ty.clone(), ty])).unwrap();
    let mut h = std::mem::take(dfb.hugr_mut());
    let [inp, outp] = h.get_io(h.entrypoint()).unwrap();
    h.connect(inp, 0, outp, 0);
    h.connect(inp, 0, outp, 1);
    h
}

fn barray_get_dest(size: u64, elem_ty: Type) -> NodeTemplate {
    let array_ty = borrow_array_type(size, elem_ty.clone());
    let opt_el = option_type(elem_ty.clone());
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
        .add_dataflow_op(Tag::new(0, vec![type_row![], elem_ty.clone().into()]), [])
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
    let [elem1, elem2] = in_range
        .add_hugr_with_wires(copy_dfg(elem_ty.clone()), [elem])
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
        .add_dataflow_op(Tag::new(1, vec![type_row![], elem_ty.into()]), [elem2])
        .unwrap()
        .outputs_arr();
    in_range.finish_with_outputs([some, arr]).unwrap();

    let outs = cb.finish_sub_container().unwrap().outputs();
    // Do not finish DFG: it contains "invalid" copy_dfg that needs linearizing
    dfb.set_outputs(outs).unwrap();
    let h = std::mem::take(dfb.hugr_mut());
    NodeTemplate::CompoundOp(Box::new(h))
}

/// The configuration used for replacing tket.bool extension types and ops.
fn lowerer() -> ReplaceTypes {
    let mut lw = ReplaceTypes::default();

    // Replace tket.bool type.
    lw.set_replace_type(bool_type().as_extension().unwrap().clone(), bool_dest());
    let dup_op = FutureOp {
        op: FutureOpDef::Dup,
        typ: bool_t(),
    }
    .to_extension_op()
    .unwrap();
    let free_op = FutureOp {
        op: FutureOpDef::Free,
        typ: bool_t(),
    }
    .to_extension_op()
    .unwrap();
    lw.linearizer_mut()
        .register_simple(
            future_type(bool_t()).as_extension().unwrap().clone(),
            NodeTemplate::SingleOp(dup_op.into()),
            NodeTemplate::SingleOp(free_op.into()),
        )
        .unwrap();

    // Replace tket.bool constants.
    lw.replace_consts(
        bool_type().as_extension().unwrap().clone(),
        |const_bool, _| {
            Ok(Value::sum(
                0,
                [Value::from_bool(
                    const_bool
                        .value()
                        .downcast_ref::<ConstBool>()
                        .unwrap()
                        .value(),
                )],
                SumType::new([vec![bool_t()], vec![future_type(bool_t())]]),
            )
            .unwrap())
        },
    );

    // Replace all tket.bool ops.
    let read_op = BoolOp::read.to_extension_op().unwrap();
    lw.set_replace_op(&read_op, read_op_dest());
    let make_opaque_op = BoolOp::make_opaque.to_extension_op().unwrap();
    lw.set_replace_op(&make_opaque_op, make_opaque_op_dest());
    for op in [BoolOp::eq, BoolOp::and, BoolOp::or, BoolOp::xor] {
        lw.set_replace_op(&op.to_extension_op().unwrap(), binary_logic_op_dest(&op));
    }
    let not_op = BoolOp::not.to_extension_op().unwrap();
    lw.set_replace_op(&not_op, not_op_dest());

    // Replace measure ops with lazy versions.
    let tket_measure_free = TketOp::MeasureFree.to_extension_op().unwrap();
    let qsystem_measure = QSystemOp::Measure.to_extension_op().unwrap();
    let qsystem_measure_reset = QSystemOp::MeasureReset.to_extension_op().unwrap();
    lw.set_replace_op(&tket_measure_free, measure_dest());
    lw.set_replace_op(&qsystem_measure, measure_dest());
    lw.set_replace_op(&qsystem_measure_reset, measure_reset_dest());

    // Replace (borrow/)array ops that used to have with copyable bounds with DFGs that
    // the linearizer can act on now that the elements are no longer copyable.
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
        lw.set_replace_parametrized_op(
            array_ext.get_op(ARRAY_CLONE_OP_ID.as_str()).unwrap(),
            move |args, _| {
                let [size, elem_ty] = args else {
                    unreachable!()
                };
                let size = size.as_nat().unwrap();
                let elem_ty = elem_ty.as_runtime().unwrap();
                let template = (!elem_ty.copyable()).then(|| {
                    NodeTemplate::CompoundOp(Box::new(copy_dfg(type_fn(size, elem_ty.clone()))))
                });
                Ok(template)
            },
        );
        let drop_op_def = GUPPY_EXTENSION.get_op(DROP_OP_NAME.as_str()).unwrap();

        lw.set_replace_parametrized_op(
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

    lw.set_replace_parametrized_op(
        borrow_array::EXTENSION
            .get_op(GenericArrayOpDef::<BorrowArray>::get.opdef_id().as_str())
            .unwrap(),
        |args, _| {
            let [Term::BoundedNat(size), Term::Runtime(elem_ty)] = args else {
                unreachable!()
            };
            if elem_ty.copyable() {
                return Ok(None);
            }
            Ok(Some(barray_get_dest(*size, elem_ty.clone())))
        },
    );

    lw
}

#[cfg(test)]
mod test {
    use crate::extension::qsystem::{QSystemOp, QSystemOpBuilder};

    use super::*;
    use hugr::extension::prelude::{UnwrapBuilder, option_type, usize_t};
    use hugr::extension::simple_op::HasDef;
    use hugr::ops::OpType;
    use hugr::std_extensions::collections::array::op_builder::GenericArrayOpBuilder;
    use hugr::std_extensions::collections::array::{Array, ArrayKind};
    use hugr::std_extensions::collections::borrow_array::{
        BArrayOpBuilder, BorrowArray, borrow_array_type,
    };
    use hugr::type_row;
    use hugr::{
        HugrView,
        builder::{DFGBuilder, Dataflow, DataflowHugr, inout_sig},
        extension::prelude::qb_t,
        types::TypeRow,
    };
    use rstest::rstest;
    use tket::{
        TketOp,
        extension::bool::{BoolOp, BoolOpBuilder},
    };

    fn tket_bool_t() -> Type {
        bool_type()
    }

    #[test]
    fn test_consts() {
        let mut dfb = DFGBuilder::new(inout_sig(vec![], vec![tket_bool_t()])).unwrap();
        let const_wire = dfb.add_load_value(ConstBool::new(true));
        let mut h = dfb.finish_hugr_with_outputs([const_wire]).unwrap();

        h.validate().unwrap();
        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();
        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.output(), &TypeRow::from(vec![bool_dest()]));
    }

    #[test]
    fn test_read() {
        let mut dfb = DFGBuilder::new(inout_sig(vec![tket_bool_t()], vec![bool_t()])).unwrap();
        let [b] = dfb.input_wires_arr();
        let output = dfb.add_bool_read(b).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(output).unwrap();

        assert_eq!(h.num_nodes(), 8);

        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.input(), &TypeRow::from(vec![bool_dest()]));
        assert_eq!(sig.output(), &TypeRow::from(vec![bool_t()]));

        assert_eq!(h.num_nodes(), 18);
    }

    #[test]
    fn test_make_opaque() {
        let mut dfb = DFGBuilder::new(inout_sig(vec![bool_t()], vec![tket_bool_t()])).unwrap();
        let [b] = dfb.input_wires_arr();
        let output = dfb.add_bool_make_opaque(b).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(output).unwrap();

        assert_eq!(h.num_nodes(), 8);

        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.input(), &TypeRow::from(vec![bool_t()]));
        assert_eq!(sig.output(), &TypeRow::from(vec![bool_dest()]));

        assert_eq!(h.num_nodes(), 11);
    }

    #[rstest]
    #[case(BoolOp::eq)]
    #[case(BoolOp::and)]
    #[case(BoolOp::or)]
    #[case(BoolOp::xor)]
    fn test_logic(#[case] logic_op: BoolOp) {
        let mut dfb = DFGBuilder::new(inout_sig(
            vec![tket_bool_t(), tket_bool_t()],
            vec![tket_bool_t()],
        ))
        .unwrap();
        let [b1, b2] = dfb.input_wires_arr();
        let result = dfb.add_dataflow_op(logic_op, [b1, b2]).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(result.outputs()).unwrap();

        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.input(), &TypeRow::from(vec![bool_dest(), bool_dest()]));
        assert_eq!(sig.output(), &TypeRow::from(vec![bool_dest()]));
    }

    #[test]
    fn test_not() {
        let mut dfb = DFGBuilder::new(inout_sig(vec![tket_bool_t()], vec![tket_bool_t()])).unwrap();
        let [b] = dfb.input_wires_arr();
        let result = dfb.add_dataflow_op(BoolOp::not, [b]).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(result.outputs()).unwrap();

        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.input(), &TypeRow::from(vec![bool_dest()]));
        assert_eq!(sig.output(), &TypeRow::from(vec![bool_dest()]));
    }

    #[rstest]
    #[case(TketOp::MeasureFree)]
    #[case(QSystemOp::Measure)]
    fn test_measure<T: Into<OpType>>(#[case] measure_op: T) {
        let mut dfb = DFGBuilder::new(inout_sig(vec![qb_t()], vec![bool_type()])).unwrap();
        let [q] = dfb.input_wires_arr();
        let output = dfb.add_dataflow_op(measure_op, [q]).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(output.outputs()).unwrap();

        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.output(), &TypeRow::from(vec![bool_dest()]));

        // TODO: Figure out how to do this when the op is inside a DFB child.
        // let top_ops = h.children(h.entrypoint()).map(|n| h.get_optype(n)).collect_vec();
        // assert!(top_ops.iter().any(|op| {
        //    if let Some(ext_op) = op.as_extension_op() {
        //        ext_op.def().name() == "LazyMeasure"
        //    } else {
        //        false
        //    }
        //}));
    }

    #[test]
    fn test_measure_reset() {
        let mut dfb = DFGBuilder::new(inout_sig(vec![qb_t()], vec![qb_t(), bool_type()])).unwrap();
        let [q] = dfb.input_wires_arr();
        let output = dfb.add_measure_reset(q).unwrap();
        let mut h = dfb.finish_hugr_with_outputs(output).unwrap();

        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        assert_eq!(sig.output(), &TypeRow::from(vec![qb_t(), bool_dest()]));
    }

    #[rstest]
    #[case(Array)]
    #[case(BorrowArray)]
    fn test_array_clone_bool<AK: ArrayKind>(#[case] _ak: AK) {
        let elem_ty = bool_type();
        let size = 4;
        let arr_ty = AK::ty(size, elem_ty.clone());
        let mut dfb = DFGBuilder::new(inout_sig(
            vec![arr_ty.clone()],
            vec![arr_ty.clone(), arr_ty.clone()],
        ))
        .unwrap();
        let [arr_in] = dfb.input_wires_arr();
        let (arr1, arr2) = dfb
            .add_generic_array_clone::<AK>(elem_ty, size, arr_in)
            .unwrap();
        let mut h = dfb.finish_hugr_with_outputs([arr1, arr2]).unwrap();

        h.validate().unwrap();
        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        let bool_dest_ty = bool_dest();
        let arr_dest_ty = AK::ty(size, bool_dest_ty);
        assert_eq!(sig.input(), &TypeRow::from(vec![arr_dest_ty.clone()]));
        assert_eq!(
            sig.output(),
            &TypeRow::from(vec![arr_dest_ty.clone(), arr_dest_ty])
        );
    }

    #[rstest]
    #[case(Array)]
    #[case(BorrowArray)]
    fn test_array_discard_bool<AK: ArrayKind>(#[case] _ak: AK) {
        let elem_ty = bool_type();
        let size = 4;
        let arr_ty = AK::ty(size, elem_ty.clone());
        let mut dfb = DFGBuilder::new(inout_sig(vec![arr_ty.clone()], type_row![])).unwrap();
        let [arr_in] = dfb.input_wires_arr();
        dfb.add_generic_array_discard::<AK>(elem_ty, size, arr_in)
            .unwrap();
        let mut h = dfb.finish_hugr_with_outputs([]).unwrap();

        h.validate().unwrap();
        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap();
        h.validate().unwrap();
    }

    #[rstest]
    #[case(Type::new_tuple(vec![tket_bool_t(), usize_t()]), Type::new_tuple(vec![bool_dest(), usize_t()]), true)]
    #[case(tket_bool_t(), bool_dest(), true)]
    #[case(usize_t(), usize_t(), false)]
    fn test_barray_get(#[case] src_ty: Type, #[case] dest_ty: Type, #[case] expect_dup: bool) {
        let arr_ty = borrow_array_type(4, src_ty.clone());
        let mut dfb = DFGBuilder::new(inout_sig(
            vec![arr_ty.clone(), usize_t()],
            vec![arr_ty, src_ty.clone()],
        ))
        .unwrap();
        let [arr_in, idx] = dfb.input_wires_arr();
        let (opt_elem, arr) = dfb
            .add_borrow_array_get(src_ty.clone(), 4, arr_in, idx)
            .unwrap();
        let [elem] = dfb
            .build_unwrap_sum(1, option_type(src_ty.clone()), opt_elem)
            .unwrap();
        let mut h = dfb.finish_hugr_with_outputs([arr, elem]).unwrap();

        h.validate().unwrap();
        let pass = ReplaceBoolPass;
        pass.run(&mut h).unwrap(); // Fails here with mismatch, option_type(bool_dest()) is not Copyable
        h.validate().unwrap();

        let sig = h.signature(h.entrypoint()).unwrap();
        let dest_arr_ty = borrow_array_type(4, dest_ty.clone());
        assert_eq!(
            sig.as_ref(),
            &inout_sig(
                vec![dest_arr_ty.clone(), usize_t()],
                vec![dest_arr_ty, dest_ty]
            )
        );
        let contains_dup = h.nodes().any(|n| {
            h.get_optype(n).as_extension_op().is_some_and(|eop| {
                FutureOp::from_op(eop).is_ok_and(|fop| fop.op == FutureOpDef::Dup)
            })
        });
        assert_eq!(contains_dup, expect_dup);
    }
}
