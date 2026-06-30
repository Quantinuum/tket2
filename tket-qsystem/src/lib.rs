//! Provides a preparation and validation workflow for Hugrs targeting
//! Quantinuum H-series quantum computers.

#[cfg(feature = "cli")]
pub mod cli;
pub mod extension;
pub(crate) mod helpers;
#[cfg(feature = "llvm")]
pub mod llvm;
pub mod lower_drops;
pub mod passes;
pub mod pytket;

pub use extension::qsystem::QSystemPlatform;
pub use passes::{
    QSystemLLVMPass, QSystemLLVMPassError, QSystemRebasePass, QSystemRebasePassError,
};
#[expect(deprecated)]
pub use passes::{QSystemPass, QSystemPassError};

#[cfg(test)]
mod test {
    use super::*;

    use hugr::{
        Hugr, HugrView,
        builder::{
            Container, Dataflow, DataflowHugr, DataflowSubContainer, FunctionBuilder, HugrBuilder,
        },
        core::Visibility,
        extension::prelude::qb_t,
        extension::simple_op::MakeExtensionOp,
        hugr::hugrmut::HugrMut,
        ops::{CallIndirect, ExtensionOp, OpType, handle::NodeHandle},
        std_extensions::{
            arithmetic::float_types::ConstF64,
            collections::array::{ArrayOpBuilder, array_type},
        },
        type_row,
        types::{Signature, Type},
    };

    use hugr::extension::prelude::bool_t;
    use itertools::Itertools as _;
    use petgraph::visit::{Topo, Walker as _};
    use rstest::rstest;
    use tket::extension::guppy::{DROP_OP_NAME, GUPPY_EXTENSION};
    use tket::extension::measurement::measurement_type;
    use tket::extension::modifier::{CONTROL_OP_ID, MODIFIER_EXTENSION, Modifier};
    use tket::passes::ComposablePass;
    use tket::{TketOp, metadata};

    use crate::extension::{
        futures::{FutureOpBuilder, FutureOpDef, future_type},
        qsystem::{QSystemOp, QSystemPlatform},
    };

    #[rstest]
    #[case(QSystemPlatform::Helios)]
    #[case(QSystemPlatform::Sol)]
    fn qsystem_passes(#[case] platform: QSystemPlatform) {
        let mut mb = hugr::builder::ModuleBuilder::new();
        let func = mb
            .define_function("func", Signature::new_endo(type_row![]))
            .unwrap()
            .finish_with_outputs([])
            .unwrap();

        let (mut hugr, [call_node, h_node, f_node, rx_node, main_node]) = {
            let mut builder = mb
                .define_function_vis(
                    "main",
                    Signature::new(vec![qb_t()], vec![bool_t(), bool_t()]),
                    Visibility::Public,
                )
                .unwrap();
            let [qb] = builder.input_wires_arr();

            // This call node has no dependencies, so it should be lifted above
            // Future Reads and sunk below quantum ops.
            let call_node = builder.call(func.handle(), &[], []).unwrap().node();

            // this LoadConstant should be pushed below the quantum ops where possible
            let angle = builder.add_load_value(ConstF64::new(0.0));
            let f_node = angle.node();

            // with no dependencies, this Reset should be lifted to the start
            let [qb] = builder
                .add_dataflow_op(QSystemOp::Reset, [qb])
                .unwrap()
                .outputs_arr();
            let h_node = qb.node();

            // depending on the angle means this op can't be lifted above the angle ops
            let [qb] = builder
                .add_dataflow_op(QSystemOp::Rz, [qb, angle])
                .unwrap()
                .outputs_arr();
            let rx_node = qb.node();

            let [measure_result] = builder
                .add_dataflow_op(QSystemOp::LazyMeasure, [qb])
                .unwrap()
                .outputs_arr();

            let [bool_result] = builder.add_read(measure_result, bool_t()).unwrap();

            let main_n = builder
                .finish_with_outputs([bool_result, bool_result])
                .unwrap()
                .node();
            let hugr = mb.finish_hugr().unwrap();
            (hugr, [call_node, h_node, f_node, rx_node, main_n])
        };
        // set the entrypoint to the main function
        hugr.set_entrypoint(main_node);

        QSystemRebasePass::defaults(platform)
            .run(&mut hugr)
            .unwrap();
        QSystemLLVMPass::default().run(&mut hugr).unwrap();

        let sg = hugr.scheduling_graph(main_node);
        let topo_sorted = Topo::new(sg.petgraph()).iter(&sg.petgraph()).collect_vec();

        let get_pos = |x| {
            topo_sorted
                .iter()
                .position(|&y| y == sg.node_to_pg(x))
                .unwrap()
        };
        assert!(get_pos(h_node) < get_pos(f_node));
        assert!(get_pos(h_node) < get_pos(call_node));
        assert!(get_pos(rx_node) < get_pos(call_node));

        for n in topo_sorted
            .iter()
            .map(|&pg_n| sg.pg_to_node(pg_n))
            .filter(|&n| FutureOpDef::try_from(hugr.get_optype(n)) == Ok(FutureOpDef::Read))
        {
            assert!(get_pos(call_node) < get_pos(n));
        }
    }

    #[rstest]
    #[case(QSystemPlatform::Helios)]
    #[case(QSystemPlatform::Sol)]
    fn qsystem_split_passes(#[case] platform: QSystemPlatform) {
        let mut builder =
            FunctionBuilder::new("main", Signature::new(vec![qb_t()], vec![qb_t()])).unwrap();
        let [qb] = builder.input_wires_arr();
        let [qb] = builder
            .add_dataflow_op(TketOp::H, [qb])
            .unwrap()
            .outputs_arr();
        let mut hugr = builder.finish_hugr_with_outputs([qb]).unwrap();

        QSystemRebasePass::defaults(platform)
            .run(&mut hugr)
            .unwrap();
        assert!(
            hugr.nodes()
                .all(|node| hugr.get_optype(node).cast::<TketOp>().is_none())
        );

        QSystemLLVMPass::default().run(&mut hugr).unwrap();
        hugr.validate().unwrap();
    }

    #[test]
    fn no_public_funcs() {
        let orig = {
            let arr_t = || array_type(4, measurement_type());
            let mut dfb = FunctionBuilder::new("main", Signature::new_endo(vec![arr_t()])).unwrap();
            let [arr] = dfb.input_wires_arr();
            let (arr1, arr2) = dfb.add_array_clone(measurement_type(), 4, arr).unwrap();
            let dop = GUPPY_EXTENSION.get_op(&DROP_OP_NAME).unwrap();
            dfb.add_dataflow_op(
                ExtensionOp::new(dop.clone(), [arr_t().into()]).unwrap(),
                [arr1],
            )
            .unwrap();
            dfb.finish_hugr_with_outputs([arr2]).unwrap()
        };

        let count_pub_funcs = |hugr: &Hugr| {
            hugr.children(hugr.module_root())
                .filter(|n| match hugr.get_optype(*n) {
                    OpType::FuncDefn(fd) => fd.visibility() == &Visibility::Public,
                    OpType::FuncDecl(fd) => fd.visibility() == &Visibility::Public,
                    _ => false,
                })
                .count()
        };

        // Check there are no public funcs (after hiding).
        let mut hugr = orig.clone();
        // TODO: add sol case?
        QSystemRebasePass::defaults(QSystemPlatform::Helios)
            .run(&mut hugr)
            .unwrap();
        QSystemLLVMPass::default().run(&mut hugr).unwrap();
        assert_eq!(count_pub_funcs(&hugr), 0);

        // Run the same passes without hiding public functions.
        let mut hugr_public = orig;
        QSystemRebasePass::defaults(QSystemPlatform::Helios)
            .with_hide_funcs(false)
            .run(&mut hugr_public)
            .unwrap();
        QSystemLLVMPass::default().run(&mut hugr_public).unwrap();

        assert_eq!(count_pub_funcs(&hugr_public), 4);
        assert_eq!(
            hugr.children(hugr.module_root()).count(),
            hugr_public.children(hugr_public.module_root()).count()
        );
        assert_eq!(hugr.num_nodes(), hugr_public.num_nodes());
    }

    #[test]
    fn measurement_drop_lowering() {
        // Additional test outside of the `LowerTketToQSystemPass` to check the
        // interaction with `LowerDropsPass`.
        let mut hugr = {
            let arr_t = || array_type(4, measurement_type());
            let mut dfb =
                FunctionBuilder::new("main", Signature::new(vec![arr_t()], vec![])).unwrap();
            let [arr] = dfb.input_wires_arr();
            dfb.add_array_discard(measurement_type(), 4, arr).unwrap();
            dfb.finish_hugr_with_outputs([]).unwrap()
        };

        QSystemRebasePass::defaults(QSystemPlatform::Helios)
            .run(&mut hugr)
            .unwrap();
        QSystemLLVMPass::default().run(&mut hugr).unwrap();

        // Check a function for discarding measurements has been introduced.
        let expected_sig = Signature::new(vec![future_type(bool_t())], vec![Type::UNIT]);
        let has_discard_load_fn = hugr.nodes().any(|n| {
            matches!(
                hugr.get_optype(n),
                OpType::LoadFunction(lf) if lf.instantiation == expected_sig
            )
        });
        assert!(has_discard_load_fn);
    }

    #[rstest]
    #[case(QSystemPlatform::Helios)]
    #[case(QSystemPlatform::Sol)]
    fn qsystem_native_pass_resolves_modifiers(#[case] platform: QSystemPlatform) {
        let mut module = hugr::builder::ModuleBuilder::new();

        let foo_sig = Signature::new_endo([qb_t()]);
        let foo = {
            let mut func = module.define_function("foo", foo_sig.clone()).unwrap();
            let func_node = func.container_node();
            func.hugr_mut()
                .set_metadata::<metadata::UnitaryFlags>(func_node, 7);
            let [qb] = func.input_wires_arr();
            let [qb] = func.add_dataflow_op(TketOp::X, [qb]).unwrap().outputs_arr();
            func.finish_with_outputs([qb]).unwrap()
        };

        let ctrl_num = 1;
        let controlled_sig = Signature::new_endo([array_type(ctrl_num, qb_t()), qb_t()]);
        let control_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    hugr::types::Term::BoundedNat(ctrl_num),
                    vec![qb_t().into()].into(),
                    vec![].into(),
                ],
            )
            .unwrap();

        {
            let mut func = module
                .define_function_vis("main", controlled_sig.clone(), Visibility::Public)
                .unwrap();
            let [controls, target] = func.input_wires_arr();

            let loaded = func.load_func(foo.handle(), &[]).unwrap();
            let modified = func
                .add_dataflow_op(control_op, [loaded])
                .unwrap()
                .out_wire(0);
            let [controls, target] = func
                .add_dataflow_op(
                    CallIndirect {
                        signature: controlled_sig,
                    },
                    [modified, controls, target],
                )
                .unwrap()
                .outputs_arr();

            func.finish_with_outputs([controls, target]).unwrap();
        }

        let mut hugr = module.finish_hugr().unwrap();
        hugr.set_entrypoint(foo.node());
        assert!(
            hugr.nodes()
                .any(|node| Modifier::from_optype(hugr.get_optype(node)).is_some())
        );

        QSystemRebasePass::defaults(platform)
            .run(&mut hugr)
            .unwrap();
        QSystemLLVMPass::default().run(&mut hugr).unwrap();

        assert!(
            hugr.nodes()
                .all(|node| Modifier::from_optype(hugr.get_optype(node)).is_none())
        );
        hugr.validate().unwrap();
    }
}
