mod barrier_inserter;
mod wrapped_barrier;
pub use barrier_inserter::BarrierInserter;

#[cfg(test)]
mod test {
    use super::*;

    use crate::extension::qsystem::{self, QSystemPlatform, lower_tk2_ops};
    use hugr::builder::{Dataflow, DataflowHugr};
    use hugr::extension::prelude::Barrier;
    use hugr::std_extensions::collections::borrow_array::borrow_array_type;
    use hugr::{HugrView, ops::handle::NodeHandle};
    use hugr::{
        builder::DFGBuilder,
        extension::prelude::{bool_t, option_type, qb_t},
        std_extensions::collections::array::array_type,
    };
    use itertools::Itertools;
    use rstest::rstest;
    use tket::passes::composable::Preserve;

    fn opt_q_arr(size: u64) -> hugr::types::Type {
        array_type(size, option_type(vec![qb_t()]).into())
    }

    #[rstest]
    #[case(vec![qb_t(), qb_t()], 2, false)]
    #[case(vec![qb_t(), bool_t(), qb_t()], 2, false)]
    // special case, array of option qubit is unwrapped and unpacked
    #[case(vec![qb_t(), opt_q_arr(2)], 3, false)]
    // bare option of qubit is ignored
    #[case(vec![qb_t(), option_type(vec![qb_t()]).into()], 1, false)]
    #[case(vec![array_type(2, bool_t())], 0, false)]
    #[case(vec![borrow_array_type(2, qb_t())], 2, false)]
    // special case, single array of qubits is passed directly to op without unpacking
    #[case(vec![array_type(3, qb_t())], 1, true)]
    #[case(vec![qb_t(), array_type(2, qb_t()), array_type(2, array_type(2, qb_t()))], 7, false)]
    #[case(vec![hugr::types::Type::new_tuple(vec![bool_t(), qb_t()]), qb_t()], 2, false)]
    #[case(vec![hugr::types::Type::new_tuple(vec![bool_t(), qb_t(), opt_q_arr(2)]), qb_t()], 4, false)]
    #[case(vec![hugr::types::Type::new_tuple(vec![bool_t(), qb_t(), array_type(2, hugr::types::Type::new_tuple(vec![bool_t(), qb_t()]))]), qb_t()], 4, false)]
    fn test_barrier(
        #[case] type_row: Vec<hugr::types::Type>,
        #[case] num_qb: usize,
        // whether it is the array[qubit] special case
        #[case] no_parent: bool,
    ) {
        // build a dfg with a generic barrier

        let (mut h, barr_n) = {
            let mut b =
                DFGBuilder::new(hugr::types::Signature::new_endo(type_row.clone())).unwrap();

            let barr_n = b.add_barrier(b.input_wires()).unwrap();
            (
                b.finish_hugr_with_outputs(barr_n.outputs()).unwrap(),
                barr_n.node(),
            )
        };

        // lower barrier to barrier + runtime barrier
        let lowered = lower_tk2_ops(&mut h, Preserve::Public, QSystemPlatform::Helios)
            .unwrap_or_else(|e| panic!("{}", e));
        h.validate().unwrap_or_else(|e| panic!("{}", e));
        assert!(matches!(&lowered[..], [n] if barr_n == *n));

        let _barr_op: Barrier = h.get_optype(barr_n).cast().unwrap();

        let run_bar_n = if no_parent {
            h.nodes()
                .filter(|&r_barr_n| {
                    h.get_optype(r_barr_n).as_extension_op().is_some_and(|op| {
                        op.def()
                            .name()
                            .contains(qsystem::helios::RUNTIME_BARRIER_NAME.as_str())
                    })
                })
                .exactly_one()
                .ok()
                .unwrap()
        } else {
            let run_barr_func_n = h
                .children(h.module_root())
                .filter(|&r_barr_n| {
                    h.get_optype(r_barr_n).as_func_defn().is_some_and(|op| {
                        op.func_name()
                            .contains(wrapped_barrier::WRAPPED_BARRIER_NAME.as_str())
                    })
                })
                .exactly_one()
                .ok();
            let Some(run_barr_func_n) = run_barr_func_n else {
                // if the runtime barrier function is never called
                // make sure it is because there are no qubits in the barrier

                use tket::passes::utils::unpack_container::type_unpack::TypeUnpacker;

                let analyzer = TypeUnpacker::for_qubits();
                let tuple_type = hugr::types::Type::new_tuple(type_row);
                assert!(!analyzer.contains_element_type(&tuple_type));
                assert_eq!(num_qb, 0);
                return;
            };

            let num_run_bar_ops = h
                .children(run_barr_func_n)
                .filter(|&n| {
                    h.get_optype(n).as_extension_op().is_some_and(|op| {
                        op.def()
                            .name()
                            .contains(qsystem::helios::RUNTIME_BARRIER_NAME.as_str())
                    })
                })
                .count();
            assert_eq!(
                num_run_bar_ops, 1,
                "Should be exactly one runtime barrier op in the function"
            );
            h.single_linked_input(run_barr_func_n, 0).unwrap().0
        };

        assert_eq!(h.all_linked_inputs(run_bar_n).count(), num_qb);

        // Check all temporary ops are removed
        for n in h.nodes() {
            if let Some(op) = h.get_optype(n).as_extension_op() {
                for factory_ext in [
                    &tket::passes::utils::unpack_container::TEMP_UNPACK_EXT_NAME,
                    &wrapped_barrier::TEMP_BARRIER_EXT_NAME,
                ] {
                    assert_ne!(
                        op.extension_id(),
                        factory_ext,
                        "temporary op: {} {}",
                        op.unqualified_id(),
                        op.args().iter().map(|a| a.to_string()).join(","),
                    );
                }
            }
        }
    }

    #[test]
    fn repeated_barriers_share_wrapped_function() {
        let mut b =
            DFGBuilder::new(hugr::types::Signature::new_endo(vec![qb_t(), qb_t()])).unwrap();
        let first = b.add_barrier(b.input_wires()).unwrap();
        let second = b.add_barrier(first.outputs()).unwrap();
        let mut h = b.finish_hugr_with_outputs(second.outputs()).unwrap();

        lower_tk2_ops(&mut h, Preserve::Public, QSystemPlatform::Helios).unwrap();
        h.validate().unwrap();

        let wrapped_function = h
            .children(h.module_root())
            .filter(|&node| {
                h.get_optype(node).as_func_defn().is_some_and(|op| {
                    op.func_name()
                        .contains(wrapped_barrier::WRAPPED_BARRIER_NAME.as_str())
                })
            })
            .exactly_one()
            .ok()
            .expect("identical barriers should share one wrapped function");
        assert_eq!(h.output_neighbours(wrapped_function).count(), 2);
    }

    #[rstest]
    #[case(QSystemPlatform::Helios)]
    #[case(QSystemPlatform::Sol)]
    fn wrapped_barrier_does_not_reuse_user_function(#[case] platform: QSystemPlatform) {
        use hugr::builder::{DataflowSubContainer, HugrBuilder, ModuleBuilder};
        use hugr::core::Visibility;
        use hugr::types::Signature;
        use tket::passes::ComposablePass;

        let mut b = ModuleBuilder::new();
        let identity = b
            .define_function_vis(
                "$wrapped_barrier$$n(2)$list($e(qubit)$e(qubit))",
                Signature::new_endo([qb_t(), qb_t()]),
                Visibility::Public,
            )
            .unwrap();
        let inputs = identity.input_wires().collect_vec();
        let identity_id = identity.finish_with_outputs(inputs).unwrap();

        let mut main = b
            .define_function_vis("main", Signature::new_endo([]), Visibility::Public)
            .unwrap();
        let q0 = main.add_dataflow_op(tket::TketOp::QAlloc, []).unwrap();
        let q1 = main.add_dataflow_op(tket::TketOp::QAlloc, []).unwrap();
        let identity_call = main
            .call(identity_id.handle(), &[], [q0.out_wire(0), q1.out_wire(0)])
            .unwrap();
        let barrier = main.add_barrier(identity_call.outputs()).unwrap();
        for q in barrier.outputs() {
            main.add_dataflow_op(tket::TketOp::QFree, [q]).unwrap();
        }
        let main_id = main.finish_with_outputs([]).unwrap();
        let mut h = b.finish_hugr().unwrap();
        h.validate().unwrap();

        crate::QSystemRebasePass::defaults(platform)
            .run(&mut h)
            .unwrap();
        h.validate().unwrap();

        assert_eq!(
            h.static_source(identity_call.node()),
            Some(identity_id.node())
        );
        assert_eq!(h.children(identity_id.node()).count(), 2);
        let [input, output] = h.get_io(identity_id.node()).unwrap();
        for port in 0..2 {
            assert_eq!(
                h.single_linked_output(output, port),
                Some((input, port.into()))
            );
        }
        assert_eq!(
            h.get_optype(identity_id.node())
                .as_func_defn()
                .unwrap()
                .visibility(),
            &Visibility::Public
        );
        let runtime_barriers = h
            .nodes()
            .filter(|&n| {
                h.get_optype(n)
                    .as_extension_op()
                    .is_some_and(|op| op.unqualified_id() == "RuntimeBarrier")
            })
            .collect_vec();
        assert_eq!(
            runtime_barriers.len(),
            1,
            "the runtime barrier must survive linking"
        );
        let runtime_op = h.get_optype(runtime_barriers[0]).as_extension_op().unwrap();
        assert_eq!(runtime_op.args(), &[hugr::types::TypeArg::BoundedNat(2)]);
        assert_eq!(
            runtime_op.extension_id(),
            &match platform {
                QSystemPlatform::Helios => qsystem::helios::EXTENSION_ID,
                QSystemPlatform::Sol => qsystem::sol::EXTENSION_ID,
            }
        );
        let wrapper = h.get_parent(runtime_barriers[0]).unwrap();
        assert_ne!(wrapper, identity_id.node());
        assert_eq!(
            h.get_optype(wrapper).as_func_defn().unwrap().visibility(),
            &Visibility::Private
        );
        let call = h
            .output_neighbours(wrapper)
            .exactly_one()
            .unwrap_or_else(|error| panic!("{error}"));
        assert!(h.descendants(main_id.node()).contains(&call));
    }

    #[rstest]
    #[case(QSystemPlatform::Helios)]
    #[case(QSystemPlatform::Sol)]
    fn repeated_container_barriers_share_helpers(
        #[case] platform: QSystemPlatform,
        #[values(false, true)] hide_funcs: bool,
    ) {
        use hugr::builder::FunctionBuilder;
        use hugr::core::Visibility;
        use hugr::types::Signature;
        use tket::passes::ComposablePass;

        let mut b =
            FunctionBuilder::new("main", Signature::new_endo([borrow_array_type(2, qb_t())]))
                .unwrap();
        let mut wires = b.input_wires().collect_vec();
        for _ in 0..64 {
            wires = b.add_barrier(wires).unwrap().outputs().collect_vec();
        }
        let mut h = b.finish_hugr_with_outputs(wires).unwrap();
        crate::QSystemRebasePass::defaults(platform)
            .with_hide_funcs(hide_funcs)
            .run(&mut h)
            .unwrap();
        h.validate().unwrap();

        let helpers = h
            .children(h.module_root())
            .filter(|&n| {
                h.get_optype(n)
                    .as_func_defn()
                    .is_some_and(|f| f.func_name() != "main")
            })
            .collect_vec();
        assert_eq!(helpers.len(), 3, "unpack, wrapped barrier, repack");
        for helper in helpers {
            assert_eq!(h.output_neighbours(helper).count(), 64);
            assert_eq!(
                h.get_optype(helper).as_func_defn().unwrap().visibility(),
                &if hide_funcs {
                    Visibility::Private
                } else {
                    Visibility::Public
                }
            );
        }
        assert_eq!(
            h.nodes()
                .filter(|&n| h
                    .get_optype(n)
                    .as_extension_op()
                    .is_some_and(|op| op.unqualified_id() == "RuntimeBarrier"))
                .count(),
            1
        );
    }
}
