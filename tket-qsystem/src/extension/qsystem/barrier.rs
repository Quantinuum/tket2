mod barrier_inserter;
mod wrapped_barrier;
pub use barrier_inserter::BarrierInserter;

#[cfg(test)]
mod test {
    use super::*;

    use crate::extension::qsystem::{self, lower_tk2_op};
    use hugr::builder::{Dataflow, DataflowHugr};
    use hugr::extension::prelude::Barrier;
    use hugr::std_extensions::collections::borrow_array::borrow_array_type;
    use hugr::std_extensions::collections::value_array::value_array_type;
    use hugr::{HugrView, ops::handle::NodeHandle};
    use hugr::{
        builder::DFGBuilder,
        extension::prelude::{bool_t, option_type, qb_t},
        std_extensions::collections::array::array_type,
    };
    use itertools::Itertools;
    use rstest::rstest;

    fn opt_q_arr(size: u64) -> hugr::types::Type {
        array_type(size, option_type(qb_t()).into())
    }

    #[rstest]
    #[case(vec![qb_t(), qb_t()], 2, false)]
    #[case(vec![qb_t(), bool_t(), qb_t()], 2, false)]
    // special case, array of option qubit is unwrapped and unpacked
    #[case(vec![qb_t(), opt_q_arr(2)], 3, false)]
    // bare option of qubit is ignored
    #[case(vec![qb_t(), option_type(qb_t()).into()], 1, false)]
    #[case(vec![array_type(2, bool_t())], 0, false)]
    #[case(vec![value_array_type(2, option_type(qb_t()).into())], 2, false)]
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
        let lowered = lower_tk2_op(&mut h).unwrap_or_else(|e| panic!("{}", e));
        h.validate().unwrap_or_else(|e| panic!("{}", e));
        assert!(matches!(&lowered[..], [n] if barr_n == *n));

        let _barr_op: Barrier = h.get_optype(barr_n).cast().unwrap();

        let run_bar_n = if no_parent {
            h.nodes()
                .filter(|&r_barr_n| {
                    h.get_optype(r_barr_n).as_extension_op().is_some_and(|op| {
                        op.def()
                            .name()
                            .contains(qsystem::RUNTIME_BARRIER_NAME.as_str())
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

                use tket::passes::unpack_container::type_unpack::TypeUnpacker;

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
                            .contains(qsystem::RUNTIME_BARRIER_NAME.as_str())
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
                    &tket::passes::unpack_container::TEMP_UNPACK_EXT_NAME,
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
}
