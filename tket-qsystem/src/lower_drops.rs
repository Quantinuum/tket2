/// Contains a pass to lower "drop" ops from the Guppy extension
use hugr::algorithms::replace_types::{NodeTemplate, ReplaceTypesError, ReplacementOptions};
use hugr::algorithms::{ComposablePass, ReplaceTypes};
use hugr::builder::{Container, DFGBuilder};
use hugr::extension::prelude::bool_t;
use hugr::extension::simple_op::MakeRegisteredOp;
use hugr::types::{Signature, Term};
use hugr::{Node, hugr::hugrmut::HugrMut};
use tket::extension::guppy::{DROP_OP_NAME, GUPPY_EXTENSION};

use crate::extension::futures::{FutureOp, FutureOpDef, future_type};

/// A pass that lowers "drop" ops from [GUPPY_EXTENSION]
#[derive(Default, Debug, Clone)]
pub struct LowerDropsPass;

impl<H: HugrMut<Node = Node>> ComposablePass<H> for LowerDropsPass {
    type Error = ReplaceTypesError;

    /// Returns whether any drops were lowered
    type Result = bool;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let mut rt = ReplaceTypes::default();

        // future(bool) is not in the default linearizer handler so we add it here.
        // TODO: Create ReplaceTypes with future(bool) linearized by default to avoid
        // code duplication with ReplaceBools pass.
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
        rt.linearizer()
            .register_simple(
                future_type(bool_t()).as_extension().unwrap().clone(),
                NodeTemplate::SingleOp(dup_op.into()),
                NodeTemplate::SingleOp(free_op.into()),
            )
            .unwrap();

        rt.replace_parametrized_op_with(
            GUPPY_EXTENSION.get_op(DROP_OP_NAME.as_str()).unwrap(),
            |targs| {
                let [Term::Runtime(ty)] = targs else {
                    panic!("Expected just one type")
                };
                // The Hugr here is invalid, so we have to pull it out manually
                let mut dfb = DFGBuilder::new(Signature::new(ty.clone(), vec![])).unwrap();
                let h = std::mem::take(dfb.hugr_mut());
                Some(NodeTemplate::CompoundOp(Box::new(h)))
            },
            ReplacementOptions::default().with_linearization(true),
        );
        rt.run(hugr)
    }
}

#[cfg(test)]
mod test {
    use std::sync::Arc;

    use hugr::builder::{Dataflow, DataflowHugr, inout_sig};
    use hugr::ops::ExtensionOp;
    use hugr::{Hugr, HugrView};
    use hugr::{extension::prelude::usize_t, std_extensions::collections::array::array_type};

    use super::*;

    #[test]
    fn test_lower_drop() {
        let arr_type = array_type(2, usize_t());
        let drop_op = GUPPY_EXTENSION.get_op(DROP_OP_NAME.as_str()).unwrap();
        let drop_node = ExtensionOp::new(drop_op.clone(), [arr_type.clone().into()]).unwrap();
        let mut b = DFGBuilder::new(inout_sig(arr_type, vec![])).unwrap();
        let inp = b.input_wires();
        b.add_dataflow_op(drop_node, inp).unwrap();
        let mut h = b.finish_hugr_with_outputs([]).unwrap();
        let count_drops = |h: &Hugr| {
            h.nodes()
                .filter(|n| {
                    h.get_optype(*n)
                        .as_extension_op()
                        .is_some_and(|e| Arc::ptr_eq(e.def_arc(), drop_op))
                })
                .count()
        };
        assert_eq!(count_drops(&h), 1);
        LowerDropsPass.run(&mut h).unwrap();
        h.validate().unwrap();
        assert_eq!(count_drops(&h), 0);
    }
}
