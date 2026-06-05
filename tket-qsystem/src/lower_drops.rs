//! Contains a pass to lower "drop" ops from the Guppy extension
use hugr::{Node, hugr::hugrmut::HugrMut};
use hugr_core::types::Type;
use tket::extension::guppy::{DROP_OP_NAME, GUPPY_EXTENSION};
use tket::passes::composable::WithScope;
use tket::passes::replace_types::{Linearizer, ReplaceTypesError};
use tket::passes::{ComposablePass, PassScope};

use crate::helpers::lowerer_with_future_linearization;

/// A pass that lowers "drop" ops from [GUPPY_EXTENSION]
#[derive(Default, Debug, Clone)]
pub struct LowerDropsPass {
    /// Where to apply the pass.
    ///
    /// Configurable via [`WithScope::with_scope`].
    scope: PassScope,
}

impl WithScope for LowerDropsPass {
    fn with_scope(mut self, scope: impl Into<PassScope>) -> Self {
        self.scope = scope.into();
        self
    }
}

impl<H: HugrMut<Node = Node>> ComposablePass<H> for LowerDropsPass {
    type Error = ReplaceTypesError;

    /// Returns whether any drops were lowered
    type Result = bool;

    fn run(&self, hugr: &mut H) -> Result<Self::Result, Self::Error> {
        let mut rt = lowerer_with_future_linearization().with_scope(self.scope.clone());

        rt.set_replace_parametrized_op(
            GUPPY_EXTENSION.get_op(DROP_OP_NAME.as_str()).unwrap(),
            |args, rt| {
                let [ty] = args else {
                    panic!("Expected just one type")
                };
                let ty = Type::try_from(ty.clone()).unwrap();
                Ok(Some(rt.get_linearizer().copy_discard_op(&ty, 0)?))
            },
        );
        rt.run(hugr)
    }
}

#[cfg(test)]
mod test {
    use std::sync::Arc;

    use hugr::builder::{DFGBuilder, Dataflow, DataflowHugr, inout_sig};
    use hugr::ops::ExtensionOp;
    use hugr::{Hugr, HugrView};
    use hugr::{extension::prelude::usize_t, std_extensions::collections::array::array_type};

    use super::*;

    #[test]
    fn test_lower_drop() {
        let arr_type = array_type(2, usize_t());
        let drop_op = GUPPY_EXTENSION.get_op(DROP_OP_NAME.as_str()).unwrap();
        let drop_node = ExtensionOp::new(drop_op.clone(), [arr_type.clone().into()]).unwrap();
        let mut b = DFGBuilder::new(inout_sig(vec![arr_type], vec![])).unwrap();
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
        LowerDropsPass::default().run(&mut h).unwrap();
        h.validate().unwrap();
        assert_eq!(count_drops(&h), 0);
    }
}
