//! Common pass configurations.
use hugr::extension::prelude::bool_t;
use hugr::extension::simple_op::MakeRegisteredOp;
use tket::passes::ReplaceTypes;
use tket::passes::replace_types::NodeTemplate;

use crate::extension::futures::{FutureOp, FutureOpDef, future_type};

/// Default `ReplaceTypes` lowerer which registers linearizers for the `Future`
/// type.
pub(crate) fn lowerer_with_future_linearization() -> ReplaceTypes {
    let mut res = ReplaceTypes::default();
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
    res.linearizer_mut()
        .register_simple(
            future_type(bool_t()).as_extension().unwrap().clone(),
            NodeTemplate::SingleOp(dup_op.into()),
            NodeTemplate::SingleOp(free_op.into()),
        )
        .unwrap();
    res
}
