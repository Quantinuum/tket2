//! This module defines a Hugr extension for operations to be used in Guppy.
use std::sync::Arc;

use hugr::{
    Extension,
    extension::{ExtensionId, Version},
    hugr::IdentList,
    type_row,
    types::{FuncValueType, PolyFuncTypeRV, Type, TypeBound},
};
use lazy_static::lazy_static;
use smol_str::SmolStr;

/// The ID of the `tket.guppy` extension.
pub const GUPPY_EXTENSION_ID: ExtensionId = IdentList::new_unchecked("tket.guppy");
/// The "tket.guppy" extension version.
pub const GUPPY_EXTENSION_VERSION: Version = Version::new(0, 2, 0);

/// The drop operation, used to handle affine types in Guppy.
pub const DROP_OP_NAME: SmolStr = SmolStr::new_inline("drop");

lazy_static! {
    /// The "tket.bool" extension.
    pub static ref GUPPY_EXTENSION: Arc<Extension>  = {
        Extension::new_arc(GUPPY_EXTENSION_ID, GUPPY_EXTENSION_VERSION, |ext, ext_ref| {
            ext.add_op(DROP_OP_NAME,
                "Drop the input wire. Applicable to guppy affine types only.".into(),
                // drop<T: Any>(t: T) -> ()
                PolyFuncTypeRV::new(
                    [TypeBound::Linear.into()],
                    FuncValueType::new(
                        vec![Type::new_var_use(0, TypeBound::Linear)],
                        type_row![],
                    )
            ),
            ext_ref
            ).unwrap();
        })
    };
}

#[cfg(test)]
mod test {

    use hugr::{
        builder::{DFGBuilder, Dataflow, DataflowHugr, inout_sig},
        extension::prelude::usize_t,
        std_extensions::collections::array::array_type,
    };

    use super::*;

    #[test]
    fn test_drop() {
        let arr_type = array_type(2, usize_t());
        let drop_op = GUPPY_EXTENSION
            .instantiate_extension_op(&DROP_OP_NAME, [arr_type.clone().into()])
            .unwrap();
        let mut b = DFGBuilder::new(inout_sig(vec![arr_type], type_row![])).unwrap();
        let inp = b.input_wires();
        b.add_dataflow_op(drop_op, inp).unwrap();
        b.finish_hugr_with_outputs([]).unwrap();
    }
}
