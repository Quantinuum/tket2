//! This module defines the "tket.qsystem.utils" extension, which includes the
//! utility functions available for Quantinuum systems.

use std::sync::{Arc, Weak};

use derive_more::derive::Display;
use hugr::{
    Extension, Wire,
    builder::{BuildError, Dataflow},
    extension::{
        ExtensionId, ExtensionRegistry, OpDef, PRELUDE, SignatureFunc, Version,
        prelude::UnwrapBuilder,
        simple_op::{MakeOpDef, MakeRegisteredOp, try_from_name},
    },
    std_extensions::arithmetic::int_types::int_type,
    type_row,
    types::{Signature, TypeArg, TypeRowRV},
};
use itertools::Itertools;
use lazy_static::lazy_static;
use strum::{EnumIter, EnumString, IntoStaticStr};

/// The extension ID for the utils extension.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qsystem.utils");
/// The version of the "tket.qsystem.utils" extension.
pub const EXTENSION_VERSION: Version = Version::new(0, 3, 0);

lazy_static! {
    /// The "tket.qsystem.utils" extension.
    pub static ref EXTENSION: Arc<Extension> = {
         Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            UtilsOp::load_all_ops( ext, ext_ref).unwrap();
        })
    };

    /// Extension registry including the "tket.qsystem.utils" extension and
    /// dependencies.
    pub static ref REGISTRY: ExtensionRegistry = ExtensionRegistry::new([
        EXTENSION.to_owned(),
        PRELUDE.to_owned(),
    ]);
}

#[derive(
    Clone,
    Copy,
    Debug,
    serde::Serialize,
    serde::Deserialize,
    Hash,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    EnumIter,
    IntoStaticStr,
    EnumString,
    Display,
)]
#[non_exhaustive]
/// The operations provided by the utils extension.
pub enum UtilsOp {
    /// `fn get_current_shot() -> usize`
    GetCurrentShot,
}

impl MakeOpDef for UtilsOp {
    fn opdef_id(&self) -> hugr::ops::OpName {
        <&'static str>::from(self).into()
    }

    fn init_signature(&self, _extension_ref: &std::sync::Weak<Extension>) -> SignatureFunc {
        match self {
            UtilsOp::GetCurrentShot => Signature::new(type_row![], int_type(6)),
        }
        .into()
    }

    fn from_def(op_def: &OpDef) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> std::sync::Weak<Extension> {
        Arc::downgrade(&EXTENSION)
    }

    fn description(&self) -> String {
        match self {
            UtilsOp::GetCurrentShot => "Get current shot number.",
        }
        .to_string()
    }
}

impl MakeRegisteredOp for UtilsOp {
    fn extension_id(&self) -> ExtensionId {
        EXTENSION_ID
    }

    fn extension_ref(&self) -> Arc<Extension> {
        EXTENSION.clone()
    }
}

/// An extension trait for [Dataflow] providing methods to add
/// "tket.qsystem.utils" operations.
pub trait UtilsOpBuilder: Dataflow + UnwrapBuilder {
    /// Add a "tket.qsystem.utils.GetCurrentShot" op.
    fn add_get_current_shot(&mut self) -> Result<Wire, BuildError> {
        Ok(self
            .add_dataflow_op(UtilsOp::GetCurrentShot, [])?
            .out_wire(0))
    }
}

pub(crate) fn row_to_arg(row: impl Into<TypeRowRV>) -> TypeArg {
    TypeArg::List(row.into().into_owned().into_iter().map_into().collect())
}

impl<D: Dataflow> UtilsOpBuilder for D {}

#[cfg(test)]
mod test {
    use std::sync::Arc;

    use hugr::HugrView;
    use hugr::{
        builder::{DataflowHugr, FunctionBuilder},
        extension::simple_op::MakeExtensionOp,
    };
    use strum::IntoEnumIterator;

    use super::*;

    fn get_opdef(op: UtilsOp) -> Option<&'static Arc<OpDef>> {
        EXTENSION.get_op(&op.op_id())
    }

    #[test]
    fn create_extension() {
        assert_eq!(EXTENSION.name(), &EXTENSION_ID);

        for o in UtilsOp::iter() {
            assert_eq!(UtilsOp::from_def(get_opdef(o).unwrap()), Ok(o));
        }
    }

    #[test]
    fn get_current_shot() {
        let hugr = {
            let mut func_builder = FunctionBuilder::new(
                "get_current_shot",
                Signature::new(vec![], vec![int_type(6)]),
            )
            .unwrap();
            let shot = func_builder.add_get_current_shot().unwrap();
            func_builder.finish_hugr_with_outputs([shot]).unwrap()
        };
        hugr.validate().unwrap()
    }
}
