//! Utilities for dagger modifiers
use std::str::FromStr;

use hugr::{
    extension::SignatureFunc,
    types::{FuncValueType, PolyFuncTypeRV, TypeBound, type_param::TypeParam},
};
use hugr_core::types::{Type, TypeRowRV};

/// Dagger modifier.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct ModifierDagger;

impl ModifierDagger {
    /// Create a new ModifierDagger.
    fn new() -> Self {
        ModifierDagger
    }
}
impl Default for ModifierDagger {
    fn default() -> Self {
        Self::new()
    }
}
impl FromStr for ModifierDagger {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == "ModifierDagger" {
            Ok(Self::new())
        } else {
            Err(())
        }
    }
}
impl ModifierDagger {
    /// Signature for the dagger modifier.
    pub(crate) fn signature() -> SignatureFunc {
        PolyFuncTypeRV::new(
            [
                TypeParam::new_list_kind(TypeBound::Linear),
                TypeParam::new_list_kind(TypeBound::Linear),
            ],
            FuncValueType::new(
                [Type::new_function(FuncValueType::new(
                    TypeRowRV::new_var_use(0, TypeBound::Linear)
                        .concat(TypeRowRV::new_var_use(1, TypeBound::Linear)),
                    TypeRowRV::new_var_use(0, TypeBound::Linear),
                ))],
                [Type::new_function(FuncValueType::new(
                    TypeRowRV::new_var_use(0, TypeBound::Linear)
                        .concat(TypeRowRV::new_var_use(1, TypeBound::Linear)),
                    TypeRowRV::new_var_use(0, TypeBound::Linear),
                ))],
            ),
        )
        .into()
    }
}
