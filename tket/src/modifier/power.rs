//! Utilities for power modifiers
use std::str::FromStr;

use hugr::{
    extension::SignatureFunc,
    std_extensions::arithmetic::int_types::int_type,
    types::{FuncValueType, PolyFuncTypeRV, TypeBound, type_param::TypeParam},
};
use hugr_core::types::{Type, TypeRowRV};

/// Power modifier.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct ModifierPower;

impl ModifierPower {
    /// Create a new ModifierPower.
    fn new() -> Self {
        ModifierPower
    }
}
impl Default for ModifierPower {
    fn default() -> Self {
        Self::new()
    }
}
impl FromStr for ModifierPower {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == "ModifierPower" {
            Ok(Self::new())
        } else {
            Err(())
        }
    }
}
impl ModifierPower {
    /// signature for the power modifier.
    /// The Copyable bound of the second parameter is needed while constructing `TailLoop`.
    pub(crate) fn signature() -> SignatureFunc {
        PolyFuncTypeRV::new(
            [
                TypeParam::new_list_kind(TypeBound::Linear),
                TypeParam::new_list_kind(TypeBound::Copyable),
            ],
            FuncValueType::new(
                vec![
                    Type::new_function(FuncValueType::new(
                        TypeRowRV::new_var_use(0, TypeBound::Linear)
                            .concat(TypeRowRV::new_var_use(1, TypeBound::Copyable)),
                        TypeRowRV::new_var_use(0, TypeBound::Linear),
                    )),
                    int_type(6),
                ],
                [Type::new_function(FuncValueType::new(
                    TypeRowRV::new_var_use(0, TypeBound::Linear)
                        .concat(TypeRowRV::new_var_use(1, TypeBound::Copyable)),
                    TypeRowRV::new_var_use(0, TypeBound::Linear),
                ))],
            ),
        )
        .into()
    }
}
