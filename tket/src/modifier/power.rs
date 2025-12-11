//! Utilities for power modifiers
use std::str::FromStr;

use hugr::{
    extension::SignatureFunc,
    std_extensions::arithmetic::int_types::int_type,
    types::{FuncValueType, PolyFuncTypeRV, TypeBound, TypeRV, type_param::TypeParam},
};

#[allow(missing_docs)]
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct ModifierPower;

impl ModifierPower {
    /// Create a new ModifierPower.
    pub fn new() -> Self {
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
    pub fn signature() -> SignatureFunc {
        PolyFuncTypeRV::new(
            [
                TypeParam::new_list_type(TypeBound::Linear),
                TypeParam::new_list_type(TypeBound::Copyable),
            ],
            FuncValueType::new(
                vec![
                    TypeRV::new_function(FuncValueType::new(
                        vec![
                            TypeRV::new_row_var_use(0, TypeBound::Linear),
                            TypeRV::new_row_var_use(1, TypeBound::Copyable),
                        ],
                        vec![TypeRV::new_row_var_use(0, TypeBound::Linear)],
                    )),
                    int_type(6).into(),
                ],
                TypeRV::new_function(FuncValueType::new(
                    vec![
                        TypeRV::new_row_var_use(0, TypeBound::Linear),
                        TypeRV::new_row_var_use(1, TypeBound::Copyable),
                    ],
                    TypeRV::new_row_var_use(0, TypeBound::Linear),
                )),
            ),
        )
        .into()
    }
}
