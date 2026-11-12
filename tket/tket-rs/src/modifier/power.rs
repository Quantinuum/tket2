//! Utilities for power modifiers
//!
//! See [`super::modifier_resolver`] for how runtime exponents are lowered to loops.
use std::str::FromStr;

use hugr::{
    extension::SignatureFunc,
    std_extensions::arithmetic::int_types::int_type,
    types::{FuncValueType, PolyFuncTypeRV, TypeBound, type_param::TypeParam},
};
use hugr_core::types::{Type, TypeRowRV};

/// A modifier that repeats a function according to a runtime 64-bit integer exponent.
///
/// The function must return its linear inputs in order and may take additional
/// copyable inputs, which are reused unchanged on every iteration. Non-positive
/// exponents, interpreted as signed integers, leave the linear inputs unchanged.
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
    /// Signature: `((L, C) -> L, int<6>) -> ((L, C) -> L)`.
    ///
    /// `L` is the row of linear inputs and outputs, and `C` is the row of
    /// additional copyable inputs. The copyable bound allows the resolver to
    /// reuse `C` on every iteration of the generated `TailLoop`. `int<6>` is
    /// HUGR's 64-bit integer type.
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
