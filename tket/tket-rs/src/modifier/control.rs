//! Utilities for Control modifiers
use hugr::{
    extension::{SignatureFunc, prelude::qb_t},
    std_extensions::collections::array::array_type_parametric,
    types::{
        FuncValueType, PolyFuncTypeRV, Type, TypeArg, TypeBound, TypeRowRV, type_param::TypeParam,
    },
};

/// Control modifier.
///
/// Stores the number of controls qubits to apply.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct ModifierControl(usize);

impl ModifierControl {
    /// Create a new ModifierControl with a specific number of controls.
    fn new(num: usize) -> Self {
        ModifierControl(num)
    }
}
impl Default for ModifierControl {
    fn default() -> Self {
        Self::new(0)
    }
}
impl ModifierControl {
    /// Signature for the control modifier.
    pub(crate) fn signature() -> SignatureFunc {
        PolyFuncTypeRV::new(
            [
                TypeParam::max_nat_kind(),
                TypeParam::new_list_kind(TypeBound::Linear),
                TypeParam::new_list_kind(TypeBound::Linear),
            ],
            FuncValueType::new(
                [Type::new_function(FuncValueType::new(
                    TypeRowRV::new_var_use(1, TypeBound::Linear)
                        .concat(TypeRowRV::new_var_use(2, TypeBound::Linear)),
                    TypeRowRV::new_var_use(1, TypeBound::Linear),
                ))],
                [Type::new_function(FuncValueType::new(
                    TypeRowRV::from([array_type_parametric(
                        TypeArg::new_var_use(0, TypeParam::max_nat_kind()),
                        qb_t(),
                    )
                    .unwrap()])
                    .concat(TypeRowRV::new_var_use(1, TypeBound::Linear))
                    .concat(TypeRowRV::new_var_use(2, TypeBound::Linear)),
                    TypeRowRV::from([array_type_parametric(
                        TypeArg::new_var_use(0, TypeParam::max_nat_kind()),
                        qb_t(),
                    )
                    .unwrap()])
                    .concat(TypeRowRV::new_var_use(1, TypeBound::Linear)),
                ))],
            ),
        )
        .into()
    }
}
