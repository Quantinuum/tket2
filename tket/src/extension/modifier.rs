//! This module defines a Hugr extension for modifier operations.
//! These operations modify circuits by applying modifiers: control, dagger, or power.
use lazy_static::lazy_static;
use std::sync::{Arc, Weak};

use hugr::{
    Extension,
    extension::{
        ExtensionId, OpDef, SignatureFunc, Version,
        simple_op::{MakeOpDef, OpLoadError},
    },
    ops::OpName,
};
use serde::{Deserialize, Serialize};
use strum::{EnumIter, EnumString, IntoStaticStr};

use crate::modifier::{control::ModifierControl, dagger::ModifierDagger, power::ModifierPower};

/// Types of modifers.
#[derive(
    Clone,
    Copy,
    Debug,
    Serialize,
    Deserialize,
    Hash,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    EnumIter,
    IntoStaticStr,
    EnumString,
)]
pub enum Modifier {
    /// Control modifier.
    ControlModifier,
    /// Dagger modifier.
    DaggerModifier,
    /// Power modifier.
    PowerModifier,
}

#[allow(missing_docs)]
pub const MODIFIER_EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.modifier");
#[allow(missing_docs)]
pub const MODIFIER_VERSION: Version = Version::new(0, 1, 0);

lazy_static! {
    /// The extension definition for modifier operations.
    pub static ref MODIFIER_EXTENSION: Arc<Extension> =  {
            Extension::new_arc(MODIFIER_EXTENSION_ID, MODIFIER_VERSION, |modifier, extension_ref| {
                modifier.add_op(
                    CONTROL_OP_ID,
                    "Quantum control operation".to_string(),
                    ModifierControl::signature(),
                    extension_ref,
                ).unwrap();

                modifier.add_op(
                    DAGGER_OP_ID,
                    "Dagger Operator".to_string(),
                    ModifierDagger::signature(),
                    extension_ref,
                ).unwrap();

                modifier.add_op(
                    POWER_OP_ID,
                    "Power Operator".to_string(),
                    ModifierPower::signature(),
                    extension_ref,
                ).unwrap();
            }
    )};
}

#[allow(missing_docs)]
pub const CONTROL_OP_ID: OpName = OpName::new_inline("ControlModifier");
#[allow(missing_docs)]
pub const DAGGER_OP_ID: OpName = OpName::new_inline("DaggerModifier");
#[allow(missing_docs)]
pub const POWER_OP_ID: OpName = OpName::new_inline("PowerModifier");

impl MakeOpDef for Modifier {
    fn opdef_id(&self) -> OpName {
        match self {
            Modifier::ControlModifier => CONTROL_OP_ID.clone(),
            Modifier::DaggerModifier => DAGGER_OP_ID.clone(),
            Modifier::PowerModifier => POWER_OP_ID.clone(),
        }
    }

    fn from_def(op_def: &OpDef) -> Result<Self, OpLoadError>
    where
        Self: Sized,
    {
        hugr::extension::simple_op::try_from_name(op_def.name(), op_def.extension_id())
    }

    fn init_signature(&self, _extension_ref: &std::sync::Weak<hugr::Extension>) -> SignatureFunc {
        match self {
            Modifier::ControlModifier => ModifierControl::signature(),
            Modifier::DaggerModifier => ModifierDagger::signature(),
            Modifier::PowerModifier => ModifierPower::signature(),
        }
    }

    fn extension_ref(&self) -> Weak<hugr::Extension> {
        Arc::downgrade(&MODIFIER_EXTENSION)
    }

    fn extension(&self) -> ExtensionId {
        MODIFIER_EXTENSION_ID.to_owned()
    }

    fn description(&self) -> String {
        match self {
            Modifier::ControlModifier => {
                "Generates a quantum-controlled circuit from a circuit.".into()
            }
            Modifier::DaggerModifier => "Dagger operation on a circuit.".into(),
            Modifier::PowerModifier => {
                "Generates a circuit that applies a circuit many times.".into()
            }
        }
    }

    // [TODO]: Do we need this?
    // fn post_opdef(&self, _def: &mut OpDef);
}

#[cfg(test)]
mod test {
    use super::{
        CONTROL_OP_ID, DAGGER_OP_ID, MODIFIER_EXTENSION, MODIFIER_EXTENSION_ID, Modifier,
        POWER_OP_ID,
    };
    use cool_asserts::assert_matches;
    use hugr::{
        builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder},
        extension::{
            OpDef,
            prelude::{bool_t, qb_t},
            simple_op::{MakeExtensionOp, MakeOpDef},
        },
        ops::{CallIndirect, ExtensionOp},
        std_extensions::{
            arithmetic::int_types::{ConstInt, int_type},
            collections::array::array_type,
        },
        types::{Signature, Term, Type},
    };
    use rstest::rstest;
    use std::sync::Arc;
    use strum::IntoEnumIterator;

    fn get_modifier_opdef(op: Modifier) -> Option<&'static Arc<OpDef>> {
        MODIFIER_EXTENSION.get_op(&op.op_id())
    }

    #[test]
    fn create_modifier_extension() {
        assert_eq!(MODIFIER_EXTENSION.name(), &MODIFIER_EXTENSION_ID);

        for o in Modifier::iter() {
            assert_eq!(Modifier::from_def(get_modifier_opdef(o).unwrap()), Ok(o));
        }
    }

    fn control_op(inout: Type, other_inputs: Type) -> (ExtensionOp, Signature) {
        let modified_sig = Signature::new(
            vec![array_type(1, qb_t()), inout.clone(), other_inputs.clone()],
            vec![array_type(1, qb_t()), inout.clone()],
        );
        let control_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &CONTROL_OP_ID,
                [
                    Term::BoundedNat(1),
                    Term::new_list([inout.into()]),
                    Term::new_list([other_inputs.into()]),
                ],
            )
            .unwrap();
        (control_op, modified_sig)
    }

    fn dagger_op(inout: Type, other_inputs: Type) -> (ExtensionOp, Signature) {
        let modified_sig = Signature::new(
            vec![inout.clone(), other_inputs.clone()],
            vec![inout.clone()],
        );
        let dagger_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &DAGGER_OP_ID,
                [
                    Term::new_list([inout.into()]),
                    Term::new_list([other_inputs.into()]),
                ],
            )
            .unwrap();
        (dagger_op, modified_sig)
    }

    fn power_op(inout: Type, other_inputs: Type) -> (ExtensionOp, Signature) {
        let modified_sig = Signature::new(
            vec![inout.clone(), other_inputs.clone()],
            vec![inout.clone()],
        );
        let power_op = MODIFIER_EXTENSION
            .instantiate_extension_op(
                &POWER_OP_ID,
                [
                    Term::new_list([inout.into()]),
                    Term::new_list([other_inputs.into()]),
                ],
            )
            .unwrap();
        (power_op, modified_sig)
    }

    #[rstest]
    #[case(control_op, false)]
    #[case(dagger_op, false)]
    #[case(power_op, true)]
    fn modifier_op(
        #[case] op_fn: fn(Type, Type) -> (ExtensionOp, Signature),
        #[case] needs_extra_param: bool,
    ) {
        let original_sig = Signature::new(vec![int_type(6), bool_t()], int_type(6));
        let (control_op, modified_sig) = op_fn(int_type(6), bool_t());
        let main_sig = modified_sig.clone();

        let mut module = ModuleBuilder::new();

        let decl = module.declare("dummy_decl", original_sig.into()).unwrap();

        let mut main = module.define_function("_main", main_sig).unwrap();
        let inputs = main.input_wires();
        let loaded_func = main.load_func(&decl, &[]).unwrap();
        let modifier_arg = if needs_extra_param {
            let int = main.add_load_value(ConstInt::new_u(6, 3).unwrap());
            vec![loaded_func, int]
            // vec![loaded_func]
        } else {
            vec![loaded_func]
        };
        let modified = main
            .add_dataflow_op(control_op, modifier_arg)
            .unwrap()
            .out_wire(0);
        let outputs = main
            .add_dataflow_op(
                CallIndirect {
                    signature: modified_sig,
                },
                [modified].into_iter().chain(inputs),
            )
            .unwrap()
            .outputs();

        main.finish_with_outputs(outputs).unwrap();

        assert_matches!(module.finish_hugr(), Ok(_));
    }
}
