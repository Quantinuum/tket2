//! This module defines a Hugr extension for the global phase operation.
//! Such operation applies some phase to a circuit.
//! This does not change the semantics of the circuit during simulation,
//! but may affect when the the controlled operators are applied.
use std::{
    str::FromStr,
    sync::{Arc, Weak},
};

use crate::extension::rotation::rotation_type;
use hugr::{
    Extension,
    extension::{
        ExtensionId, OpDef, SignatureFunc, Version,
        simple_op::{MakeOpDef, MakeRegisteredOp, OpLoadError},
    },
    ops::{ExtensionOp, OpName},
    type_row,
    types::Signature,
};
use lazy_static::lazy_static;

/// Identifier for the `tket.global_phase` extension.
pub const GLOBAL_PHASE_EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.global_phase");
/// Version of the `tket.global_phase` extension.
pub const GLOBAL_PHASE_VERSION: Version = Version::new(0, 1, 0);

lazy_static! {
    /// The extension definition for global phase operation.
    pub static ref GLOBAL_PHASE_EXTENSION: Arc<Extension> =  {
            Extension::new_arc(GLOBAL_PHASE_EXTENSION_ID, GLOBAL_PHASE_VERSION, |op, extension_ref| {
                op.add_op(
                    GLOBAL_PHASE_OP_ID.clone(),
                    "Global phase of a circuit".to_string(),
                    GlobalPhase::signature(),
                    extension_ref,
                ).unwrap();
            }
    )};
}

/// Identifier for the `global_phase` operation.
pub static GLOBAL_PHASE_OP_ID: OpName = OpName::new_inline("global_phase");

/// Global phase of a circuit.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub struct GlobalPhase;

impl GlobalPhase {
    /// Wraps the operation in an [`ExtensionOp`]
    pub fn into_extension_op(self) -> ExtensionOp {
        <Self as MakeRegisteredOp>::to_extension_op(self)
            .expect("Failed to convert to extension op.")
    }

    fn signature() -> Signature {
        Signature::new(vec![rotation_type()], type_row![])
    }
}

impl FromStr for GlobalPhase {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == Self.opdef_id() {
            Ok(Self)
        } else {
            Err(())
        }
    }
}

impl MakeOpDef for GlobalPhase {
    fn opdef_id(&self) -> OpName {
        GLOBAL_PHASE_OP_ID.clone()
    }

    fn from_def(op_def: &OpDef) -> Result<Self, OpLoadError>
    where
        Self: Sized,
    {
        hugr::extension::simple_op::try_from_name(op_def.name(), op_def.extension_id())
    }

    fn init_signature(&self, _extension_ref: &Weak<Extension>) -> SignatureFunc {
        SignatureFunc::from(GlobalPhase::signature())
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&GLOBAL_PHASE_EXTENSION)
    }

    fn extension(&self) -> ExtensionId {
        GLOBAL_PHASE_EXTENSION_ID.to_owned()
    }

    fn description(&self) -> String {
        "Global phase of a circuit.".into()
    }

    // TODO: Do we need this?
    // fn post_opdef(&self, def: &mut OpDef) {
}

impl MakeRegisteredOp for GlobalPhase {
    fn extension_id(&self) -> ExtensionId {
        GLOBAL_PHASE_EXTENSION_ID.to_owned()
    }

    fn extension_ref(&self) -> Arc<hugr::Extension> {
        GLOBAL_PHASE_EXTENSION.clone()
    }
}

#[cfg(test)]
mod test {
    use cool_asserts::assert_matches;
    use hugr::{
        HugrView,
        builder::{Dataflow, DataflowHugr, FunctionBuilder},
        extension::simple_op::{MakeExtensionOp, MakeOpDef},
        type_row,
        types::Signature,
    };

    use super::{
        GLOBAL_PHASE_EXTENSION, GLOBAL_PHASE_EXTENSION_ID, GLOBAL_PHASE_OP_ID, GlobalPhase,
    };
    use crate::extension::rotation::ConstRotation;

    #[test]
    fn create_global_phase_extension() {
        assert_eq!(GLOBAL_PHASE_EXTENSION.name(), &GLOBAL_PHASE_EXTENSION_ID);
        assert_eq!(
            GlobalPhase::from_def(GLOBAL_PHASE_EXTENSION.get_op(&GlobalPhase.op_id()).unwrap()),
            Ok(GlobalPhase)
        );
    }

    #[test]
    fn global_phase_op() {
        let mut func = FunctionBuilder::new("test_func", Signature::new_endo(type_row![])).unwrap();
        let rot = func.add_load_value(ConstRotation::new(1.0).unwrap());
        let global_phase_op = GLOBAL_PHASE_EXTENSION
            .instantiate_extension_op(&GLOBAL_PHASE_OP_ID, [])
            .unwrap();
        func.add_dataflow_op(global_phase_op, [rot]).unwrap();
        let hugr = func.finish_hugr_with_outputs([]).unwrap();
        assert_matches!(hugr.validate(), Ok(_));
    }
}
