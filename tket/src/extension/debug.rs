//! This module defines a Hugr extension for operations to be used by users debugging
//! with a simulator.
use std::sync::{Arc, Weak};

use hugr::{
    Extension,
    extension::{
        ExtensionId, SignatureError, SignatureFunc, Version,
        prelude::qb_t,
        simple_op::{
            HasConcrete, HasDef, MakeExtensionOp, MakeOpDef, MakeRegisteredOp, OpLoadError,
            try_from_name,
        },
    },
    ops::OpName,
    std_extensions::collections::array::array_type_parametric,
    types::{FuncValueType, PolyFuncTypeRV, TypeArg, type_param::TypeParam},
};
use lazy_static::lazy_static;

/// The ID of the `tket.debug` extension.
pub const DEBUG_EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.debug");
/// The "tket.debug" extension version
pub const DEBUG_EXTENSION_VERSION: Version = Version::new(0, 2, 0);

lazy_static! {
    /// The "tket.bool" extension.
    pub static ref DEBUG_EXTENSION: Arc<Extension>  = {
        Extension::new_arc(DEBUG_EXTENSION_ID, DEBUG_EXTENSION_VERSION, |ext, ext_ref| {
            StateResultDef.add_to_extension(ext, ext_ref).unwrap();
        })
    };
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
/// A `tket.StateResult` operation definition.
pub struct StateResultDef;

/// Name of the `tket.StateResult` operation.
pub const STATE_RESULT_OP_ID: OpName = OpName::new_inline("StateResult");

impl std::str::FromStr for StateResultDef {
    type Err = ();

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        if s == StateResultDef.opdef_id() {
            Ok(Self)
        } else {
            Err(())
        }
    }
}

impl MakeOpDef for StateResultDef {
    fn opdef_id(&self) -> hugr::ops::OpName {
        STATE_RESULT_OP_ID
    }

    fn init_signature(&self, _extension_ref: &Weak<Extension>) -> SignatureFunc {
        PolyFuncTypeRV::new(
            vec![TypeParam::StringType, TypeParam::max_nat_type()],
            FuncValueType::new(
                vec![
                    array_type_parametric(
                        TypeArg::new_var_use(1, TypeParam::max_nat_type()),
                        qb_t(),
                    )
                    .unwrap(),
                ],
                vec![
                    array_type_parametric(
                        TypeArg::new_var_use(1, TypeParam::max_nat_type()),
                        qb_t(),
                    )
                    .unwrap(),
                ],
            ),
        )
        .into()
    }

    fn from_def(
        op_def: &hugr::extension::OpDef,
    ) -> Result<Self, hugr::extension::simple_op::OpLoadError> {
        try_from_name(op_def.name(), op_def.extension_id())
    }

    fn extension(&self) -> ExtensionId {
        DEBUG_EXTENSION_ID
    }

    fn description(&self) -> String {
        "Report the state of given qubits in the given order.".to_string()
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&DEBUG_EXTENSION)
    }
}

#[derive(Debug, Clone, PartialEq)]
/// A debug operation for requesting the state of some qubits to be recorded if the
/// program is executed on a simulator.
pub struct StateResult {
    /// Static string tag for the result.
    pub tag: String,
    /// The number of qubits in the result.
    pub num_qubits: u64,
}

impl StateResult {
    /// Create a new `StateResult` operation.
    pub fn new(tag: String, num_qubits: u64) -> Self {
        StateResult { tag, num_qubits }
    }
}

impl MakeExtensionOp for StateResult {
    fn op_id(&self) -> OpName {
        STATE_RESULT_OP_ID
    }

    fn from_extension_op(ext_op: &hugr::ops::ExtensionOp) -> Result<Self, OpLoadError>
    where
        Self: Sized,
    {
        let def = StateResultDef::from_def(ext_op.def())?;
        def.instantiate(ext_op.args())
    }

    fn type_args(&self) -> Vec<TypeArg> {
        vec![
            TypeArg::String(self.tag.clone()),
            TypeArg::BoundedNat(self.num_qubits),
        ]
    }
}

impl MakeRegisteredOp for StateResult {
    fn extension_id(&self) -> ExtensionId {
        DEBUG_EXTENSION_ID
    }

    fn extension_ref(&self) -> Weak<Extension> {
        Arc::downgrade(&DEBUG_EXTENSION)
    }
}

impl HasDef for StateResult {
    type Def = StateResultDef;
}

impl HasConcrete for StateResultDef {
    type Concrete = StateResult;

    fn instantiate(&self, type_args: &[TypeArg]) -> Result<Self::Concrete, OpLoadError> {
        let [TypeArg::String(arg), TypeArg::BoundedNat(n)] = type_args else {
            return Err(SignatureError::InvalidTypeArgs)?;
        };
        Ok(StateResult {
            tag: arg.to_string(),
            num_qubits: *n,
        })
    }
}

#[cfg(test)]
pub(crate) mod test {
    use hugr::HugrView;
    use hugr::{
        builder::{DFGBuilder, Dataflow, DataflowHugr},
        ops::OpType,
        std_extensions::collections::array::array_type,
        types::Signature,
    };

    use super::*;

    #[test]
    fn test_state_result() {
        let op = StateResult::new("test".into(), 22);
        let optype: OpType = op.clone().into();
        let new_op = StateResult::from_extension_op(optype.as_extension_op().unwrap()).unwrap();
        assert_eq!(new_op, op);

        let qb_array_type = array_type(22, qb_t());
        let hugr = {
            let mut builder =
                DFGBuilder::new(Signature::new(qb_array_type.clone(), qb_array_type)).unwrap();
            let inputs: [hugr::Wire; 1] = builder.input_wires_arr();
            let output = builder.add_dataflow_op(op, inputs).unwrap();
            builder.finish_hugr_with_outputs(output.outputs()).unwrap()
        };
        hugr.validate().unwrap();
    }
}
