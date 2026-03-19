//! This module defines the Hugr extensions used to represent circuits.
//!
//! This includes a extension for the opaque TKET1 operations.

use std::sync::Arc;

use crate::TketOp;
use crate::serialize::pytket::extension::OpaqueTk1Op;
use hugr::Extension;
use hugr::extension::simple_op::MakeOpDef;
use hugr::extension::{
    CustomSignatureFunc, ExtensionId, ExtensionRegistry, SignatureError, Version,
};
use hugr::hugr::IdentList;
use hugr::std_extensions::STD_REG;
use hugr::types::type_param::{TypeArg, TypeParam};
use hugr::types::{PolyFuncType, PolyFuncTypeRV};
use lazy_static::lazy_static;
use smol_str::SmolStr;

/// Definition for bool type and ops.
pub mod bool;
/// Definition for debug ops.
pub mod debug;
pub mod global_phase;
/// Definition for ops used by Guppy.
pub mod guppy;
pub mod modifier;
/// Definition for Angle ops and types.
pub mod rotation;
pub mod sympy;

pub mod globals;

use sympy::SympyOpDef;

/// The ID of the TKET1 extension.
pub const TKET1_EXTENSION_ID: ExtensionId = IdentList::new_unchecked("TKET1");

/// The name for opaque TKET1 operations.
pub const TKET1_OP_NAME: SmolStr = SmolStr::new_inline("tk1op");

/// Current version of the legacy TKET 1 extension
pub const TKET1_EXTENSION_VERSION: Version = Version::new(0, 2, 0);

lazy_static! {
/// The TKET1 extension, containing the opaque TKET1 operations.
pub static ref TKET1_EXTENSION: Arc<Extension>  = {
    Extension::new_arc(TKET1_EXTENSION_ID, TKET1_EXTENSION_VERSION, |res, ext_ref| {
        res.add_op(
            TKET1_OP_NAME,
            "An opaque TKET1 operation.".into(),
            Tk1Signature([TypeParam::StringType]),
            ext_ref
        ).unwrap();
    })
};

/// Extension registry including the prelude, std, TKET1, and TketOps extensions.
pub(crate) static ref REGISTRY: ExtensionRegistry = ExtensionRegistry::new(
    STD_REG.iter().map(|e| e.to_owned()).chain([
    TKET1_EXTENSION.to_owned(),
    TKET_EXTENSION.to_owned(),
    bool::BOOL_EXTENSION.to_owned(),
    debug::DEBUG_EXTENSION.to_owned(),
    guppy::GUPPY_EXTENSION.to_owned(),
    rotation::ROTATION_EXTENSION.to_owned(),
    globals::EXTENSION.to_owned()
]));

}

struct Tk1Signature([TypeParam; 1]);

impl CustomSignatureFunc for Tk1Signature {
    fn compute_signature<'o, 'a: 'o>(
        &'a self,
        arg_values: &[TypeArg],
        _def: &'o hugr::extension::OpDef,
    ) -> Result<PolyFuncTypeRV, SignatureError> {
        let [TypeArg::String(arg)] = arg_values else {
            // This should have already been checked.
            panic!("Wrong number of arguments");
        };
        let op: OpaqueTk1Op = serde_json::from_str(arg).unwrap(); // TODO Errors!
        let poly_func: PolyFuncType = op.signature().into();
        Ok(poly_func.into())
    }

    fn static_params(&self) -> &[TypeParam] {
        &self.0
    }
}

/// Name of tket extension.
pub const TKET_EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.quantum");

/// Current version of the TKET extension
pub const TKET_EXTENSION_VERSION: Version = Version::new(0, 2, 1);

lazy_static! {
    /// The extension definition for TKET ops and types.
    pub static ref TKET_EXTENSION: Arc<Extension> = {
        Extension::new_arc(TKET_EXTENSION_ID, TKET_EXTENSION_VERSION, |res, ext_ref| {
            TketOp::load_all_ops(res, ext_ref).expect("add_fail");
            SympyOpDef.add_to_extension(res, ext_ref).unwrap();
        })
    };
}
