//! This module defines the Hugr extension used to represent H-series
//! quantum operations.
//!
//! In the case of lazy operations,
//! laziness is represented by returning `tket.futures.Future` classical
//! values. Qubits are never lazy.
use std::sync::Arc;

use hugr::{
    Extension,
    extension::{ExtensionId, ExtensionRegistry, PRELUDE, Version, simple_op::MakeOpDef},
    std_extensions::arithmetic::float_types::EXTENSION as FLOAT_TYPES,
};

use crate::extension::futures;
use lazy_static::lazy_static;

mod barrier;
mod common;
pub mod helios;
mod lower;
pub mod sol;
mod synth_tket_op;
pub(crate) use common::SharedOp;
pub use lower::{LowerTk2Error, LowerTketToQSystemPass, check_lowered, lower_tk2_ops};

/// The "tket.qsystem" extension id.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qsystem");
/// The "tket.qsystem" extension version.
pub const EXTENSION_VERSION: Version = Version::new(0, 5, 1);

lazy_static! {
    /// The "tket.qsystem" extension.
    pub static ref EXTENSION: Arc<Extension> = {
         Extension::new_arc(EXTENSION_ID, EXTENSION_VERSION, |ext, ext_ref| {
            QSystemOp::load_all_ops( ext, ext_ref).unwrap();
            RuntimeBarrierDef.add_to_extension(ext, ext_ref).unwrap();
        })
    };

    /// Extension registry including the "tket.qsystem" extension and
    /// dependencies.
    pub static ref REGISTRY: ExtensionRegistry = ExtensionRegistry::new([
        EXTENSION.to_owned(),
        helios::EXTENSION.to_owned(),
        sol::EXTENSION.to_owned(),
        futures::EXTENSION.to_owned(),
        PRELUDE.to_owned(),
        FLOAT_TYPES.to_owned(),
    ]);
}

/// Target platform for QSystem operations. This can determine supported operations,
/// the native gateset, and steer optimisation choices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum QSystemPlatform {
    /// Quantinuum Helios, supporting PhasedX, ZZPhase, Rz
    Helios,
    /// Quantinuum Sol, supporting PhasedX, PhasedXX
    Sol,
}

impl QSystemPlatform {
    /// Convert a [`SharedOp`] into the platform-appropriate [`hugr::ops::OpType`].
    pub(crate) fn shared_op_type(self, op: SharedOp) -> hugr::ops::OpType {
        match self {
            QSystemPlatform::Helios => helios::HeliosOp::from(op).into(),
            QSystemPlatform::Sol => sol::SolOp::from(op).into(),
        }
    }
}
#[deprecated(
    since = "0.25.0",
    note = "Use helios::HeliosOp instead of QSystemOp for Helios-specific operations"
)]
pub use helios::HeliosOp as QSystemOp;

#[deprecated(since = "0.25.0", note = "Use helios::RUNTIME_BARRIER_NAME instead.")]
pub use helios::RUNTIME_BARRIER_NAME;

#[deprecated(since = "0.25.0", note = "Use helios::RuntimeBarrierDef instead.")]
pub use helios::RuntimeBarrierDef;
