//! Encoder hooks for futures.

use hugr::HugrView;
use hugr::extension::ExtensionId;
use hugr::ops::ExtensionOp;
use tket::serialize::pytket::encoder::EncodeStatus;
use tket::serialize::pytket::extension::{PytketTypeTranslator, RegisterCount};
use tket::serialize::pytket::{
    PytketEmitter, PytketEncodeError, PytketEncoderContext, TypeTranslatorSet,
};

use crate::extension::futures;

/// Emitter for [futures](crate::extension::futures) operations and types.
///
/// Futures have no pytket representation.
#[derive(Debug, Clone, Default)]
pub struct FutureEmitter;

impl<H: HugrView> PytketEmitter<H> for FutureEmitter {
    fn extensions(&self) -> Option<Vec<ExtensionId>> {
        Some(vec![futures::EXTENSION_ID])
    }

    fn op_to_pytket(
        &self,
        _node: H::Node,
        _op: &ExtensionOp,
        _hugr: &H,
        _encoder: &mut PytketEncoderContext<H>,
    ) -> Result<EncodeStatus, PytketEncodeError<H::Node>> {
        Ok(EncodeStatus::Unsupported)
    }
}

impl PytketTypeTranslator for FutureEmitter {
    fn extensions(&self) -> Vec<ExtensionId> {
        vec![futures::EXTENSION_ID]
    }

    fn type_to_pytket(
        &self,
        _typ: &hugr::types::CustomType,
        _type_translators: &TypeTranslatorSet,
    ) -> Option<RegisterCount> {
        None
    }
}
