//! Encoder/decoder definitions for translating tket-qsystem operations to/from legacy Pytket circuits.

mod futures;
mod qsystem;

pub use futures::FutureEmitter;
use hugr::HugrView;
pub use qsystem::QSystemEmitter;
use tket::serialize::pytket::{
    PytketDecoderConfig, PytketEncoderConfig, default_decoder_config, default_encoder_config,
};

use crate::extension::qsystem::QSystemPlatform;

/// Default pytket decoder configuration for [`Circuit`][tket::Circuit]s with
/// native qsystem operations targeting `platform`.
///
/// Contains a list of custom decoders that define translations of legacy tket
/// primitives into HUGR operations.
pub fn qsystem_decoder_config(platform: QSystemPlatform) -> PytketDecoderConfig {
    let mut config = default_decoder_config();
    config.add_decoder(QSystemEmitter(platform));
    config.add_type_translator(FutureEmitter);
    config
}

/// Default pytket encoder configuration for [`Circuit`][tket::Circuit]s with
/// native qsystem operations targeting `platform`.
///
/// Contains emitters for std and tket operations.
pub fn qsystem_encoder_config<H: HugrView>(platform: QSystemPlatform) -> PytketEncoderConfig<H> {
    let mut config = default_encoder_config();
    config.add_emitter(QSystemEmitter(platform));
    config.add_emitter(FutureEmitter);
    config.add_type_translator(FutureEmitter);
    config
}

#[cfg(test)]
mod tests;
