//! Encoder/decoder definitions for translating tket-qsystem operations to/from legacy Pytket circuits.

mod futures;
mod qsystem;

pub use futures::FutureEmitter;
use hugr::HugrView;
pub use qsystem::QSystemEmitter;
use tket::serialize::pytket::{
    PytketDecoderConfig, PytketEncoderConfig, add_default_decoders, default_encoder_config,
};

use crate::extension::qsystem::QSystemPlatform;

/// Default pytket decoder configuration for [`Circuit`][tket::Circuit]s with
/// native qsystem operations targeting `platform`.
///
/// Contains a list of custom decoders that define translations of legacy tket
/// primitives into HUGR operations.
pub fn qsystem_decoder_config(platform: QSystemPlatform) -> PytketDecoderConfig {
    let mut config = PytketDecoderConfig::new();

    // Platform-specific decoders take priority over the base quantum ones.
    add_qsystem_decoders(&mut config, platform);
    add_default_decoders(&mut config);

    config
}

/// Add the platform-specific HUGR decoders and type translators to an existing config.
///
/// This registers the base `qsystem` operation decoders together with the future type translators.
///
/// Decoders are tried in registration order, so this is useful when building
/// custom decoder configs that register additional decoders *before* the qsystem
/// ones to give them higher priority, while still keeping the qsystem decoders as
/// a fallback.
pub fn add_qsystem_decoders(config: &mut PytketDecoderConfig, platform: QSystemPlatform) {
    config.add_decoder(QSystemEmitter(platform));
    config.add_type_translator(FutureEmitter);
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
