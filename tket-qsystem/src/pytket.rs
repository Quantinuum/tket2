//! Encoder/decoder definitions for translating tket-qsystem operations to/from legacy Pytket circuits.

mod futures;
mod qsystem;

pub use futures::FutureEmitter;
use hugr::HugrView;
pub use qsystem::QSystemEmitter;
use tket::serialize::pytket::{
    PytketDecoderConfig, PytketEncoderConfig, default_decoder_config, default_encoder_config,
};

/// Default pytket decoder configuration for [`Circuit`][tket::Circuit]s with
/// native qsystem operations.
///
/// Contains a list of custom decoders that define translations of legacy tket
/// primitives into HUGR operations.
pub fn qsystem_decoder_config() -> PytketDecoderConfig {
    let mut config = default_decoder_config();
    config.add_decoder(QSystemEmitter);

    config.add_type_translator(FutureEmitter);

    config
}

/// Default pytket encoder configuration for [`Circuit`][tket::Circuit]s with
/// native qsystem operations.
///
/// Contains emitters for std and tket operations.
pub fn qsystem_encoder_config<H: HugrView>() -> PytketEncoderConfig<H> {
    let mut config = default_encoder_config();
    config.add_emitter(QSystemEmitter);
    config.add_emitter(FutureEmitter);

    config.add_type_translator(FutureEmitter);

    config
}

#[cfg(test)]
mod tests;
