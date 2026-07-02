//! Configuration structs for pytket encoders and decoders.

mod decoder_config;
mod encoder_config;
mod type_translators;

pub use decoder_config::PytketDecoderConfig;
pub use encoder_config::PytketEncoderConfig;
pub use type_translators::TypeTranslatorSet;

use crate::serialize::pytket::extension::{
    CoreDecoder, FloatEmitter, MeasurementEmitter, PreludeEmitter, RotationEmitter, Tk1Emitter,
    TketOpEmitter,
};
use hugr::HugrView;

/// Default pytket decoder configuration for Hugrs.
///
/// Contains a list of custom decoders that define translations of legacy tket
/// primitives into HUGR operations.
pub fn default_decoder_config() -> PytketDecoderConfig {
    let mut config = PytketDecoderConfig::new();
    add_default_decoders(&mut config);
    config
}

/// Add the default HUGR decoders and type translators to an existing config.
///
/// This registers the base `tket` decoders ([`CoreDecoder`], [`PreludeEmitter`],
/// and [`TketOpEmitter`]) together with the default type translators.
///
/// Decoders are tried in registration order, so this is useful when building
/// custom decoder configs that register additional decoders *before* the base
/// ones to give them higher priority, while still keeping the base decoders as
/// a fallback.
pub fn add_default_decoders(config: &mut PytketDecoderConfig) {
    config.add_decoder(CoreDecoder);
    config.add_decoder(PreludeEmitter);
    config.add_decoder(TketOpEmitter);

    config.add_type_translator(PreludeEmitter);
    config.add_type_translator(FloatEmitter);
    config.add_type_translator(RotationEmitter);
}

/// Default encoder configuration for Hugrs.
///
/// Contains emitters for std and tket operations.
pub fn default_encoder_config<H: HugrView>() -> PytketEncoderConfig<H> {
    let mut config = PytketEncoderConfig::new();
    config.add_emitter(PreludeEmitter);
    config.add_emitter(FloatEmitter);
    config.add_emitter(MeasurementEmitter);
    config.add_emitter(RotationEmitter);
    config.add_emitter(Tk1Emitter);
    config.add_emitter(TketOpEmitter);

    config.add_type_translator(PreludeEmitter);
    config.add_type_translator(FloatEmitter);
    config.add_type_translator(RotationEmitter);

    config
}
