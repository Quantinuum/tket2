//! Target platform selection for translating between HUGRs and pytket circuits.

use hugr::HugrView;
use strum::{EnumIter, EnumString, IntoStaticStr};
use tket::serialize::pytket::{
    PytketDecoderConfig, PytketEncoderConfig, default_decoder_config, default_encoder_config,
};

use crate::extension::qsystem::QSystemPlatform;
use crate::pytket::{add_qsystem_decoders, qsystem_decoder_config, qsystem_encoder_config};

/// Selects which set of encoder/decoder extensions is used when translating
/// between HUGRs and pytket circuits.
///
/// This is a superset of [`QSystemPlatform`]: in addition to the native
/// Quantinuum platforms it includes a platform-agnostic [`PlatformTarget::Tket`]
/// variant that only uses the base `tket` extensions. The
/// [`qsystem_platform`][PlatformTarget::qsystem_platform] conversion maps each
/// target onto its [`QSystemPlatform`], returning `None` for the non-native
/// `Tket` target.
///
/// The string representation of each variant (used by the python bindings) is
/// the lowercase variant name, e.g. `"tket"`, `"sol"`, or `"helios"`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, EnumIter, EnumString, IntoStaticStr)]
#[strum(serialize_all = "lowercase")]
#[non_exhaustive]
pub enum PlatformTarget {
    /// Platform-agnostic target using only the base `tket` extensions.
    #[default]
    Tket,
    /// Quantinuum Sol platform.
    Sol,
    /// Quantinuum Helios platform.
    Helios,
}

impl PlatformTarget {
    /// The string identifier for this target, as used by the python bindings.
    pub fn as_str(self) -> &'static str {
        self.into()
    }

    /// The native [`QSystemPlatform`] for this target, if any.
    ///
    /// A qsystem platform is only relevant when a native target is selected;
    /// the platform-agnostic [`PlatformTarget::Tket`] returns `None`.
    pub fn qsystem_platform(self) -> Option<QSystemPlatform> {
        match self {
            PlatformTarget::Tket => None,
            PlatformTarget::Sol => Some(QSystemPlatform::Sol),
            PlatformTarget::Helios => Some(QSystemPlatform::Helios),
        }
    }

    /// Pytket encoder configuration for this target.
    ///
    /// Only the extensions supported by the target are registered, so that a
    /// HUGR is not implicitly rebased into a different gate set when encoded.
    pub fn encoder_config<H: HugrView>(self) -> PytketEncoderConfig<H> {
        match self.qsystem_platform() {
            None => default_encoder_config(),
            Some(platform) => qsystem_encoder_config(platform),
        }
    }

    /// Pytket decoder configuration for this target.
    ///
    /// Native targets decode into their platform-specific operations. The
    /// platform-agnostic [`PlatformTarget::Tket`] decodes into base
    /// `tket.quantum` operations, with the Helios decoders registered as a
    /// fallback for pytket commands without a base counterpart (e.g. `ZZPhase`
    /// or `PhasedX`).
    pub fn decoder_config(self) -> PytketDecoderConfig {
        match self.qsystem_platform() {
            None => {
                let mut config = default_decoder_config();
                add_qsystem_decoders(&mut config, QSystemPlatform::Helios);
                config
            }
            Some(platform) => qsystem_decoder_config(platform),
        }
    }
}
