//! Prepared emulator state for repeated LLVM emission.

use anyhow::{Result, anyhow};
use tket_qsystem::QSystemPlatform;

use crate::hugr::Hugr;
use crate::inkwell::{context::Context, module::Module};
use crate::{CompileArgs, compile_prepared, process_hugr, validate};

/// A validated HUGR prepared for emission by the Selene emulator.
///
/// Construction applies the platform-specific QSystem lowering passes exactly
/// once. The resulting HUGR is then borrowed immutably by all emission calls,
/// so producing multiple output formats does not clone or reprocess it.
#[derive(Debug)]
#[cfg_attr(feature = "py", pyo3::pyclass(frozen))]
pub struct EmulatorState {
    hugr: Hugr,
    platform: QSystemPlatform,
}

impl EmulatorState {
    /// Validate and prepare a HUGR for the selected emulator platform.
    ///
    /// # Errors
    ///
    /// Returns an error if the HUGR is invalid, contains unsupported opaque
    /// pytket operations, or cannot be lowered for `platform`.
    pub fn new(mut hugr: Hugr, platform: QSystemPlatform) -> Result<Self> {
        validate(&hugr)?;
        process_hugr(platform, &mut hugr)?;
        Ok(Self { hugr, platform })
    }

    /// Prepare a HUGR that has already passed [`validate`].
    pub(crate) fn from_validated_hugr(mut hugr: Hugr, platform: QSystemPlatform) -> Result<Self> {
        process_hugr(platform, &mut hugr)?;
        Ok(Self { hugr, platform })
    }

    /// Return the emulator platform fixed when this state was prepared.
    pub(crate) fn platform(&self) -> QSystemPlatform {
        self.platform
    }

    /// Emit this prepared HUGR as an LLVM module.
    ///
    /// # Errors
    ///
    /// Returns an error if `args` target a different emulator platform or LLVM
    /// emission and optimization fail.
    pub fn compile<'c>(&self, args: &CompileArgs, ctx: &'c Context) -> Result<Module<'c>> {
        if self.platform != args.platform {
            return Err(anyhow!(
                "emulator state targets {:?}, but compilation arguments target {:?}",
                self.platform,
                args.platform
            ));
        }
        compile_prepared(args, ctx, &self.hugr)
    }
}
