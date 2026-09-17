//! Extension registries and LLVM code-generation extensions.

use crate::hugr::extension::Version;
use crate::hugr::llvm::CodegenExtsBuilder;
use crate::hugr::llvm::custom::CodegenExtsMap;
use crate::hugr::llvm::extension::int::IntCodegenExtension;
use anyhow::{Context as _, Result};
use tket::hugr::Hugr;
use tket::llvm::rotation::RotationCodegenExtension;
use tket_qsystem::QSystemPlatform;
use tket_qsystem::extension::REGISTRY;
use tket_qsystem::llvm::array_utils::ArrayLowering;
use tket_qsystem::llvm::futures::FuturesCodegenExtension;
use tket_qsystem::llvm::globals::GlobalsCodegenExtension;
use tket_qsystem::llvm::{
    argument::ArgumentCodegenExtension, debug::DebugCodegenExtension, prelude::QISPreludeCodegen,
    qsystem::QSystemCodegenExtension, random::RandomCodegenExtension,
    result::ResultsCodegenExtension, utils::UtilsCodegenExtension,
};

use crate::array::SeleneHeapArrayCodegen;
use crate::gpu::GpuCodegen;

/// Return the extension names and versions available to the envelope loader.
///
/// The result is sorted so callers can compare or serialize it deterministically.
pub(crate) fn embedded_extensions() -> Vec<(String, String)> {
    let mut extensions = REGISTRY
        .iter_all()
        .map(|ext| (ext.name.to_string(), ext.version.to_string()))
        .collect::<Vec<_>>();
    extensions.sort_unstable();
    extensions
}

/// Return whether the envelope loader can provide a compatible extension.
///
/// # Errors
///
/// Returns an error if `version` is not a valid semantic version.
pub(crate) fn has_compatible_extension(name: &str, version: &str) -> Result<bool> {
    let version = Version::parse(version)
        .with_context(|| format!("invalid extension version: {version:?}"))?;
    Ok(REGISTRY.get_compatible(name, &version).is_some())
}

/// Build the LLVM code-generation extension map for an emulator platform.
pub(crate) fn codegen_extensions(platform: QSystemPlatform) -> CodegenExtsMap<'static, Hugr> {
    let prelude = QISPreludeCodegen;
    CodegenExtsBuilder::default()
        .add_prelude_extensions(prelude.clone())
        .add_extension(IntCodegenExtension::new(prelude.clone()))
        .add_float_extensions()
        .add_conversion_extensions()
        .add_logic_extensions()
        .add_extension(SeleneHeapArrayCodegen::LOWERING.codegen_extension())
        .add_default_static_array_extensions()
        .add_borrow_array_extensions(crate::array::SeleneHeapBorrowArrayCodegen(prelude.clone()))
        .add_extension(FuturesCodegenExtension)
        .add_extension(GlobalsCodegenExtension::new(prelude.clone()))
        .add_extension(QSystemCodegenExtension::new(platform, prelude.clone()))
        .add_extension(RandomCodegenExtension)
        // Results use standard arrays.
        .add_extension(ResultsCodegenExtension::new(
            SeleneHeapArrayCodegen::LOWERING,
        ))
        .add_extension(RotationCodegenExtension::new(prelude))
        .add_extension(UtilsCodegenExtension)
        // State results use standard arrays.
        .add_extension(DebugCodegenExtension::new(SeleneHeapArrayCodegen::LOWERING))
        .add_extension(GpuCodegen)
        // Argument reading uses standard arrays.
        .add_extension(ArgumentCodegenExtension::new(
            SeleneHeapArrayCodegen::LOWERING,
        ))
        .finish()
}
