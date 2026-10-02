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
        .add_ptr_extensions(tket_qsystem::llvm::ptr::QisPtrCodegen)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hugr;
    use hugr::builder::{Dataflow, DataflowHugr};
    use hugr::extension::prelude::{UnwrapBuilder, bool_t, option_type};
    use hugr::llvm::emit::{EmitDebugInfo, Namer, test::SimpleHugrConfig};
    use hugr::std_extensions::{
        arithmetic::int_types::{ConstInt, int_type},
        ptr::{self, PtrOpBuilder},
    };
    use std::rc::Rc;

    #[test]
    fn pointer_registry_and_codegen_for_both_platforms() {
        assert!(has_compatible_extension(&ptr::EXTENSION_ID, &ptr::VERSION.to_string()).unwrap());
        for platform in [QSystemPlatform::Sol, QSystemPlatform::Helios] {
            let ty = int_type(6);
            let mut hugr = SimpleHugrConfig::new()
                .with_extensions(REGISTRY.to_owned())
                .with_outs([ty.clone()])
                .finish(|mut b| {
                    let value = b.add_load_value(ConstInt::new_u(6, 42).unwrap());
                    let p = b.add_new_ptr(value).unwrap();
                    let (p, read) = b.add_read_ptr(p, ty.clone()).unwrap();
                    let result = b.add_free_ptr(p, ty.clone()).unwrap();
                    let [_] = b.build_unwrap_sum(1, option_type([ty]), result).unwrap();
                    b.finish_hugr_with_outputs([read]).unwrap()
                });
            crate::process_hugr(platform, &mut hugr).unwrap();
            let ctx = hugr::llvm::inkwell::context::Context::create();
            let (module, _) = crate::get_hugr_llvm_module(
                &ctx,
                Rc::new(Namer::new("", false)),
                &hugr,
                "pointers",
                Rc::new(codegen_extensions(platform)),
                EmitDebugInfo::Exclude,
            )
            .unwrap();
            module.verify().unwrap();
            for symbol in [
                "___ptr_create",
                "___ptr_get_ptr",
                "___ptr_lock",
                "___ptr_unlock",
                "___ptr_inc_refcount",
            ] {
                assert!(module.get_function(symbol).is_some(), "missing {symbol}");
            }
        }
    }

    #[test]
    fn pointer_equality_uses_identity_without_runtime_hooks() {
        for platform in [QSystemPlatform::Sol, QSystemPlatform::Helios] {
            // A linear payload needs no read or payload-specific codegen for Eq.
            let ty = ptr::ptr_type(int_type(6));
            let pointer = ptr::ptr_type(ty.clone());
            let mut hugr = SimpleHugrConfig::new()
                .with_extensions(REGISTRY.to_owned())
                .with_ins([pointer.clone(), pointer.clone()])
                .with_outs([pointer.clone(), pointer, bool_t()])
                .finish(|mut b| {
                    let [lhs, rhs] = b.input_wires_arr();
                    let (lhs, rhs, equal) = b.add_eq_ptr(lhs, rhs, ty).unwrap();
                    b.finish_hugr_with_outputs([lhs, rhs, equal]).unwrap()
                });
            crate::process_hugr(platform, &mut hugr).unwrap();
            let ctx = hugr::llvm::inkwell::context::Context::create();
            let (module, _) = crate::get_hugr_llvm_module(
                &ctx,
                Rc::new(Namer::new("", false)),
                &hugr,
                "pointer_equality",
                Rc::new(codegen_extensions(platform)),
                EmitDebugInfo::Exclude,
            )
            .unwrap();
            module.verify().unwrap();
            let ir = module.print_to_string().to_string();
            assert!(ir.contains("icmp eq ptr"));
            assert!(!ir.contains("___ptr_"), "Eq must not emit runtime hooks");
            assert!(!ir.contains("getelementptr"));
            assert!(!ir.contains("atomic"));
        }
    }
}
