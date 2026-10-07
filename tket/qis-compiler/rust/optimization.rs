//! LLVM optimization configuration.

use std::ffi::CString;
use std::sync::Once;

use crate::inkwell::OptimizationLevel;
use crate::inkwell::llvm_sys::support::LLVMParseCommandLineOptions;
use crate::inkwell::module::Module;
use crate::inkwell::passes::PassBuilderOptions;
use anyhow::{Result, anyhow};

use crate::CompileArgs;

/// Default cap for the LLVM SLP vectorizer's tree-building recursion depth.
///
/// LLVM's own default is 12. Programs with many `if <a or b or ...>:` branches
/// over distinct runtime booleans lower to a wide/deep graph of `i1` phi/branch
/// values, and `llvm::slpvectorizer::BoUpSLP::buildTreeRec` explores it with a
/// cost that is superlinear in the width. Capping the recursion depth avoids
/// that blowup while retaining the shallow, profitable vectorization the pass
/// normally finds.
const SLP_RECURSION_MAX_DEPTH: u32 = 4;

/// Resolve the SLP recursion cap, honouring a process-level override.
fn resolve_slp_recursion_depth(override_value: Option<String>) -> u32 {
    override_value
        .and_then(|value| value.trim().parse::<u32>().ok())
        .unwrap_or(SLP_RECURSION_MAX_DEPTH)
}

/// Cap the SLP vectorizer's recursion depth via LLVM's global `cl::opt`.
///
/// `-slp-recursion-max-depth` has no C API or [`PassBuilderOptions`] setter, so
/// `LLVMParseCommandLineOptions` is the only available lever. This mutates
/// process-global state and therefore runs exactly once.
fn configure_slp_recursion_depth() {
    static INIT: Once = Once::new();
    INIT.call_once(|| {
        let depth =
            resolve_slp_recursion_depth(std::env::var("SELENE_SLP_RECURSION_MAX_DEPTH").ok());
        let prog = CString::new("selene-hugr-qis-compiler").expect("static string is NUL-free");
        let flag = CString::new(format!("-slp-recursion-max-depth={depth}"))
            .expect("formatted flag is NUL-free");
        let overview = CString::new("").expect("empty string is NUL-free");
        let argv = [prog.as_ptr(), flag.as_ptr()];
        // SAFETY: `argv` holds `argv.len()` valid, NUL-terminated pointers that
        // outlive the call, and `overview` is a valid NUL-terminated string.
        unsafe {
            LLVMParseCommandLineOptions(
                argv.len() as std::ffi::c_int,
                argv.as_ptr(),
                overview.as_ptr(),
            );
        }
    });
}

/// Optimize an LLVM module using the configured optimization level.
///
/// # Errors
///
/// Returns an error when LLVM rejects or fails to run the pass pipeline.
pub(crate) fn optimize_module(module: &Module, args: &CompileArgs) -> Result<()> {
    configure_slp_recursion_depth();
    let passes = match args.opt_level {
        OptimizationLevel::Aggressive => "default<O3>",
        OptimizationLevel::Less => "default<O1>",
        OptimizationLevel::None => "default<O0>",
        OptimizationLevel::Default => "default<O2>",
    };
    module
        .run_passes(passes, args.target_machine, PassBuilderOptions::create())
        .map_err(|error| anyhow!(error.to_string()))
}

#[cfg(test)]
mod tests {
    use super::{SLP_RECURSION_MAX_DEPTH, resolve_slp_recursion_depth};

    #[test]
    fn recursion_depth_parses_override_or_falls_back() {
        assert_eq!(resolve_slp_recursion_depth(None), SLP_RECURSION_MAX_DEPTH);
        assert_eq!(resolve_slp_recursion_depth(Some(" 2 ".to_string())), 2);
        assert_eq!(
            resolve_slp_recursion_depth(Some("garbage".to_string())),
            SLP_RECURSION_MAX_DEPTH
        );
    }
}
