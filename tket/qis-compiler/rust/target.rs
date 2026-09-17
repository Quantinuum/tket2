//! LLVM target-machine and compiler-option parsing.

use crate::inkwell::OptimizationLevel;
use crate::inkwell::targets::{
    CodeModel, InitializationConfig, RelocMode, Target, TargetMachine, TargetTriple,
};
use anyhow::{Result, anyhow};
use tket_qsystem::QSystemPlatform;
use tracing::debug;

/// Get an LLVM target machine for the current host.
///
/// # Errors
///
/// Returns an error when LLVM cannot initialize or construct the native target.
pub fn get_native_target_machine(opt_level: OptimizationLevel) -> Result<TargetMachine> {
    Target::initialize_native(&InitializationConfig::default())
        .map_err(|error| anyhow!("failed to initialize native target: {error}"))?;
    let triple = TargetMachine::get_default_triple();
    let target = Target::from_triple(&triple).map_err(|error| anyhow!(error.to_string()))?;
    target
        .create_target_machine(
            &triple,
            &TargetMachine::get_host_cpu_name().to_string_lossy(),
            &TargetMachine::get_host_cpu_features().to_string_lossy(),
            opt_level,
            RelocMode::PIC,
            CodeModel::Default,
        )
        .ok_or_else(|| anyhow!("failed to create native target machine"))
}

/// Get an LLVM target machine for an explicit target triple.
///
/// # Errors
///
/// Returns an error when LLVM cannot resolve or construct the requested target.
pub fn get_target_machine_from_triple(
    target_triple: &str,
    opt_level: OptimizationLevel,
) -> Result<TargetMachine> {
    Target::initialize_all(&InitializationConfig::default());
    let triple = TargetTriple::create(target_triple);
    let target = Target::from_triple(&triple).map_err(|error| anyhow!(error.to_string()))?;
    let target_name = target.get_name().to_string_lossy();
    debug!(target_triple = %triple, target = %target_name, "resolved LLVM target");
    target
        .create_target_machine(
            &triple,
            &target_name,
            "",
            opt_level,
            RelocMode::PIC,
            CodeModel::Default,
        )
        .ok_or_else(|| anyhow!("failed to create target machine for {triple}"))
}

/// Convert a numeric optimization level into LLVM's representation.
///
/// # Errors
///
/// Returns an error unless `opt_level` is between zero and three inclusive.
pub fn get_opt_level(opt_level: u32) -> Result<OptimizationLevel> {
    match opt_level {
        0 => Ok(OptimizationLevel::None),
        1 => Ok(OptimizationLevel::Less),
        2 => Ok(OptimizationLevel::Default),
        3 => Ok(OptimizationLevel::Aggressive),
        _ => Err(anyhow!("invalid optimization level: {opt_level}")),
    }
}

/// Parse a supported QSystem platform name.
///
/// # Errors
///
/// Returns an error unless `platform` is `"helios"` or `"sol"`, ignoring case.
pub fn get_platform(platform: &str) -> Result<QSystemPlatform> {
    match platform.to_lowercase().as_str() {
        "helios" => Ok(QSystemPlatform::Helios),
        "sol" => Ok(QSystemPlatform::Sol),
        _ => Err(anyhow!(
            "unknown platform: {platform} (expected 'helios' or 'sol')"
        )),
    }
}

#[cfg(test)]
mod tests {
    use crate::inkwell::OptimizationLevel;
    use rstest::rstest;

    use super::get_opt_level;

    #[rstest]
    #[case(0, OptimizationLevel::None)]
    #[case(1, OptimizationLevel::Less)]
    #[case(2, OptimizationLevel::Default)]
    #[case(3, OptimizationLevel::Aggressive)]
    fn optimization_levels_are_parsed(#[case] value: u32, #[case] expected: OptimizationLevel) {
        assert_eq!(get_opt_level(value).unwrap(), expected);
    }

    #[test]
    fn invalid_optimization_level_is_an_error() {
        let error = get_opt_level(4).unwrap_err();
        assert_eq!(error.to_string(), "invalid optimization level: 4");
    }
}
