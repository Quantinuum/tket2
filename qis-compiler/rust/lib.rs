//! The compiler for HUGR to QIS
pub mod array;

use anyhow::{Result, anyhow};
use hugr::envelope::EnvelopeConfig;
use hugr::llvm::custom::CodegenExtsMap;
use hugr::llvm::emit::{EmitDebugInfo, EmitHugr, Namer, debug_info::DebugInfoContext};
use hugr::llvm::utils::fat::FatExt as _;
use inkwell::OptimizationLevel;
use inkwell::context::Context;
use inkwell::module::Module;
use inkwell::targets::TargetMachine;
use itertools::Itertools;
#[cfg(feature = "py")]
use pyo3::prelude::*;
use tket::hugr::ops::DataflowParent;
use tket::passes::composable::ComposablePass;
use tket_qsystem::{QSystemLLVMPass, QSystemRebasePass};

use extensions::codegen_extensions;
use optimization::optimize_module;
use std::path::PathBuf;
use std::rc::Rc;
use std::vec::Vec;
use std::{fs, str};
use tket::hugr::{Hugr, HugrView, Node};
use tket_qsystem::extension::qsystem;
use tracing::{Level, event, instrument};
use utils::read_hugr_envelope;

pub use tket::hugr::{self, llvm::inkwell};
pub use tket_qsystem::{self, extension::REGISTRY, llvm::futures::FuturesCodegenExtension};
pub use utils::validate;

mod emulator;
mod extensions;
mod gpu;
mod optimization;
mod selene_specific;
mod target;
mod utils;

pub use emulator::EmulatorState;
pub use target::{
    get_native_target_machine, get_opt_level, get_platform, get_target_machine_from_triple,
};

const LLVM_MAIN: &str = "qmain";
const METADATA: &[(&str, &[&str])] = &[("name", &["mainlib"])];

/// create an llvm module from hugr via hugr-llvm
fn get_hugr_llvm_module<'c, 'a: 'c>(
    context: &'c Context,
    namer: Rc<Namer>,
    hugr: &Hugr,
    module_name: impl AsRef<str>,
    exts: Rc<CodegenExtsMap<'a, Hugr>>,
    emit_debug: EmitDebugInfo,
) -> Result<(Module<'c>, Option<DebugInfoContext<'c>>)> {
    let module = context.create_module(module_name.as_ref());
    let emit = EmitHugr::new(context, module, namer, exts);
    let module_root = hugr
        .try_fat(hugr.module_root())
        .ok_or_else(|| anyhow!("module root has an unexpected HUGR operation type"))?;
    Ok(emit.emit_module(module_root, emit_debug)?.finish())
}

fn process_hugr(platform: qsystem::QSystemPlatform, hugr: &mut Hugr) -> Result<()> {
    QSystemRebasePass::defaults(platform).run(hugr)?;
    QSystemLLVMPass::default().run(hugr)?;
    Ok(())
}

/// given an LLVM context and hugr, compile to an LLVM module.
/// Returns the LLVM Module and the [Node] of the entry point.
fn get_module_from_prepared_hugr<'c>(
    args: &CompileArgs,
    context: &'c Context,
    namer: Rc<Namer>,
    hugr: &Hugr,
) -> Result<(Module<'c>, Option<DebugInfoContext<'c>>)> {
    if let Some(filename) = &args.save_hugr {
        let file = fs::File::create(PathBuf::from(filename))?;
        hugr.store(file, EnvelopeConfig::text())?;
    }
    get_hugr_llvm_module(
        context,
        namer,
        hugr,
        &args.name,
        Rc::new(codegen_extensions(args.platform)),
        args.emit_debug,
    )
}

/// Copy LLVM bitcode into a public byte buffer.
///
/// LLVM's in-memory bitcode writer appends an implicit trailing NUL byte. That
/// terminator is required for some in-process LLVM APIs but must not be exposed
/// in the public bitcode payload.
fn public_bitcode_bytes(memory_buffer: &inkwell::memory_buffer::MemoryBuffer<'_>) -> Vec<u8> {
    let bytes = memory_buffer.as_slice();
    match bytes.last() {
        Some(0) => bytes[..bytes.len() - 1].to_vec(),
        _ => bytes.to_vec(),
    }
}

fn get_entry_point_name(namer: &Namer, hugr: &impl HugrView<Node = Node>) -> Result<String> {
    const HUGR_MAIN: &str = "main";
    let (name, entry_point_node) = if hugr.entrypoint_optype().is_module() {
        // for backwards compatibility with old Guppy versions:
        // assume entrypoint is "main" function in module.

        let node = hugr
            .children(hugr.module_root())
            .filter(|&n| {
                hugr.get_optype(n)
                    .as_func_defn()
                    .is_some_and(|f| f.func_name() == HUGR_MAIN)
            })
            .exactly_one()
            .map_err(|_| {
                anyhow!("Module entrypoint must have a single function named {HUGR_MAIN} as child")
            })?;
        (HUGR_MAIN, node)
    } else {
        let func_defn = hugr
            .entrypoint_optype()
            .as_func_defn()
            .ok_or_else(|| anyhow!("Entry point node is not a function definition"))?;
        if func_defn.inner_signature().input_count() != 0 {
            return Err(anyhow!(
                "Entry point function must have no input parameters (found {})",
                func_defn.inner_signature().input_count()
            ));
        }
        (func_defn.func_name().as_ref(), hugr.entrypoint())
    };

    Ok(namer.name_func(name, entry_point_node))
}

fn wrap_main<'c>(
    ctx: &'c Context,
    module: &Module<'c>,
    hugr_entry: &str,
    module_entry: &str,
    mut maybe_di_ctx: Option<&mut DebugInfoContext<'c>>,
) -> Result<()> {
    let entry_ty = ctx.i64_type().fn_type(&[ctx.i64_type().into()], false);
    let entry_fun = module.add_function(module_entry, entry_ty, None);
    let setup_type = ctx.void_type().fn_type(&[ctx.i64_type().into()], false);
    let setup = module.add_function("setup", setup_type, None);
    let teardown_type = ctx.i64_type().fn_type(&[], false);
    let teardown = module.add_function("teardown", teardown_type, None);
    let block = ctx.append_basic_block(entry_fun, "entry");
    let builder = ctx.create_builder();

    if let Some(ref mut di_ctx) = maybe_di_ctx {
        di_ctx.set_compiler_generated(entry_fun, ctx, &builder)?;
    }

    builder.position_at_end(block);

    let initial_tc = entry_fun.get_nth_param(0).unwrap().into_int_value();
    let hugr_main = module
        .get_function(hugr_entry)
        .ok_or_else(|| anyhow!("Entrypoint function '{hugr_entry}' not found in Module"))?;

    let _ = builder.build_call(setup, &[initial_tc.into()], "")?;
    let _ = builder.build_call(hugr_main, &[], "")?;
    let tc = builder
        .build_call(teardown, &[], "")?
        .try_as_basic_value()
        .basic()
        .ok_or_else(|| anyhow!("get_tc has no return value"))?;
    // Return the initial time cursor
    let _ = builder.build_return(Some(&tc))?;

    if let Some(di_ctx) = maybe_di_ctx {
        di_ctx.unset_debug_loc(&builder)?;
    }
    Ok(())
}

/// Compilation arguments.
#[derive(Debug)]
pub struct CompileArgs<'a> {
    /// Entry point symbol
    entry: Option<String>,
    /// LLVM module name
    name: String,
    /// Save Hugr to file
    save_hugr: Option<String>,
    /// Target machine
    target_machine: &'a TargetMachine,
    /// Optimization level
    opt_level: OptimizationLevel,
    /// Target quantum platform
    platform: qsystem::QSystemPlatform,
    /// Debug info configuration
    emit_debug: EmitDebugInfo,
}

impl<'a> CompileArgs<'a> {
    /// Create compiler arguments.
    pub fn new(
        name: &impl ToString,
        target_machine: &'a TargetMachine,
        opt_level: OptimizationLevel,
        platform: qsystem::QSystemPlatform,
        iw_ctx: &Context,
        emit_debug: bool,
    ) -> Self {
        let emit_debug_arg = if emit_debug {
            EmitDebugInfo::Include {
                ptr_bits: iw_ctx
                    .ptr_sized_int_type(&target_machine.get_target_data(), Default::default())
                    .get_bit_width(),
            }
        } else {
            EmitDebugInfo::Exclude
        };

        Self {
            entry: None,
            name: name.to_string(),
            save_hugr: None,
            target_machine,
            opt_level,
            platform,
            emit_debug: emit_debug_arg,
        }
    }
}

/// Compile the given HUGR to an LLVM module.
/// This function is the primary entry point for the compiler.
#[instrument(skip(ctx, hugr),parent = None)]
pub fn compile<'c, 'hugr: 'c>(
    args: &CompileArgs,
    ctx: &'c Context,
    hugr: &'hugr mut Hugr,
) -> Result<Module<'c>> {
    process_hugr(args.platform, hugr)?;
    compile_prepared(args, ctx, hugr)
}

/// Compile a HUGR after the platform-specific lowering passes have run.
fn compile_prepared<'c>(args: &CompileArgs, ctx: &'c Context, hugr: &Hugr) -> Result<Module<'c>> {
    event!(Level::DEBUG, "starting primary compilation");
    let namer = Rc::new(Namer::new("__hugr__.", true));

    // Find the name of the LLVM function that corresponds to the entry point in
    // the HUGR.
    let hugr_entry = get_entry_point_name(&namer, hugr)?;

    // The name of the entry point in the LLVM module.
    // The function will wrap `hugr_entry`.
    let module_entry = args.entry.as_ref().map_or(LLVM_MAIN, |x| x.as_ref());

    // Create a new LLVM module using hugr-llvm
    let (module, mut maybe_di_ctx) = get_module_from_prepared_hugr(args, ctx, namer, hugr)?;

    wrap_main(
        ctx,
        &module,
        &hugr_entry,
        module_entry,
        maybe_di_ctx.as_mut(),
    )?;

    let (data_layout, triple) = {
        (
            args.target_machine.get_target_data().get_data_layout(),
            args.target_machine.get_triple(),
        )
    };
    module.set_triple(&triple);
    module.set_data_layout(&data_layout);

    optimize_module(&module, args)?;

    // Add metadata to the module
    for (key, values) in METADATA {
        let md_vec = values
            .iter()
            .map(|v| ctx.metadata_string(v).into())
            .collect::<Vec<_>>();
        let node = ctx.metadata_node(md_vec.as_slice());
        module
            .add_global_metadata(key, &node)
            .map_err(|error| anyhow!(error.to_string()))?;
    }

    if let Some(di_ctx) = maybe_di_ctx.take() {
        di_ctx.finish();
    }
    module
        .verify()
        .map_err(|error| anyhow!(error.to_string()))?;

    Ok(module)
}

// -------------------- Python bindings -----------------------
#[cfg(feature = "py")]
mod exceptions {
    use pyo3::exceptions::PyException;

    pyo3::create_exception!(selene_hugr_qis_compiler, HugrReadError, PyException);
}
#[cfg(feature = "py")]
#[pymodule]
mod selene_hugr_qis_compiler {
    use super::{
        CompileArgs, Context, Hugr, PyResult, get_native_target_machine, get_opt_level,
        get_platform, get_target_machine_from_triple, public_bitcode_bytes, pyfunction,
        read_hugr_envelope,
    };
    use crate::extensions::{
        embedded_extensions as rust_extensions,
        has_compatible_extension as rust_has_compatible_extension,
    };
    use pyo3::pymethods;

    #[pymodule_export]
    use super::EmulatorState;
    #[pymodule_export]
    use super::exceptions::HugrReadError;

    fn py_read_envelope(pkg_bytes: &[u8]) -> PyResult<Hugr> {
        read_hugr_envelope(pkg_bytes).map_err(|e| HugrReadError::new_err(format!("{e:?}")))
    }

    pub(crate) fn py_emulator_state(pkg_bytes: &[u8], platform: &str) -> PyResult<EmulatorState> {
        let platform = get_platform(platform)?;
        let hugr = py_read_envelope(pkg_bytes)?;
        Ok(EmulatorState::from_validated_hugr(hugr, platform)?)
    }

    #[pymethods]
    impl EmulatorState {
        #[new]
        #[pyo3(signature = (pkg_bytes, *, platform="helios"))]
        fn py_new(pkg_bytes: &[u8], platform: &str) -> PyResult<Self> {
            py_emulator_state(pkg_bytes, platform)
        }

        /// Compile the prepared HUGR to an LLVM IR string.
        #[pyo3(signature = (*, opt_level=2, target_triple="native", emit_debug=false))]
        pub(crate) fn compile_to_llvm_ir(
            &self,
            opt_level: u32,
            target_triple: &str,
            emit_debug: bool,
        ) -> PyResult<String> {
            let opt = get_opt_level(opt_level)?;
            let target_machine = if target_triple == "native" {
                get_native_target_machine(opt)
            } else {
                get_target_machine_from_triple(target_triple, opt)
            }?;
            let ctx = Context::create();
            let llvm_module = self.compile(
                &CompileArgs::new(
                    &"hugr",
                    &target_machine,
                    opt,
                    self.platform(),
                    &ctx,
                    emit_debug,
                ),
                &ctx,
            )?;
            Ok(llvm_module.to_string())
        }

        /// Compile the prepared HUGR to LLVM bitcode.
        #[pyo3(signature = (*, opt_level=2, target_triple="native", emit_debug=false))]
        pub(crate) fn compile_to_bitcode(
            &self,
            opt_level: u32,
            target_triple: &str,
            emit_debug: bool,
        ) -> PyResult<Vec<u8>> {
            let opt = get_opt_level(opt_level)?;
            let target_machine = if target_triple == "native" {
                get_native_target_machine(opt)
            } else {
                get_target_machine_from_triple(target_triple, opt)
            }?;
            let ctx = Context::create();
            let llvm_module = self.compile(
                &CompileArgs::new(
                    &"hugr",
                    &target_machine,
                    opt,
                    self.platform(),
                    &ctx,
                    emit_debug,
                ),
                &ctx,
            )?;
            Ok(public_bitcode_bytes(&llvm_module.write_bitcode_to_memory()))
        }
    }

    /// Return extension names and versions available to the native loader.
    #[pyfunction]
    fn embedded_extensions() -> Vec<(String, String)> {
        rust_extensions()
    }

    /// Return whether the native loader can provide a compatible extension.
    #[pyfunction]
    fn has_compatible_extension(name: &str, version: &str) -> PyResult<bool> {
        Ok(rust_has_compatible_extension(name, version)?)
    }

    /// Load serialized HUGR and validate it
    #[pyfunction]
    pub fn check_hugr(pkg_bytes: &[u8]) -> PyResult<()> {
        py_read_envelope(pkg_bytes).map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "py")]
    use super::selene_hugr_qis_compiler::py_emulator_state;
    use std::{
        fs,
        time::{SystemTime, UNIX_EPOCH},
    };
    use tket::hugr::llvm::inkwell::{
        context::Context, memory_buffer::MemoryBuffer, module::Module,
    };

    fn parse_bitcode_as_file(bitcode: &[u8]) -> Result<Module<'static>, String> {
        let file_name = format!(
            "selene-hugr-qis-compiler-{}.bc",
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_err(|e| format!("Failed to compute timestamp: {e}"))?
                .as_nanos()
        );
        let path = std::env::temp_dir().join(file_name);
        fs::write(&path, bitcode).map_err(|e| format!("Failed to write temp bitcode: {e}"))?;
        let ctx: &'static Context = Box::leak(Box::new(Context::create()));
        let result = MemoryBuffer::create_from_file(&path)
            .map_err(|e| format!("Failed to read temp bitcode: {e}"))
            .and_then(|memory_buffer| {
                Module::parse_bitcode_from_buffer(&memory_buffer, ctx)
                    .map_err(|e| format!("Failed to parse bitcode: {e}"))
            });
        let _ = fs::remove_file(&path);
        result
    }

    #[cfg(feature = "py")]
    #[test]
    fn test_compile_to_bitcode_returns_file_safe_public_bytes() {
        let hugr = include_bytes!("../python/tests/resources/check.hugr");
        let state = py_emulator_state(hugr, "helios")
            .expect("preparing fixture for the emulator should work");
        let bitcode = state
            .compile_to_bitcode(2, "native", false)
            .expect("compiling fixture to bitcode should work");

        let module =
            parse_bitcode_as_file(&bitcode).expect("returned bitcode should parse from file");
        let raw_buffer = module.write_bitcode_to_memory();
        assert_eq!(raw_buffer.as_slice().last(), Some(&0));
        assert_eq!(
            bitcode,
            raw_buffer.as_slice()[..raw_buffer.as_slice().len() - 1],
            "Public bitcode should match LLVM's raw buffer without the implicit trailing NUL"
        );
    }

    /// A program with many `if <a or b or ...>:` branches over distinct booleans
    /// used to trigger a superlinear SLP-vectorizer blowup at the default opt
    /// level. It must still compile to valid, parseable bitcode with SLP off.
    #[cfg(feature = "py")]
    #[test]
    fn test_compile_or_chain_program() {
        let hugr = include_bytes!("../python/tests/resources/slp_or_chain.hugr");
        let state = py_emulator_state(hugr, "helios")
            .expect("preparing fixture for the emulator should work");
        let bitcode = state
            .compile_to_bitcode(2, "native", false)
            .expect("compiling the or-chain fixture to bitcode should work");

        parse_bitcode_as_file(&bitcode).expect("or-chain bitcode should parse from file");
    }
}
