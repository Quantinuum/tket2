//! The compiler for HUGR to QIS
pub mod array;

use anyhow::{Result, anyhow};
use hugr::envelope::EnvelopeConfig;
use hugr::llvm::CodegenExtsBuilder;
use hugr::llvm::custom::CodegenExtsMap;
use hugr::llvm::emit::{EmitHugr, Namer};
use hugr::llvm::extension::int::IntCodegenExtension;
use hugr::llvm::utils::fat::FatExt as _;
use hugr::llvm::utils::inline_constant_functions;
use inkwell::OptimizationLevel;
use inkwell::context::Context;
use inkwell::module::Module;
use inkwell::passes::PassBuilderOptions;
use inkwell::support::LLVMString;
use inkwell::targets::{
    CodeModel, InitializationConfig, RelocMode, Target, TargetMachine, TargetTriple,
};
use itertools::Itertools;
use pyo3::prelude::*;
use tket::hugr::ops::DataflowParent;

use std::error::Error;
use std::fmt::{self, Display, Formatter};
use std::path::PathBuf;
use std::rc::Rc;
use std::vec::Vec;
use std::{fs, str, vec};
use tket::extension::rotation::ROTATION_EXTENSION;
use tket::extension::{TKET_EXTENSION, TKET1_EXTENSION};
use tket::hugr::extension::{ExtensionRegistry, prelude};
use tket::hugr::std_extensions::arithmetic::{
    conversions, float_ops, float_types, int_ops, int_types,
};
use tket::hugr::std_extensions::{collections, logic, ptr};
use tket::hugr::{self, llvm::inkwell};
use tket::hugr::{Hugr, HugrView, Node};
use tket::llvm::rotation::RotationCodegenExtension;
use tket_qsystem::QSystemPass;
use tket_qsystem::extension::{futures as qsystem_futures, qsystem, result as qsystem_result};
use tket_qsystem::llvm::array_utils::ArrayLowering;
pub use tket_qsystem::llvm::futures::FuturesCodegenExtension;
use tket_qsystem::llvm::{
    debug::DebugCodegenExtension, prelude::QISPreludeCodegen, qsystem::QSystemCodegenExtension,
    random::RandomCodegenExtension, result::ResultsCodegenExtension, utils::UtilsCodegenExtension,
};
use tracing::{Level, event, instrument};
use utils::read_hugr_envelope;

mod gpu;
mod selene_specific;
mod utils;

const LLVM_MAIN: &str = "qmain";
const METADATA: &[(&str, &[&str])] = &[("name", &["mainlib"])];

static REGISTRY: std::sync::LazyLock<ExtensionRegistry> = std::sync::LazyLock::new(|| {
    ExtensionRegistry::new([
        prelude::PRELUDE.to_owned(),
        int_types::EXTENSION.to_owned(),
        int_ops::EXTENSION.to_owned(),
        float_types::EXTENSION.to_owned(),
        float_ops::EXTENSION.to_owned(),
        conversions::EXTENSION.to_owned(),
        logic::EXTENSION.to_owned(),
        ptr::EXTENSION.to_owned(),
        collections::list::EXTENSION.to_owned(),
        collections::array::EXTENSION.to_owned(),
        collections::static_array::EXTENSION.to_owned(),
        collections::borrow_array::EXTENSION.to_owned(),
        collections::value_array::EXTENSION.to_owned(),
        qsystem_futures::EXTENSION.to_owned(),
        qsystem_result::EXTENSION.to_owned(),
        qsystem::EXTENSION.to_owned(),
        ROTATION_EXTENSION.to_owned(),
        TKET_EXTENSION.to_owned(),
        TKET1_EXTENSION.to_owned(),
        tket::extension::bool::BOOL_EXTENSION.to_owned(),
        tket::extension::debug::DEBUG_EXTENSION.to_owned(),
        tket_qsystem::extension::gpu::EXTENSION.to_owned(),
        tket_qsystem::extension::wasm::EXTENSION.to_owned(),
    ])
});

#[derive(Debug)]
/// Handles a series of errors
struct ProcessErrs(Vec<String>);

impl Display for ProcessErrs {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        for s in &self.0 {
            f.write_str(&format!("{s}\n"))?;
        }
        Ok(())
    }
}

impl From<String> for ProcessErrs {
    fn from(value: String) -> Self {
        Self(vec![value])
    }
}

impl From<LLVMString> for ProcessErrs {
    fn from(value: LLVMString) -> Self {
        Self(vec![value.to_string()])
    }
}

impl From<&str> for ProcessErrs {
    fn from(value: &str) -> Self {
        Self(vec![value.to_string()])
    }
}

impl From<Vec<String>> for ProcessErrs {
    fn from(value: Vec<String>) -> Self {
        Self(value)
    }
}

impl Error for ProcessErrs {}

/// create an llvm module from hugr via hugr-llvm
fn get_hugr_llvm_module<'c, 'hugr, 'a: 'c>(
    context: &'c Context,
    namer: Rc<Namer>,
    hugr: &'hugr Hugr,
    module_name: impl AsRef<str>,
    exts: Rc<CodegenExtsMap<'a, Hugr>>,
) -> Result<Module<'c>> {
    let module = context.create_module(module_name.as_ref());
    let emit = EmitHugr::new(context, module, namer, exts);
    Ok(emit
        .emit_module(hugr.try_fat(hugr.module_root()).unwrap())?
        .finish())
}

fn process_hugr(hugr: &mut Hugr) -> Result<()> {
    QSystemPass::default().run(hugr)?;
    // with_entrypoint(hugr, hugr.module_root(), |hugr| {
    //     // `with_entrypoint` returns Rerooted, which the pass expects a bare Hugr.
    // })?;
    inline_constant_functions(hugr)?;
    Ok(())
}

fn codegen_extensions() -> CodegenExtsMap<'static, Hugr> {
    use array::SeleneHeapArrayCodegen;
    let pcg = QISPreludeCodegen;
    CodegenExtsBuilder::default()
        .add_prelude_extensions(pcg.clone())
        .add_extension(IntCodegenExtension::new(pcg.clone()))
        .add_float_extensions()
        .add_conversion_extensions()
        .add_logic_extensions()
        .add_extension(SeleneHeapArrayCodegen::LOWERING.codegen_extension())
        .add_default_static_array_extensions()
        .add_borrow_array_extensions(array::SeleneHeapBorrowArrayCodegen(pcg.clone()))
        .add_extension(FuturesCodegenExtension)
        .add_extension(QSystemCodegenExtension::from(pcg.clone()))
        .add_extension(RandomCodegenExtension)
        // Results use standard arrays.
        .add_extension(ResultsCodegenExtension::new(
            SeleneHeapArrayCodegen::LOWERING,
        ))
        .add_extension(RotationCodegenExtension::new(pcg))
        .add_extension(UtilsCodegenExtension)
        // State results use standard arrays.
        .add_extension(DebugCodegenExtension::new(SeleneHeapArrayCodegen::LOWERING))
        .add_extension(gpu::GpuCodegen)
        .finish()
}

/// given an LLVM context and hugr, compile to an LLVM module.
/// Returns the LLVM Module and the [Node] of the entry point.
fn get_module_with_std_exts<'c>(
    args: &CompileArgs,
    context: &'c Context,
    namer: Rc<Namer>,
    hugr: &'c mut Hugr,
) -> Result<Module<'c>> {
    process_hugr(hugr)?;
    if let Some(filename) = &args.save_hugr {
        let file = fs::File::create(PathBuf::from(filename))?;
        hugr.store(file, EnvelopeConfig::text())?;
    }
    get_hugr_llvm_module(
        context,
        namer,
        hugr,
        &args.name,
        Rc::new(codegen_extensions()),
    )
}

/// Optimize the module using LLVM passes
fn optimize_module(module: &Module, args: &CompileArgs) -> Result<()> {
    let opt_str = match args.opt_level {
        OptimizationLevel::Aggressive => "default<O3>",
        OptimizationLevel::Less => "default<O1>",
        OptimizationLevel::None => "default<O0>",
        OptimizationLevel::Default => "default<O2>",
    };
    module
        .run_passes(opt_str, args.target_machine, PassBuilderOptions::create())
        .map_err(Into::<ProcessErrs>::into)?;
    Ok(())
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
) -> Result<()> {
    let entry_ty = ctx.i64_type().fn_type(&[ctx.i64_type().into()], false);
    let entry_fun = module.add_function(module_entry, entry_ty, None);
    let setup_type = ctx.void_type().fn_type(&[ctx.i64_type().into()], false);
    let setup = module.add_function("setup", setup_type, None);
    let teardown_type = ctx.i64_type().fn_type(&[], false);
    let teardown = module.add_function("teardown", teardown_type, None);
    let block = ctx.append_basic_block(entry_fun, "entry");
    let builder = ctx.create_builder();
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
        .left()
        .ok_or_else(|| anyhow!("get_tc has no return value"))?;
    // Return the initial time cursor
    let _ = builder.build_return(Some(&tc))?;
    Ok(())
}

#[derive(Debug)]
struct CompileArgs<'a> {
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
}

impl<'a> CompileArgs<'a> {
    fn new(
        name: &impl ToString,
        target_machine: &'a TargetMachine,
        opt_level: OptimizationLevel,
    ) -> Self {
        Self {
            entry: None,
            name: name.to_string(),
            save_hugr: None,
            target_machine,
            opt_level,
        }
    }
}

/// Compile the given HUGR to an LLVM module.
/// This function is the primary entry point for the compiler.
#[instrument(skip(ctx, hugr),parent = None)]
fn compile<'c, 'hugr: 'c>(
    args: &CompileArgs,
    ctx: &'c Context,
    hugr: &'hugr mut Hugr,
) -> Result<Module<'c>> {
    event!(Level::DEBUG, "starting primary compilation");
    let namer = Rc::new(Namer::new("__hugr__.", true));

    // Find the name of the LLVM function that corresponds to the entry point in
    // the HUGR.
    let hugr_entry = get_entry_point_name(&namer, hugr)?;

    // The name of the entry point in the LLVM module.
    // The function will wrap `hugr_entry`.
    let module_entry = args.entry.as_ref().map_or(LLVM_MAIN, |x| x.as_ref());

    // Create a new LLVM module using hugr-llvm
    let module = get_module_with_std_exts(args, ctx, namer, hugr)?;

    wrap_main(ctx, &module, &hugr_entry, module_entry)?;

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
        let _ = module
            .add_global_metadata(key, &node)
            .map_err(Into::<ProcessErrs>::into);
    }
    module.verify().map_err(Into::<ProcessErrs>::into)?;
    Ok(module)
}

/// Get the Inkwell TargetMachine for the current platform, given
/// the provided optimization level.
pub fn get_native_target_machine(opt_level: OptimizationLevel) -> Result<TargetMachine> {
    let reloc_mode = RelocMode::PIC;
    let code_model = CodeModel::Default;
    Target::initialize_native(&InitializationConfig::default()).unwrap();
    let triple = TargetMachine::get_default_triple();
    let target = Target::from_triple(&triple).map_err(|e| anyhow!("{e}"))?;
    target
        .create_target_machine(
            &triple,
            &TargetMachine::get_host_cpu_name().to_string_lossy(),
            &TargetMachine::get_host_cpu_features().to_string_lossy(),
            opt_level,
            reloc_mode,
            code_model,
        )
        .ok_or_else(|| anyhow!("Failed to create target machine"))
}

/// Get the Inkwell TargetMachine for the current platform, given
/// the provided optimization level.
pub fn get_target_machine_from_triple(
    target_triple: &str,
    opt_level: OptimizationLevel,
) -> Result<TargetMachine> {
    let reloc_mode = RelocMode::PIC;
    let code_model = CodeModel::Default;
    Target::initialize_all(&InitializationConfig::default());
    let triple = TargetTriple::create(target_triple);
    eprintln!("Using target triple: {triple}");
    let target = Target::from_triple(&triple).map_err(|e| anyhow!("{e}"))?;
    eprintln!("Using target: {:?}", target.get_name());
    let cpu: String = target.get_name().to_string_lossy().to_string();
    target
        .create_target_machine(&triple, &cpu, "", opt_level, reloc_mode, code_model)
        .ok_or_else(|| anyhow!("Failed to create target machine"))
}

/// Get the optimization level for the given integer value.
pub fn get_opt_level(opt_level: u32) -> Result<OptimizationLevel> {
    match opt_level {
        0 => Ok(OptimizationLevel::None),
        1 => Ok(OptimizationLevel::Less),
        2 => Ok(OptimizationLevel::Default),
        3 => Ok(OptimizationLevel::Aggressive),
        _ => panic!("Invalid optimization level: {opt_level}"),
    }
}

// -------------------- Python bindings -----------------------
mod exceptions {
    use pyo3::exceptions::PyException;

    pyo3::create_exception!(selene_hugr_qis_compiler, HugrReadError, PyException);
}
#[pymodule]
mod selene_hugr_qis_compiler {
    use super::{
        CompileArgs, Context, Hugr, PyResult, compile, get_native_target_machine, get_opt_level,
        get_target_machine_from_triple, pyfunction, read_hugr_envelope,
    };

    #[pymodule_export]
    use super::exceptions::HugrReadError;

    fn py_read_envelope(pkg_bytes: &[u8]) -> PyResult<Hugr> {
        read_hugr_envelope(pkg_bytes).map_err(|e| HugrReadError::new_err(format!("{e:?}")))
    }

    /// Load serialized HUGR and validate it
    #[pyfunction]
    pub fn check_hugr(pkg_bytes: &[u8]) -> PyResult<()> {
        py_read_envelope(pkg_bytes).map(|_| ())
    }

    /// Compile HUGR package to LLVM IR string
    #[pyfunction]
    #[pyo3(signature = (pkg_bytes, opt_level=2, target_triple="native"))]
    pub fn compile_to_llvm_ir(
        pkg_bytes: &[u8],
        opt_level: u32,
        target_triple: &str,
    ) -> PyResult<String> {
        let opt = get_opt_level(opt_level)?;
        let target_machine = if target_triple == "native" {
            get_native_target_machine(opt)
        } else {
            get_target_machine_from_triple(target_triple, opt)
        }?;
        let mut hugr = py_read_envelope(pkg_bytes)?;
        let ctx = Context::create();
        let llvm_module = compile(
            &CompileArgs::new(&"hugr", &target_machine, opt),
            &ctx,
            &mut hugr,
        )?;
        Ok(llvm_module.to_string())
    }

    /// Compile HUGR package to LLVM bitcode
    #[pyfunction]
    #[pyo3(signature = (pkg_bytes, opt_level=2, target_triple="native"))]
    pub fn compile_to_bitcode(
        pkg_bytes: &[u8],
        opt_level: u32,
        target_triple: &str,
    ) -> PyResult<Vec<u8>> {
        let opt = get_opt_level(opt_level)?;
        let target_machine = if target_triple == "native" {
            get_native_target_machine(opt)
        } else {
            get_target_machine_from_triple(target_triple, opt)
        }?;
        let mut hugr = py_read_envelope(pkg_bytes)?;
        let ctx = Context::create();
        let llvm_module = compile(
            &CompileArgs::new(&"hugr", &target_machine, opt),
            &ctx,
            &mut hugr,
        )?;
        Ok(llvm_module.write_bitcode_to_memory().as_slice().to_vec())
    }
}
