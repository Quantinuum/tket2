//! Utilities for serializing circuits.
//!
//! See [`crate::serialize::pytket`] for serialization to and from the legacy pytket format.
pub mod pytket;

pub use hugr::envelope::{EnvelopeConfig, EnvelopeError};
use hugr::hugr::hugrmut::HugrMut;

use std::io;

use hugr::extension::{ExtensionRegistry, ExtensionRegistryError};
use hugr::hugr::ValidationError;
pub use pytket::{
    TKETDecode, load_tk1_json_file, load_tk1_json_reader, load_tk1_json_str, save_tk1_json_file,
    save_tk1_json_str, save_tk1_json_writer,
};

use derive_more::{Display, Error, From};
use hugr::ops::{OpTag, OpTrait, OpType};
use hugr::package::{Package, PackageValidationError};
use hugr::{Hugr, HugrView, Node};

use crate::extension::REGISTRY;
use crate::{Circuit, CircuitError};

impl<T: HugrView> Circuit<T> {
    /// Store the circuit as a HUGR envelope, using the given configuration.
    pub fn store(
        &self,
        writer: impl io::Write,
        config: EnvelopeConfig,
    ) -> Result<(), EnvelopeError> {
        let pkg = self.wrap_package()?;
        pkg.store(writer, config)?;
        Ok(())
    }

    /// Store the circuit as a String in HUGR envelope format.
    pub fn store_str(&self, config: EnvelopeConfig) -> Result<String, EnvelopeError> {
        let pkg = self.wrap_package()?;
        pkg.store_str(config)
    }

    /// Wrap the circuit in a package.
    fn wrap_package(&self) -> Result<Package, EnvelopeError> {
        let hugr = Circuit::to_owned(self).into_hugr();
        Ok(Package::from_hugr(hugr))
    }
}

impl Circuit<Hugr> {
    /// Load a circuit from a HUGR envelope.
    ///
    /// Returns the first module in the encoded package, using the given entrypoint.
    pub fn load(
        reader: impl io::BufRead,
        extensions: Option<&ExtensionRegistry>,
    ) -> Result<Self, CircuitLoadError> {
        let extensions = extensions.unwrap_or(&REGISTRY);
        let hugr = Hugr::load(reader, Some(extensions))?;
        hugr.validate()?;
        Ok(Self::try_new(hugr)?)
    }
    /// Load a circuit from a string-encoded HUGR envelope.
    ///
    /// Returns the first module in the encoded package, using the given entrypoint.
    pub fn load_str(
        envelope: impl AsRef<str>,
        extensions: Option<&ExtensionRegistry>,
    ) -> Result<Self, CircuitLoadError> {
        let extensions = extensions.unwrap_or(&REGISTRY);
        let hugr = Hugr::load_str(envelope, Some(extensions))?;
        hugr.validate()?;
        Ok(Self::try_new(hugr)?)
    }

    /// Load a circuit from a HUGR envelope.
    ///
    /// Searches each module in the package for the function name, and return
    /// the first match.
    pub fn load_function(
        reader: impl io::BufRead,
        function_name: impl AsRef<str>,
    ) -> Result<Self, CircuitLoadError> {
        let pkg = Package::load(reader, Some(&REGISTRY))?;
        pkg.validate()?;
        Self::unwrap_package(pkg, function_name)
    }

    /// Load a circuit from a String in HUGR envelope format.
    ///
    /// Searches each module in the package for the function name, and return
    /// the first match.
    pub fn load_function_str(
        envelope: impl AsRef<str>,
        function_name: impl AsRef<str>,
    ) -> Result<Self, CircuitLoadError> {
        let pkg = Package::load_str(envelope, Some(&REGISTRY))?;
        pkg.validate()?;
        Self::unwrap_package(pkg, function_name)
    }

    /// Unwrap a circuit from a function in the package.
    ///
    /// Searches each module in the package for the function name, and return
    /// the first match.
    fn unwrap_package(
        pkg: Package,
        function_name: impl AsRef<str>,
    ) -> Result<Self, CircuitLoadError> {
        let Package {
            modules,
            extensions: _,
        } = pkg;
        let (_module_idx, circ) = find_function_in_modules(modules, function_name.as_ref())?;
        Ok(circ)
    }
}

/// Error type for deserialization operations on [`Circuit`]s.
#[derive(Debug, Display, Error, From)]
#[non_exhaustive]
pub enum CircuitLoadError {
    /// Cannot load the circuit file.
    #[display("Cannot load the circuit file: {_0}")]
    #[from]
    InvalidFile(io::Error),
    /// Invalid JSON
    #[display("Invalid HUGR JSON. {_0}")]
    #[from]
    InvalidJson(serde_json::Error),
    /// The root node is not a module operation.
    #[display(
        "Expected a HUGR with a module at the root, but found a {} instead.",
        root_op
    )]
    NonModuleRoot {
        /// The root operation.
        root_op: OpType,
    },
    /// The function is not found in the module.
    #[display(
        "Function '{function}' not found in the loaded module. Available functions: [{}]",
        available_functions.join(", ")
    )]
    FunctionNotFound {
        /// The function name.
        function: String,
        /// The available functions.
        available_functions: Vec<String>,
    },
    /// The function has an invalid control flow structure.
    #[display(
        "Function '{function}' has an invalid control flow structure. Currently only flat functions with no control flow primitives are supported."
    )]
    InvalidControlFlow {
        /// The function name.
        function: String,
    },
    /// Error loading the circuit.
    #[display("Error loading the circuit: {_0}")]
    #[from]
    CircuitLoadError(CircuitError),
    /// Error loading an envelope.
    #[from]
    EnvelopeError(EnvelopeError),
    /// Error validating the loaded circuit.
    #[from]
    ValidationError(ValidationError<Node>),
    /// An error that can occur in defining an extension registry while loading the circuit.
    #[from]
    ExtensionError(ExtensionRegistryError),
    /// The encoded HUGR package must have a single HUGR.
    #[display("The encoded HUGR package must have a single HUGR, but it has {count} HUGRs.")]
    InvalidNumHugrs {
        /// The number of HUGRs encountered in the encoded HUGR package.
        count: usize,
    },
}

impl From<PackageValidationError> for CircuitLoadError {
    fn from(e: PackageValidationError) -> Self {
        match e {
            PackageValidationError::Validation(e) => CircuitLoadError::ValidationError(e),
            _ => panic!("Unexpected package validation error: {e}"),
        }
    }
}

/// Looks for the circuit entrypoint in a list of modules, and returns a new
/// circuit pointing to it.
///
/// The modules are searched in order, and the first match is returned.
///
/// # Errors
///
/// - If any of the HUGR roots is not a module operation.
/// - If the function is not found in any module.
fn find_function_in_modules(
    modules: impl IntoIterator<Item = Hugr>,
    function_name: &str,
) -> Result<(usize, Circuit), CircuitLoadError> {
    let mut available_functions = Vec::new();
    for (i, hugr) in modules.into_iter().enumerate() {
        match find_function(hugr, function_name) {
            Ok(circ) => return Ok((i, circ)),
            Err(CircuitLoadError::FunctionNotFound {
                available_functions: fns,
                ..
            }) => {
                available_functions.extend(fns);
                continue;
            }
            Err(e) => return Err(e),
        }
    }
    Err(CircuitLoadError::FunctionNotFound {
        function: function_name.to_string(),
        available_functions,
    })
}

/// Looks for the circuit entrypoint in a HUGR, and returns a new circuit pointing to it.
///
/// # Errors
///
/// - If the root of the HUGR is not a module operation.
/// - If the function is not found in the module.
fn find_function(mut hugr: Hugr, function_name: &str) -> Result<Circuit, CircuitLoadError> {
    // Find the root module.
    let module = hugr.module_root();
    if !OpTag::ModuleRoot.is_superset(hugr.get_optype(module).tag()) {
        return Err(CircuitLoadError::NonModuleRoot {
            root_op: hugr.get_optype(module).clone(),
        });
    }

    // Find the function definition.
    fn func_name(op: &OpType) -> &str {
        match op {
            OpType::FuncDefn(decl) => decl.func_name(),
            _ => "",
        }
    }

    let Some(function) = hugr
        .children(module)
        .find(|&n| func_name(hugr.get_optype(n)) == function_name)
    else {
        let available_functions = hugr
            .children(module)
            .map(|n| func_name(hugr.get_optype(n)).to_string())
            .collect();
        return Err(CircuitLoadError::FunctionNotFound {
            function: function_name.to_string(),
            available_functions,
        });
    };

    hugr.set_entrypoint(function);
    let circ = Circuit::try_new(hugr)?;
    Ok(circ)
}

#[cfg(test)]
mod tests {
    use crate::TketOp;
    use crate::circuit::CircuitHash;

    use super::*;

    use cool_asserts::assert_matches;
    use hugr::builder::{
        Dataflow, DataflowHugr, DataflowSubContainer, FunctionBuilder, HugrBuilder, ModuleBuilder,
    };
    use hugr::extension::prelude::qb_t;
    use hugr::ops::handle::NodeHandle;

    use hugr::types::Signature;
    use itertools::Itertools;
    use rstest::{fixture, rstest};

    /// A circuit based on a DFG-rooted HUGR.
    #[fixture]
    fn root_circ() -> Circuit {
        let mut h = FunctionBuilder::new("main", Signature::new(vec![], vec![qb_t()])).unwrap();

        let res = h.add_dataflow_op(TketOp::QAlloc, []).unwrap();
        let q = res.out_wire(0);

        h.finish_hugr_with_outputs([q]).unwrap().into()
    }

    #[fixture]
    fn function_circ() -> Circuit {
        let mut f = FunctionBuilder::new("banana", Signature::new(vec![], vec![qb_t()])).unwrap();
        let res = f.add_dataflow_op(TketOp::QAlloc, []).unwrap();
        let q = res.out_wire(0);
        let hugr = f.finish_hugr_with_outputs([q]).unwrap();
        Circuit::new(hugr)
    }

    /// A circuit located inside a function in a module.
    #[fixture]
    fn nested_circ() -> Circuit {
        let mut f = FunctionBuilder::new("banana", Signature::new(vec![], vec![qb_t()])).unwrap();
        let dfg = {
            let mut dfg = f
                .dfg_builder(Signature::new(vec![], vec![qb_t()]), [])
                .unwrap();
            let res = dfg.add_dataflow_op(TketOp::QAlloc, []).unwrap();
            let q = res.out_wire(0);
            dfg.finish_with_outputs([q]).unwrap()
        };
        let mut hugr = f.finish_hugr_with_outputs(dfg.outputs()).unwrap();
        hugr.set_entrypoint(dfg.node());

        Circuit::new(hugr)
    }

    #[fixture]
    fn multi_module_pkg() -> Package {
        fn define(name: &str, h: &mut ModuleBuilder<Hugr>) -> Node {
            let f = h
                .define_function(name, Signature::new(vec![qb_t()], vec![qb_t()]))
                .unwrap();
            let inputs = f.input_wires().collect_vec();
            f.finish_with_outputs(inputs).unwrap().handle().node()
        }

        let mut mod1 = ModuleBuilder::new();
        define("apple", &mut mod1);
        define("banana", &mut mod1);
        let mod1 = mod1.finish_hugr().unwrap();

        let mut mod2 = ModuleBuilder::new();
        define("foo", &mut mod2);
        define("bar", &mut mod2);
        define("banana", &mut mod2);
        let mod2 = mod2.finish_hugr().unwrap();

        Package::new([mod1, mod2])
    }

    /// Test roundtrips of a circuit with a root parent.
    #[rstest]
    fn root_circuit_store(root_circ: Circuit) {
        let mut buf = Vec::new();
        root_circ.store(&mut buf, EnvelopeConfig::text()).unwrap();
        let circ = Circuit::load(buf.as_slice(), None).unwrap();
        assert_eq!(
            root_circ.circuit_hash(root_circ.parent()),
            circ.circuit_hash(circ.parent())
        );

        let envelope = root_circ.store_str(EnvelopeConfig::text()).unwrap();
        let circ = Circuit::load_function_str(envelope, "main").unwrap();
        assert_eq!(
            root_circ.circuit_hash(root_circ.parent()),
            circ.circuit_hash(circ.parent())
        );
    }

    #[rstest]
    fn func_circuit_store(function_circ: Circuit) {
        let mut buf = Vec::new();
        function_circ
            .store(&mut buf, EnvelopeConfig::text())
            .unwrap();
        let circ2 = Circuit::load_function(buf.as_slice(), "banana").unwrap();

        assert_eq!(function_circ, circ2);
    }

    #[rstest]
    fn serialize_package_errors(multi_module_pkg: Package) {
        let pkg_json = multi_module_pkg.store_str(EnvelopeConfig::text()).unwrap();

        match Circuit::load_function_str(&pkg_json, "not_found") {
            Err(CircuitLoadError::FunctionNotFound {
                function,
                available_functions,
            }) => {
                assert_eq!(function, "not_found");
                assert_eq!(
                    available_functions,
                    ["apple", "banana", "foo", "bar", "banana"]
                );
            }
            Err(e) => panic!("Expected FunctionNotFound error got {e}."),
            Ok(_) => panic!("Expected an error."),
        };

        assert_matches!(Circuit::load_function_str(&pkg_json, "banana"), Ok(_))
    }
}
