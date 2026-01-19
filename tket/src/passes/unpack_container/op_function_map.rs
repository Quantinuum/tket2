//! Mapping from extension operation instances to function definitions
//! that can be used to replace them.
use hugr::HugrView;
use hugr::builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder};
use hugr::hugr::linking::OnMultiDefn;
use hugr::ops::handle::{FuncID, NodeHandle};
use hugr::{
    Hugr, Node, Wire,
    algorithms::{ReplaceTypes, mangle_name, replace_types::NodeTemplate},
    builder::{BuildError, DataflowHugr, FunctionBuilder},
    hugr::hugrmut::HugrMut,
    ops::{DataflowOpTrait, ExtensionOp},
    types::TypeArg,
};
use hugr_core::Visibility;
use hugr_core::hugr::linking::NameLinkingPolicy;
use indexmap::IndexMap;
use std::{cell::RefCell, ops::Deref};

/// Wrapper for ExtensionOp that implements Hash
#[derive(Clone, PartialEq, Eq)]
struct OpHashWrapper(ExtensionOp);

impl From<ExtensionOp> for OpHashWrapper {
    fn from(op: ExtensionOp) -> Self {
        Self(op)
    }
}

impl std::hash::Hash for OpHashWrapper {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.0.extension_id().hash(state);
        self.0.unqualified_id().hash(state);
        self.0.args().hash(state);
    }
}

/// Map from extension operation instances to a function definition
/// that can be used to replace it.
#[derive(Clone)]
pub struct OpFunctionMap {
    // RefCell for interior mutability, allowing
    // recursive function building.
    // Value is option so that None can be used as a
    // placeholder while building a function.
    map: RefCell<IndexMap<OpHashWrapper, Option<Hugr>>>,
}

impl OpFunctionMap {
    /// Create a new empty map
    pub fn new() -> Self {
        Self {
            map: RefCell::new(IndexMap::new()),
        }
    }

    /// Insert a function definition for the given operation instance,
    /// if it is not already present.
    ///
    /// * `op`: The extension operation to store the function for.
    /// * `mangle_args`: Type arguments to use when mangling the function name,
    ///   to ensure uniqueness, with op name as base.
    /// * `func_builder`: Closure that takes a [`FunctionBuilder`] and
    ///   builds the function body, returning the output wires.
    ///   The function signature and name will already have been set
    ///   in the builder.
    pub fn insert_with<O, F>(
        &self,
        op: &ExtensionOp,
        mangle_args: &[TypeArg],
        func_builder: F,
    ) -> Result<(), BuildError>
    where
        O: IntoIterator<Item = Wire>,
        F: FnOnce(&mut FunctionBuilder<Hugr>) -> Result<O, BuildError>,
    {
        let key = OpHashWrapper::from(op.clone());
        if self.map.borrow().contains_key(&key) {
            return Ok(());
        }
        let name = mangle_name(op.def().name(), mangle_args);
        let sig = op.signature().deref().clone();
        let mut func_b = FunctionBuilder::new(name, sig)?;
        // insert None as a placeholder to avoid cyclic recursion in func_builder call
        self.map.borrow_mut().insert(key.clone(), None);

        let outputs = func_builder(&mut func_b)?;
        let hugr = func_b.finish_hugr_with_outputs(outputs)?;

        // replace placeholder
        let out = self.map.borrow_mut().insert(key, Some(hugr));
        debug_assert_eq!(out, Some(None));
        Ok(())
    }

    /// Return the number of stored functions
    pub fn len(&self) -> usize {
        self.map.borrow().len()
    }

    /// Return true if the map is empty
    pub fn is_empty(&self) -> bool {
        self.map.borrow().is_empty()
    }

    /// Consume the map and return an iterator over (operation, function) pairs
    pub fn into_function_iter(self) -> impl Iterator<Item = (ExtensionOp, Hugr)> {
        self.map
            .into_inner()
            .into_iter()
            .map(|(k, v)| (k.0, v.expect("All placeholders should have been replaced")))
    }

    /// Register function replacements for all temporary operations.
    /// Inserts function definitions in the given Hugr and
    /// adds replacements to the given [`ReplaceTypes`] lowerer,
    /// which can be used to replace extension operations with calls to the
    /// corresponding function definitions.
    pub fn register_operation_replacements(
        self,
        _hugr: &mut impl HugrMut<Node = Node>,
        lowerer: &mut ReplaceTypes,
    ) {
        // Use the centralized cache for all operation replacements
        for (op, mut func_def) in self.into_function_iter() {
            // Create a replacement hugr for the op nodes: Add a `call` node in the `func_def` hugr and set it as entrypoint.
            let func_node = func_def.entrypoint();
            let func_signature = func_def.inner_function_type().unwrap().into_owned();
            let func_id = FuncID::<true>::from(func_node);
            func_def.set_entrypoint(func_def.module_root());
            let mut b = ModuleBuilder::with_hugr(func_def);
            let call = {
                let mut f = b
                    .define_function_vis("", func_signature, Visibility::Private)
                    .unwrap();
                let call = f.call(&func_id, &[], f.input_wires()).unwrap();
                f.finish_with_outputs(call.outputs()).unwrap();
                call
            };
            let mut call_hugr = b.finish_hugr().unwrap();
            call_hugr.set_entrypoint(call.node());

            lowerer.set_replace_op(
                &op,
                NodeTemplate::LinkedHugr(
                    Box::new(call_hugr),
                    NameLinkingPolicy::default().on_multiple_defn(OnMultiDefn::UseTarget),
                ),
            );
        }
    }
}

impl Default for OpFunctionMap {
    fn default() -> Self {
        Self::new()
    }
}
