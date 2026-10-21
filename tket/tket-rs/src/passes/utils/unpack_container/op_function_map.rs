//! Mapping from extension operation instances to function definitions
//! that can be used to replace them.
use crate::passes::monomorphize::mangle_name;
use crate::passes::{
    ReplaceTypes,
    replace_types::{NodeTemplate, reserve_function_name},
};
use hugr::HugrView;
use hugr::{
    Hugr, Node, Wire,
    builder::{BuildError, DataflowHugr, FunctionBuilder},
    ops::{DataflowOpTrait, ExtensionOp, OpType},
    types::TypeArg,
};
use hugr_core::Visibility;
use hugr_core::hugr::internal::HugrMutInternals;
use indexmap::IndexMap;
use std::{cell::RefCell, collections::HashSet, ops::Deref};

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
    /// * `mangle_args`: Additional type arguments to include in the function name,
    ///   alongside the complete operation arguments, with op name as base.
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
        // Definitions are made public while name linking deduplicates them, so
        // the symbol must distinguish every cached operation instance. Keep
        // caller-selected arguments for readability, and append the complete
        // operation arguments to distinguish operation shapes.
        let name_args = mangle_args
            .iter()
            .chain(op.args())
            .cloned()
            .collect::<Vec<_>>();
        let name = mangle_name(op.def().name(), name_args);
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

    /// Register calls to helpers with collision-safe symbols for their definitions.
    ///
    /// Helper symbols are allocated for this target at registration time. Do not
    /// introduce conflicting symbols before running `lowerer`; register fresh
    /// replacements for each target Hugr.
    pub fn register_operation_replacements(
        self,
        hugr: &impl HugrView<Node = Node>,
        lowerer: &mut ReplaceTypes,
    ) {
        let mut functions = self.into_function_iter().collect::<Vec<_>>();
        // Reserve each name once; declarations remain in their HUGRs for linker type checking.
        let declarations = functions
            .iter()
            .flat_map(|(_, h)| {
                h.children(h.module_root())
                    .filter_map(|n| match h.get_optype(n) {
                        OpType::FuncDecl(f) => Some(f.func_name().clone()),
                        _ => None,
                    })
            })
            .collect::<HashSet<_>>();
        let mut occupied = lowerer.reserve_function_declarations(hugr, &declarations);
        // Allocate all dependency names before entrypoints so dependencies get first choice.
        for (_, h) in &mut functions {
            let entrypoint = h.entrypoint();
            let dependencies = h
                .children(h.module_root())
                .filter(|&n| n != entrypoint)
                .collect::<Vec<_>>();
            for dependency in dependencies {
                if let OpType::FuncDefn(f) = h.optype_mut(dependency) {
                    *f.func_name_mut() = reserve_function_name(f.func_name(), &mut occupied);
                }
            }
        }
        for (op, mut func_def) in functions {
            let entrypoint = func_def.entrypoint();
            let OpType::FuncDefn(func) = func_def.optype_mut(entrypoint) else {
                panic!("OpFunctionMap entries must be function definitions");
            };
            *func.func_name_mut() = reserve_function_name(func.func_name(), &mut occupied);
            lowerer.set_replace_generated_op(&op, func_as_node_template(func_def));
        }
    }
}

/// Given a HUGR with a function definition as entrypoint, constructs a
/// [`NodeTemplate::LinkedHugr`] that produces a call to the function.
fn func_as_node_template(mut func_def: Hugr) -> NodeTemplate {
    let entrypoint = func_def.entrypoint();
    let OpType::FuncDefn(func) = func_def.optype_mut(entrypoint) else {
        panic!("OpFunctionMap entries must be function definitions");
    };
    *func.visibility_mut() = Visibility::Public;

    NodeTemplate::call_to_function(func_def, &[])
        .expect("OpFunctionMap entries must be monomorphic function definitions")
}

impl Default for OpFunctionMap {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::passes::ComposablePass;
    use hugr::builder::{Container, Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder};
    use hugr::extension::prelude::bool_t;
    use hugr::extension::{Extension, Version};
    use hugr::hugr::hugrmut::HugrMut;
    use hugr::ops::handle::{FuncID, NodeHandle};
    use hugr::std_extensions::logic::LogicOp;
    use hugr::types::Signature;
    use rstest::rstest;

    fn helper_op(extension: &str) -> (std::sync::Arc<Extension>, ExtensionOp) {
        let ext = Extension::new_arc(
            extension.try_into().unwrap(),
            Version::new(0, 0, 0),
            |e, w| {
                e.add_op(
                    "helper".into(),
                    Default::default(),
                    Signature::new_endo([bool_t()]),
                    w,
                )
                .unwrap();
            },
        );
        let op = ExtensionOp::new(ext.get_op("helper").unwrap().clone(), []).unwrap();
        (ext, op)
    }

    #[derive(Clone, Copy)]
    enum EarlierHelper {
        None,
        SameMap,
        SeparateMap,
        Replaced,
    }

    #[rstest]
    #[case::no_earlier_definition(EarlierHelper::None, false)]
    #[case::same_map_definition(EarlierHelper::SameMap, false)]
    #[case::separate_maps_definition(EarlierHelper::SeparateMap, false)]
    #[case::no_earlier_declaration(EarlierHelper::None, true)]
    #[case::same_map_declaration(EarlierHelper::SameMap, true)]
    #[case::separate_maps_declaration(EarlierHelper::SeparateMap, true)]
    #[case::replaced_declaration(EarlierHelper::Replaced, true)]
    fn helper_symbol_avoids_its_incoming_dependency(
        #[case] earlier: EarlierHelper,
        #[case] declaration: bool,
        #[values(false, true)] target_dependency: bool,
    ) {
        let earlier_helper = !matches!(earlier, EarlierHelper::None);
        let separate_maps = matches!(
            earlier,
            EarlierHelper::SeparateMap | EarlierHelper::Replaced
        );
        let (_ext, op) = helper_op("test.incoming");
        let (_earlier_ext, earlier_op) = helper_op("test.earlier");
        let map = OpFunctionMap::new();
        let earlier_map = OpFunctionMap::new();
        if earlier_helper {
            let earlier_builder = if separate_maps { &earlier_map } else { &map };
            earlier_builder
                .insert_with(&earlier_op, &[], |b| Ok(b.input_wires()))
                .unwrap();
        }
        map.insert_with(&op, &[], |b| {
            if declaration {
                let dependency = b
                    .module_root_builder()
                    .declare("$helper$$1", Signature::new_endo([bool_t()]).into())?;
                return Ok(b.call(&dependency, &[], b.input_wires())?.outputs());
            }
            let mut dependency = FunctionBuilder::new_vis(
                "$helper$$1",
                Signature::new_endo([bool_t()]),
                Visibility::Public,
            )?;
            let negated = dependency.add_dataflow_op(LogicOp::Not, dependency.input_wires())?;
            let dependency = dependency.finish_hugr_with_outputs(negated.outputs())?;
            let dependency = FuncID::<true>::from(
                b.module_root_builder()
                    .add_hugr(dependency)
                    .inserted_entrypoint,
            );
            Ok(b.call(&dependency, &[], b.input_wires())?.outputs())
        })
        .unwrap();

        let mut b = ModuleBuilder::new();
        let user_dependency = if target_dependency {
            let user = b
                .define_function_vis(
                    "$helper$$1",
                    Signature::new_endo([bool_t()]),
                    Visibility::Public,
                )
                .unwrap();
            let inputs = user.input_wires().collect::<Vec<_>>();
            Some(user.finish_with_outputs(inputs).unwrap())
        } else {
            None
        };
        let user = b
            .define_function_vis(
                "$helper$",
                Signature::new_endo([bool_t()]),
                Visibility::Public,
            )
            .unwrap();
        let inputs = user.input_wires().collect::<Vec<_>>();
        let user = user.finish_with_outputs(inputs).unwrap();
        let mut main = b
            .define_function("main", Signature::new_endo([bool_t()]))
            .unwrap();
        let mut inputs = main.input_wires().collect::<Vec<_>>();
        let dependency_user_call = user_dependency.as_ref().map(|user| {
            let call = main.call(user.handle(), &[], inputs.clone()).unwrap();
            inputs = call.outputs().collect();
            call.node()
        });
        let original_call = main.call(user.handle(), &[], inputs).unwrap();
        let mut inputs = original_call.outputs().collect::<Vec<_>>();
        let earlier_call = if earlier_helper {
            let call = main.add_dataflow_op(earlier_op.clone(), inputs).unwrap();
            inputs = call.outputs().collect();
            Some(call.node())
        } else {
            None
        };
        let first = main.add_dataflow_op(op.clone(), inputs).unwrap();
        let second = main.add_dataflow_op(op, first.outputs()).unwrap();
        main.finish_with_outputs(second.outputs()).unwrap();
        let mut h = b.finish_hugr().unwrap();
        h.validate().unwrap();

        let mut lowerer = ReplaceTypes::new_empty();
        if separate_maps {
            earlier_map.register_operation_replacements(&h, &mut lowerer);
        }
        if matches!(earlier, EarlierHelper::Replaced) {
            let f = FunctionBuilder::new_vis(
                "$helper$$1",
                Signature::new_endo([bool_t()]),
                Visibility::Public,
            )
            .unwrap();
            let inputs = f.input_wires().collect::<Vec<_>>();
            let f = f.finish_hugr_with_outputs(inputs).unwrap();
            lowerer.set_replace_op(&earlier_op, NodeTemplate::call_to_function(f, &[]).unwrap());
        }
        map.register_operation_replacements(&h, &mut lowerer);
        lowerer.run(&mut h).unwrap();
        h.validate().unwrap();

        let helper = h.static_source(first.node()).unwrap();
        assert_eq!(h.static_source(second.node()), Some(helper));
        assert_eq!(h.static_source(original_call.node()), Some(user.node()));
        assert_eq!(h.children(user.node()).count(), 2);
        let call = h
            .children(helper)
            .find(|&n| h.get_optype(n).is_call())
            .unwrap();
        let dependency = h.static_source(call).unwrap();
        assert_ne!(helper, dependency);
        if let Some(user) = user_dependency {
            assert_eq!(h.children(user.node()).count(), 2);
            assert_eq!(
                h.static_source(dependency_user_call.unwrap()),
                Some(user.node())
            );
            if declaration {
                assert_eq!(dependency, user.node());
            } else {
                assert_ne!(dependency, user.node());
            }
        }
        if declaration && !target_dependency && !matches!(earlier, EarlierHelper::Replaced) {
            assert_eq!(
                h.get_optype(dependency).as_func_decl().unwrap().func_name(),
                "$helper$$1"
            );
        } else if !declaration {
            if !target_dependency && !separate_maps {
                assert_eq!(
                    h.get_optype(dependency).as_func_defn().unwrap().func_name(),
                    "$helper$$1"
                );
            }
            assert!(
                h.children(dependency)
                    .any(|n| h.get_optype(n) == &LogicOp::Not.into())
            );
        }
        if let Some(call) = earlier_call {
            let earlier_function = h.static_source(call).unwrap();
            if matches!(earlier, EarlierHelper::Replaced) {
                assert_eq!(earlier_function, dependency);
            } else {
                assert_ne!(earlier_function, dependency);
            }
            assert_eq!(h.children(earlier_function).count(), 2);
        }
    }

    #[rstest]
    fn helper_symbols_handle_nested_dependencies_and_repeated_lowering(
        #[values(false, true)] separate_maps: bool,
    ) {
        let (_outer_ext, outer) = helper_op("test.outer");
        let (_inner_ext, inner) = helper_op("test.inner");
        let outer_map = OpFunctionMap::new();
        outer_map
            .insert_with(&outer, &[], |b| {
                Ok(b.add_dataflow_op(inner.clone(), b.input_wires())?.outputs())
            })
            .unwrap();
        let inner_map = OpFunctionMap::new();
        let inner_builder = if separate_maps {
            &inner_map
        } else {
            &outer_map
        };
        inner_builder
            .insert_with(&inner, &[], |b| {
                Ok(b.add_dataflow_op(LogicOp::Not, b.input_wires())?.outputs())
            })
            .unwrap();

        let mut b = ModuleBuilder::new();
        let declaration = b
            .declare("$helper$$1", Signature::new_endo([bool_t()]).into())
            .unwrap();
        let private = b
            .define_function("$helper$$2", Signature::new_endo([bool_t()]))
            .unwrap();
        let inputs = private.input_wires().collect::<Vec<_>>();
        let private = private.finish_with_outputs(inputs).unwrap();
        let user = b
            .define_function_vis(
                "$helper$",
                Signature::new_endo([bool_t()]),
                Visibility::Public,
            )
            .unwrap();
        let inputs = user.input_wires().collect::<Vec<_>>();
        let user = user.finish_with_outputs(inputs).unwrap();
        let mut main = b
            .define_function("main", Signature::new_endo([bool_t()]))
            .unwrap();
        let original_call = main.call(user.handle(), &[], main.input_wires()).unwrap();
        let first = main
            .add_dataflow_op(outer.clone(), original_call.outputs())
            .unwrap();
        let second = main.add_dataflow_op(outer, first.outputs()).unwrap();
        main.finish_with_outputs(second.outputs()).unwrap();
        let mut h = b.finish_hugr().unwrap();
        h.validate().unwrap();
        let mut lowerer = ReplaceTypes::new_empty();
        outer_map.register_operation_replacements(&h, &mut lowerer);
        inner_map.register_operation_replacements(&h, &mut lowerer);
        lowerer.run(&mut h).unwrap();
        h.validate().unwrap();

        let outer_fn = h.static_source(first.node()).unwrap();
        assert!(h.get_optype(declaration.node()).is_func_decl());
        assert_eq!(h.children(private.node()).count(), 2);
        assert_eq!(h.static_source(second.node()), Some(outer_fn));
        assert_eq!(h.static_source(original_call.node()), Some(user.node()));
        assert_eq!(h.children(user.node()).count(), 2);
        let inner_call = h
            .children(outer_fn)
            .find(|&n| h.get_optype(n).is_call())
            .unwrap();
        let inner_fn = h.static_source(inner_call).unwrap();
        assert_ne!(outer_fn, inner_fn);
        assert_ne!(outer_fn, user.node());
        assert_ne!(inner_fn, user.node());
        assert!(
            h.children(inner_fn)
                .any(|n| h.get_optype(n) == &LogicOp::Not.into())
        );

        let mut later = FunctionBuilder::new("later", Signature::new_endo([bool_t()])).unwrap();
        let op = later
            .add_dataflow_op(inner.clone(), later.input_wires())
            .unwrap();
        let later = later.finish_hugr_with_outputs(op.outputs()).unwrap();
        let later = h.insert_hugr(h.module_root(), later).inserted_entrypoint;
        let new_map = OpFunctionMap::new();
        new_map
            .insert_with(&inner, &[], |b| Ok(b.input_wires()))
            .unwrap();
        let mut lowerer = ReplaceTypes::new_empty();
        new_map.register_operation_replacements(&h, &mut lowerer);
        lowerer.run(&mut h).unwrap();
        h.validate().unwrap();
        let later_call = h
            .children(later)
            .find(|&n| h.get_optype(n).is_call())
            .unwrap();
        let later_fn = h.static_source(later_call).unwrap();
        assert_ne!(later_fn, inner_fn);
        assert_eq!(
            h.children(later_fn).count(),
            2,
            "new helper is identity, not Not"
        );
        assert!(
            h.children(inner_fn)
                .any(|n| h.get_optype(n) == &LogicOp::Not.into())
        );
    }
}
