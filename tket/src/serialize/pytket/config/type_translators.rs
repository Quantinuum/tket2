//! Sets of extensions for translating HUGR types into pytket registers.
//!
//! Defines sets of [`PytketTypeTranslator`]s used by the encoders and decoders to
//! translate HUGR types into pytket primitives.

use std::collections::HashMap;
use std::sync::RwLock;

use hugr::builder::{BuildError, DFGBuilder, Dataflow};
use hugr::extension::prelude::bool_t;
use hugr::extension::ExtensionId;
use hugr::std_extensions::arithmetic::float_types;
use hugr::types::{Type, TypeEnum};
use hugr::{Hugr, Wire};
use itertools::Itertools;

use crate::extension::bool::BoolOp;
use crate::extension::rotation;
use crate::serialize::pytket::extension::{PytketTypeTranslator, RegisterCount};
use crate::serialize::pytket::{PytketDecodeError, PytketDecodeErrorInner};

/// A set of [`PytketTypeTranslator`]s that can be used to translate HUGR types
/// into pytket registers (qubits, bits, and parameter expressions).
#[derive(Default, derive_more::Debug)]
pub struct TypeTranslatorSet {
    /// Registered type translators
    #[debug(skip)]
    pub(super) type_translators: Vec<Box<dyn PytketTypeTranslator + Send + Sync>>,
    /// Pre-computed map from extension ids to corresponding type translators in
    /// `type_translators`, identified by their index.
    #[debug("{:?}", extension_translators.keys().collect_vec())]
    extension_translators: HashMap<ExtensionId, Vec<usize>>,
    /// A cache of translated types.
    #[debug(skip)]
    type_cache: RwLock<HashMap<Type, Option<RegisterCount>>>,
}

impl TypeTranslatorSet {
    /// Add a translator to the set.
    ///
    /// This operation invalidates the type translation cache.
    pub(super) fn add_type_translator(
        &mut self,
        translator: impl PytketTypeTranslator + Send + Sync + 'static,
    ) {
        let idx = self.type_translators.len();

        for ext in translator.extensions() {
            self.extension_translators.entry(ext).or_default().push(idx);
        }

        self.type_translators.push(Box::new(translator));

        // Clear the cache, or create a new one if it's poisoned.
        let cache = self.type_cache.write();
        if let Ok(mut cache) = cache {
            cache.clear();
        } else {
            drop(cache);
            self.type_cache = RwLock::new(HashMap::new());
        }
    }

    /// Translate a HUGR type into a count of qubits, bits, and parameters,
    /// using the registered custom translator.
    ///
    /// Only tuple sums, bools, and custom types are supported.
    /// Other types will return `None`.
    pub fn type_to_pytket(&self, typ: &Type) -> Option<RegisterCount> {
        self.type_to_pytket_internal(typ).filter(|c| !c.is_empty())
    }

    /// Recursive call for [`Self::type_to_pytket`].
    ///
    /// This allows returning empty register counts, for types that may be included inside other types.
    fn type_to_pytket_internal(&self, typ: &Type) -> Option<RegisterCount> {
        let cache = self.type_cache.read().ok();
        if let Some(count) = cache.and_then(|c| c.get(typ).cloned()) {
            return count;
        }

        // We currently don't allow user types to contain parameters,
        // so we handle rotations and floats manually here.
        if typ.as_extension().is_some_and(|ext| {
            [float_types::EXTENSION_ID, rotation::ROTATION_EXTENSION_ID].contains(ext.extension())
        }) {
            return Some(RegisterCount::only_params(1));
        }

        let res = match typ.as_type_enum() {
            TypeEnum::Sum(sum) => {
                if sum.num_variants() == 0 {
                    return Some(RegisterCount::default());
                }
                if typ == &bool_t() {
                    return Some(RegisterCount::only_bits(1));
                }
                if let Some(tuple) = sum.as_tuple() {
                    let count: Option<RegisterCount> = tuple
                        .iter()
                        .map(|ty| {
                            match ty.clone().try_into() {
                                Ok(ty) => self.type_to_pytket_internal(&ty),
                                // Sum types with row variables (variable tuple lengths) are not supported.
                                Err(_) => None,
                            }
                        })
                        .sum();
                    // Don't allow parameters nested inside other types
                    count.filter(|c| c.params == 0)
                } else {
                    None
                }
            }
            TypeEnum::Extension(custom) => 'outer: {
                let type_ext = custom.extension();
                for encoder in self.translators_for_extension(type_ext) {
                    if let Some(count) = encoder.type_to_pytket(custom, self) {
                        // Don't allow user types with nested parameters
                        if count.params == 0 {
                            break 'outer Some(count);
                        } else {
                            break 'outer None;
                        }
                    }
                }
                None
            }
            _ => None,
        };

        // Insert the result into the cache. Ignoring it if it's poisoned.
        if let Ok(mut cache) = self.type_cache.write() {
            cache.insert(typ.clone(), res);
        }

        res
    }

    /// Lists the translators that can handle a given extension.
    fn translators_for_extension(
        &self,
        ext: &ExtensionId,
    ) -> impl Iterator<Item = &Box<dyn PytketTypeTranslator + Send + Sync>> + use<'_> {
        self.extension_translators
            .get(ext)
            .into_iter()
            .flatten()
            .map(move |idx| &self.type_translators[*idx])
    }

    /// Returns `true` if the two types are isomorphic. I.e. they can be translated
    /// into each other without losing information.
    //
    // TODO: We should allow custom TypeTranslators to expand this checks,
    // and implement their own translations.
    pub fn types_are_isomorphic(&self, typ1: &Type, typ2: &Type) -> bool {
        if typ1 == typ2 {
            return true;
        }

        // For now, we just hard-code this to the two kind of bits we support.
        let native_bool = bool_t();
        let tket_bool = crate::extension::bool::bool_type();
        if (typ1 == &native_bool && typ2 == &tket_bool)
            || (typ1 == &tket_bool && typ2 == &native_bool)
        {
            return true;
        }

        false
    }

    /// Inserts the necessary operations to translate a type into an isomorphic
    /// type.
    ///
    /// This operation fails if [`Self::types_are_isomorphic`] returns `false`.
    pub(super) fn transform_typed_value(
        &self,
        wire: Wire,
        initial_type: &Type,
        target_type: &Type,
        builder: &mut DFGBuilder<&mut Hugr>,
    ) -> Result<Wire, PytketDecodeError> {
        if initial_type == target_type {
            return Ok(wire);
        }

        let map_build_error = |e: BuildError| PytketDecodeErrorInner::CannotTranslateWire {
            wire,
            initial_type: initial_type.to_string(),
            target_type: target_type.to_string(),
            context: Some(e.to_string()),
        };

        // Hard-coded transformations until customs calls are added to [`PytketTypeTranslator`].
        let native_bool = bool_t();
        let tket_bool = crate::extension::bool::bool_type();
        if initial_type == &native_bool && target_type == &tket_bool {
            let [wire] = builder
                .add_dataflow_op(BoolOp::make_opaque, [wire])
                .map_err(map_build_error)?
                .outputs_arr();
            return Ok(wire);
        }
        if initial_type == &tket_bool && target_type == &native_bool {
            let [wire] = builder
                .add_dataflow_op(BoolOp::read, [wire])
                .map_err(map_build_error)?
                .outputs_arr();
            return Ok(wire);
        }

        Err(PytketDecodeErrorInner::CannotTranslateWire {
            wire,
            initial_type: initial_type.to_string(),
            target_type: target_type.to_string(),
            context: None,
        }
        .wrap())
    }
}

#[cfg(test)]
mod tests {
    use hugr::extension::prelude::{qb_t, PRELUDE_ID};
    use hugr::types::SumType;

    use crate::extension::bool::BOOL_EXTENSION_ID;

    use super::*;

    struct TestBoolTranslator;

    impl PytketTypeTranslator for TestBoolTranslator {
        fn extensions(&self) -> Vec<ExtensionId> {
            vec![BOOL_EXTENSION_ID, PRELUDE_ID]
        }

        fn type_to_pytket(
            &self,
            typ: &hugr::types::CustomType,
            _set: &TypeTranslatorSet,
        ) -> Option<RegisterCount> {
            match typ.name().as_str() {
                "usize" => Some(RegisterCount::only_bits(64)),
                "qubit" => Some(RegisterCount::only_qubits(1)),
                "bool" => Some(RegisterCount::only_bits(1)),
                _ => None,
            }
        }
    }

    #[rstest::fixture]
    fn translator_set() -> TypeTranslatorSet {
        let mut set = TypeTranslatorSet::default();
        set.add_type_translator(TestBoolTranslator);
        set
    }

    #[rstest::rstest]
    #[case::empty(SumType::new_unary(0).into(), None)]
    #[case::native_bool(SumType::new_unary(2).into(), Some(RegisterCount::only_bits(1)))]
    #[case::simple(bool_t(), Some(RegisterCount::only_bits(1)))]
    #[case::tuple(SumType::new_tuple(vec![bool_t(), qb_t(), bool_t(), SumType::new_unary(1).into()]).into(), Some(RegisterCount::new(1, 2, 0)))]
    #[case::unsupported(SumType::new([vec![bool_t(), qb_t()], vec![bool_t()]]).into(), None)]
    fn test_translations(
        translator_set: TypeTranslatorSet,
        #[case] typ: Type,
        #[case] count: Option<RegisterCount>,
    ) {
        assert_eq!(translator_set.type_to_pytket(&typ), count);
    }
}
