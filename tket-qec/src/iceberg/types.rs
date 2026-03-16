//! Extension providing a type for the Iceberg codeblock.

use std::sync::{Arc, LazyLock};

use hugr::{
    Extension,
    extension::ExtensionId,
    types::{CustomType, Type, TypeArg, TypeBound, TypeName, type_param::TypeParam},
};

/// The extension identifier.
pub const EXTENSION_ID: ExtensionId = ExtensionId::new_unchecked("tket.qec.iceberg.types");
/// Extension version.
pub const VERSION: semver::Version = semver::Version::new(0, 1, 0);

/// Type name for logical Iceberg block.
pub const BLOCK_TYPENAME: TypeName = TypeName::new_inline("block");

/// Type of an Iceberg block of a given size.
///
/// * `k_arg` - The number of logical qubits in the code block.
pub fn block_type(k_arg: impl Into<TypeArg>) -> Type {
    CustomType::new(
        BLOCK_TYPENAME,
        [k_arg.into()],
        EXTENSION_ID,
        TypeBound::Linear,
        &Arc::<Extension>::downgrade(&EXTENSION),
    )
    .into()
}

/// Extension for logical Iceberg block type.
fn extension() -> Arc<Extension> {
    Extension::new_arc(EXTENSION_ID, VERSION, |extension, extension_ref| {
        extension
            .add_type(
                BLOCK_TYPENAME,
                vec![TypeParam::max_nat_type()],
                "logical iceberg block".to_owned(),
                TypeBound::Linear.into(),
                extension_ref,
            )
            .unwrap();
    })
}

/// Lazy reference to extension for logical Iceberg block type.
pub static EXTENSION: LazyLock<Arc<Extension>> = LazyLock::new(extension);

/// Get an Iceberg block type with size corresponding to a type variable with a
/// given ID.
pub fn block_tv(var_id: usize) -> Type {
    Type::new_extension(
        EXTENSION
            .get_type(&BLOCK_TYPENAME)
            .unwrap()
            .instantiate(vec![TypeArg::new_var_use(
                var_id,
                TypeParam::max_nat_type(),
            )])
            .unwrap(),
    )
}

#[cfg(test)]
mod tests {
    use hugr::{
        HugrView,
        builder::{Dataflow, DataflowSubContainer, HugrBuilder, ModuleBuilder},
        types::Signature,
    };

    use super::*;

    #[test]
    fn test_iceberg_types_extension() {
        let extn = extension();
        assert_eq!(extn.name() as &str, "tket.qec.iceberg.types");
        assert_eq!(extn.types().count(), 1);
        assert_eq!(extn.operations().count(), 0);
    }

    #[test]
    fn test_iceberg_block_type() {
        let block = block_type(6);
        assert!(!block.copyable());
    }

    #[test]
    fn test_hugr() {
        let block = block_type(2);
        let mut module_builder = ModuleBuilder::new();
        let signature = Signature::new_endo(vec![block]);
        let f_build = module_builder.define_function("main", signature).unwrap();
        let wires: Vec<_> = f_build.input_wires().collect();
        f_build.finish_with_outputs(wires).unwrap();
        let h = module_builder.finish_hugr().unwrap();
        h.validate().unwrap();
    }
}
