//! Load local extension definitions for migration.

use std::sync::LazyLock;

use hugr::{
    Extension,
    extension::{ExtensionRegistry, Version},
    ops::ExtensionOp,
};

use crate::error::ExtensionLookupError;

/// Registry containing local extensions for migration.
pub(crate) static LOCAL_REGISTRY: LazyLock<ExtensionRegistry> = LazyLock::new(local_registry);

/// Looks up a bundled extension by name and exact version.
pub(crate) fn get_extension(
    name: &str,
    version: &Version,
) -> Result<&'static Extension, ExtensionLookupError> {
    LOCAL_REGISTRY
        .get_exact(name, version)
        .map(AsRef::as_ref)
        .ok_or_else(|| ExtensionLookupError::MissingExtension {
            name: name.to_owned(),
            version: version.clone(),
        })
}

/// Instantiates a named extension operation that takes no type arguments.
pub(crate) fn get_op_by_name(
    name: &str,
    ext: &Extension,
) -> Result<ExtensionOp, ExtensionLookupError> {
    let definition = ext
        .get_op(name)
        .ok_or_else(|| ExtensionLookupError::MissingOperation { name: name.into() })?;
    ExtensionOp::new(definition.clone(), []).map_err(|source| {
        ExtensionLookupError::InstantiateOperation {
            name: name.into(),
            source,
        }
    })
}

/// Builds the extension registry used for migration.
///
/// The registry starts from the standard HUGR extensions and is extended with
/// the legacy extension definitions bundled in the `extensions` directory.
/// The bundled definitions are resolved against the standard registry.
///
/// # Panics
///
/// Panics if a bundled extension is not valid JSON, or if the bundled
/// extensions cannot be resolved.
fn local_registry() -> ExtensionRegistry {
    let legacy_extensions = [
        include_str!("../extensions/TKET1-0.2.0.json"),
        include_str!("../extensions/tket/argument-0.1.0.json"),
        include_str!("../extensions/tket/bool-0.2.0.json"),
        include_str!("../extensions/tket/debug-0.2.0.json"),
        include_str!("../extensions/tket/futures-0.2.0.json"),
        include_str!("../extensions/tket/global_phase-0.1.0.json"),
        include_str!("../extensions/tket/globals-0.1.0.json"),
        include_str!("../extensions/tket/gpu-0.2.0.json"),
        include_str!("../extensions/tket/guppy-0.2.0.json"),
        include_str!("../extensions/tket/measurement-0.1.0.json"),
        include_str!("../extensions/tket/modifier-0.1.0.json"),
        include_str!("../extensions/tket/pauli_exp-0.1.0.json"),
        include_str!("../extensions/tket/qsystem-0.6.0.json"),
        include_str!("../extensions/tket/qsystem/helios-0.6.0.json"),
        include_str!("../extensions/tket/qsystem/random-0.2.1.json"),
        include_str!("../extensions/tket/qsystem/sol-0.6.0.json"),
        include_str!("../extensions/tket/qsystem/utils-0.3.0.json"),
        include_str!("../extensions/tket/quantum-0.2.1.json"),
        include_str!("../extensions/tket/quantum-0.3.0.json"),
        include_str!("../extensions/tket/result-0.2.0.json"),
        include_str!("../extensions/tket/rotation-0.2.0.json"),
        include_str!("../extensions/tket/wasm-0.5.0.json"),
    ]
    .into_iter()
    .map(|json| {
        serde_json::from_str::<Extension>(json)
            .expect("Bundled extension declarations must be valid JSON")
    })
    .collect::<Vec<_>>();

    let mut registry = hugr::std_extensions::STD_REG.clone();

    let legacy_registry =
        ExtensionRegistry::new_with_extension_resolution(legacy_extensions, &(&registry).into())
            .expect("Bundled migration extensions must resolve");

    registry.extend(legacy_registry);
    registry
}
