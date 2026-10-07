use crate::REGISTRY;
use anyhow::{Error, Result, anyhow};
use itertools::Itertools;
use tket::hugr::envelope::read_envelope;
use tket::hugr::ops::OpType;
use tket::hugr::types::Term;
use tket::hugr::{Hugr, HugrView};

use tket::extension::{TKET1_EXTENSION_ID, TKET1_OP_NAME};

/// Loads a HUGR package from a binary [Envelope][tket::hugr::envelope::Envelope].
///
/// Interprets the string as a hugr package and, verifies there is exactly one module in the
/// package, then extracts and returns that module.
pub fn read_hugr_envelope(bytes: &[u8]) -> Result<Hugr> {
    let (desc, package) = read_envelope(bytes, &REGISTRY)
        .map_err(|e| Error::new(e).context("Error loading HUGR package."))?;

    let num = package.modules.len();
    let Ok(hugr) = package.modules.into_iter().exactly_one() else {
        return Err(anyhow!(
            "Expected exactly one module in the package, found {num}"
        ));
    };

    validate(&hugr).map_err(|e| {
        if let Some(generator) = desc.generator() {
            e.context(format!("in package with generator {generator}"))
        } else {
            e
        }
    })?;

    Ok(hugr)
}

/// Checks whether the hugr is expected to be processable by the QIS compiler.
pub fn validate(hugr: &Hugr) -> Result<(), Error> {
    hugr.validate()?;

    // Check that no opaque tket1 operations are present.
    for node in hugr.nodes() {
        if let Some(name) = is_opaque_tket1_op(hugr.get_optype(node)) {
            return Err(anyhow!(
                "Pytket op '{name}' is not currently supported by the Selene HUGR-QIS compiler"
            ));
        }
    }

    // some more validation can be done here, e.g. extension version checking.
    Ok(())
}

/// Check if the optype is an opaque tket1 operation,
/// and return its name if so.
///
// TODO: Interpreting the operation payload to get the name is a bit hacky atm,
// since `tket` does not make the `OpaqueTk1Op` payload definition public.
fn is_opaque_tket1_op(op: &OpType) -> Option<String> {
    let ext_op = op.as_extension_op()?;

    if ext_op.extension_id() != &TKET1_EXTENSION_ID || ext_op.unqualified_id() != TKET1_OP_NAME {
        return None;
    }

    fn get_pytket_op_name(payload: Option<&Term>) -> Option<String> {
        let payload = match payload {
            Some(Term::String(payload)) => payload,
            _ => return None,
        };
        let json_payload: serde_json::Value = serde_json::from_str(payload).ok()?;
        let name = json_payload
            .as_object()?
            .get("op")?
            .as_object()?
            .get("type")?
            .as_str()?;
        Some(name.to_string())
    }

    Some(get_pytket_op_name(ext_op.args().first()).unwrap_or_else(|| "unknown".to_string()))
}
