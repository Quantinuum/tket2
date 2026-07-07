//! Passes that call to tket1-passes using the tket-c-api.

use std::str::FromStr;

use rayon::iter::ParallelIterator;
use std::sync::Arc;
use tket::passes::composable::PassScope;
use tket_qsystem::PlatformTarget;

use crate::passes::PyPassScope;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use tket::serialize::pytket::{EncodeOptions, EncodedCircuit};

use crate::state::CompilationState;
use crate::utils::{ConvertPyErr, create_py_exception};

/// Runs a pytket pass on all circuit-like regions under the entrypoint of the
/// HUGR.
///
/// The `target` string selects which set of encoder/decoder extensions is used
/// when translating between the HUGR and pytket circuits:
///
/// - `"tket"` (default): Only base `tket` operations are encoded. When decoding,
///   pytket commands are translated into base `tket.quantum` operations, falling
///   back to Helios qsystem operations for commands without a base counterpart
///   (e.g. `ZZPhase`).
/// - `"sol"`: Base `tket` and native Sol operations are encoded. When decoding,
///   commands are translated into native Sol qsystem operations where possible,
///   falling back to base `tket.quantum` operations.
/// - `"helios"`: Base `tket` and native Helios operations are encoded. When
///   decoding, commands are translated into native Helios qsystem operations
///   where possible, falling back to base `tket.quantum` operations.
///
/// Operations without a valid encoder are kept as-is on a pytket roundtrip.
/// Pytket commands without a valid decoder produce an unsupported `TKET1.tk1op`
/// operation in the decoded HUGR.
///
/// Parameters:
/// - program: The CompilationState to run the pass on.
/// - pass_json: The JSON string of the pytket pass to run. See [pytket
///   documentation](https://docs.quantinuum.com/tket/api-docs/passes.html#pytket.passes.BasePass.to_dict)
///   for more details.
/// - target: The platform target identifier selecting which encoder/decoder
///   extension set to use. Defaults to the platform-agnostic `"tket"` target.
#[pyfunction]
#[pyo3(signature = (program, pass_json, *, scope = None, target = None))]
pub(crate) fn tket1_pass(
    program: &mut CompilationState,
    pass_json: &str,
    scope: Option<PyPassScope>,
    target: Option<&str>,
) -> PyResult<()> {
    // TODO: Implement a Tket1Pass: ComposablePass, use it here with the right scope.
    // <https://github.com/Quantinuum/tket2/issues/1494>
    let scope: PassScope = scope.unwrap_or_default().scope;
    let target = match target {
        None => PlatformTarget::default(),
        Some(s) => PlatformTarget::from_str(s)
            .map_err(|_| PyValueError::new_err(format!("Unknown platform target: {s:?}")))?,
    };
    let encode_options = EncodeOptions::new()
        .with_config(target.encoder_config())
        .with_subcircuits(scope.recursive());

    let Some(root) = scope.root(&program.hugr) else {
        return Ok(());
    };

    let mut encoded_circ = EncodedCircuit::new_with_entrypoint(&program.hugr, root, encode_options)
        .convert_pyerrs()?;

    encoded_circ
        .par_iter_mut()
        .try_for_each(|(_, circ)| -> Result<(), tket1_passes::PassError> {
            let mut tk1_circ = tket1_passes::Tket1Circuit::from_serial_circuit(circ)?;
            tket1_passes::Tket1Pass::run_from_json(pass_json, &mut tk1_circ)?;
            *circ = tk1_circ.to_serial_circuit()?;
            Ok(())
        })
        .convert_pyerrs()?;

    encoded_circ
        .reassemble_inplace(&mut program.hugr, Some(Arc::new(target.decoder_config())))
        .convert_pyerrs()?;

    Ok(())
}

create_py_exception!(
    tket1_passes::PassError,
    PytketPassError,
    "Error from a call to tket-c-api"
);
