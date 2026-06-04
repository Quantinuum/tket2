//! Passes that call to tket1-passes using the tket-c-api.

use rayon::iter::ParallelIterator;
use std::sync::Arc;
use tket::passes::composable::PassScope;
use tket_qsystem::QSystemPlatform;

use crate::passes::PyPassScope;
use pyo3::prelude::*;
use tket::serialize::pytket::{EncodeOptions, EncodedCircuit};
use tket_qsystem::pytket::{qsystem_decoder_config, qsystem_encoder_config};

use crate::state::CompilationState;
use crate::utils::{ConvertPyErr, create_py_exception};

/// Runs a pytket pass on all circuit-like regions under the entrypoint of the
/// HUGR.
///
/// Parameters:
/// - program: The CompilationState to run the pass on.
/// - pass_json: The JSON string of the pytket pass to run. See [pytket
///   documentation](https://docs.quantinuum.com/tket/api-docs/passes.html#pytket.passes.BasePass.to_dict)
///   for more details.
#[pyfunction]
#[pyo3(signature = (program, pass_json, *, scope = None))]
pub(crate) fn tket1_pass(
    program: &mut CompilationState,
    pass_json: &str,
    scope: Option<PyPassScope>,
) -> PyResult<()> {
    // TODO: Implement a Tket1Pass: ComposablePass, use it here with the right scope.
    // <https://github.com/Quantinuum/tket2/issues/1494>
    let scope: PassScope = scope.unwrap_or_default().scope;
    let encode_options = EncodeOptions::new()
        .with_config(qsystem_encoder_config(QSystemPlatform::Helios))
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
        .reassemble_inplace(
            &mut program.hugr,
            // TODO: Make the decoder set configurable.
            // <https://github.com/Quantinuum/tket2/issues/1619>
            Some(Arc::new(qsystem_decoder_config(QSystemPlatform::Helios))),
        )
        .convert_pyerrs()?;

    Ok(())
}

create_py_exception!(
    tket1_passes::PassError,
    PytketPassError,
    "Error from a call to tket-c-api"
);
