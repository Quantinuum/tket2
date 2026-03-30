//! Passes that call to tket1-passes using the tket-c-api.

use rayon::iter::ParallelIterator;
use std::sync::Arc;

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
/// - traverse_subcircuits: Whether to recurse into the children of the
///   circuit-like regions, and optimise them too.
#[pyfunction]
#[pyo3(signature = (program, pass_json, *, traverse_subcircuits = true))]
pub fn tket1_pass(
    program: &mut CompilationState,
    pass_json: &str,
    traverse_subcircuits: bool,
) -> PyResult<()> {
    let mut encoded_circ = EncodedCircuit::new(
        &program.hugr,
        EncodeOptions::new()
            .with_config(qsystem_encoder_config())
            .with_subcircuits(traverse_subcircuits),
    )
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
        .reassemble_inplace(&mut program.hugr, Some(Arc::new(qsystem_decoder_config())))
        .convert_pyerrs()?;

    Ok(())
}

create_py_exception!(
    tket1_passes::PassError,
    PytketPassError,
    "Error from a call to tket-c-api"
);
