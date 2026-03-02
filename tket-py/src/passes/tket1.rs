//! Passes that call to tket1-passes using the tket-c-api.

use hugr_passes::PassScope;
use rayon::iter::ParallelIterator;
use std::sync::Arc;

use pyo3::prelude::*;
use tket::serialize::pytket::{EncodeOptions, EncodedCircuit};
use tket_qsystem::pytket::{qsystem_decoder_config, qsystem_encoder_config};

use crate::circuit::try_with_circ;
use crate::passes::PyPassScope;
use crate::utils::{ConvertPyErr, create_py_exception};

/// Runs a pytket pass on all circuit-like regions under the entrypoint of the
/// HUGR.
///
/// Parameters:
/// - circ: The circuit to run the pass on.
/// - pass_json: The JSON string of the pytket pass to run. See [pytket
///   documentation](https://docs.quantinuum.com/tket/api-docs/passes.html#pytket.passes.BasePass.to_dict)
///   for more details.
#[pyfunction]
#[pyo3(signature = (circ, pass_json, *, scope = None))]
pub(crate) fn tket1_pass<'py>(
    circ: &Bound<'py, PyAny>,
    pass_json: &str,
    scope: Option<PyPassScope>,
) -> PyResult<Bound<'py, PyAny>> {
    let py = circ.py();

    try_with_circ(circ, |mut circ, typ| {
        // TODO: Implement a Tket1Pass: ComposablePass, use it here with the right scope.
        // (Create an issue and link it here).
        let scope: PassScope = scope.unwrap_or_default().scope;
        let encode_options = EncodeOptions::new()
            .with_config(qsystem_encoder_config())
            .with_subcircuits(scope.recursive());

        let Some(root) = scope.root(circ.hugr()) else {
            // Local scope with module entrypoint, do not modify the hugr.
            let circ = typ.convert(py, circ)?;
            return PyResult::Ok(circ);
        };

        let mut encoded_circ =
            EncodedCircuit::new_with_entrypoint(circ.hugr(), root, encode_options)
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
            .reassemble_inplace(circ.hugr_mut(), Some(Arc::new(qsystem_decoder_config())))
            .convert_pyerrs()?;

        let circ = typ.convert(py, circ)?;
        PyResult::Ok(circ)
    })
}

create_py_exception!(
    tket1_passes::PassError,
    PytketPassError,
    "Error from a call to tket-c-api"
);
