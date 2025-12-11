//! Utility functions for the library.

use hugr::Hugr;
use hugr::builder::FunctionBuilder;
use hugr::types::{Type, TypeBound};
use hugr::{
    builder::{BuildError, CircuitBuilder, Dataflow, DataflowHugr},
    extension::prelude::qb_t,
    types::Signature,
};

use crate::circuit::Circuit;

pub(crate) fn type_is_linear(typ: &Type) -> bool {
    !TypeBound::Copyable.contains(typ.least_upper_bound())
}

/// Utility for building simple qubit-only circuits.
#[allow(unused)]
pub(crate) fn build_simple_circuit<F>(num_qubits: usize, f: F) -> Result<Circuit, BuildError>
where
    F: FnOnce(&mut CircuitBuilder<'_, FunctionBuilder<Hugr>>) -> Result<(), BuildError>,
{
    let qb_row = vec![qb_t(); num_qubits];
    let signature = Signature::new(qb_row.clone(), qb_row);
    let mut h = FunctionBuilder::new("main", signature)?;

    let qbs = h.input_wires();

    let mut circ = h.as_circuit(qbs);

    f(&mut circ)?;

    let qbs = circ.finish();

    let hugr = h.finish_hugr_with_outputs(qbs)?;
    Ok(hugr.into())
}
