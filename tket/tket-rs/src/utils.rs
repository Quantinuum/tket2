//! Utility functions for the library.

use hugr::types::{Type, TypeBound};

pub(crate) fn type_is_linear(typ: &Type) -> bool {
    !TypeBound::Copyable.contains(typ.least_upper_bound())
}

/// Utility for building simple qubit-only circuits.
#[cfg(test)]
pub(crate) fn build_simple_circuit<F>(
    num_qubits: usize,
    f: F,
) -> Result<crate::Circuit, hugr::builder::BuildError>
where
    F: FnOnce(
        &mut hugr::builder::CircuitBuilder<'_, hugr::builder::FunctionBuilder<hugr::Hugr>>,
    ) -> Result<(), hugr::builder::BuildError>,
{
    use hugr::builder::FunctionBuilder;
    use hugr::{
        builder::{Dataflow, DataflowHugr},
        extension::prelude::qb_t,
        types::Signature,
    };

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
