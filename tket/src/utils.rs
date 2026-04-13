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
    use hugr::{extension::prelude::qb_t, types::Signature};

    let qb_row = vec![qb_t(); num_qubits];
    build_circuit(Signature::new(qb_row.clone(), qb_row), f)
}

#[cfg(test)]
pub(crate) fn build_circuit<F>(
    signature: hugr_core::types::Signature,
    f: F,
) -> Result<crate::Circuit, hugr::builder::BuildError>
where
    F: FnOnce(
        &mut hugr::builder::CircuitBuilder<'_, hugr::builder::FunctionBuilder<hugr_core::Hugr>>,
    ) -> Result<(), hugr::builder::BuildError>,
{
    use hugr::builder::{Dataflow, DataflowHugr};
    use hugr_core::builder::FunctionBuilder;

    let mut h = FunctionBuilder::new("main", signature)?;
    let mut circ = h.as_circuit(h.input_wires());
    f(&mut circ)?;
    let outputs = circ.finish();
    let hugr = h.finish_hugr_with_outputs(outputs)?;
    Ok(hugr.into())
}
