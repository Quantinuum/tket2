//! Utility functions for the python interface.

use itertools::Itertools;
/// A trait for types wrapping rust errors that may be converted into python exception.
///
/// In addition to raw errors, this is implemented for wrapper types such as `Result`.
/// [`ConvertPyErr::convert_errors`] will be called on the internal error type.
pub trait ConvertPyErr {
    /// The output type after conversion.
    type Output;

    /// Convert any internal errors to python errors.
    fn convert_pyerrs(self) -> Self::Output;
}

impl ConvertPyErr for pyo3::PyErr {
    type Output = Self;

    fn convert_pyerrs(self) -> Self::Output {
        self
    }
}

impl<T, E> ConvertPyErr for Result<T, E>
where
    E: ConvertPyErr,
{
    type Output = Result<T, E::Output>;

    fn convert_pyerrs(self) -> Self::Output {
        self.map_err(|e| e.convert_pyerrs())
    }
}

macro_rules! create_py_exception {
    ($err:path, $py_err:ident, $doc:expr) => {
        pyo3::create_exception!(tket, $py_err, pyo3::exceptions::PyException, $doc);

        impl $crate::utils::ConvertPyErr for $err {
            type Output = pyo3::PyErr;

            fn convert_pyerrs(self) -> Self::Output {
                $py_err::new_err(<Self as std::string::ToString>::to_string(&self))
            }
        }
    };
}
pub(crate) use create_py_exception;

/// Convert an iterator of one type into vector of another type.
pub fn into_vec<T, S: From<T>>(v: impl IntoIterator<Item = T>) -> Vec<S> {
    v.into_iter().map_into().collect()
}

#[cfg(test)]
pub(crate) mod test {
    use hugr::Hugr;
    use hugr::builder::{
        BuildError, CircuitBuilder, Container, Dataflow, DataflowHugr, DataflowSubContainer,
        FunctionBuilder, HugrBuilder, ModuleBuilder,
    };
    use hugr::extension::prelude::qb_t;
    use hugr::ops::handle::NodeHandle;
    use pyo3::{Bound, PyResult, Python};
    use tket::Circuit;
    use tket::TketOp;

    use crate::circuit::Tk2Circuit;

    /// Utility for building a module with a single circuit definition.
    pub fn build_module_with_circuit<F>(num_qubits: usize, f: F) -> Result<Circuit, BuildError>
    where
        F: FnOnce(&mut CircuitBuilder<'_, FunctionBuilder<Hugr>>) -> Result<(), BuildError>,
    {
        let qb_row = vec![qb_t(); num_qubits];
        let circ_signature = FunctionType::new(qb_row.clone(), qb_row);
        let mut dfg = FunctionBuilder::new("main", circ_signature.into())?;
        let mut circ = dfg.as_circuit(dfg.input_wires());
        f(&mut circ)?;
        let qbs = circ.finish();
        let hugr = dfg.finish_hugr_with_outputs(qbs)?;
        Ok(Circuit::new(hugr))
    }

    /// Generates a simple tket circuit for testing,
    /// defined as a function inside a module.
    pub fn make_module_tk2_circuit<'py>(py: Python<'py>) -> PyResult<Bound<'py, Tk2Circuit>> {
        let circ = build_module_with_circuit(2, |circ| {
            circ.append(TketOp::H, [0])?;
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::X, [1])?;
            Ok(())
        })
        .unwrap();
        Bound::new(py, Tk2Circuit { circ })
    }
}
