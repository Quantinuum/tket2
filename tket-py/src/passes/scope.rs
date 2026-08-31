//! Bindings for pass scopes.

use pyo3::prelude::*;
use pyo3::types::PyAnyMethods;
use tket::passes::composable::{PassScope, Preserve};

#[derive(Debug, Default, Clone, derive_more::From, derive_more::Into)]
pub(crate) struct PyPassScope {
    pub scope: PassScope,
}

impl<'a, 'py> FromPyObject<'a, 'py> for PyPassScope {
    type Error = PyErr;

    fn extract(ob: pyo3::Borrowed<'a, 'py, PyAny>) -> Result<Self, Self::Error> {
        let value: String = ob.getattr("value")?.extract()?;
        let scope = match value.as_str() {
            "EntrypointFlat" => PassScope::EntrypointFlat,
            "EntrypointRecursive" => PassScope::EntrypointRecursive,
            "GlobalAll" => PassScope::Global(Preserve::All),
            "GlobalPublic" => PassScope::Global(Preserve::Public),
            "GlobalEntrypoint" => PassScope::Global(Preserve::Entrypoint),
            _ => {
                return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Unknown PassScope value: {value:?}"
                )));
            }
        };
        Ok(PyPassScope { scope })
    }
}

impl<'py> IntoPyObject<'py> for PyPassScope {
    type Target = PyAny;
    type Output = Bound<'py, Self::Target>;
    type Error = PyErr;

    fn into_pyobject(self, py: Python<'py>) -> Result<Self::Output, Self::Error> {
        let scope_mod = py.import("hugr.passes._scope")?;
        let obj = match self.scope {
            PassScope::EntrypointFlat => scope_mod.getattr("LocalScope")?.getattr("FLAT")?,
            PassScope::EntrypointRecursive => {
                scope_mod.getattr("LocalScope")?.getattr("RECURSIVE")?
            }
            PassScope::Global(Preserve::All) => {
                scope_mod.getattr("GlobalScope")?.getattr("PRESERVE_ALL")?
            }
            PassScope::Global(Preserve::Public) => scope_mod
                .getattr("GlobalScope")?
                .getattr("PRESERVE_PUBLIC")?,
            PassScope::Global(Preserve::Entrypoint) => scope_mod
                .getattr("GlobalScope")?
                .getattr("PRESERVE_ENTRYPOINT")?,
            _ => {
                return Err(PyErr::new::<pyo3::exceptions::PyValueError, _>(format!(
                    "Unknown PassScope variant: {:?}",
                    self.scope
                )));
            }
        };
        Ok(obj)
    }
}
