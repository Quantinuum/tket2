//! Cost functions defined from python objects.

use std::cmp::Ordering;
use std::iter::Sum;
use std::ops::{Add, AddAssign, Sub};

use pyo3::{Py, PyAny, PyResult, PyTypeInfo, Python, pyclass, pymethods};
use tket::circuit::cost::{CircuitCost, CostDelta};

/// A generic circuit cost, backed by an arbitrary python object.
#[pyclass]
#[derive(Clone, Debug)]
#[pyo3(name = "CircuitCost")]
pub struct PyCircuitCost {
    /// Generic python cost object.
    pub cost: Py<PyAny>,
}

#[pymethods]
impl PyCircuitCost {
    /// Create a new circuit cost.
    #[new]
    pub fn new(cost: Py<PyAny>) -> Self {
        Self { cost }
    }
}

impl Default for PyCircuitCost {
    fn default() -> Self {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| PyCircuitCost { cost: py.None() })
    }
}

impl Add for PyCircuitCost {
    type Output = PyCircuitCost;

    fn add(self, rhs: PyCircuitCost) -> Self::Output {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| {
            let cost = self
                .cost
                .call_method1(py, "__add__", (rhs.cost,))
                .expect("Could not add circuit cost objects.");
            PyCircuitCost { cost }
        })
    }
}

impl AddAssign for PyCircuitCost {
    fn add_assign(&mut self, rhs: Self) {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| {
            let cost = self
                .cost
                .call_method1(py, "__add__", (rhs.cost,))
                .expect("Could not add circuit cost objects.");
            self.cost = cost;
        })
    }
}

impl Sub for PyCircuitCost {
    type Output = PyCircuitCost;

    fn sub(self, rhs: PyCircuitCost) -> Self::Output {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| {
            let cost = self
                .cost
                .call_method1(py, "__sub__", (rhs.cost,))
                .expect("Could not subtract circuit cost objects.");
            PyCircuitCost { cost }
        })
    }
}

impl Sum for PyCircuitCost {
    fn sum<I: Iterator<Item = Self>>(iter: I) -> Self {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| {
            let cost = iter
                .fold(None, |acc: Option<Py<PyAny>>, c| {
                    Some(match acc {
                        None => c.cost,
                        Some(cost) => cost
                            .call_method1(py, "__add__", (c.cost,))
                            .expect("Could not add circuit cost objects."),
                    })
                })
                .unwrap_or_else(|| py.None());
            PyCircuitCost { cost }
        })
    }
}

impl PartialEq for PyCircuitCost {
    fn eq(&self, other: &Self) -> bool {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| {
            let res = self
                .cost
                .call_method1(py, "__eq__", (&other.cost,))
                .expect("Could not compare circuit cost objects.");
            res.is_truthy(py)
                .expect("Could not compare circuit cost objects.")
        })
    }
}

impl Eq for PyCircuitCost {}

impl PartialOrd for PyCircuitCost {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PyCircuitCost {
    fn cmp(&self, other: &Self) -> Ordering {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| -> PyResult<Ordering> {
            let res = self.cost.call_method1(py, "__lt__", (&other.cost,))?;
            if res.is_truthy(py)? {
                return Ok(Ordering::Less);
            }
            let res = self.cost.call_method1(py, "__eq__", (&other.cost,))?;
            if res.is_truthy(py)? {
                return Ok(Ordering::Equal);
            }
            Ok(Ordering::Greater)
        })
        .expect("Could not compare circuit cost objects.")
    }
}

impl CostDelta for PyCircuitCost {
    fn as_isize(&self) -> isize {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| {
            let res = self
                .cost
                .call_method0(py, "__int__")
                .expect("Could not convert the circuit cost object to an integer.");
            res.extract(py)
                .expect("Could not convert the circuit cost object to an integer.")
        })
    }
}

impl CircuitCost for PyCircuitCost {
    type CostDelta = PyCircuitCost;

    fn as_usize(&self) -> usize {
        self.as_isize() as usize
    }

    fn sub_cost(&self, other: &Self) -> Self::CostDelta {
        self.clone() - other.clone()
    }

    fn add_delta(&self, delta: &Self::CostDelta) -> Self {
        self.clone() + delta.clone()
    }

    fn div_cost(&self, n: std::num::NonZeroUsize) -> Self {
        #[allow(deprecated, reason = "deprecated and renamed in pyo3 0.26")]
        Python::with_gil(|py| {
            let res = self
                .cost
                .call_method0(py, "__div__")
                .expect("Could not divide the circuit cost object.");
            Self { cost: res }
        })
    }
}
