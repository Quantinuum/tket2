//! Python bindings for portmatching features

use std::fmt;

use derive_more::{From, Into};
use itertools::Itertools;
use portmatching::PatternID;
use pyo3::{prelude::*, types::PyIterator};

use tket::portmatching::{CircuitPattern, PatternMatch, PatternMatcher};

use crate::circuit::{PyNode, try_with_circ, with_circ};

/// A pattern that match a circuit exactly
///
/// Python equivalent of [`CircuitPattern`].
///
/// [`CircuitPattern`]: tket::portmatching::matcher::CircuitPattern
#[pyclass]
#[pyo3(name = "CircuitPattern")]
#[repr(transparent)]
#[derive(Debug, Clone, From)]
pub struct PyCircuitPattern {
    /// Rust representation of the pattern
    pub pattern: CircuitPattern,
}

#[pymethods]
impl PyCircuitPattern {
    /// Construct a pattern from a TKET1 circuit
    #[new]
    pub fn from_circuit(circ: &Bound<PyAny>) -> PyResult<Self> {
        let pattern = try_with_circ(circ, |circ, _| CircuitPattern::try_from_circuit(&circ))?;
        Ok(pattern.into())
    }

    /// A string representation of the pattern.
    pub fn __repr__(&self) -> String {
        format!("{:?}", self.pattern)
    }
}

/// A matcher object for fast pattern matching on circuits.
///
/// This uses a state automaton internally to match against a set of patterns
/// simultaneously.
///
/// Python equivalent of [`PatternMatcher`].
///
/// [`PatternMatcher`]: tket::portmatching::matcher::PatternMatcher
#[pyclass]
#[pyo3(name = "PatternMatcher")]
#[repr(transparent)]
#[derive(Debug, Clone, From)]
pub struct PyPatternMatcher {
    /// Rust representation of the matcher
    pub matcher: PatternMatcher,
}

#[pymethods]
impl PyPatternMatcher {
    /// Construct a matcher from a list of patterns.
    #[new]
    pub fn py_from_patterns(patterns: &Bound<PyIterator>) -> PyResult<Self> {
        Ok(PatternMatcher::from_patterns(
            patterns
                .try_iter()?
                .map(|p| {
                    let py_pattern = p?.extract::<PyCircuitPattern>()?;
                    Ok(py_pattern.pattern)
                })
                .collect::<PyResult<Vec<_>>>()?,
        )
        .into())
    }
    /// A string representation of the pattern.
    pub fn __repr__(&self) -> PyResult<String> {
        Ok(format!("{:?}", self.matcher))
    }

    /// Find one convex match in a circuit.
    pub fn find_match(&self, circ: &Bound<PyAny>) -> PyResult<Option<PyPatternMatch>> {
        with_circ(circ, |circ, _| {
            self.matcher.find_matches_iter(&circ).next().map(Into::into)
        })
    }

    /// Find all convex matches in a circuit.
    pub fn find_matches(&self, circ: &Bound<PyAny>) -> PyResult<Vec<PyPatternMatch>> {
        with_circ(circ, |circ, _| {
            self.matcher
                .find_matches(&circ)
                .into_iter()
                .map_into()
                .collect()
        })
    }
}

/// A convex pattern match in a circuit, available from Python.
///
/// Python equivalent of [`PatternMatch`].
///
/// [`PatternMatch`]: tket::portmatching::matcher::PatternMatch
#[pyclass]
#[derive(Debug, Clone, From)]
#[pyo3(name = "PatternMatch")]
pub struct PyPatternMatch {
    pmatch: PatternMatch,
}

#[pymethods]
impl PyPatternMatch {
    /// The matched pattern ID.
    pub fn pattern_id(&self) -> PyPatternID {
        self.pmatch.pattern_id().into()
    }

    /// Returns the root of the pattern in the circuit.
    pub fn root(&self) -> PyNode {
        self.pmatch.root().into()
    }

    /// A string representation of the pattern.
    pub fn __repr__(&self) -> String {
        format!("{:?}", self.pmatch)
    }
}

/// A [`hugr::Node`] wrapper for Python.
#[pyclass]
#[pyo3(name = "PatternID")]
#[repr(transparent)]
#[derive(From, Into, PartialEq, Eq, Hash, Clone, Copy)]
pub struct PyPatternID {
    /// Rust representation of the pattern ID
    pub id: PatternID,
}

#[pymethods]
impl PyPatternID {
    /// A string representation of the pattern.
    pub fn __repr__(&self) -> String {
        format!("{:?}", self.id)
    }

    /// Cast the pattern ID to an integer.
    pub fn __int__(&self) -> usize {
        self.id.into()
    }
}

impl fmt::Display for PyPatternID {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.id.fmt(f)
    }
}

impl fmt::Debug for PyPatternID {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.id.fmt(f)
    }
}
