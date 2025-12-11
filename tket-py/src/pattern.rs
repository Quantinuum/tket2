//! Pattern matching on circuits.

pub mod portmatching;

use crate::circuit::Tk2Circuit;
use crate::rewrite::PyCircuitRewrite;
use crate::utils::{create_py_exception, ConvertPyErr};
use derive_more::From;

use pyo3::prelude::*;
use tket::portmatching::{Rule, RuleMatcher};

/// The module definition
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "pattern")?;
    m.add_class::<PyRule>()?;
    m.add_class::<PyRuleMatcher>()?;
    m.add_class::<self::portmatching::PyCircuitPattern>()?;
    m.add_class::<self::portmatching::PyPatternMatcher>()?;
    m.add_class::<self::portmatching::PyPatternMatch>()?;
    m.add_class::<self::portmatching::PyPatternID>()?;

    m.add(
        "InvalidPatternError",
        py.get_type::<PyInvalidPatternError>(),
    )?;
    m.add(
        "InvalidReplacementError",
        py.get_type::<PyInvalidReplacementError>(),
    )?;

    Ok(m)
}

create_py_exception!(
    hugr::hugr::views::sibling_subgraph::InvalidReplacement,
    PyInvalidReplacementError,
    "Errors that can occur while constructing a HUGR replacement."
);

create_py_exception!(
    tket::portmatching::pattern::InvalidPattern,
    PyInvalidPatternError,
    "Conversion error from circuit to pattern."
);

/// A rewrite rule defined by a left hand side and right hand side of an equation.
#[pyclass]
#[pyo3(name = "Rule")]
#[repr(transparent)]
#[derive(Debug, Clone, From)]
pub struct PyRule(pub Rule);

#[pymethods]
impl PyRule {
    #[new]
    fn new_rule(l: &Bound<PyAny>, r: &Bound<PyAny>) -> PyResult<PyRule> {
        let l = Tk2Circuit::new(l)?;
        let r = Tk2Circuit::new(r)?;
        let rule = Rule::new(l.circ, r.circ);
        Ok(PyRule(rule))
    }

    /// The left hand side of the rule.
    ///
    /// This is the pattern that will be matched against the target circuit.
    fn lhs(&self) -> Tk2Circuit {
        Tk2Circuit { circ: self.0.lhs() }
    }

    /// The right hand side of the rule.
    ///
    /// This is the replacement that will be applied to the target circuit.
    fn rhs(&self) -> Tk2Circuit {
        Tk2Circuit { circ: self.0.rhs() }
    }
}

#[pyclass]
#[pyo3(name = "RuleMatcher")]
#[repr(transparent)]
#[derive(Debug, Clone, From)]
struct PyRuleMatcher {
    rmatcher: RuleMatcher,
}

#[pymethods]
impl PyRuleMatcher {
    #[new]
    pub fn from_rules(rules: Vec<PyRule>) -> PyResult<Self> {
        let rules: Vec<Rule> = rules.into_iter().map(|r| r.0).collect();
        let rmatcher = RuleMatcher::from_rules(rules).convert_pyerrs()?;

        Ok(Self { rmatcher })
    }

    pub fn find_match(&self, target: &Tk2Circuit) -> PyResult<Option<PyCircuitRewrite>> {
        let circ = &target.circ;
        self.rmatcher
            .find_match(circ)
            .convert_pyerrs()
            .map(|optn| optn.map(|rewrite| rewrite.into()))
    }

    pub fn find_matches(&self, target: &Tk2Circuit) -> PyResult<Vec<PyCircuitRewrite>> {
        let circ = &target.circ;
        self.rmatcher
            .find_matches(circ)
            .convert_pyerrs()
            .map(|vec| vec.into_iter().map(|rewrite| rewrite.into()).collect())
    }
}
