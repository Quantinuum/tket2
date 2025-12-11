//! Pattern matching on circuits.

pub mod portmatching;

use crate::circuit::Tk2Circuit;
use crate::rewrite::PyCircuitRewrite;
use crate::utils::{create_py_exception, ConvertPyErr};

use hugr::{HugrView, Node};
use pyo3::prelude::*;
use tket::portmatching::{CircuitPattern, PatternMatch, PatternMatcher};
use tket::Circuit;

/// The module definition
pub fn module(py: Python<'_>) -> PyResult<Bound<'_, PyModule>> {
    let m = PyModule::new(py, "pattern")?;
    m.add_class::<Rule>()?;
    m.add_class::<RuleMatcher>()?;
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

#[derive(Clone)]
#[pyclass]
/// A rewrite rule defined by a left hand side and right hand side of an equation.
pub struct Rule(pub [Circuit; 2]);

#[pymethods]
impl Rule {
    #[new]
    fn new_rule(l: &Bound<PyAny>, r: &Bound<PyAny>) -> PyResult<Rule> {
        let l = Tk2Circuit::new(l)?;
        let r = Tk2Circuit::new(r)?;

        Ok(Rule([l.circ, r.circ]))
    }

    /// The left hand side of the rule.
    ///
    /// This is the pattern that will be matched against the target circuit.
    fn lhs(&self) -> Tk2Circuit {
        Tk2Circuit {
            circ: self.0[0].clone(),
        }
    }

    /// The right hand side of the rule.
    ///
    /// This is the replacement that will be applied to the target circuit.
    fn rhs(&self) -> Tk2Circuit {
        Tk2Circuit {
            circ: self.0[1].clone(),
        }
    }
}
#[pyclass]
struct RuleMatcher {
    matcher: PatternMatcher,
    rights: Vec<Circuit>,
}

#[pymethods]
impl RuleMatcher {
    #[new]
    pub fn from_rules(rules: Vec<Rule>) -> PyResult<Self> {
        let (lefts, rights): (Vec<_>, Vec<_>) =
            rules.into_iter().map(|Rule([l, r])| (l, r)).unzip();
        let patterns: Result<Vec<CircuitPattern>, _> =
            lefts.iter().map(CircuitPattern::try_from_circuit).collect();
        let matcher = PatternMatcher::from_patterns(patterns.convert_pyerrs()?);

        Ok(Self { matcher, rights })
    }

    pub fn find_match(&self, target: &Tk2Circuit) -> PyResult<Option<PyCircuitRewrite>> {
        let circ = &target.circ;
        let Some(pmatch) = self.matcher.find_matches_iter(circ).next() else {
            return Ok(None);
        };
        Ok(Some(self.match_to_rewrite(pmatch, circ)?))
    }

    pub fn find_matches(&self, target: &Tk2Circuit) -> PyResult<Vec<PyCircuitRewrite>> {
        let circ = &target.circ;
        self.matcher
            .find_matches_iter(circ)
            .map(|m| self.match_to_rewrite(m, circ))
            .collect()
    }
}

impl RuleMatcher {
    fn match_to_rewrite(
        &self,
        pmatch: PatternMatch,
        target: &Circuit<impl HugrView<Node = Node>>,
    ) -> PyResult<PyCircuitRewrite> {
        let r = self.rights.get(pmatch.pattern_id().0).unwrap().clone();
        let rw = pmatch.to_rewrite(target, r).convert_pyerrs()?;
        Ok(rw.into())
    }
}
