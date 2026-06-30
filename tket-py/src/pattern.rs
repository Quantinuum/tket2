//! Pattern matching on circuits.

pub mod portmatching;

use anyhow::Context;

use crate::passes::PyPassScope;
use crate::rewrite::PyCircuitRewrite;
use crate::state::CompilationState;
use crate::utils::{ConvertPyErr, create_py_exception};

use hugr::{HugrView, Node, hugr::hugrmut::HugrMut};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use tket::portmatching::{CircuitPattern, PatternMatch, PatternMatcher};
use tket::{Circuit, CircuitError};

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
#[pyclass(from_py_object)]
/// A rewrite rule defined by a left hand side and right hand side of an equation.
pub struct Rule(pub [Circuit; 2]);

fn rule_circuit(state: &CompilationState) -> PyResult<Circuit> {
    let mut hugr = state.hugr.clone();

    if hugr.get_optype(hugr.entrypoint()).is_module() {
        let module = hugr.entrypoint();
        let entrypoint = {
            let mut children = hugr.children(module);
            let entrypoint = children
                .next()
                .ok_or_else(|| PyValueError::new_err("Rule module contains no circuit"))?;

            if children.next().is_some() {
                return Err(PyValueError::new_err(
                    "Rule module must contain exactly one circuit",
                ));
            }

            entrypoint
        };

        hugr.set_entrypoint(entrypoint);
    }

    Circuit::try_new(hugr).map_err(|error| PyValueError::new_err(error.to_string()))
}

#[pymethods]
impl Rule {
    #[new]
    fn new_rule(l: &CompilationState, r: &CompilationState) -> PyResult<Rule> {
        let l = rule_circuit(l)?;
        let r = rule_circuit(r)?;
        Ok(Rule([l, r]))
    }

    /// The left hand side of the rule.
    ///
    /// This is the pattern that will be matched against the target circuit.
    fn lhs(&self) -> CompilationState {
        CompilationState {
            hugr: self.0[0].clone().into_hugr(),
        }
    }

    /// The right hand side of the rule.
    ///
    /// This is the replacement that will be applied to the target circuit.
    fn rhs(&self) -> CompilationState {
        CompilationState {
            hugr: self.0[1].clone().into_hugr(),
        }
    }
}
#[pyclass(skip_from_py_object)]
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

    pub fn find_match(&self, target: &CompilationState) -> PyResult<Option<PyCircuitRewrite>> {
        let circ = Circuit::try_new(&target.hugr)
            .map_err(|error| PyValueError::new_err(error.to_string()))?;
        let Some(pmatch) = self.matcher.find_matches_iter(&circ).next() else {
            return Ok(None);
        };
        Ok(Some(self.match_to_rewrite(pmatch, &circ)?))
    }

    pub fn find_matches(&self, target: &CompilationState) -> PyResult<Vec<PyCircuitRewrite>> {
        let circ = Circuit::try_new(&target.hugr)
            .map_err(|error| PyValueError::new_err(error.to_string()))?;
        self.matcher
            .find_matches_iter(&circ)
            .map(|m| self.match_to_rewrite(m, &circ))
            .collect()
    }

    /// Apply the first matching rule repeatedly within each circuit-compatible
    /// region in the selected scope.
    ///
    /// Non-circuit regions are skipped. Returns the number of rewrites applied
    /// and restores the original HUGR entrypoint before returning.
    ///
    /// Returns a count of applied rewrites.
    #[pyo3(signature = (target, scope = None))]
    pub fn apply_exhaustive(
        &self,
        target: &mut CompilationState,
        scope: Option<PyPassScope>,
    ) -> anyhow::Result<usize> {
        let scope = scope.unwrap_or_default().scope;
        let original_entrypoint = target.hugr.entrypoint();
        let regions: Vec<_> = scope.regions(&target.hugr).collect();

        let result = (|| {
            let mut rewrite_count = 0;
            for region in regions {
                target.hugr.set_entrypoint(region);
                match Circuit::try_new(&target.hugr) {
                    Ok(_) => {}
                    Err(CircuitError::InvalidParentOp { .. }) => continue,
                    Err(error) => return Err(anyhow::Error::msg(error.to_string())),
                }

                while let Some(rewrite) = self.find_match(target)? {
                    target
                        .apply_rewrite(rewrite)
                        .context("Could not apply exhaustive rule rewrite")?;
                    rewrite_count += 1;
                }
            }
            Ok(rewrite_count)
        })();

        target.hugr.set_entrypoint(original_entrypoint);
        result
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
