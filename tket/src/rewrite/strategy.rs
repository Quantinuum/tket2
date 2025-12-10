//! Rewriting strategies for circuit optimisation.
//!
//! This module contains the [`RewriteStrategy`] trait, which is currently
//! implemented by
//! - [`GreedyRewriteStrategy`], which applies as many rewrites as possible on
//!   one circuit, and
//! - Exhaustive strategies, which clone the original circuit and explore every
//!   possible rewrite (with some pruning strategy):
//!    - [`ExhaustiveGreedyStrategy`], which applies multiple combinations of
//!      non-overlapping rewrites.
//!    - [`ExhaustiveThresholdStrategy`], which tries every rewrite below
//!      threshold function.
//!
//! The exhaustive strategies are parametrised by a strategy cost function:
//!    - [`LexicographicCostFunction`] allows rewrites that do
//!      not increase some coarse cost function (e.g. CX count), whilst
//!      ordering them according to a lexicographic ordering of finer cost
//!      functions (e.g. total gate count). See
//!      [`LexicographicCostFunction::default_cx_strategy`]) for a default implementation.
//!    - [`GammaStrategyCost`] ignores rewrites that increase the cost
//!      function beyond a percentage given by a f64 parameter gamma.

use std::iter;
use std::{collections::HashSet, fmt::Debug};

use derive_more::From;
use hugr::ops::OpType;
use hugr::{HugrView, Node};
use itertools::Itertools;

use crate::circuit::cost::{is_cx, is_quantum, CircuitCost, CostDelta, LexicographicCost};
use crate::{op_matches, Circuit, TketOp};

use super::trace::RewriteTrace;
use super::CircuitRewrite;

/// Rewriting strategies for circuit optimisation.
///
/// A rewrite strategy takes a set of possible rewrites and applies them
/// to a circuit according to a strategy. It returns a list of new circuits,
/// each obtained by applying one or several non-overlapping rewrites to the
/// original circuit.
///
/// It also assign every circuit a totally ordered cost that can be used when
/// using rewrites for circuit optimisation.
pub trait RewriteStrategy {
    /// The circuit cost to be minimised.
    type Cost: CircuitCost;

    /// Apply a set of rewrites to a circuit.
    fn apply_rewrites(
        &self,
        rewrites: impl IntoIterator<Item = CircuitRewrite>,
        circ: &Circuit,
    ) -> impl Iterator<Item = RewriteResult<Self::Cost>>;

    /// The cost of a single operation for this strategy's cost function.
    fn op_cost(&self, op: &OpType) -> Self::Cost;

    /// The cost of a circuit using this strategy's cost function.
    #[inline]
    fn circuit_cost(&self, circ: &Circuit<impl HugrView<Node = Node>>) -> Self::Cost {
        circ.circuit_cost(|op| self.op_cost(op))
    }

    /// Returns the cost of a rewrite's matched subcircuit before replacing it.
    #[inline]
    fn pre_rewrite_cost(&self, rw: &CircuitRewrite, circ: &Circuit) -> Self::Cost {
        circ.nodes_cost(rw.subgraph().nodes().iter().copied(), |op| self.op_cost(op))
    }

    /// Returns the expected cost of a rewrite's matched subcircuit after replacing it.
    fn post_rewrite_cost(&self, rw: &CircuitRewrite) -> Self::Cost {
        rw.replacement().circuit_cost(|op| self.op_cost(op))
    }
}

/// A possible rewrite result returned by a rewrite strategy.
#[derive(Debug, Clone)]
pub struct RewriteResult<C: CircuitCost> {
    /// The rewritten circuit.
    pub circ: Circuit,
    /// The cost delta of the rewrite.
    pub cost_delta: C::CostDelta,
}

impl<C: CircuitCost, T: HugrView<Node = Node>> From<(Circuit<T>, C::CostDelta)>
    for RewriteResult<C>
{
    #[inline]
    fn from((circ, cost_delta): (Circuit<T>, C::CostDelta)) -> Self {
        Self {
            circ: circ.to_owned(),
            cost_delta,
        }
    }
}

/// A rewrite strategy applying as many non-overlapping rewrites as possible.
///
/// All possible rewrites are sorted by the number of gates they remove from
/// the circuit and are applied in order. If a rewrite overlaps with a rewrite
/// that has already been applied, it is skipped.
///
/// This strategy will always return exactly one circuit: the original circuit
/// with as many rewrites applied as possible.
///
/// Rewrites are only applied if they strictly decrease gate count.
#[derive(Debug, Copy, Clone)]
pub struct GreedyRewriteStrategy;

impl RewriteStrategy for GreedyRewriteStrategy {
    type Cost = usize;

    #[tracing::instrument(skip_all)]
    fn apply_rewrites(
        &self,
        rewrites: impl IntoIterator<Item = CircuitRewrite>,
        circ: &Circuit,
    ) -> impl Iterator<Item = RewriteResult<Self::Cost>> {
        let rewrites = rewrites
            .into_iter()
            .sorted_by_key(|rw| rw.node_count_delta())
            .take_while(|rw| rw.node_count_delta() < 0);
        let mut changed_nodes = HashSet::new();
        let mut cost_delta = 0;
        let mut circ = circ.clone();
        for rewrite in rewrites {
            if rewrite
                .subgraph()
                .nodes()
                .iter()
                .any(|n| changed_nodes.contains(n))
            {
                continue;
            }
            changed_nodes.extend(rewrite.subgraph().nodes().iter().copied());
            cost_delta += rewrite.node_count_delta();
            rewrite
                .apply(&mut circ)
                .expect("Could not perform rewrite in greedy strategy");
        }
        iter::once((circ, cost_delta).into())
    }

    fn circuit_cost(&self, circ: &Circuit<impl HugrView<Node = Node>>) -> Self::Cost {
        circ.num_operations()
    }

    fn op_cost(&self, _op: &OpType) -> Self::Cost {
        1
    }
}

/// Exhaustive strategies based on cost functions and thresholds.
///
/// Every possible rewrite is applied to a copy of the input circuit. In
/// addition, other non-overlapping rewrites are applied greedily in ascending
/// cost delta.
///
/// Thus for one circuit, up to `n` rewritten circuits will be returned, each
/// obtained by applying at least one of the `n` rewrites to the original
/// circuit.
///
/// Whether a rewrite is allowed or not is determined by a cost function and a
/// threshold function: if the cost of the target of the rewrite is below the
/// threshold given by the cost of the original circuit, the rewrite is
/// performed.
///
/// The cost function must return a value of type `Self::OpCost`. All op costs
/// are summed up to obtain a total cost that is then compared using the
/// threshold function.
///
/// This kind of strategy is not recommended for thresholds that allow positive
/// cost deltas, as these will always be greedily applied even if they increase
/// the final cost.
#[derive(Debug, Copy, Clone, From)]
pub struct ExhaustiveGreedyStrategy<T> {
    /// The cost function.
    pub strat_cost: T,
}

impl<T: StrategyCost> RewriteStrategy for ExhaustiveGreedyStrategy<T> {
    type Cost = T::OpCost;

    #[tracing::instrument(skip_all)]
    fn apply_rewrites(
        &self,
        rewrites: impl IntoIterator<Item = CircuitRewrite>,
        circ: &Circuit,
    ) -> impl Iterator<Item = RewriteResult<Self::Cost>> {
        // Check only the rewrites that reduce the size of the circuit.
        let rewrites = rewrites
            .into_iter()
            .filter_map(|rw| {
                let pattern_cost = self.pre_rewrite_cost(&rw, circ);
                let target_cost = self.post_rewrite_cost(&rw);
                if !self.strat_cost.under_threshold(&pattern_cost, &target_cost) {
                    return None;
                }
                Some((rw, target_cost.sub_cost(&pattern_cost)))
            })
            .sorted_by_key(|(_, delta)| delta.clone())
            .collect_vec();

        (0..rewrites.len()).map(move |i| {
            let mut curr_circ = circ.clone();
            let mut changed_nodes = HashSet::new();
            let mut cost_delta = Default::default();
            let mut composed_rewrite_count = 0;
            for (rewrite, delta) in &rewrites[i..] {
                if !changed_nodes.is_empty()
                    && rewrite
                        .invalidation_set()
                        .any(|n| changed_nodes.contains(&n))
                {
                    continue;
                }
                changed_nodes.extend(rewrite.invalidation_set());
                cost_delta += delta.clone();

                composed_rewrite_count += 1;

                rewrite
                    .clone()
                    .apply_notrace(&mut curr_circ)
                    .expect("Could not perform rewrite in exhaustive greedy strategy");
            }

            curr_circ.add_rewrite_trace(RewriteTrace::new(composed_rewrite_count));
            (curr_circ, cost_delta).into()
        })
    }

    #[inline]
    fn op_cost(&self, op: &OpType) -> Self::Cost {
        self.strat_cost.op_cost(op)
    }
}

/// Exhaustive strategies based on cost functions and thresholds.
///
/// Every possible rewrite is applied to a copy of the input circuit. Thus for
/// one circuit, up to `n` rewritten circuits will be returned, each obtained
/// by applying one of the `n` rewrites to the original circuit.
///
/// Whether a rewrite is allowed or not is determined by a cost function and a
/// threshold function: if the cost of the target of the rewrite is below the
/// threshold given by the cost of the original circuit, the rewrite is
/// performed.
///
/// The cost function must return a value of type `Self::OpCost`. All op costs
/// are summed up to obtain a total cost that is then compared using the
/// threshold function.
#[derive(Debug, Copy, Clone, From)]
pub struct ExhaustiveThresholdStrategy<T> {
    /// The cost function.
    pub strat_cost: T,
}

impl<T: StrategyCost> RewriteStrategy for ExhaustiveThresholdStrategy<T> {
    type Cost = T::OpCost;

    #[tracing::instrument(skip_all)]
    fn apply_rewrites(
        &self,
        rewrites: impl IntoIterator<Item = CircuitRewrite>,
        circ: &Circuit,
    ) -> impl Iterator<Item = RewriteResult<Self::Cost>> {
        rewrites.into_iter().filter_map(|rw| {
            let pattern_cost = self.pre_rewrite_cost(&rw, circ);
            let target_cost = self.post_rewrite_cost(&rw);
            if !self.strat_cost.under_threshold(&pattern_cost, &target_cost) {
                return None;
            }
            let mut circ = circ.clone();
            rw.apply(&mut circ).expect("invalid pattern match");
            Some((circ, target_cost.sub_cost(&pattern_cost)).into())
        })
    }

    #[inline]
    fn op_cost(&self, op: &OpType) -> Self::Cost {
        self.strat_cost.op_cost(op)
    }
}

/// Cost function definitions required in exhaustive strategies.
///
/// See [`ExhaustiveThresholdStrategy`], [`ExhaustiveGreedyStrategy`].
pub trait StrategyCost {
    /// The cost of a single operation.
    type OpCost: CircuitCost;

    /// Returns true if the rewrite is allowed, based on the cost of the pattern and target.
    #[inline]
    fn under_threshold(&self, pattern_cost: &Self::OpCost, target_cost: &Self::OpCost) -> bool {
        target_cost.sub_cost(pattern_cost).as_isize() <= 0
    }

    /// The cost of a single operation.
    fn op_cost(&self, op: &OpType) -> Self::OpCost;
}

/// Rewrite strategy cost allowing smaller or equal cost rewrites.
///
/// Rewrites are permitted based on a coarse cost function: if
/// the coarse cost of the target of the rewrite is smaller or equal to the
/// coarse cost of the pattern, the rewrite is allowed.
///
/// Further more fine-grained cost functions can be used as tie breakers: within
/// circuits with the same coarse cost, circuits are ranked according to a
/// lexicographic ordering of their cost functions.
///
/// An example would be to use the number of CX gates as coarse cost and the
/// total number of gates as the only fine grained cost function.
///
/// Lexicographic orderings may be useful to add relief to an otherwise flat
/// optimisation landscape.
#[derive(Debug, Clone)]
pub struct LexicographicCostFunction<F, const N: usize> {
    cost_fns: [F; N],
}

impl<F, const N: usize> StrategyCost for LexicographicCostFunction<F, N>
where
    F: Fn(&OpType) -> usize,
{
    type OpCost = LexicographicCost<usize, N>;

    #[inline]
    fn op_cost(&self, op: &OpType) -> Self::OpCost {
        let mut costs = [0; N];
        for (cost_fn, cost_mut) in self.cost_fns.iter().zip(&mut costs) {
            *cost_mut = cost_fn(op);
        }
        costs.into()
    }
}

impl LexicographicCostFunction<fn(&OpType) -> usize, 2> {
    /// Non-increasing rewrite strategy based on CX count.
    ///
    /// A fine-grained cost function given by the total number of quantum gates
    /// is used to rank circuits with equal CX count.
    ///
    /// This is probably a good default for NISQ-y circuit optimisation.
    pub fn default_cx_strategy() -> ExhaustiveGreedyStrategy<Self> {
        Self::cx_count().into_greedy_strategy()
    }

    /// Non-increasing rewrite cost function based on CX gate count.
    ///
    /// A fine-grained cost function given by the total number of quantum gates
    /// is used to rank circuits with equal Rz gate count.
    #[inline]
    pub fn cx_count() -> Self {
        Self {
            cost_fns: [|op| is_cx(op) as usize, |op| is_quantum(op) as usize],
        }
    }

    // TODO: Ideally, do not count Clifford rotations in the cost function.
    /// Non-increasing rewrite cost function based on Rz gate count.
    ///
    /// A fine-grained cost function given by the total number of quantum gates
    /// is used to rank circuits with equal Rz gate count.
    #[inline]
    pub fn rz_count() -> Self {
        Self {
            cost_fns: [
                |op| op_matches(op, TketOp::Rz) as usize,
                |op| is_quantum(op) as usize,
            ],
        }
    }

    /// Consume the cost function and create a greedy rewrite strategy out of
    /// it.
    pub fn into_greedy_strategy(self) -> ExhaustiveGreedyStrategy<Self> {
        ExhaustiveGreedyStrategy { strat_cost: self }
    }

    /// Consume the cost function and create a threshold rewrite strategy out
    /// of it.
    pub fn into_threshold_strategy(self) -> ExhaustiveThresholdStrategy<Self> {
        ExhaustiveThresholdStrategy { strat_cost: self }
    }
}

impl Default for LexicographicCostFunction<fn(&OpType) -> usize, 2> {
    fn default() -> Self {
        LexicographicCostFunction::cx_count()
    }
}

/// Rewrite strategy cost allowing rewrites with bounded cost increase.
///
/// The parameter gamma controls how greedy the algorithm should be. It allows a
/// rewrite C1 -> C2 if C2 has at most gamma times the cost of C1:
///
/// $cost(C2) < gamma * cost(C1)$
///
/// The cost function is given by the sum of the cost of each operation in the
/// circuit. This allows for instance to use of the total number of gates (true
/// predicate), the number of CX gates or a weighted sum of gate types as cost
/// functions.
///
/// gamma = 1 is the greedy strategy where a rewrite is only allowed if it
/// strictly reduces the gate count. The default is gamma = 1.0001 (as set in
/// the Quartz paper) and the number of CX gates. This essentially allows
/// rewrites that improve or leave the number of CX unchanged.
#[derive(Debug, Clone)]
pub struct GammaStrategyCost<C> {
    /// The gamma parameter.
    pub gamma: f64,
    /// A cost function for each operation.
    pub op_cost: C,
}

impl<C: Fn(&OpType) -> usize> StrategyCost for GammaStrategyCost<C> {
    type OpCost = usize;

    #[inline]
    fn under_threshold(&self, &pattern_cost: &Self::OpCost, &target_cost: &Self::OpCost) -> bool {
        (target_cost as f64) < self.gamma * (pattern_cost as f64)
    }

    #[inline]
    fn op_cost(&self, op: &OpType) -> Self::OpCost {
        (self.op_cost)(op)
    }
}

impl<C> GammaStrategyCost<C> {
    /// New exhaustive rewrite strategy with provided predicate.
    ///
    /// The gamma parameter is set to the default 1.0001.
    #[inline]
    pub fn with_cost(op_cost: C) -> ExhaustiveThresholdStrategy<Self> {
        Self {
            gamma: 1.0001,
            op_cost,
        }
        .into()
    }

    /// New exhaustive rewrite strategy with provided gamma and predicate.
    #[inline]
    pub fn new(gamma: f64, op_cost: C) -> ExhaustiveThresholdStrategy<Self> {
        Self { gamma, op_cost }.into()
    }
}

impl GammaStrategyCost<fn(&OpType) -> usize> {
    /// Exhaustive rewrite strategy with CX count cost function.
    ///
    /// The gamma parameter is set to the default 1.0001. This is a good default
    /// choice for NISQ-y circuits, where CX gates are the most expensive.
    #[inline]
    pub fn exhaustive_cx() -> ExhaustiveThresholdStrategy<Self> {
        GammaStrategyCost::with_cost(|op| is_cx(op) as usize)
    }

    /// Exhaustive rewrite strategy with CX count cost function and provided gamma.
    #[inline]
    pub fn exhaustive_cx_with_gamma(gamma: f64) -> ExhaustiveThresholdStrategy<Self> {
        GammaStrategyCost::new(gamma, |op| is_cx(op) as usize)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hugr::hugr::views::SiblingSubgraph;
    use hugr::Node;
    use itertools::Itertools;

    use crate::rewrite::trace::REWRITE_TRACING_ENABLED;
    use crate::{circuit::Circuit, rewrite::CircuitRewrite, utils::build_simple_circuit};

    /// Create a rewrite rule to replace the subcircuit with a new circuit.
    /// TODO: this should use the new Subcircuit; TEMP TEST WORKAROUND until that arrives.
    ///
    /// # Parameters
    /// * `circuit` - The base circuit that contains the subcircuit.
    /// * `replacement` - The new circuit to replace the subcircuit with.
    fn create_rewrite(
        ssg: &SiblingSubgraph<Node>,
        circuit: &Circuit<impl HugrView<Node = Node>>,
        replacement: Circuit<impl HugrView<Node = Node>>,
    ) -> CircuitRewrite {
        // The replacement must be a Dfg rooted hugr.
        let replacement = replacement.extract_dfg().unwrap().into_hugr();
        ssg.create_simple_replacement(circuit.hugr(), replacement)
            .unwrap()
            .into()
    }

    fn n_cx(n_gates: usize) -> Circuit {
        let qbs = [0, 1];
        build_simple_circuit(2, |circ| {
            for _ in 0..n_gates {
                circ.append(TketOp::CX, qbs).unwrap();
            }
            Ok(())
        })
        .unwrap_or_else(|e| panic!("{}", e))
    }

    /// Rewrite cx_nodes -> empty
    fn rw_to_empty(circ: &Circuit, cx_nodes: impl Into<Vec<Node>>) -> CircuitRewrite {
        let subcirc = SiblingSubgraph::try_from_nodes(cx_nodes, circ.hugr()).unwrap();
        create_rewrite(&subcirc, circ, n_cx(0))
    }

    /// Rewrite cx_nodes -> 10x CX
    fn rw_to_full(circ: &Circuit, cx_nodes: impl Into<Vec<Node>>) -> CircuitRewrite {
        let subcirc = SiblingSubgraph::try_from_nodes(cx_nodes, circ.hugr()).unwrap();
        create_rewrite(&subcirc, circ, n_cx(10))
    }

    #[test]
    fn test_greedy_strategy() {
        let mut circ = n_cx(10);
        let cx_gates = circ.commands().map(|cmd| cmd.node()).collect_vec();

        assert!(circ.rewrite_trace().is_none());
        circ.enable_rewrite_tracing();
        match REWRITE_TRACING_ENABLED {
            true => assert_eq!(circ.rewrite_trace().unwrap().collect_vec(), []),
            false => assert!(circ.rewrite_trace().is_none()),
        }

        let rws = [
            rw_to_empty(&circ, cx_gates[0..2].to_vec()),
            rw_to_full(&circ, cx_gates[4..7].to_vec()),
            rw_to_empty(&circ, cx_gates[4..6].to_vec()),
            rw_to_empty(&circ, cx_gates[9..10].to_vec()),
        ];

        let strategy = GreedyRewriteStrategy;
        let rewritten = strategy.apply_rewrites(rws, &circ).collect_vec();
        assert_eq!(rewritten.len(), 1);
        assert_eq!(rewritten[0].circ.num_operations(), 5);

        if REWRITE_TRACING_ENABLED {
            assert_eq!(rewritten[0].circ.rewrite_trace().unwrap().count(), 3);
        }
    }

    #[test]
    fn test_exhaustive_default_strategy() {
        let mut circ = n_cx(10);
        let cx_gates = circ.commands().map(|cmd| cmd.node()).collect_vec();
        circ.enable_rewrite_tracing();

        let rws = [
            rw_to_empty(&circ, cx_gates[0..2].to_vec()),
            rw_to_full(&circ, cx_gates[4..7].to_vec()),
            rw_to_empty(&circ, cx_gates[4..8].to_vec()),
            rw_to_empty(&circ, cx_gates[9..10].to_vec()),
        ];

        let strategy = LexicographicCostFunction::cx_count().into_greedy_strategy();
        let rewritten = strategy.apply_rewrites(rws, &circ).collect_vec();
        let exp_circ_lens = HashSet::from_iter([3, 7, 9]);
        let circ_lens: HashSet<_> = rewritten.iter().map(|r| r.circ.num_operations()).collect();
        assert_eq!(circ_lens, exp_circ_lens);

        if REWRITE_TRACING_ENABLED {
            // Each strategy branch applies a single rewrite, composed of
            // multiple individual elements from `rws`.
            assert_eq!(
                rewritten[0].circ.rewrite_trace().unwrap().collect_vec(),
                vec![RewriteTrace::new(3)]
            );
            assert_eq!(
                rewritten[1].circ.rewrite_trace().unwrap().collect_vec(),
                vec![RewriteTrace::new(2)]
            );
            assert_eq!(
                rewritten[2].circ.rewrite_trace().unwrap().collect_vec(),
                vec![RewriteTrace::new(1)]
            );
        }
    }

    #[test]
    fn test_exhaustive_gamma_strategy() {
        let circ = n_cx(10);
        let cx_gates = circ.commands().map(|cmd| cmd.node()).collect_vec();

        let rws = [
            rw_to_empty(&circ, cx_gates[0..2].to_vec()),
            rw_to_full(&circ, cx_gates[4..7].to_vec()),
            rw_to_empty(&circ, cx_gates[4..8].to_vec()),
            rw_to_empty(&circ, cx_gates[9..10].to_vec()),
        ];

        let strategy = GammaStrategyCost::exhaustive_cx_with_gamma(10.);
        let rewritten = strategy.apply_rewrites(rws, &circ);
        let exp_circ_lens = HashSet::from_iter([8, 17, 6, 9]);
        let circ_lens: HashSet<_> = rewritten.map(|r| r.circ.num_operations()).collect();
        assert_eq!(circ_lens, exp_circ_lens);
    }

    #[test]
    fn test_exhaustive_default_cx_cost() {
        let strat = LexicographicCostFunction::cx_count().into_greedy_strategy();
        let circ = n_cx(3);
        assert_eq!(strat.circuit_cost(&circ), (3, 3).into());
        let circ = build_simple_circuit(2, |circ| {
            circ.append(TketOp::CX, [0, 1])?;
            circ.append(TketOp::X, [0])?;
            circ.append(TketOp::X, [1])?;
            Ok(())
        })
        .unwrap();
        assert_eq!(strat.circuit_cost(&circ), (1, 3).into());
    }

    #[test]
    fn test_exhaustive_default_cx_threshold() {
        let strat = LexicographicCostFunction::cx_count();
        assert!(strat.under_threshold(&(3, 0).into(), &(3, 0).into()));
        assert!(strat.under_threshold(&(3, 0).into(), &(3, 5).into()));
        assert!(!strat.under_threshold(&(3, 10).into(), &(4, 0).into()));
        assert!(strat.under_threshold(&(3, 0).into(), &(1, 5).into()));
    }
}
