//! Concurrent storage and selection of sampled TQE candidates based on depth.

use crate::frontier::{Frontier, FrontierReductionGate};
use crate::packed_pg_slice::SliceIndex;
use crate::tqe::TQE;
use crossbeam_skiplist::SkipSet;
use std::cmp::Ordering;

const DEPTH_WEIGHT: f64 = 0.3;
const MAX_SELECTION_CANDIDATES: usize = 5;

#[derive(Debug, Clone)]
pub(crate) struct GateCandidate {
    pub(crate) gate: FrontierReductionGate,
    q0_version: usize,
    q1_version: usize,
    cost: f64,
    pub(crate) stop: Option<SliceIndex>,
}

impl PartialEq for GateCandidate {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for GateCandidate {}

impl PartialOrd for GateCandidate {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for GateCandidate {
    /// Orders by cost, then by the complete candidate identity.
    ///
    /// The identity includes the TQE, frontier witness, qubit versions and stop
    /// point. Exact repeated samples collapse in the skip set, while refreshed
    /// candidates and separate witnesses remain distinct until stale entries
    /// are removed lazily.
    fn cmp(&self, other: &Self) -> Ordering {
        self.cost
            .partial_cmp(&other.cost)
            .expect("candidate costs must be finite")
            .then_with(|| self.gate.tqe.q0.cmp(&other.gate.tqe.q0))
            .then_with(|| self.gate.tqe.q1.cmp(&other.gate.tqe.q1))
            .then_with(|| self.gate.tqe.gate_type.cmp(&other.gate.tqe.gate_type))
            .then_with(|| self.gate.frontier_index.cmp(&other.gate.frontier_index))
            .then_with(|| self.q0_version.cmp(&other.q0_version))
            .then_with(|| self.q1_version.cmp(&other.q1_version))
            .then_with(|| self.stop.cmp(&other.stop))
    }
}

pub(crate) struct GatePool {
    candidates: SkipSet<GateCandidate>,
    qubit_versions: Vec<usize>,
}

impl GatePool {
    pub(crate) fn new(n_qubits: usize) -> Self {
        Self {
            candidates: SkipSet::new(),
            qubit_versions: vec![0; n_qubits],
        }
    }

    pub(crate) fn clear(&mut self) {
        self.candidates.clear();
        self.qubit_versions.fill(0);
    }

    pub(crate) fn is_empty(&self) -> bool {
        self.candidates.is_empty()
    }

    /// Inserts a scored candidate into the concurrent skip set.
    pub(crate) fn insert(&self, gate: FrontierReductionGate, cost: f64, stop: Option<SliceIndex>) {
        assert!(cost.is_finite(), "candidate costs must be finite");
        let tqe = gate.tqe;
        self.candidates.insert(GateCandidate {
            gate,
            q0_version: self.qubit_versions[tqe.q0],
            q1_version: self.qubit_versions[tqe.q1],
            cost,
            stop,
        });
    }

    /// Invalidates every candidate touching the selected TQE qubits by
    /// advancing the version for each qubit without scanning the pool.
    pub(crate) fn invalidate(&mut self, tqe: TQE) {
        self.qubit_versions[tqe.q0] += 1;
        self.qubit_versions[tqe.q1] += 1;
    }

    /// Checks both endpoint versions and the candidate's frontier cost bucket.
    fn is_current(&self, candidate: &GateCandidate, frontier: &Frontier) -> bool {
        let tqe = candidate.gate.tqe;
        candidate.q0_version == self.qubit_versions[tqe.q0]
            && candidate.q1_version == self.qubit_versions[tqe.q1]
            && frontier.is_least_cost(candidate.gate.frontier_index)
    }
}

/// Selects the best candidate using gate cost and projected TQE depth.
///
/// Selection examines at most five current entries with the lowest costs. It
/// normalises cost and depth, then gives depth a weight of 0.3. These values are
/// inherited tuning heuristics and should only change with representative
/// benchmarks and an output quality comparison. Unselected candidates are
/// returned to the pool only when they are disjoint from the chosen gate.
pub(crate) fn select_candidate(
    pool: &GatePool,
    frontier: &Frontier,
    qubit_depth: &[u64],
) -> GateCandidate {
    // Limit the secondary depth comparison to the cheapest few gates. This
    // keeps selection bounded while still allowing a slightly more expensive
    // candidate to reduce the projected TQE depth.
    let mut candidates = Vec::with_capacity(MAX_SELECTION_CANDIDATES);
    while candidates.len() < MAX_SELECTION_CANDIDATES {
        let Some(entry) = pool.candidates.pop_front() else {
            break;
        };
        let candidate = entry.value().clone();
        if pool.is_current(&candidate, frontier) {
            candidates.push(candidate);
        }
    }
    assert!(!candidates.is_empty(), "no valid candidates found");

    let depths: Vec<_> = candidates
        .iter()
        .map(|candidate| {
            let tqe = candidate.gate.tqe;
            qubit_depth[tqe.q0].max(qubit_depth[tqe.q1]) + 1
        })
        .collect();

    // Normalisation based on the minimum and maximum gate costs and depths.
    let mut minimum_depth = u64::MAX;
    let mut maximum_depth = 0;
    let mut minimum_depth_index = 0;
    let mut minimum_gate_cost = f64::INFINITY;
    let mut maximum_gate_cost = f64::NEG_INFINITY;
    let mut minimum_gate_cost_index = 0;

    for (index, candidate) in candidates.iter().enumerate() {
        if candidate.cost < minimum_gate_cost {
            minimum_gate_cost = candidate.cost;
            minimum_gate_cost_index = index;
        }
        maximum_gate_cost = maximum_gate_cost.max(candidate.cost);
    }
    for (index, depth) in depths.iter().copied().enumerate() {
        if depth <= minimum_depth {
            minimum_depth = depth;
            minimum_depth_index = index;
        }
        maximum_depth = maximum_depth.max(depth);
    }

    let gate_cost_range = maximum_gate_cost - minimum_gate_cost;
    let depth_range = maximum_depth - minimum_depth;
    let best_index = if gate_cost_range.abs() <= 1e-8 {
        minimum_depth_index
    } else if depth_range == 0 {
        minimum_gate_cost_index
    } else {
        candidates
            .iter()
            .zip(&depths)
            .enumerate()
            // Selection is based on the weighted sum of normalised gate cost and depth.
            .map(|(index, (candidate, depth))| {
                let normalised_gate = (candidate.cost - minimum_gate_cost) / gate_cost_range;
                let normalised_depth = (*depth - minimum_depth) as f64 / depth_range as f64;
                (index, normalised_gate + DEPTH_WEIGHT * normalised_depth)
            })
            .min_by(|(_, left), (_, right)| left.partial_cmp(right).unwrap())
            .unwrap()
            .0
    };

    let selected = candidates[best_index].clone();
    let selected_tqe = selected.gate.tqe;
    for (index, candidate) in candidates.into_iter().enumerate() {
        let tqe = candidate.gate.tqe;
        if index != best_index
            && tqe.q0 != selected_tqe.q0
            && tqe.q0 != selected_tqe.q1
            && tqe.q1 != selected_tqe.q0
            && tqe.q1 != selected_tqe.q1
        {
            pool.candidates.insert(candidate);
        }
    }
    selected
}

#[cfg(test)]
#[path = "gate_pool/tests.rs"]
mod tests;
