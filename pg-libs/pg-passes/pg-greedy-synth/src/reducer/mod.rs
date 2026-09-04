//! Windowed greedy reduction of canonical Pauli graph operation sets.

mod gate_pool;

pub(crate) use gate_pool::{GatePool, select_candidate};

use crate::backend::TQECostBackend;
use crate::frontier::Frontier;
use crate::packed_pg_slice::{PackedBackend, PackedOpMeta, PackedPGSlice};
use crate::synthesis::{ParallelMode, synthesise_tableau};
use crate::synthesis::{progress_bar, tqe_op, update_depth};
use pg_canonical_form::CanonicalFormPass;
use pg_core::{GateData, GateType, Op, PGPass, PauliGraph};
use pg_optimise::GroupCommutingOpsPass;
use pg_qm_tableau::Tableau;
use rand::{SeedableRng, rngs::StdRng};
use rayon::prelude::*;

pub(crate) struct Reducer<P, C> {
    packed_backend: P,
    cost_backend: C,
    window_size: usize,
    pool_size: usize,
    top_up_size: usize,
    seed: u64,
    parallel_mode: ParallelMode,
    enable_progress: bool,
}

impl<P, C> Reducer<P, C>
where
    P: PackedBackend,
    C: TQECostBackend + Sync,
{
    pub(crate) fn new(
        packed_backend: P,
        cost_backend: C,
        window_size: usize,
        pool_size: usize,
        top_up_size: usize,
        seed: u64,
        parallel_mode: ParallelMode,
        enable_progress: bool,
    ) -> Self {
        assert!(window_size > 0, "window size must be positive");
        Self {
            packed_backend,
            cost_backend,
            window_size,
            pool_size,
            top_up_size,
            seed,
            parallel_mode,
            enable_progress,
        }
    }

    /// Reduces a canonical graph one complete commuting set at a time.
    ///
    /// This fills a [`PackedPGSlice`], rebuilds a [`Frontier`] for its
    /// first visible set, emits operations with zero cost and repeatedly
    /// applies a sampled TQE from the cheapest positive cost bucket. Once all
    /// sets are retired, it synthesises the remaining Clifford tableau with the
    /// same frontier and candidate pool machinery.
    pub(crate) fn reduce(&mut self, input: &PauliGraph) -> PauliGraph {
        let n_qubits = input.get_n_qubits();
        let input_ops = input.get_ops();
        if input_ops.is_empty() {
            return PauliGraph::new(n_qubits);
        }
        assert!(
            input_ops.iter().any(|op| matches!(op, Op::SetBoundary)),
            "input PauliGraph must contain a SetBoundary"
        );

        let mut slice = PackedPGSlice::new(n_qubits);
        let mut frontier = Frontier::new(n_qubits);
        let mut pool = GatePool::new(n_qubits);
        let mut output = PauliGraph::new(n_qubits);
        let mut qubit_depth = vec![0; n_qubits];
        let mut rng = StdRng::seed_from_u64(self.seed);
        let mut next_input = 0;
        let progress = progress_bar(
            self.enable_progress,
            input_ops
                .iter()
                .filter(|op| matches!(op, Op::SetBoundary))
                .count()
                .saturating_sub(1),
        );

        loop {
            while slice.len() < self.window_size && next_input < input_ops.len() {
                self.load_set(&mut slice, input_ops, &mut next_input);
            }
            let input_exhausted = next_input == input_ops.len();
            while !slice.is_empty() && (slice.len() >= self.window_size || input_exhausted) {
                self.reduce_front_set(
                    &mut slice,
                    &mut frontier,
                    &mut pool,
                    &mut output,
                    &mut qubit_depth,
                    &mut rng,
                );
                if let Some(progress) = &progress {
                    progress.inc(1);
                }
            }
            if input_exhausted {
                break;
            }
        }

        if let Some(progress) = progress {
            progress.finish();
        }
        let trailing = slice.into_trailing_tableau();
        let trailing_output = synthesise_tableau(
            trailing,
            &self.packed_backend,
            &self.cost_backend,
            self.pool_size,
            self.top_up_size,
            &mut qubit_depth,
            self.seed,
            self.parallel_mode,
            self.enable_progress,
        );
        output.extend(trailing_output);
        output
    }

    /// Loads one complete commuting set into the packed lookahead slice.
    fn load_set(&self, slice: &mut PackedPGSlice, ops: &[Op], next: &mut usize) {
        let mut saw_operation = false;
        while *next < ops.len() {
            let op = &ops[*next];
            *next += 1;
            match op {
                Op::SetBoundary => {
                    slice.start_new_set();
                    if saw_operation {
                        break;
                    }
                }
                Op::Rotation { .. }
                | Op::Measure { .. }
                | Op::Reset { .. }
                | Op::BlackBox { .. }
                | Op::ConditionalBox { .. }
                | Op::Tableau { .. } => {
                    saw_operation = true;
                    slice.push_op(op, &self.packed_backend);
                }
                _ => panic!("unsupported operation in input PauliGraph"),
            }
        }
    }

    /// Synthesises all operations in the current front set.
    ///
    /// Black boxes and conditional boxes are standalone sets. A black box first
    /// emits the Clifford tableau captured immediately before it. A conditional
    /// box is decoded, canonicalised and reduced recursively, then its condition
    /// is copied to every emitted gate.
    fn reduce_front_set(
        &mut self,
        slice: &mut PackedPGSlice,
        frontier: &mut Frontier,
        pool: &mut GatePool,
        output: &mut PauliGraph,
        qubit_depth: &mut [u64],
        rng: &mut StdRng,
    ) {
        enum FrontSetKind {
            BlackBox(pg_core::BlackBoxData, Tableau),
            Conditional(pg_core::ConditionalBoxData),
            Ordinary,
        }

        let set_kind = {
            let mut front = slice.op_sets().next().expect("visible slice has no set");
            let Some((index, op)) = front.next() else {
                panic!("visible operation set is empty")
            };
            // A set with multiple operations cannot be a black box or conditional box.
            if front.next().is_some() {
                FrontSetKind::Ordinary
            } else {
                match op.meta {
                    PackedOpMeta::BlackBox(black_box) => FrontSetKind::BlackBox(
                        black_box.data().clone(),
                        black_box.tableau().clone(),
                    ),
                    PackedOpMeta::ConditionalBox(_) => {
                        FrontSetKind::Conditional(slice.decode_conditional_box(index))
                    }
                    _ => FrontSetKind::Ordinary,
                }
            }
        };

        match set_kind {
            FrontSetKind::BlackBox(data, tableau) => {
                let mut nested_depth = vec![0; qubit_depth.len()];
                output.extend(synthesise_tableau(
                    tableau,
                    &self.packed_backend,
                    &self.cost_backend,
                    self.pool_size,
                    self.top_up_size,
                    &mut nested_depth,
                    self.seed,
                    self.parallel_mode,
                    self.enable_progress,
                ));
                output.add_op(Op::Gate {
                    data: GateData::new(GateType::BlackBox, data.get_qubits().clone())
                        .with_data(data.get_content().clone()),
                });
            }
            FrontSetKind::Conditional(data) => {
                let inner = PauliGraph::new(qubit_depth.len()).with_ops(data.get_ops().clone());
                let canonical = CanonicalFormPass::new().transform(&inner);
                let grouped = GroupCommutingOpsPass::new().transform(&canonical);
                let reduced = self.reduce(&grouped);
                for op in reduced.get_ops() {
                    let Op::Gate { data: gate } = op else {
                        panic!("conditional synthesis emitted an operation that is not a gate")
                    };
                    output.add_op(Op::Gate {
                        data: gate.clone().with_conditional(
                            data.get_conditional_bits().clone(),
                            data.get_conditional_values().clone(),
                        ),
                    });
                }
            }
            FrontSetKind::Ordinary => {
                self.reduce_ordinary_set(slice, frontier, pool, output, qubit_depth, rng)
            }
        }
        slice.progress_set();
    }

    /// Reduces an ordinary set until every operation reaches zero cost.
    ///
    /// The frontier and cost strategy are rebuilt because their prepared state
    /// is tied to the current visible front of the packed slice. A full candidate
    /// pool is sampled after emissions, while iterations without emissions only
    /// top up the existing pool.
    fn reduce_ordinary_set(
        &mut self,
        slice: &mut PackedPGSlice,
        frontier: &mut Frontier,
        pool: &mut GatePool,
        output: &mut PauliGraph,
        qubit_depth: &mut [u64],
        rng: &mut StdRng,
    ) {
        assert!(frontier.is_empty());
        frontier.rebuild(slice);
        self.cost_backend.update(slice);
        pool.clear();
        debug_assert!(pool.is_empty());

        loop {
            let emissions = frontier.pop_ops();
            let emitted_gate_count = emissions.iter().map(|item| item.ops.len()).sum::<usize>();
            for emission in emissions {
                output.extend(PauliGraph::new(qubit_depth.len()).with_ops(emission.ops));
                slice.clear_op_mask(emission.slice_index);
            }
            if frontier.is_empty() {
                break;
            }
            let sample_count = if emitted_gate_count > 0 {
                self.pool_size
            } else {
                self.top_up_size
            };
            self.sample_packed_candidates(slice, frontier, pool, sample_count, rng);
            let selected = select_candidate(pool, frontier, qubit_depth);
            let tqe = selected.gate.tqe;
            let stop = selected.stop.expect("packed candidate has no stop point");
            update_depth(qubit_depth, tqe);
            output.add_op(tqe_op(tqe));
            slice.apply_tqe(tqe, stop, &self.packed_backend);
            frontier.apply_tqe(tqe);
            pool.invalidate(tqe);
        }
    }

    /// Samples gates from the cheapest frontier bucket and scores their visible
    /// packed ranges, using Rayon when the configured workload mode allows it.
    fn sample_packed_candidates(
        &self,
        slice: &PackedPGSlice,
        frontier: &Frontier,
        pool: &GatePool,
        count: usize,
        rng: &mut StdRng,
    ) {
        let gates = frontier.sample_gates(count, rng);
        let cost_backend = &self.cost_backend;
        let insert = |gate: &crate::frontier::FrontierReductionGate| {
            let stop = slice.stop_point(gate.tqe);
            let cost = cost_backend.tqe_cost(slice, gate.tqe, stop);
            pool.insert(*gate, cost, Some(stop));
        };
        if self
            .parallel_mode
            .should_use_parallel(slice.len(), gates.len())
        {
            gates.par_iter().for_each(insert);
        } else {
            gates.iter().for_each(insert);
        }
    }
}
