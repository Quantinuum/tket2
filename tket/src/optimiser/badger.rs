//! Badger circuit optimiser.
//!
//! This module implements the Badger circuit optimiser. It relies on a rewriter
//! and a RewriteStrategy instance to repeatedly rewrite a circuit and optimising
//! it according to some cost metric (typically gate count).
//!
//! The optimiser is implemented as a priority queue of circuits to be processed.
//! On top of the queue are the circuits with the lowest cost. They are popped
//! from the queue and replaced by the new circuits obtained from the rewriter
//! and the rewrite strategy. A hash of every circuit computed is stored to
//! detect and ignore duplicates. The priority queue is truncated whenever
//! it gets too large.

mod eq_circ_class;
pub mod log;
mod qtz_circuit;
mod worker;

use crossbeam_channel::select;
pub use eq_circ_class::{EqCircClass, load_eccs_json_file};
use hugr::hugr::HugrError;
use hugr::{HugrView, Node};
pub use log::BadgerLogger;
use rayon::iter::{IndexedParallelIterator, IntoParallelRefMutIterator, ParallelIterator};

use std::num::NonZeroUsize;
use std::time::{Duration, Instant};
use std::{mem, thread};

use crate::Circuit;
use crate::circuit::CircuitHash;
use crate::circuit::cost::CircuitCost;
use crate::optimiser::badger::worker::BadgerWorker;
use crate::optimiser::{BacktrackingOptimiser, Optimiser, State, StatePQWorker, pqueue_worker};
use crate::passes::CircuitChunks;
use crate::rewrite::Rewriter;
use crate::rewrite::strategy::{RewriteResult, RewriteStrategy};

/// Configuration options for the Badger optimiser.
#[derive(Copy, Clone, Debug)]
pub struct BadgerOptions {
    /// The maximum time (in seconds) to run the optimiser.
    ///
    /// Defaults to `None`, which means no timeout.
    pub timeout: Option<u64>,
    /// The maximum time (in seconds) to search for new improvements to the
    /// circuit. If no progress is made in this time, the optimiser will stop.
    ///
    /// Defaults to `None`, which means no timeout.
    pub progress_timeout: Option<u64>,
    /// The maximum number of circuits to process before stopping the optimisation.
    ///
    /// For data parallel multi-threading, (split_circuit=true), applies on a
    /// per-thread basis, otherwise applies globally.
    ///
    /// Defaults to `None`, which means no limit.
    pub max_circuit_count: Option<usize>,
    /// The number of threads to use.
    ///
    /// Defaults to `1`.
    pub n_threads: NonZeroUsize,
    /// Whether to split the circuit into chunks and process each in a separate thread.
    ///
    /// If this option is set to `true`, the optimiser will split the circuit into `n_threads`
    /// chunks.
    ///
    /// If this option is set to `false`, the optimiser will run parallel searches on the whole
    /// circuit.
    ///
    /// Defaults to `false`.
    pub split_circuit: bool,
    /// The maximum size of the circuit candidates priority queue.
    ///
    /// Defaults to `20`.
    pub queue_size: usize,
}

impl Default for BadgerOptions {
    fn default() -> Self {
        Self {
            timeout: Default::default(),
            progress_timeout: Default::default(),
            n_threads: NonZeroUsize::new(1).unwrap(),
            split_circuit: Default::default(),
            queue_size: 20,
            max_circuit_count: None,
        }
    }
}

/// The Badger optimiser.
///
/// Adapted from [Quartz][], and originally [TASO][].
///
/// Using a rewriter and a rewrite strategy, the optimiser
/// will repeatedly rewrite the circuit, optimising the circuit according to
/// the cost function provided.
///
/// Optimisation is done by maintaining a priority queue of circuits and
/// always processing the circuit with the lowest cost first. Rewrites are
/// computed for that circuit and all new circuit obtained are added to the queue.
///
/// There are a single-threaded and two multi-threaded versions of the optimiser,
/// controlled by setting the [`BadgerOptions::n_threads`] and
/// [`BadgerOptions::split_circuit`] fields.
///
/// [Quartz]: https://arxiv.org/abs/2204.09033
/// [TASO]: https://dl.acm.org/doi/10.1145/3341301.3359630
#[derive(Clone, Debug)]
pub struct BadgerOptimiser<R, S> {
    rewriter: R,
    strategy: S,
}

impl<R, S> BadgerOptimiser<R, S> {
    /// Create a new Badger optimiser.
    pub fn new(rewriter: R, strategy: S) -> Self {
        Self { rewriter, strategy }
    }

    fn cost(&self, circ: &Circuit<impl HugrView<Node = Node>>) -> S::Cost
    where
        S: RewriteStrategy,
    {
        self.strategy.circuit_cost(circ)
    }
}

/// A state in the Badger search space.
#[derive(Clone, Debug)]
struct BadgerState<C> {
    /// The current circuit
    circ: Circuit,
    /// The circuit cost
    cost: C,
}

impl<R: Rewriter, S: RewriteStrategy> State<&BadgerOptimiser<R, S>> for BadgerState<S::Cost>
where
    S::Cost: serde::Serialize,
{
    type Cost = S::Cost;

    fn hash(&self, _context: &&BadgerOptimiser<R, S>) -> Option<u64> {
        self.circ.circuit_hash(self.circ.parent()).ok()
    }

    fn cost(&self, _context: &&BadgerOptimiser<R, S>) -> Option<Self::Cost> {
        Some(self.cost.clone())
    }

    fn next_states(&self, &mut context: &mut &BadgerOptimiser<R, S>) -> Vec<Self> {
        let rewrites = context.rewriter.get_rewrites(&self.circ);
        context
            .strategy
            .apply_rewrites(rewrites, &self.circ)
            .map(|RewriteResult { circ, cost_delta }| BadgerState {
                circ,
                cost: self.cost.add_delta(&cost_delta),
            })
            .collect()
    }
}

impl<R, S> BadgerOptimiser<R, S>
where
    R: Rewriter + Send + Clone + Sync + 'static,
    S: RewriteStrategy + Send + Sync + Clone + 'static,
    S::Cost: serde::Serialize + Send + Sync,
{
    /// Run the Badger optimiser on a circuit.
    ///
    /// A timeout (in seconds) can be provided.
    pub fn optimise(
        &self,
        circ: &Circuit<impl HugrView<Node = Node>>,
        options: BadgerOptions,
    ) -> Circuit {
        self.optimise_with_log(circ, Default::default(), options)
    }

    /// Run the Badger optimiser on a circuit with logging activated.
    ///
    /// A timeout (in seconds) can be provided.
    pub fn optimise_with_log(
        &self,
        circ: &Circuit<impl HugrView<Node = Node>>,
        log_config: BadgerLogger,
        options: BadgerOptions,
    ) -> Circuit {
        match options.n_threads.get() {
            1 => self.badger(circ, log_config, options),
            _ => {
                if options.split_circuit {
                    self.badger_split_multithreaded(circ, log_config, options)
                        .unwrap()
                } else {
                    self.badger_multithreaded(circ, log_config, options)
                }
            }
        }
    }

    /// Run the Badger optimiser on a circuit, using a single thread.
    #[tracing::instrument(target = "badger::metrics", skip(self, circ, logger))]
    fn badger(
        &self,
        circ: &Circuit<impl HugrView<Node = Node>>,
        logger: BadgerLogger,
        opt: BadgerOptions,
    ) -> Circuit {
        let backtracking = BacktrackingOptimiser::with_badger_options(&opt);
        let init_state = BadgerState {
            circ: circ.to_owned(),
            cost: self.cost(circ),
        };
        backtracking
            .optimise_with_options(init_state, self, logger.into())
            .expect("optimisation failed")
            .best_state
            .circ
    }

    /// Run the Badger optimiser on a circuit, using multiple threads.
    ///
    /// This is the multi-threaded version of [`Self::badger`], using a single
    /// priority queue and multiple workers to process the circuits in parallel.
    #[tracing::instrument(target = "badger::metrics", skip(self, circ, logger))]
    fn badger_multithreaded(
        &self,
        circ: &Circuit<impl HugrView<Node = Node>>,
        mut logger: BadgerLogger,
        opt: BadgerOptions,
    ) -> Circuit {
        let start_time = Instant::now();
        let n_threads: usize = opt.n_threads.get();
        let circ = circ.to_owned();

        // multi-consumer priority channel for queuing circuits to be processed by the
        // workers
        let (pq, rx_log) = StatePQWorker::init(opt.queue_size);

        let initial_circ_hash = circ.circuit_hash(circ.parent()).unwrap();
        let mut best_circ = circ.clone();
        let mut best_circ_cost = self.cost(&best_circ);

        // Initialise the work channels and send the initial circuit.
        pq.send(vec![pqueue_worker::Work {
            cost: best_circ_cost.clone(),
            hash: initial_circ_hash,
            state: circ,
        }])
        .unwrap();

        // Each worker waits for circuits to scan for rewrites using all the
        // patterns and sends the results back to main.
        let joins: Vec<_> = (0..n_threads)
            .map(|i| {
                BadgerWorker::spawn(i, pq.clone(), self.rewriter.clone(), self.strategy.clone())
            })
            .collect();

        // Deadline for the optimisation timeout
        let timeout_event = match opt.timeout {
            None => crossbeam_channel::never(),
            Some(t) => crossbeam_channel::at(Instant::now() + Duration::from_secs(t)),
        };

        // Deadline for the timeout when no progress is made
        let mut progress_timeout_event = match opt.progress_timeout {
            None => crossbeam_channel::never(),
            Some(t) => crossbeam_channel::at(Instant::now() + Duration::from_secs(t)),
        };

        // Main loop: log best circuits as they come in from the priority queue,
        // until the timeout is reached.
        let mut timeout_flag = false;
        let mut processed_count = 0;
        let mut seen_count = 0;
        loop {
            select! {
                recv(rx_log) -> msg => {
                    match msg {
                        Ok(pqueue_worker::LogMessage::NewBestState(circ, cost)) => {
                            if cost < best_circ_cost {
                                best_circ = circ;
                                best_circ_cost = cost;
                                let num_rewrites = best_circ.rewrite_trace().map(|rs| rs.count());
                                logger.log_best(&best_circ_cost, num_rewrites);
                                if let Some(t) = opt.progress_timeout {
                                    progress_timeout_event = crossbeam_channel::at(Instant::now() + Duration::from_secs(t));
                                }
                            }
                        },
                        Ok(pqueue_worker::LogMessage::StateCount{processed_count: proc, seen_count: seen, queue_length}) => {
                            processed_count = proc;
                            seen_count = seen;
                            if let Some(max_circuit_count) = opt.max_circuit_count
                                && seen_count > max_circuit_count {
                                    timeout_flag = true;
                                    // Signal the workers to stop.
                                    let _ = pq.close();
                                    break;
                                }
                            logger.log_progress(processed_count, Some(queue_length), seen_count);
                        }
                        Err(crossbeam_channel::RecvError) => {
                            logger.log("The priority channel panicked. Stopping Badger optimisation.");
                            let _ = pq.close();
                            break;
                        }
                    }
                }
                recv(timeout_event) -> _ => {
                    timeout_flag = true;
                    // Signal the workers to stop.
                    let _ = pq.close();
                    break;
                }
                recv(progress_timeout_event) -> _ => {
                    timeout_flag = true;
                    // Signal the workers to stop.
                    let _ = pq.close();
                    break;
                }
            }
        }

        // Empty the log from the priority queue and store final circuit count.
        while let Ok(log) = rx_log.recv() {
            match log {
                pqueue_worker::LogMessage::NewBestState(circ, cost) => {
                    if cost < best_circ_cost {
                        best_circ = circ;
                        best_circ_cost = cost;
                        let num_rewrites = best_circ.rewrite_trace().map(|rs| rs.count());
                        logger.log_best(&best_circ_cost, num_rewrites);
                    }
                }
                pqueue_worker::LogMessage::StateCount {
                    processed_count: proc,
                    seen_count: seen,
                    queue_length,
                } => {
                    processed_count = proc;
                    seen_count = seen;
                    logger.log_progress(processed_count, Some(queue_length), seen_count);
                }
            }
        }
        logger.log_processing_end(
            processed_count,
            Some(seen_count),
            best_circ_cost,
            true,
            timeout_flag,
            start_time.elapsed(),
        );

        joins.into_iter().for_each(|j| j.join().unwrap());

        best_circ
    }

    /// Run the Badger optimiser on a circuit, with data parallel multithreading.
    ///
    /// Split the circuit into chunks and process each in a separate thread.
    #[tracing::instrument(target = "badger::metrics", skip(self, circ, logger))]
    fn badger_split_multithreaded(
        &self,
        circ: &Circuit<impl HugrView<Node = Node>>,
        mut logger: BadgerLogger,
        opt: BadgerOptions,
    ) -> Result<Circuit, HugrError> {
        let start_time = Instant::now();
        let circ = circ.to_owned();
        let circ_cost = self.cost(&circ);
        let max_chunk_cost = circ_cost.clone().div_cost(opt.n_threads);
        logger.log(format!(
            "Splitting circuit with cost {:?} into chunks of at most {max_chunk_cost:?}.",
            circ_cost.clone()
        ));
        let mut chunks =
            CircuitChunks::split_with_cost(&circ, max_chunk_cost, |op| self.strategy.op_cost(op));

        let num_rewrites = circ.rewrite_trace().map(|rs| rs.count());
        logger.log_best(circ_cost.clone(), num_rewrites);

        let (joins, rx_work): (Vec<_>, Vec<_>) = chunks
            .par_iter_mut()
            .enumerate()
            .map(|(i, chunk)| {
                let (tx, rx) = crossbeam_channel::unbounded();
                let badger = self.clone();
                let chunk = mem::take(chunk);
                let chunk_cx_cost = chunk.circuit_cost(|op| self.strategy.op_cost(op));
                logger.log(format!("Chunk {i} has {chunk_cx_cost:?} CX gates",));
                let join = thread::Builder::new()
                    .name(format!("chunk-{i}"))
                    .spawn(move || {
                        let res = badger.optimise(
                            &chunk,
                            BadgerOptions {
                                n_threads: NonZeroUsize::new(1).unwrap(),
                                split_circuit: false,
                                ..opt
                            },
                        );
                        tx.send(res).unwrap();
                    })
                    .unwrap();
                (join, rx)
            })
            .unzip();

        for i in 0..chunks.len() {
            let res = rx_work[i]
                .recv()
                .unwrap_or_else(|_| panic!("Worker thread panicked"));
            chunks[i] = res;
        }

        let best_circ = chunks.reassemble()?;
        let best_circ_cost = self.cost(&best_circ);
        if best_circ_cost.clone() < circ_cost {
            let num_rewrites = best_circ.rewrite_trace().map(|rs| rs.count());
            logger.log_best(best_circ_cost.clone(), num_rewrites);
        }

        logger.log_processing_end(
            opt.n_threads.get(),
            None,
            best_circ_cost,
            true,
            false,
            start_time.elapsed(),
        );
        joins.into_iter().for_each(|j| j.join().unwrap());

        Ok(best_circ)
    }
}

#[cfg(feature = "portmatching")]
mod badger_default {
    use std::io;
    use std::path::Path;

    use hugr::ops::OpType;

    use crate::rewrite::ECCRewriter;
    use crate::rewrite::ecc_rewriter::RewriterSerialisationError;
    use crate::rewrite::strategy::{ExhaustiveGreedyStrategy, LexicographicCostFunction};

    use super::*;

    /// The default optimisation strategy for the Badger optimiser.
    pub type DefaultBadgerStrategy = ExhaustiveGreedyStrategy<StrategyCost>;
    pub type StrategyCost = LexicographicCostFunction<fn(&OpType) -> usize, 2>;

    /// The Badger optimiser using ECC sets.
    pub type ECCBadgerOptimiser = BadgerOptimiser<ECCRewriter, DefaultBadgerStrategy>;

    impl Default for DefaultBadgerStrategy {
        fn default() -> Self {
            Self::cx_count()
        }
    }

    impl DefaultBadgerStrategy {
        /// A strategy minimising CX gate count.
        pub fn cx_count() -> Self {
            LexicographicCostFunction::default_cx_strategy()
        }

        /// A strategy minimising Rz gate count.
        pub fn rz_count() -> Self {
            LexicographicCostFunction::rz_count().into_greedy_strategy()
        }
    }

    impl ECCBadgerOptimiser {
        /// A sane default optimiser using the given ECC sets.
        pub fn default_with_eccs_json_file(eccs_path: impl AsRef<Path>) -> io::Result<Self> {
            let rewriter = ECCRewriter::try_from_eccs_json_file(eccs_path)?;
            let strategy = DefaultBadgerStrategy::default();
            Ok(BadgerOptimiser::new(rewriter, strategy))
        }

        /// A sane default optimiser using a precompiled binary rewriter.
        #[cfg(feature = "binary-eccs")]
        pub fn default_with_rewriter_binary(
            rewriter_path: impl AsRef<Path>,
        ) -> Result<Self, RewriterSerialisationError> {
            let rewriter = ECCRewriter::load_binary(rewriter_path)?;
            let strategy = DefaultBadgerStrategy::default();
            Ok(BadgerOptimiser::new(rewriter, strategy))
        }

        /// An optimiser minimising Rz gate count using the given ECC sets.
        pub fn rz_opt_with_eccs_json_file(eccs_path: impl AsRef<Path>) -> io::Result<Self> {
            let rewriter = ECCRewriter::try_from_eccs_json_file(eccs_path)?;
            let strategy = LexicographicCostFunction::rz_count().into_greedy_strategy();
            Ok(BadgerOptimiser::new(rewriter, strategy))
        }

        /// An optimiser minimising Rz gate count using a precompiled binary rewriter.
        #[cfg(feature = "binary-eccs")]
        pub fn rz_opt_with_rewriter_binary(
            rewriter_path: impl AsRef<Path>,
        ) -> Result<Self, RewriterSerialisationError> {
            let rewriter = ECCRewriter::load_binary(rewriter_path)?;
            let strategy = LexicographicCostFunction::rz_count().into_greedy_strategy();
            Ok(BadgerOptimiser::new(rewriter, strategy))
        }
    }
}
#[cfg(feature = "portmatching")]
pub use badger_default::{DefaultBadgerStrategy, ECCBadgerOptimiser};

#[cfg(test)]
#[cfg(feature = "portmatching")]
mod tests {
    use hugr::{
        HugrView,
        builder::{DFGBuilder, Dataflow, DataflowHugr},
        extension::prelude::qb_t,
        types::Signature,
    };
    use rstest::{fixture, rstest};

    use crate::serialize::load_tk1_json_str;
    use crate::serialize::pytket::DecodeOptions;
    use crate::{Circuit, TketOp};
    use crate::{extension::rotation::rotation_type, optimiser::badger::BadgerOptions};

    use super::{BadgerOptimiser, ECCBadgerOptimiser};

    #[fixture]
    fn rz_rz() -> Circuit {
        let input_t = vec![qb_t(), rotation_type(), rotation_type()];
        let output_t = vec![qb_t()];
        let mut h = DFGBuilder::new(Signature::new(input_t, output_t)).unwrap();

        let mut inps = h.input_wires();
        let qb = inps.next().unwrap();
        let f1 = inps.next().unwrap();
        let f2 = inps.next().unwrap();

        let res = h.add_dataflow_op(TketOp::Rz, [qb, f1]).unwrap();
        let qb = res.outputs().next().unwrap();
        let res = h.add_dataflow_op(TketOp::Rz, [qb, f2]).unwrap();
        let qb = res.outputs().next().unwrap();

        h.finish_hugr_with_outputs([qb]).unwrap().into()
    }

    /// This hugr corresponds to the qasm circuit:
    ///
    /// ```skip
    /// OPENQASM 2.0;
    /// include "qelib1.inc";
    ///
    /// qreg q[5];
    /// cx q[4],q[1];
    /// cx q[3],q[4];
    /// cx q[1],q[2];
    /// cx q[4],q[0];
    /// u3(0.5*pi,0.0*pi,0.5*pi) q[1];
    /// cx q[0],q[2];
    /// cx q[3],q[1];
    /// cx q[0],q[2];
    /// ```
    const NON_COMPOSABLE: &str = r#"{"phase":"0.0","commands":[{"op":{"type":"CX","n_qb":2,"signature":["Q","Q"]},"args":[["q",[4]],["q",[1]]]},{"op":{"type":"CX","n_qb":2,"signature":["Q","Q"]},"args":[["q",[1]],["q",[2]]]},{"op":{"type":"U3","params":["0.5","0","0.5"],"signature":["Q"]},"args":[["q",[1]]]},{"op":{"type":"CX","n_qb":2,"signature":["Q","Q"]},"args":[["q",[3]],["q",[4]]]},{"op":{"type":"CX","n_qb":2,"signature":["Q","Q"]},"args":[["q",[4]],["q",[0]]]},{"op":{"type":"CX","n_qb":2,"signature":["Q","Q"]},"args":[["q",[0]],["q",[2]]]},{"op":{"type":"CX","n_qb":2,"signature":["Q","Q"]},"args":[["q",[0]],["q",[2]]]},{"op":{"type":"CX","n_qb":2,"signature":["Q","Q"]},"args":[["q",[3]],["q",[1]]]}],"qubits":[["q",[0]],["q",[1]],["q",[2]],["q",[3]],["q",[4]]],"bits":[],"implicit_permutation":[[["q",[0]],["q",[0]]],[["q",[1]],["q",[1]]],[["q",[2]],["q",[2]]],[["q",[3]],["q",[3]]],[["q",[4]],["q",[4]]]]}"#;

    /// A circuit that would trigger non-composable rewrites, if we applied them blindly from nam_6_3 matches.
    #[fixture]
    fn non_composable_rw_hugr() -> Circuit {
        load_tk1_json_str(NON_COMPOSABLE, DecodeOptions::new()).unwrap()
    }

    /// A badger optimiser using a reduced set of rewrite rules.
    #[fixture]
    fn badger_opt_json() -> ECCBadgerOptimiser {
        BadgerOptimiser::default_with_eccs_json_file("../test_files/eccs/small_eccs.json").unwrap()
    }

    /// A badger optimiser using a reduced set of rewrite rules.
    #[fixture]
    fn badger_opt_compiled() -> ECCBadgerOptimiser {
        BadgerOptimiser::default_with_rewriter_binary("../test_files/eccs/small_eccs.rwr").unwrap()
    }

    /// A badger optimiser using the complete nam_6_3 rewrite set.
    ///
    /// NOTE: This takes a few seconds to load.
    /// Use [`badger_opt`] if possible.
    #[fixture]
    fn badger_opt_full() -> ECCBadgerOptimiser {
        BadgerOptimiser::default_with_rewriter_binary("../test_files/eccs/nam_6_3.rwr").unwrap()
    }

    #[rstest]
    #[case::compiled(badger_opt_compiled())]
    #[case::json(badger_opt_json())]
    fn rz_rz_cancellation(rz_rz: Circuit, #[case] badger_opt: ECCBadgerOptimiser) {
        use hugr::ops::OpType;

        use crate::{extension::rotation::RotationOp, op_matches};

        let opt_rz = badger_opt.optimise(
            &rz_rz,
            BadgerOptions {
                queue_size: 4,
                ..Default::default()
            },
        );
        let [op1, op2]: [&OpType; 2] = opt_rz
            .commands()
            .map(|cmd| cmd.optype())
            .collect::<Vec<_>>()
            .try_into()
            .unwrap();

        // Rzs combined into a single one.
        assert_eq!(op1.cast(), Some(RotationOp::radd));
        assert!(op_matches(op2, TketOp::Rz));
    }

    #[rstest]
    #[case::compiled(badger_opt_compiled())]
    #[case::json(badger_opt_json())]
    fn rz_rz_cancellation_parallel(rz_rz: Circuit, #[case] badger_opt: ECCBadgerOptimiser) {
        let opt_rz = badger_opt.optimise(
            &rz_rz,
            BadgerOptions {
                timeout: Some(0),
                n_threads: 2.try_into().unwrap(),
                queue_size: 4,
                ..Default::default()
            },
        );
        opt_rz.hugr().validate().unwrap();
    }

    #[rstest]
    #[case::compiled(badger_opt_compiled())]
    #[case::json(badger_opt_json())]
    fn rz_rz_cancellation_split_parallel(rz_rz: Circuit, #[case] badger_opt: ECCBadgerOptimiser) {
        let opt_rz = badger_opt.optimise(
            &rz_rz,
            BadgerOptions {
                timeout: Some(0),
                n_threads: 2.try_into().unwrap(),
                queue_size: 4,
                split_circuit: true,
                ..Default::default()
            },
        );
        opt_rz.hugr().validate().unwrap();
        assert_eq!(opt_rz.commands().count(), 2);
    }

    #[rstest]
    #[ignore = "Loading the ECC set is really slow (~5 seconds)"]
    fn non_composable_rewrites(
        non_composable_rw_hugr: Circuit,
        badger_opt_full: ECCBadgerOptimiser,
    ) {
        let opt = badger_opt_full.optimise(
            &non_composable_rw_hugr,
            BadgerOptions {
                timeout: Some(0),
                queue_size: 4,
                ..Default::default()
            },
        );
        // No rewrites applied.
        opt.hugr().validate().unwrap();
    }

    #[test]
    fn load_precompiled_bin() {
        let opt =
            BadgerOptimiser::default_with_rewriter_binary("../test_files/eccs/small_eccs.rwr");
        opt.unwrap();
    }
}
