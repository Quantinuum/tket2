//! Optimisers for circuit rewriting.
//!
//! Currently, the only optimiser is Badger

mod backtracking;
pub mod badger;
mod pqueue;
mod pqueue_worker;

use std::fmt::Debug;

#[cfg(feature = "portmatching")]
pub use badger::ECCBadgerOptimiser;
pub use badger::{BadgerLogger, BadgerOptimiser};
use pqueue::StatePQueue;
use pqueue_worker::StatePQWorker;

pub use backtracking::BacktrackingOptimiser;

/// Options for the optimisation routine.
///
/// Use `OptimiserOptions::default()` to get the default options, then set the
/// options you want to activate. Currently available options are:
/// - `badger_logger`: an object to log optimisation progressed used for the
///   Badger optimiser.
/// - `track_n_best`: Instead of returning only the state minimising the cost
///   function, return the `n` best candidates.
///
/// See [Optimiser::optimise_with_options] for more details.
#[derive(Default)]
#[non_exhaustive]
pub struct OptimiserOptions<'w> {
    /// A logger to log optimisation progressed used for the Badger optimiser.
    pub badger_logger: BadgerLogger<'w>,
    /// The number of best states to track.
    pub track_n_best: Option<usize>,
}

impl<'w> From<BadgerLogger<'w>> for OptimiserOptions<'w> {
    fn from(badger_logger: BadgerLogger<'w>) -> Self {
        Self {
            badger_logger,
            ..Default::default()
        }
    }
}

/// The result of an optimisation routine.
pub struct OptimiserResult<S> {
    /// The best state found.
    pub best_state: S,
    /// The `n` best states found.
    ///
    /// Not `None` iff the `track_n_best` option is set.
    pub n_best_states: Option<Vec<S>>,
}

/// An optimiser exploring a discrete search space, in search for the lowest
/// cost state.
///
/// The optimiser accepts a global context object, which each state can mutate
/// when computing the next states that can be transitioned to.
pub trait Optimiser: Sized {
    /// Start optimisation from the given state, using the given context.
    fn optimise<C, S>(&self, start_state: S, start_context: C) -> Option<S>
    where
        S: State<C>,
    {
        self.optimise_with_options(start_state, start_context, Default::default())
            .map(|r| r.best_state)
    }

    /// Start optimisation from the given state, using the given context and
    /// logger.
    fn optimise_with_options<C, S>(
        &self,
        start_state: S,
        start_context: C,
        options: OptimiserOptions,
    ) -> Option<OptimiserResult<S>>
    where
        S: State<C>;
}

/// A state in the search space of the optimiser.
///
/// A mutable context is shared between all states in the search space.
pub trait State<Context>: Clone {
    /// The cost of the state, to be minimised.
    type Cost: Ord + Debug + serde::Serialize + Clone;

    /// The hash of the state.
    ///
    /// This may fail, in which case the state is considered invalid and
    /// discarded.
    fn hash(&self, context: &Context) -> Option<u64>;

    /// The cost of the state, to be minimised.
    ///
    /// This may fail, in which case the state is considered invalid and
    /// discarded.
    fn cost(&self, context: &Context) -> Option<Self::Cost>;

    /// The next states from the current state.
    ///
    /// States are allowed to write to a global "context" object. This allows
    /// search problems where for instance states are nodes in an infinitely
    /// sized graph that is lazily generated.
    ///
    /// Note that the order in which the states are visited is not guaranteed,
    /// which may result in different contexts.
    fn next_states(&self, context: &mut Context) -> Vec<Self>;
}
