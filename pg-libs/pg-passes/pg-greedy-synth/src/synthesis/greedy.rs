use super::ParallelMode;
use crate::backend::{
    CostKernel, ExpandedSparseWeightedSum, GreedyCostBackend, GroupedWeightedSum, ScalarBackend,
    WeightedSumStrategy,
};
#[cfg(feature = "simd")]
use crate::backend::{ExpandedSimdWeightedSum, SimdBackend};
use crate::packed_pg_slice::PackedBackend;
use crate::reducer::Reducer;
use pg_core::{Op, PGPass, PauliGraph};
use std::cmp::max;

const DEFAULT_WINDOW_SIZE: usize = 1280;
const MINIMUM_POOL_SIZE: usize = 1000;
const MINIMUM_TOP_UP_SIZE: usize = 200;
const GROUPED_WEIGHTED_SUM_MIN_ROTATIONS_PER_SET: usize = 64;

#[derive(Clone, Copy)]
struct GreedySynthConfig {
    window_size: Option<usize>,
    pool_size: Option<usize>,
    top_up_size: Option<usize>,
    enable_progress: bool,
    seed: u64,
    parallel_mode: ParallelMode,
}

impl Default for GreedySynthConfig {
    fn default() -> Self {
        Self {
            window_size: None,
            pool_size: None,
            top_up_size: None,
            enable_progress: false,
            seed: 0,
            parallel_mode: ParallelMode::Auto,
        }
    }
}

/// Returns the average size of nonempty rotation sets.
fn average_rotations_per_set(pg: &PauliGraph) -> usize {
    let mut rotations = 0;
    let mut rotation_sets = 0;
    let mut current_set_has_rotation = false;

    for op in pg.get_ops() {
        match op {
            Op::SetBoundary => {
                rotation_sets += usize::from(current_set_has_rotation);
                current_set_has_rotation = false;
            }
            Op::Rotation { .. } => {
                rotations += 1;
                current_set_has_rotation = true;
            }
            _ => {}
        }
    }
    rotation_sets += usize::from(current_set_has_rotation);

    if rotation_sets == 0 {
        0
    } else {
        rotations / rotation_sets
    }
}

fn resolve_sizes(
    n_qubits: usize,
    window_size: Option<usize>,
    pool_size: Option<usize>,
    top_up_size: Option<usize>,
) -> (usize, usize, usize) {
    let window_size = window_size.unwrap_or(DEFAULT_WINDOW_SIZE);
    let pool_size = pool_size.unwrap_or(max(
        MINIMUM_POOL_SIZE,
        ((n_qubits * n_qubits) as f64 * 0.2) as usize,
    ));
    let top_up_size = top_up_size.unwrap_or(max(MINIMUM_TOP_UP_SIZE, pool_size / n_qubits));
    (window_size, pool_size, top_up_size)
}

/// Transforms a Pauli graph using the specified backend, the default weighted
/// sum strategy and the given synthesis configuration.
///
/// Grouped weighting replaces expanded weighting when nonempty rotation sets
/// contain an average of at least 64 rotations. This inherited tuning heuristic
/// should only change with representative benchmarks and an output quality
/// comparison.
fn transform_with_backend<P, W>(
    pg: &PauliGraph,
    config: GreedySynthConfig,
    packed_backend: P,
    expanded_weighted_sum: W,
) -> PauliGraph
where
    P: PackedBackend + CostKernel + Copy + Sync,
    W: WeightedSumStrategy,
{
    assert!(
        config.window_size.is_none_or(|size| size > 0),
        "window size must be positive"
    );
    assert!(
        config.pool_size.is_none_or(|size| size > 0),
        "pool size must be positive"
    );
    assert!(
        config.top_up_size.is_none_or(|size| size > 0),
        "top-up size must be positive"
    );
    if pg.get_ops().is_empty() {
        return PauliGraph::new(pg.get_n_qubits());
    }

    let (window_size, pool_size, top_up_size) = resolve_sizes(
        pg.get_n_qubits(),
        config.window_size,
        config.pool_size,
        config.top_up_size,
    );
    if average_rotations_per_set(pg) >= GROUPED_WEIGHTED_SUM_MIN_ROTATIONS_PER_SET {
        Reducer::new(
            packed_backend,
            GreedyCostBackend::new(packed_backend, GroupedWeightedSum::default()),
            window_size,
            pool_size,
            top_up_size,
            config.seed,
            config.parallel_mode,
            config.enable_progress,
        )
        .reduce(pg)
    } else {
        Reducer::new(
            packed_backend,
            GreedyCostBackend::new(packed_backend, expanded_weighted_sum),
            window_size,
            pool_size,
            top_up_size,
            config.seed,
            config.parallel_mode,
            config.enable_progress,
        )
        .reduce(pg)
    }
}

macro_rules! impl_pass_configuration {
    ($pass:ty, $constructor_summary:literal) => {
        impl $pass {
            #[doc = $constructor_summary]
            pub fn new() -> Self {
                Self {
                    config: GreedySynthConfig::default(),
                }
            }

            /// Sets the lookahead window size for packed operations.
            ///
            /// The default is 1280. The value must be positive.
            pub fn with_window_size(mut self, size: usize) -> Self {
                self.config.window_size = Some(size);
                self
            }

            /// Sets the number of candidates sampled after frontier progress.
            ///
            /// The default depends on the input and is at least 1000. The value
            /// must be positive.
            pub fn with_pool_size(mut self, size: usize) -> Self {
                self.config.pool_size = Some(size);
                self
            }

            /// Sets the number of candidates added when the frontier has not
            /// progressed.
            ///
            /// The default depends on the input and is at least 200. The value
            /// must be positive.
            pub fn with_top_up_size(mut self, size: usize) -> Self {
                self.config.top_up_size = Some(size);
                self
            }

            /// Enables or disables terminal progress reporting.
            ///
            /// Progress reporting is disabled by default.
            pub fn with_progress(mut self, enable: bool) -> Self {
                self.config.enable_progress = enable;
                self
            }

            /// Sets the seed for candidate sampling.
            ///
            /// The default seed is zero.
            pub fn with_seed(mut self, seed: u64) -> Self {
                self.config.seed = seed;
                self
            }

            /// Selects the parallelisation policy for candidate costing.
            ///
            /// The default is [`ParallelMode::Auto`].
            pub fn with_parallel_mode(mut self, mode: ParallelMode) -> Self {
                self.config.parallel_mode = mode;
                self
            }
        }

        impl Default for $pass {
            fn default() -> Self {
                Self::new()
            }
        }
    };
}

/// Greedily synthesises a canonical Pauli graph with scalar packed kernels.
///
/// The pass expects the output of `CanonicalFormPass` followed by
/// `GroupCommutingOpsPass`. Output contains Clifford gates on individual
/// qubits, Pauli rotations, measurements, resets, black boxes and TQE gates.
///
pub struct GreedySynthPass {
    config: GreedySynthConfig,
}

impl_pass_configuration!(
    GreedySynthPass,
    "Creates a scalar synthesis pass with sizes chosen automatically."
);

impl PGPass for GreedySynthPass {
    fn transform(&self, pg: &PauliGraph) -> PauliGraph {
        transform_with_backend(
            pg,
            self.config,
            ScalarBackend,
            ExpandedSparseWeightedSum::default(),
        )
    }
}

/// Greedily synthesises a canonical Pauli graph with portable SIMD kernels.
#[cfg(feature = "simd")]
pub struct GreedySynthSimdPass {
    config: GreedySynthConfig,
}

#[cfg(feature = "simd")]
impl_pass_configuration!(
    GreedySynthSimdPass,
    "Creates a SIMD synthesis pass with sizes chosen automatically."
);

#[cfg(feature = "simd")]
impl PGPass for GreedySynthSimdPass {
    fn transform(&self, pg: &PauliGraph) -> PauliGraph {
        transform_with_backend(
            pg,
            self.config,
            SimdBackend,
            ExpandedSimdWeightedSum::default(),
        )
    }
}
