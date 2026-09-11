/// Parallelisation mode for gate costing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParallelMode {
    /// Always use sequential execution.
    Off,
    /// Always use parallel execution.
    On,
    /// Use parallel execution when the workload reaches 150,000.
    Auto,
}

impl ParallelMode {
    /// Reports whether the two workload dimensions justify parallel execution.
    ///
    /// Auto mode uses parallel costing at a workload product of 150,000. This
    /// inherited tuning threshold should only change after representative
    /// benchmarks.
    pub(crate) fn should_use_parallel(self, work_items: usize, candidates: usize) -> bool {
        match self {
            Self::Off => false,
            Self::On => true,
            Self::Auto => work_items.saturating_mul(candidates) >= 150_000,
        }
    }
}
