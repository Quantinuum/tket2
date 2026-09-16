# Internal tuning constants

Internal constants used by synthesis heuristics and default size calculations.

## Default sizing constants

Defined in [synthesis/greedy.rs](src/synthesis/greedy.rs).

| Parameter             | Value  | Purpose                                                                                                            |
| --------------------- | ------ | ------------------------------------------------------------------------------------------------------------------ |
| `DEFAULT_WINDOW_SIZE` | `1280` | Default target number of operations in the costing window.                                                              |
| `MINIMUM_POOL_SIZE`   | `1000` | Minimum default number of TQE candidates sampled after frontier progress.                                                                        |
| `MINIMUM_TOP_UP_SIZE` | `200`  | Minimum default number of fresh TQE samples added without frontier progress, replenishing invalidated candidates.                                             |
| `TQE_SAMPLE_FRACTION` | `0.1`  | Rough proportion of the candidates for a Pauli string with maximum weight to sample. The minimum pool size still applies. |

## Internal heuristics

| Parameter | Value | Purpose | Source |
| --- | --- | --- | --- |
| `PARALLEL_COSTING_THRESHOLD` | `150_000` | Auto mode uses parallel costing when `work_items * candidates` exceeds this threshold, to offset parallel overhead. `work_items` is the number of visible operations or tableau generators being costed. | [parallel_mode.rs](src/synthesis/parallel_mode.rs) |
| `GROUPED_WEIGHTED_SUM_MIN_OPS_PER_SET` | `64` | Uses `GroupedWeightedSum` when the average set size reaches this threshold. | [greedy.rs](src/synthesis/greedy.rs) |
| `SPARSE_WEIGHTING_MAX_DENSITY` | `0.25` | `ExpandedSimdWeightedSum` strategy uses sparse iteration below this mask density, avoiding work on zero bits. | [expanded.rs](src/backend/weighted_sum/expanded.rs) |
| `ALPHA` | `0.588` | Factor by which to discount the weight of each successive set, favouring nearby operations. | [greedy.rs](src/backend/greedy.rs) |
| `MAX_SELECTION_CANDIDATES` | `5` | Limits depth comparison to this many current candidates with the lowest costs. | [gate_pool.rs](src/reducer/gate_pool.rs) |
| `DEPTH_WEIGHT` | `0.3` | Weights normalised depth relative to normalised gate cost. Larger values favour depth more strongly. | [gate_pool.rs](src/reducer/gate_pool.rs) |
