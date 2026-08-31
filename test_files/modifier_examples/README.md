# Modifier examples

This directory contains test guppy programs with modifier
operations (`dagger` and `control`). The examples are used for regression
testing of modifier compilation in tket2.

Each example is defined as a `.py` [uv script](https://docs.astral.sh/uv/guides/scripts/)
that defines the guppy program for a pinned version of guppylang.
The compiled HUGR is stored alongside it with a `.hugr` extension.

## Recompiling

| Command | Where | Effect |
|---------|-------|--------|
| `just recompile-hugrs` | this directory | Recompile all `.hugr` files |
| `just recompile-modifiers` | repo root | Recompile all `.hugr` files |
| `just recompile-test-hugrs` | repo root | Same as above, plus all other test HUGRs in `test_files/` |

All the previous commands also update
the `test_files/run_modifier_examples/hugr_results/` directory with `.npy` files
containing simulation results, and regenerate the human-readable summary in
`test_files/run_modifier_examples/hugr_results.txt`.

To recompile and run a single example, use from the repo root `just recompile-modifier <example_name>` or `just example <example_name>.py` in this directory. This will recompile the `.hugr` file and update the results only for that example., 

