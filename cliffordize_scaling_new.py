import statistics
import time
from pathlib import Path

import matplotlib.pyplot as plt
from tket._state.build import OneQbGate, from_coms
from tket.passes import Cliffordize

T = OneQbGate("T")
SIZES = (100, 200, 400, 800, 1_600, 3_200, 6_400)
REPEATS = 5
PLOT_PATH = Path(__file__).with_name("cliffordize_new_run_scaling.png")


def benchmark_new_run() -> tuple[list[float], list[int]]:
    times: list[float] = []
    rewrite_counts: list[int] = []

    print("Sequential T gates using Cliffordize.new_run")
    print(
        f"{'N':>6} {'rewrites':>9} {'median(s)':>11} "
        f"{'time/N':>10} {'ratio':>8} {'linear':>8}"
    )

    previous_time: float | None = None
    previous_size: int | None = None

    for size in SIZES:
        # Circuit construction is deliberately outside the timed region.
        hugr = from_coms(*[T(0) for _ in range(size)]).to_python().modules[0]

        samples = []
        rewrite_count = 0
        for _ in range(REPEATS):
            start = time.perf_counter()
            result = Cliffordize().new_run(hugr, inplace=False)
            samples.append(time.perf_counter() - start)
            rewrite_count = result.results[-1][1]

        median_time = statistics.median(samples)
        ratio = f"x{median_time / previous_time:.1f}" if previous_time else ""
        linear_ratio = f"x{size / previous_size:.1f}" if previous_size else ""

        print(
            f"{size:>6} {rewrite_count:>9} {median_time:>11.6f} "
            f"{median_time / size:>10.2e} {ratio:>8} {linear_ratio:>8}"
        )

        times.append(median_time)
        rewrite_counts.append(rewrite_count)
        previous_time = median_time
        previous_size = size

    return times, rewrite_counts


def plot_results(times: list[float]) -> None:
    anchor_size = SIZES[-1]
    anchor_time = times[-1]
    linear_reference = [anchor_time * size / anchor_size for size in SIZES]

    _, axis = plt.subplots()
    axis.plot(SIZES, times, "o-", label="Cliffordize.new_run")
    axis.plot(SIZES, linear_reference, "--", label=r"Linear reference $O(N)$")
    axis.set_xlabel("Number of T gates (N)")
    axis.set_ylabel("Median time (s)")
    axis.set_title("Cliffordize.new_run scaling")
    axis.grid(True, linestyle=":")
    axis.legend()
    plt.tight_layout()
    plt.savefig(PLOT_PATH, dpi=160)
    print(f"\nPlot saved to {PLOT_PATH}")


def main() -> None:
    times, rewrite_counts = benchmark_new_run()
    assert rewrite_counts == list(SIZES)
    plot_results(times)


if __name__ == "__main__":
    main()
