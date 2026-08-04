import time

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError as exc:
    if exc.name != "matplotlib":
        raise
    plt = None

from tket._state.build import H, OneQbGate, from_coms
from tket.passes import Cliffordize

T = OneQbGate("T")

print("Sequential T gates")
print(
    f"{'N':>6} {'rewrites':>9} {'time(s)':>9} {'time/N^2':>10} "
    f"{'ratio':>8} {'quadratic':>10}"
)
prev = None
prev_n = None
sizes = (
    100,
    150,
    200,
    300,
    400,
    600,
    800,
    1200,
    1600,
    2000,
    2400,
    2800,
    3200,
    4000,
    4600,
    4800,
    5200,
    5400,
    5700,
    6000,
    6400,
    6600,
    7000,
)
times = []
for n in sizes:
    # Circuit construction is deliberately outside the timed region.
    hugr = from_coms(*[T(0) for _ in range(n)]).to_python().modules[0]

    t0 = time.perf_counter()
    res = Cliffordize().run(hugr, inplace=False)
    dt = time.perf_counter() - t0

    ratio = f"x{dt / prev:.1f}" if prev else ""
    quadratic_ratio = f"x{(n / prev_n) ** 2:.1f}" if prev_n else ""
    print(
        f"{n:>6} {res.results[-1][1]:>9} {dt:>9.4f} "
        f"{dt / (n * n):>10.2e} {ratio:>8} {quadratic_ratio:>10}"
    )
    times.append(dt)
    prev = dt
    prev_n = n

if plt is not None:
    quadratic_times = [
        [anchor_time * (n / anchor_size) ** 2 for n in sizes]
        for anchor_size, anchor_time in zip(sizes[:-4], times[:-4])
    ]
    plt.plot(sizes, times, "o-", label="Measured")
    for anchor_size, expected_times in zip(sizes[:-4], quadratic_times):
        plt.plot(
            sizes,
            expected_times,
            "--",
            label=rf"Theoretical $O(N^2)$ from N={anchor_size}",
        )
    plt.xlabel("Number of T gates (N)")
    plt.ylabel("Time (s)")
    plt.title("Cliffordize scaling")
    plt.grid(True, which="both", linestyle=":")
    plt.legend()
    plt.tight_layout()
    plt.show()


print()
# print("One T in an otherwise-Clifford region")
# print(f"{'N':>6} {'rewrites':>9} {'time(s)':>9} {'time/N':>10} {'ratio':>8}")
# prev = None
# for n in (200, 400, 800, 1600, 3200, 6400):
#     hugr = from_coms(
#         *([H(0) for _ in range(n - 1)] + [T(0)])
#     ).to_python().modules[0]

#     t0 = time.perf_counter()
#     res = Cliffordize().run(hugr, inplace=False)
#     dt = time.perf_counter() - t0

#     ratio = f"x{dt / prev:.1f}" if prev else ""
#     print(
#         f"{n:>6} {res.results[-1][1]:>9} {dt:>9.4f} "
#         f"{dt / n:>10.2e} {ratio:>8}"
#     )
#     prev = dt
