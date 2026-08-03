"""Phase 7 chart generation (plan §21.3).

Reads the experiment CSVs in ``experiments/results/`` and renders the
comparison charts referenced by the plan:

  * algorithm compute time per scenario/planner   (§21.3 "계산 시간 비교")
  * algorithm path length per scenario/planner     (§21.3 "경로 길이 비교")
  * algorithm success rate per scenario/planner
  * LKA RMS lateral error vs speed per controller  (§21.3 "lateral error 그래프")

Headless (matplotlib Agg) so it runs without a display and can be re-run by the
analysis notebook or from CI. PNGs go to ``experiments/results/charts/``.

Run:
    python experiments/make_charts.py
"""
from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
CHARTS = RESULTS / "charts"
PLANNER_ORDER = ["astar", "rrt", "rrt_star"]
PLANNER_COLOR = {"astar": "#2563eb", "rrt": "#f59e0b", "rrt_star": "#10b981"}


def _read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


# --------------------------------------------------------------------------- #
def chart_algo(summary: list[dict]) -> None:
    """Grouped bars per scenario for time, path length, success rate.

    Uses the largest vehicle-count row per (scenario, planner) so each planner
    appears once per scenario.
    """
    # keep the max num_vehicles row for each (scenario, planner)
    best: dict[tuple[str, str], dict] = {}
    for r in summary:
        key = (r["scenario"], r["planner"])
        if key not in best or int(r["num_vehicles"]) > int(best[key]["num_vehicles"]):
            best[key] = r
    scenarios = sorted({r["scenario"] for r in summary})

    metrics = [
        ("mean_time_ms", "Mean compute time (ms, log scale)", True),
        ("mean_path_length", "Mean path length (m)", False),
        ("success_rate", "Success rate", False),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    x = range(len(scenarios))
    width = 0.25
    for ax, (field, title, log) in zip(axes, metrics):
        for pi, planner in enumerate(PLANNER_ORDER):
            vals = [float(best.get((s, planner), {}).get(field, 0.0)) for s in scenarios]
            ax.bar([xi + (pi - 1) * width for xi in x], vals, width,
                   label=planner, color=PLANNER_COLOR[planner])
        ax.set_title(title)
        ax.set_xticks(list(x))
        ax.set_xticklabels(scenarios, rotation=15, ha="right")
        if log:
            ax.set_yscale("log")
        ax.grid(axis="y", alpha=0.3)
    axes[0].legend(title="planner")
    fig.suptitle("A* vs RRT vs RRT*  —  compute / quality / success (plan §20.1)")
    fig.tight_layout()
    out = CHARTS / "algo_compare.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"wrote {out}")


def chart_time_vs_vehicles(summary: list[dict]) -> None:
    """Total planning load vs vehicle count, per scenario/planner."""
    fig, ax = plt.subplots(figsize=(7, 4.5))
    series: dict[tuple[str, str], list[tuple[int, float]]] = defaultdict(list)
    for r in summary:
        series[(r["scenario"], r["planner"])].append(
            (int(r["num_vehicles"]), float(r["total_time_ms"])))
    for (scenario, planner), pts in sorted(series.items()):
        pts.sort()
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, marker="o", color=PLANNER_COLOR.get(planner),
                linestyle={"astar": "-", "rrt": "--", "rrt_star": ":"}.get(planner, "-"),
                label=f"{scenario}/{planner}")
    ax.set_xlabel("vehicles (queries)")
    ax.set_ylabel("total planning time (ms, log scale)")
    ax.set_yscale("log")
    ax.set_title("Planning load vs vehicle count (plan §20.1)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    out = CHARTS / "algo_time_vs_vehicles.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"wrote {out}")


def chart_lka(lka: list[dict]) -> None:
    """RMS lateral error vs speed per controller (plan §20.2 / §21.3)."""
    if not lka:
        print("skip LKA chart: no lka_summary.csv (run experiments/run_lka_test.py)")
        return
    series: dict[str, list[tuple[float, float, float]]] = defaultdict(list)
    for r in lka:
        series[r["controller"]].append(
            (float(r["speed_kmh"]), float(r["rms_lateral_m"]), float(r["max_lateral_m"])))
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for ctrl, pts in sorted(series.items()):
        pts.sort()
        xs = [p[0] for p in pts]
        rms = [p[1] for p in pts]
        ax.plot(xs, rms, marker="o", label=ctrl)
    ax.set_xlabel("speed (km/h)")
    ax.set_ylabel("RMS lateral error (m)")
    ax.set_title("LKA lateral error vs speed (plan §20.2)")
    ax.grid(alpha=0.3)
    ax.legend(title="controller")
    fig.tight_layout()
    out = CHARTS / "lka_lateral_error.png"
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"wrote {out}")


def main() -> None:
    CHARTS.mkdir(parents=True, exist_ok=True)
    summary = _read_csv(RESULTS / "algo_compare_summary.csv")
    lka = _read_csv(RESULTS / "lka_summary.csv")
    if summary:
        chart_algo(summary)
        chart_time_vs_vehicles(summary)
    else:
        print("skip algo charts: no algo_compare_summary.csv "
              "(run experiments/run_algorithm_compare.py)")
    chart_lka(lka)


if __name__ == "__main__":
    main()
