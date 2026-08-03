"""Phase 7 experiment 1: A* vs RRT vs RRT* (plan §15, §20.1).

Runs each planner over a set of scenarios, vehicle counts, and random seeds,
measuring the three things the plan asks to compare:

  * **compute time** (ms per query),
  * **path length** (m), and
  * **success rate** (did a collision-free path to the goal come back?).

The scenarios are chosen to expose the expected result from plan §15.4:
  - ``road_open``     — clean road graph, no obstacles → A* should win
                        (fast, road-following, always solves).
  - ``road_detour``   — a lane blocked by a hazard → lane-graph A* has no
                        alternative edge and fails; RRT/RRT* detour in free
                        space and succeed.
  - ``obstacle_field``— several irregular obstacles between start and goal →
                        RRT vs RRT* quality/compute trade-off.

Pure stdlib (csv, time, statistics) so it runs headless with no extra deps.
Writes per-run rows and an aggregated summary to ``experiments/results/``.

Run:
    python experiments/run_algorithm_compare.py
"""
from __future__ import annotations

import csv
import random
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "server"))

from planners import AStarPlanner, RRTConfig, RRTPlanner, RRTStarPlanner  # noqa: E402
from planners._rrt_common import collision_free, polyline_length            # noqa: E402
from scenarios import networks                                              # noqa: E402
from world_model import LaneNetwork, Lane                                   # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
SEEDS = [0, 1, 2, 3, 4]
VEHICLE_COUNTS = [1, 5, 20]
PLANNERS = ["astar", "rrt", "rrt_star"]

# Sampling-planner iteration budgets. RRT stops at the first goal connection so
# a generous cap is cheap; RRT* runs its whole budget refining, so it's kept
# lower to keep the full experiment matrix tractable (still solves these maps).
RRT_ITERS = 3000
RRT_STAR_ITERS = 1500


# --------------------------------------------------------------------------- #
# Scenario construction. Each returns (network, list[(start, goal)]).
# --------------------------------------------------------------------------- #
def _straight_lane(lane_id: str, length: float, x: float = 0.0) -> Lane:
    n = max(2, int(length / 5.0))
    cl = [[x, 0.0, length * i / n] for i in range(n + 1)]
    return Lane(id=lane_id, centerline=cl, width=3.5, speed_limit=27.8)


def scenario_road_open(n_vehicles: int):
    """Clean 2x2 urban grid, no obstacles — the road-graph home turf of A*."""
    net = networks.urban_grid(rows=2, cols=2, block=60.0)
    rng = random.Random(9001)
    queries = []
    for _ in range(n_vehicles):
        # bottom row -> top-right corner, small lateral jitter per vehicle
        sx = rng.uniform(0.0, 4.0)
        gz = rng.uniform(112.0, 120.0)
        queries.append(([sx, 0.0, 2.0], [120.0, 0.0, gz]))
    return net, queries


def scenario_road_detour(n_vehicles: int):
    """A single straight corridor with the middle blocked by a fallen object.
    The lane graph offers no alternative edge, so A* fails; RRT/RRT* detour."""
    net = LaneNetwork([_straight_lane("corridor", 100.0)], name="detour",
                      scenario="highway")
    net.block([0.0, 0.0, 50.0], radius=3.0)      # hazard on the lane
    rng = random.Random(9002)
    queries = [([rng.uniform(-1.5, 1.5), 0.0, 2.0],
                [rng.uniform(-1.5, 1.5), 0.0, 98.0]) for _ in range(n_vehicles)]
    return net, queries


def scenario_obstacle_field(n_vehicles: int):
    """Free space between start and goal cluttered with irregular obstacles —
    the RRT/RRT* use case (parking lot / off-graph detour)."""
    net = LaneNetwork([_straight_lane("field", 100.0)], name="field",
                      scenario="highway")
    obstacles = [(-6, 25), (8, 40), (-4, 55), (10, 70), (-8, 82), (4, 90)]
    for ox, oz in obstacles:
        net.block([float(ox), 0.0, float(oz)], radius=4.0)
    rng = random.Random(9003)
    queries = [([rng.uniform(-1.5, 1.5), 0.0, 2.0],
                [rng.uniform(-1.5, 1.5), 0.0, 98.0]) for _ in range(n_vehicles)]
    return net, queries


SCENARIOS = {
    "road_open": scenario_road_open,
    "road_detour": scenario_road_detour,
    "obstacle_field": scenario_obstacle_field,
}


# --------------------------------------------------------------------------- #
def make_planner(name: str, seed: int):
    if name == "astar":
        return AStarPlanner()
    iters = RRT_ITERS if name == "rrt" else RRT_STAR_ITERS
    cfg = RRTConfig(seed=seed, step_size=4.0, goal_sample_rate=0.1,
                    max_iters=iters, goal_radius=4.0, edge_resolution=1.0,
                    margin=40.0)
    return RRTPlanner(cfg) if name == "rrt" else RRTStarPlanner(cfg)


def path_is_collision_free(path, world) -> bool:
    if len(path) < 2:
        return len(path) == 1  # single point == trivially at goal
    return all(collision_free(a, b, world, 1.0) for a, b in zip(path, path[1:]))


def run_query(name: str, seed: int, start, goal, world):
    planner = make_planner(name, seed)
    t0 = time.perf_counter()
    path = planner.plan(start, goal, world)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    success = bool(path) and path_is_collision_free(path, world)
    length = polyline_length(path) if path else 0.0
    nodes = getattr(planner, "last_expanded", getattr(planner, "last_nodes", 0))
    return {
        "success": int(success),
        "plan_time_ms": elapsed_ms,
        "path_length": length if success else 0.0,
        "nodes": nodes,
    }


def main():
    RESULTS.mkdir(parents=True, exist_ok=True)
    raw_path = RESULTS / "algo_compare_raw.csv"
    summary_path = RESULTS / "algo_compare_summary.csv"

    raw_rows = []
    for scenario, builder in SCENARIOS.items():
        for n_veh in VEHICLE_COUNTS:
            net, queries = builder(n_veh)
            for planner_name in PLANNERS:
                for seed in SEEDS:
                    for vi, (start, goal) in enumerate(queries):
                        r = run_query(planner_name, seed, start, goal, net)
                        raw_rows.append({
                            "scenario": scenario, "planner": planner_name,
                            "num_vehicles": n_veh, "seed": seed, "vehicle_id": vi,
                            **r,
                        })

    with open(raw_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=[
            "scenario", "planner", "num_vehicles", "seed", "vehicle_id",
            "success", "plan_time_ms", "path_length", "nodes"])
        w.writeheader()
        w.writerows(raw_rows)

    # Aggregate per (scenario, planner, num_vehicles) across seeds+vehicles.
    summary = _aggregate(raw_rows)
    with open(summary_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=[
            "scenario", "planner", "num_vehicles", "runs", "success_rate",
            "mean_time_ms", "std_time_ms", "total_time_ms", "mean_path_length",
            "std_path_length", "mean_nodes"])
        w.writeheader()
        w.writerows(summary)

    _print_report(summary)
    print(f"\nwrote {raw_path}  ({len(raw_rows)} rows)")
    print(f"wrote {summary_path}  ({len(summary)} rows)")


def _aggregate(raw_rows):
    groups: dict[tuple, list] = {}
    for r in raw_rows:
        groups.setdefault((r["scenario"], r["planner"], r["num_vehicles"]), []).append(r)

    out = []
    for (scenario, planner, n_veh), rows in groups.items():
        n = len(rows)
        times = [r["plan_time_ms"] for r in rows]
        succ = [r for r in rows if r["success"]]
        lengths = [r["path_length"] for r in succ]
        out.append({
            "scenario": scenario, "planner": planner, "num_vehicles": n_veh,
            "runs": n,
            "success_rate": round(len(succ) / n, 4) if n else 0.0,
            "mean_time_ms": round(statistics.fmean(times), 4) if times else 0.0,
            "std_time_ms": round(statistics.pstdev(times), 4) if len(times) > 1 else 0.0,
            "total_time_ms": round(sum(times), 4),
            "mean_path_length": round(statistics.fmean(lengths), 4) if lengths else 0.0,
            "std_path_length": round(statistics.pstdev(lengths), 4) if len(lengths) > 1 else 0.0,
            "mean_nodes": round(statistics.fmean([r["nodes"] for r in rows]), 1),
        })
    out.sort(key=lambda d: (d["scenario"], d["planner"], d["num_vehicles"]))
    return out


def _print_report(summary):
    print(f"{'scenario':16s} {'planner':9s} {'N':>3s}  "
          f"{'succ':>5s}  {'time_ms':>9s}  {'len_m':>8s}  {'nodes':>7s}")
    print("-" * 70)
    for r in summary:
        print(f"{r['scenario']:16s} {r['planner']:9s} {r['num_vehicles']:3d}  "
              f"{r['success_rate']:5.2f}  {r['mean_time_ms']:9.3f}  "
              f"{r['mean_path_length']:8.2f}  {r['mean_nodes']:7.1f}")


if __name__ == "__main__":
    main()
