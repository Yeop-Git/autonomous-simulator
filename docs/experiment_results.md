# Experiment Results (Phase 7)

Research deliverable for the centralized V2X simulator: a comparison of the
path-search algorithms (A\* / RRT / RRT\*) and the LKA lateral controllers,
produced entirely from **headless, logged CSV runs** — no Unity required.

**Reproduce:**

```bash
python experiments/run_algorithm_compare.py   # -> results/algo_compare_{raw,summary}.csv
python experiments/run_lka_test.py            # -> results/lka_{drive_log,summary}.csv
python experiments/make_charts.py             # -> results/charts/*.png
# or open experiments/analysis.ipynb for the interactive version
```

All numbers below are from `experiments/results/algo_compare_summary.csv` and
`experiments/results/lka_summary.csv` (5 seeds × {1, 5, 20} vehicles per
scenario/planner). Charts are in `experiments/results/charts/`.

---

## Experiment 1 — A\* vs RRT vs RRT\* (plan §20.1)

Three scenarios probe the boundary from plan §15.4 (road-graph search vs.
free-space sampling):

| Scenario | World | What it tests |
|---|---|---|
| `road_open` | clean 2×2 one-way urban grid, no obstacles | A\*'s home turf |
| `road_detour` | single straight lane, middle blocked by a hazard | lane graph has **no alternative edge** |
| `obstacle_field` | free corridor cluttered with 6 irregular obstacles | off-graph avoidance |

### Results (largest vehicle count, mean over seeds)

| Scenario | Planner | Success | Compute (ms) | Path len (m) | Tree/expanded nodes |
|---|---|---:|---:|---:|---:|
| road_open | **A\*** | **100%** | **0.35** | 233.8 | 8 |
| road_open | RRT | 100% | 0.05 | 164.3 | 1 |
| road_open | RRT\* | 100% | 437.0 | 168.5 | 1501 |
| road_detour | A\* | **0%** | 0.07 | — | 1 |
| road_detour | **RRT** | **100%** | **0.53** | 112.8 | 60 |
| road_detour | **RRT\*** | **100%** | 540.2 | **96.7** | 1499 |
| obstacle_field | A\* | **0%** | 0.07 | — | 1 |
| obstacle_field | **RRT** | **100%** | **1.01** | 112.2 | 64 |
| obstacle_field | **RRT\*** | **100%** | 710.5 | **97.0** | 1456 |

![A* vs RRT vs RRT*](../experiments/results/charts/algo_compare.png)

### Findings

1. **On the road graph, A\* is the right tool.** It returns a *road-legal*
   route (233.8 m following the one-way grid) in ~0.35 ms. RRT's shorter
   164 m "path" is a straight diagonal that **cuts across the grid and ignores
   lane topology / direction** — not drivable. This is exactly the plan's
   prediction that A\* dominates on structured road networks (§15.4).

2. **A\* cannot handle off-graph obstacles.** In `road_detour` and
   `obstacle_field` the blocked lane leaves the lane graph with no successor
   edge, so A\* fails outright (0% success — it returns empty in <0.1 ms). This
   is the documented A\* limitation (§15.1 단점) and the reason a sampling
   planner is needed for hazard detours and stalled-car avoidance.

3. **RRT is fast; RRT\* is better but ~700× costlier.** Both solve the obstacle
   scenarios 100% of the time. RRT finds a feasible but jagged path (~112–113 m)
   in ~1 ms; RRT\* refines to a shorter, smoother path (~97 m, a **~14%**
   reduction) but spends its full iteration budget doing so (~0.5–0.7 s). Its
   path-length variance is also far lower (std ≈ 0.1–1.2 m vs RRT's 5–9 m),
   i.e. more consistent quality across seeds.

4. **Real-time implication (plan §15.3 단점, §3.2 연구질문).** At ~0.5–0.7 s per
   query, RRT\* does **not** scale to real-time multi-vehicle replanning
   (`total_time_ms` grows linearly with vehicle count: 20 obstacle-field
   queries × 5 seeds = ~71 s of planning). RRT at ~1 ms is viable for online
   detours; RRT\* is best reserved for offline path-quality comparison or
   one-off optimization. **Recommended policy:** A\* for global road routing,
   RRT for online obstacle detours, RRT\* for quality benchmarking.

![Planning load vs vehicle count](../experiments/results/charts/algo_time_vs_vehicles.png)

---

## Experiment 2 — LKA lateral control (plan §20.2)

Curved-track sweep of the three lateral controllers at 40/60/80/100 km/h
(`experiments/run_lka_test.py`). RMS lateral error (m):

| Speed (km/h) | Pure Pursuit | Stanley | PID |
|---:|---:|---:|---:|
| 40 | 0.089 | 0.041 | 0.016 |
| 60 | 0.113 | 0.055 | 0.014 |
| 80 | 0.135 | 0.049 | **0.239** |
| 100 | 0.153 | 0.058 | **0.303** |

![LKA lateral error vs speed](../experiments/results/charts/lka_lateral_error.png)

### Findings

1. **Stanley is the most speed-robust** with the default (untuned) gains: RMS
   error stays ~0.04–0.06 m flat across all speeds. This matches its reputation
   as an LKA-style centerline tracker (plan §10.2).
2. **Pure Pursuit degrades with speed** (0.089 → 0.153 m) — the fixed lookahead
   under-steers the curve as speed rises, the classic tuning issue noted in
   §10.2. Still zero lane departures on this gentle curve.
3. **PID is excellent at low speed but unstable at high speed** (0.016 m at
   40 km/h → 0.30 m at 100 km/h). This is the expected limit of a pure
   error-feedback controller on curvature without feed-forward (§10.2 단점).
4. **No lane departures** for any controller on this radius, so gain tuning is
   about ride quality, not safety, here. Final gains are a **human task**
   (IMPLEMENTATION_PLAN Phase 4) — this harness supplies the plots.

---

## Experiments 3–5 — status

| Experiment | Plan | Server-side logic | Blocker |
|---|---|---|---|
| Highway merge control (§20.3) | merge reservation | `server/merge.py` + tests | needs Unity mainline+ramp scene (human) |
| Urban intersection (§20.4) | signal vs reservation | `server/traffic.py`, `intersection.py` + tests | needs Unity intersection scene (human) |
| Hazard response (§20.5) | detect→command latency | events→obstacle→A\* reroute + `collision_predictor` | needs Unity hazard scene (human) |

The **algorithms** for 3–5 are implemented and unit-tested headless
(Phases 5–6). Producing their logged-metric CSVs requires the corresponding
Unity scenes, which are the remaining human tasks in `TASKS.md`. The headless
sim (`server/headless_sim.py`) can drive reduced versions of these once
synthetic scenes are added to `server/scenarios/networks.py`.

---

## Reproducibility notes

- All sampling planners are **seeded** (`RRTConfig.seed`); the runner sweeps
  seeds `0..4` and reports mean ± population std.
- RRT iteration budget is 3000 (it stops at first goal connection, so this is
  cheap); RRT\* is 1500 (it refines for the whole budget). These live at the
  top of `run_algorithm_compare.py`.
- CSV schema for the algorithm comparison is defined in the runner; the LKA
  drive log uses the frozen plan-§21.1 schema via `server/logging_csv.py`.
