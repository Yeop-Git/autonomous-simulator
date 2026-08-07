# Experiment Results (Phase 7)

Research deliverable for the centralized V2X simulator: a comparison of the
path-search algorithms (A\* / RRT / RRT\*) and the LKA lateral controllers,
produced entirely from **headless, logged CSV runs** — no Unity required.

**Reproduce:**

```bash
python experiments/run_algorithm_compare.py   # -> results/algo_compare_{raw,summary}.csv
python experiments/run_lka_test.py            # -> results/lka_{drive_log,summary}.csv
python experiments/run_scene_stats.py         # -> results/scene_stats.csv
python experiments/make_charts.py             # -> results/charts/*.png
python -m pytest server/tests -q --junitxml=experiments/results/pytest.xml
python experiments/validate_results.py        # -> results/manifest.json
```

The tables below are backed by the CSV and JUnit artifacts listed in
`experiments/results/README.md`. The algorithm comparison uses 5 seeds ×
batches of {1, 5, 20}
sequential queries per scenario/planner). Batch prefixes reuse the same fixed
query-generation stream, so they are not independent samples or simultaneous
multi-vehicle interactions. Charts are in `experiments/results/charts/`.

---

## Experiment 1 — A\* vs RRT vs RRT\* (plan §20.1)

Three scenarios probe the boundary from plan §15.4 (road-graph search vs.
free-space sampling):

| Scenario | World | What it tests |
|---|---|---|
| `road_open` | clean 2×2 one-way urban grid, no obstacles | structured-road baseline |
| `road_detour` | single straight lane, middle blocked by a hazard | lane graph has **no alternative edge** |
| `obstacle_field` | free corridor cluttered with 6 irregular obstacles | off-graph avoidance |

### Results (largest vehicle count, mean over seeds)

| Scenario | Planner | Success | Compute (ms) | Path len (m) | Tree/expanded nodes |
|---|---|---:|---:|---:|---:|
| road_open | **A\*** | **100%** | **0.36** | 233.8 | 8 |
| road_open | RRT | 100% | 0.05 | 164.3 | 1 |
| road_open | RRT\* | 100% | 456.9 | 168.5 | 1501 |
| road_detour | A\* | **0%** | 0.07 | — | 1 |
| road_detour | **RRT** | **100%** | **0.50** | 112.8 | 60 |
| road_detour | **RRT\*** | **100%** | 553.3 | **96.7** | 1499 |
| obstacle_field | A\* | **0%** | 0.07 | — | 1 |
| obstacle_field | **RRT** | **100%** | **0.99** | 112.2 | 64 |
| obstacle_field | **RRT\*** | **100%** | 739.2 | **97.0** | 1456 |

![A* vs RRT vs RRT*](../experiments/results/charts/algo_compare.png)

### Findings

1. **On the road graph, A\* is the right tool.** It returns a *road-legal*
   route (233.8 m following the one-way grid) in ~0.36 ms in this run. RRT's shorter
   164 m "path" is a straight diagonal that **cuts across the grid and ignores
   lane topology / direction** — not drivable. This is exactly the plan's
   prediction that A\* dominates on structured road networks (§15.4).

2. **A\* cannot handle off-graph obstacles.** In `road_detour` and
   `obstacle_field` the blocked lane leaves the lane graph with no successor
   edge, so A\* fails outright (0% success — it returns empty in <0.1 ms). This
   is the documented A\* limitation (§15.1 단점) and the reason a sampling
   planner is needed for hazard detours and stalled-car avoidance.

3. **RRT is fast; RRT\* is better but hundreds of times costlier.** Both solve
   all sampled obstacle queries in this run. RRT finds a feasible but jagged path (~112–113 m)
   in ~1 ms; RRT\* refines to a shorter, smoother path (~97 m, a **~14%**
   reduction) but spends its full iteration budget doing so (~0.5–0.8 s). Its
   pooled path-length dispersion is also lower (std ≈ 0.2–1.0 m vs RRT's
   6.1–9.1 m). Queries and seeds are pooled, so this is not an independent
   seed-robustness estimate.

4. **Operational implication.** At ~0.5–0.8 s per query, this RRT\*
   implementation does not fit the authored Unity scenes' nominal 40 ms send
   interval. The 20-query total is a sequential batch, so it is not evidence
   about a concurrent multi-vehicle scheduler. **Current policy:** A\* for
   global road routing, RRT for online obstacle detours, and RRT\* for offline
   path-quality comparison.

![Planning load vs vehicle count](../experiments/results/charts/algo_time_vs_vehicles.png)

---

## Experiment 2 — LKA lateral control (plan §20.2)

Single deterministic curved-track runs (R=140 m, no noise or delay) of the three
lateral controllers at 40/60/80/100 km/h (`experiments/run_lka_test.py`). This
is not a measurement of the Unity `LKA_Test` scene. RMS lateral error (m):

| Speed (km/h) | Pure Pursuit | Stanley | PID |
|---:|---:|---:|---:|
| 40 | 0.089 | 0.041 | 0.016 |
| 60 | 0.113 | 0.055 | 0.014 |
| 80 | 0.135 | 0.049 | **0.239** |
| 100 | 0.153 | 0.058 | **0.303** |

![LKA lateral error vs speed](../experiments/results/charts/lka_lateral_error.png)

### Findings

1. **Stanley has the lowest speed sensitivity in this synthetic sweep** with
   the default (untuned) gains: RMS error stays ~0.04–0.06 m. Robustness to
   noise, initial offset, curvature changes, or steering delay was not tested.
2. **Pure Pursuit degrades with speed** (0.089 → 0.153 m). Its lookahead is
   speed-dependent (`4.0 + 0.4v`), so the result is attributed only to the
   current lookahead law and untuned gains. There were zero departures in this run.
3. **PID has the smallest low-speed error, but its error increases markedly at
   high speed** (0.016 m at 40 km/h → 0.30 m at 100 km/h). This run does not
   establish instability; it records the result of the current pure
   error-feedback gains without curvature feed-forward.
4. **No lane departures** were observed for any controller on this radius.
   This single deterministic condition is not a safety result; noise, initial
   offset, curvature changes, steering delay, and Unity physics still require validation.

---

## Experiment 3 — headless scene workload

`python experiments/run_scene_stats.py` drives reduced headless counterparts of
all five authored scenes and records Python `CentralController.step()` timing,
same-lane centre gaps, reported TTC, saturated deceleration episodes, behaviour
changes, and fixed-horizon arrivals. The CSV is
`experiments/results/scene_stats.csv`.

This is a model-in-the-loop workload baseline, not end-to-end latency: WebSocket,
JSON/schema processing, Unity command application, physics, and rendering are
outside the timed region. Each scene is run once, so p50/p95 describe ticks in
one run rather than a confidence interval across runs. See the root README §11.3
and `experiments/results/README.md` for the table and provenance.

---

## Reproducibility notes

- All sampling planners are **seeded** (`RRTConfig.seed`); the runner sweeps
  seeds `0..4` and reports mean ± population std.
- RRT iteration budget is 3000 (it stops at first goal connection, so this is
  cheap); RRT\* is 1500 (it refines for the whole budget). These live at the
  top of `run_algorithm_compare.py`.
- CSV schema for the algorithm comparison is defined in the runner; the LKA
  drive log uses the frozen plan-§21.1 schema via `server/logging_csv.py`.
- Exact artifact scope, environment, row counts, and hashes are recorded in
  `experiments/results/README.md` and `experiments/results/manifest.json`.
