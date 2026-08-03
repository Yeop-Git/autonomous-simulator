# TASKS.md — Progress Board

Living checklist. Claude Code updates checkboxes as work lands. Each phase
closes only when its **exit gate** (see IMPLEMENTATION_PLAN.md) passes.

Legend: `[ ]` todo · `[~]` in progress · `[x]` done · `(H)` human task

## Phase 0 — Foundation
- [x] Verify server stub boots and validates messages
- [x] Fake-Unity round-trip client (`server/tools/fake_unity_client.py`)
- [x] pytest smoke tests (schemas load, sample round-trip) — `tests/test_protocol.py`
- [ ] (H) git init, git lfs install, first push
- [x] **GATE:** server + fake client round-trip clean; pytest green (50 tests)

## Phase 1 — Unity waypoint driving + lane graph
- [x] Lane.cs / RoadNetworkManager.cs (lane model per plan §10.1)
- [x] VehicleController.cs (kinematic bicycle + Pure Pursuit/Stanley)
- [x] world_model.py lane graph (matches World protocol)
- [x] lane-data export format Unity → Python (`lane_network.schema.json` + `Editor/LaneNetworkExporter.cs`)
- [x] author one-lane scene + place waypoints — `Assets/Scenes/Main.unity`
- [ ] (H) tune vehicle motion feel
- [x] **GATE:** car drives authored lane smoothly (live Unity↔Python run)

## Phase 2 — A* server + comms (VERTICAL SLICE)
- [x] astar.py implemented + unit-tested on synthetic graph
- [x] V2XClient real StateMessage from scene (Newtonsoft + provider/sink)
- [x] apply returned path to vehicle (VehicleController.ApplyCommand)
- [x] route-request flow (`has_goal`/`goal`) + route visualization (PathVisualizer)
- [x] sync hardening (tick echo, stale/out-of-order/gap/dup warnings)
- [x] editor wiring + route viz — generated and live-validated in `Main.unity`
- [ ] (H) timestep-sync debugging if drift appears
- [x] **GATE:** Unity→server→A*→Unity live vertical slice proven

## Phase 3 — Multi-vehicle + collision prediction
- [x] central routing of N cars (CentralController) + headless_sim test bed
- [x] collision_predictor.py (TTC / closest-approach, horizon 3–5s)
- [x] following behavior (ACC + safe-speed) + FSM states (behavior.py)
- [x] conflict-resolution (emergency/stop/follow via FSM + ACC)
- [x] multi-spawn scene — `Assets/Scenes/Highway.unity`
- [x] **GATE (logic):** 6-car train + slow-leader + obstacle = zero collisions (headless tests)

## Phase 4 — LKA / ADAS test track
- [x] lateral controllers: Pure Pursuit / Stanley / PID (`controllers/lateral.py`)
- [x] simple ACC (longitudinal) (`controllers/acc.py`, safe-speed model)
- [x] lateral/heading error + curvature from centerline (`frenet_errors`)
- [x] CSV logging (frozen schema) (`logging_csv.py`, wired into headless sim)
- [x] noise modes (Full / Noisy / Local) (`noise.py`, wired into controller)
- [x] tuning harness (`experiments/run_lka_test.py` sweep 40/60/80/100)
- [x] straight + curved test tracks — `Main.unity` + `LKA_Test.unity`
- [ ] (H) controller gain tuning
- [x] **GATE (headless):** lateral_error + departures logged at 40/60/80/100 (CSV out)

## Phase 5 — Highway scenarios
- [x] merge reservation system (`merge.py`)
- [x] lane change (gap acceptance + collision prediction) (`lane_change.py`)
- [x] hazard events (falling object) + obstacle/replan (`world_model` events→obstacle)
- [x] emergency-vehicle yielding (`emergency.py`, wired into controller)
- [x] highway metrics (plan §6.4) (`metrics.py`)
- [x] highway map (three-lane mainline + ramp) — `Assets/Scenes/Highway.unity`
- [x] **GATE (logic):** lane change + merge + hazard-stop covered by tests

## Phase 6 — Urban scenarios
- [x] TrafficLightManager (`traffic.py`, fixed-cycle + should_stop)
- [x] reservation-based IntersectionManager (`intersection.py`, ETA slots)
- [x] pedestrian predicted-trajectory yield/stop (generic collision prediction)
- [x] hazard/stopped-vehicle handling via events→obstacle + A* reroute
- [x] urban metrics (plan §7.4) (`metrics.py`, shared)
- [x] urban map (intersection/crosswalks/pedestrian) — `Assets/Scenes/Urban.unity`
- [x] **GATE (logic):** light cycle + reservation + pedestrian-stop covered by tests

## Phase 7 — Algorithm comparison + report
- [x] rrt.py / rrt_star.py (same planner interface) — `planners/rrt.py`, `planners/rrt_star.py` (+ `_rrt_common.py`); unit-tested in `tests/test_rrt.py`
- [x] experiment runner (plan §20.1) + multi-seed variance — `experiments/run_algorithm_compare.py` (5 seeds × {1,5,20} vehicles × 3 scenarios)
- [x] CSV logging across experiments — `results/algo_compare_{raw,summary}.csv` (+ existing LKA §20.2 logs)
- [x] analysis.ipynb charts (plan §21.3) — `experiments/analysis.ipynb` (+ headless `experiments/make_charts.py` → `results/charts/*.png`)
- [x] docs/experiment_results.md — findings for §20.1 (A*/RRT/RRT*) and §20.2 (LKA)
- [ ] (H) run experiment matrix on Unity scenes for §20.3–20.5 (merge/intersection/hazard) — logic done, needs scenes
- [x] **GATE:** comparison charts + results doc produced from logged CSVs (A*/RRT/RRT* + LKA)
