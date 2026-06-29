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
- [ ] (H) author one-lane scene + place waypoints  ← see docs/unity_setup.md §4
- [ ] (H) tune vehicle motion feel
- [ ] **GATE:** car drives authored lane smoothly

## Phase 2 — A* server + comms (VERTICAL SLICE)
- [x] astar.py implemented + unit-tested on synthetic graph
- [x] V2XClient real StateMessage from scene (Newtonsoft + provider/sink)
- [x] apply returned path to vehicle (VehicleController.ApplyCommand)
- [x] route-request flow (`has_goal`/`goal`) + route visualization (PathVisualizer)
- [x] sync hardening (tick echo, stale/out-of-order/gap/dup warnings)
- [ ] (H) editor wiring + route viz  ← see docs/unity_setup.md §4.4
- [ ] (H) timestep-sync debugging if drift appears
- [~] **GATE:** proven in-process + live via fake client; needs Unity scene to close

## Phase 3 — Multi-vehicle + collision prediction
- [x] central routing of N cars (CentralController) + headless_sim test bed
- [x] collision_predictor.py (TTC / closest-approach, horizon 3–5s)
- [x] following behavior (ACC + safe-speed) + FSM states (behavior.py)
- [x] conflict-resolution (emergency/stop/follow via FSM + ACC)
- [ ] (H) multi-spawn scene
- [x] **GATE (logic):** 6-car train + slow-leader + obstacle = zero collisions (headless tests)

## Phase 4 — LKA / ADAS test track
- [x] lateral controllers: Pure Pursuit / Stanley / PID (`controllers/lateral.py`)
- [x] simple ACC (longitudinal) (`controllers/acc.py`, safe-speed model)
- [x] lateral/heading error + curvature from centerline (`frenet_errors`)
- [x] CSV logging (frozen schema) (`logging_csv.py`, wired into headless sim)
- [x] noise modes (Full / Noisy / Local) (`noise.py`, wired into controller)
- [x] tuning harness (`experiments/run_lka_test.py` sweep 40/60/80/100)
- [ ] (H) straight + curved test tracks  ← Unity scene
- [ ] (H) controller gain tuning
- [x] **GATE (headless):** lateral_error + departures logged at 40/60/80/100 (CSV out)

## Phase 5 — Highway scenarios
- [x] merge reservation system (`merge.py`)
- [x] lane change (gap acceptance + collision prediction) (`lane_change.py`)
- [x] hazard events (falling object) + obstacle/replan (`world_model` events→obstacle)
- [x] emergency-vehicle yielding (`emergency.py`, wired into controller)
- [x] highway metrics (plan §6.4) (`metrics.py`)
- [ ] (H) highway map (mainline + ramp)  ← Unity scene
- [x] **GATE (logic):** lane change + merge + hazard-stop covered by tests

## Phase 6 — Urban scenarios
- [x] TrafficLightManager (`traffic.py`, fixed-cycle + should_stop)
- [x] reservation-based IntersectionManager (`intersection.py`, ETA slots)
- [x] pedestrian predicted-trajectory yield/stop (generic collision prediction)
- [x] hazard/stopped-vehicle handling via events→obstacle + A* reroute
- [x] urban metrics (plan §7.4) (`metrics.py`, shared)
- [ ] (H) urban map (intersections/crosswalks/ped paths)  ← Unity scene
- [x] **GATE (logic):** light cycle + reservation + pedestrian-stop covered by tests

## Phase 7 — Algorithm comparison + report  (NOT STARTED — only sampling core stub)
- [~] rrt.py / rrt_star.py (same planner interface) — `planners/_rrt_common.py` foundation only
- [ ] experiment runners (plan §20) + multi-seed variance
- [ ] CSV logging across experiments
- [ ] analysis.ipynb charts (plan §21.3)
- [ ] docs/experiment_results.md
- [ ] (H) run experiment matrix + review
- [ ] **GATE:** comparison charts + results doc from logged CSVs
