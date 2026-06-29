# TASKS.md — Progress Board

Living checklist. Claude Code updates checkboxes as work lands. Each phase
closes only when its **exit gate** (see IMPLEMENTATION_PLAN.md) passes.

Legend: `[ ]` todo · `[~]` in progress · `[x]` done · `(H)` human task

## Phase 0 — Foundation
- [ ] Verify server stub boots and validates messages
- [ ] Fake-Unity round-trip client (one StateMessage → CommandMessage)
- [ ] pytest smoke tests (schemas load, sample round-trip)
- [ ] (H) git init, git lfs install, first push
- [ ] **GATE:** server + fake client round-trip clean; pytest green

## Phase 1 — Unity waypoint driving + lane graph
- [ ] Lane.cs / RoadNetworkManager.cs (lane model per plan §10.1)
- [ ] VehicleController.cs (kinematic waypoint follower)
- [ ] world_model.py lane graph (matches World protocol)
- [ ] lane-data export format Unity → Python
- [ ] (H) author one-lane scene + place waypoints
- [ ] (H) tune vehicle motion feel
- [ ] **GATE:** car drives authored lane smoothly

## Phase 2 — A* server + comms (VERTICAL SLICE)
- [ ] astar.py implemented + unit-tested on synthetic graph
- [ ] V2XClient real StateMessage from scene
- [ ] apply returned path to vehicle
- [ ] route-request flow + route visualization
- [ ] sync hardening (tick echo, stale warn, reconnect)
- [ ] (H) editor wiring + route viz
- [ ] (H) timestep-sync debugging if drift appears
- [ ] **GATE:** pick goal → car drives A* route → no sync warnings

## Phase 3 — Multi-vehicle + collision prediction
- [ ] MultiVehicleManager (spawn/route N cars)
- [ ] collision_predictor.py (TTC / min-distance, horizon 3–5s)
- [ ] following behavior + FSM states
- [ ] conflict-resolution handling
- [ ] (H) multi-spawn scene
- [ ] **GATE:** 5–20 cars, zero collisions, gaps held

## Phase 4 — LKA / ADAS test track
- [ ] lateral controllers: Pure Pursuit / Stanley / PID (one interface)
- [ ] simple ACC (longitudinal)
- [ ] lateral/heading error + curvature from centerline
- [ ] CSV logging (frozen schema)
- [ ] noise modes (Full / Noisy / Local)
- [ ] tuning harness + plots
- [ ] (H) straight + curved test tracks
- [ ] (H) controller gain tuning
- [ ] **GATE:** lateral_error + departures logged at 40/60/80/100; plots

## Phase 5 — Highway scenarios
- [ ] merge reservation system
- [ ] lane change (gap acceptance + collision prediction)
- [ ] hazard events (falling object, sudden stop) + replanning
- [ ] emergency-vehicle yielding
- [ ] highway metrics (plan §6.4)
- [ ] (H) highway map (mainline + ramp)
- [ ] **GATE:** merge + lane change + one hazard event logged

## Phase 6 — Urban scenarios
- [ ] TrafficLightManager
- [ ] reservation-based IntersectionManager
- [ ] pedestrian spawner + predicted-trajectory yield/stop
- [ ] sudden crossing, stopped vehicle, construction replanning
- [ ] urban metrics (plan §7.4)
- [ ] (H) urban map (intersections/crosswalks/ped paths)
- [ ] **GATE:** signalized + reservation intersection demos with peds

## Phase 7 — Algorithm comparison + report
- [ ] rrt.py / rrt_star.py (same planner interface)
- [ ] experiment runners (plan §20) + multi-seed variance
- [ ] CSV logging across experiments
- [ ] analysis.ipynb charts (plan §21.3)
- [ ] docs/experiment_results.md
- [ ] (H) run experiment matrix + review
- [ ] **GATE:** comparison charts + results doc from logged CSVs
