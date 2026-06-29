# IMPLEMENTATION_PLAN.md — Strategy & Delegation Plan for Claude Code

This is the master plan Claude Code follows to build the project. Read
`CLAUDE.md` first for standing context, then work this document phase by
phase. Do **not** jump ahead: each phase has an exit gate that must pass
before the next begins.

The repo already contains a scaffold (folders, `.gitignore`/`.gitattributes`,
protocol schemas, a vertical-slice server stub, and a Unity `V2XClient`
stub). None of the stub runtime code has been executed yet — **verifying and
completing it is Phase 2's job, not an assumption.**

---

## 0. Operating principles

- **Vertical slice first.** The single most important early outcome is one
  car going `Unity -> server -> A* -> Unity` with stable time-sync. Features
  wait until that loop is proven.
- **Protocol is the contract.** `shared/protocol/*.schema.json` is the single
  source of truth. Any message change updates the schema AND both sides in
  the same commit.
- **Human vs. automation split.** Claude Code writes Python + C# scripts,
  experiment runners, and analysis. The human does Unity editor scene work,
  controller gain tuning, and final timestep-sync debugging. When a task
  crosses into the human column, stop and flag it with a precise ask.
- **Each phase ends with a runnable artifact and an exit gate.** No "it
  should work" — run it, show output, then proceed.
- **Keep planners/controllers pure.** They must be unit-testable without
  Unity, using synthetic data.

---

## 1. Milestones at a glance

| Phase | Outcome | Exit gate |
|------|---------|-----------|
| 0 | Repo initialized, server boots | `git` clean; `python main.py` listens; pytest scaffold runs |
| 1 | Unity car follows waypoints; lane graph exists | 1 car drives a hand-built lane in editor |
| 2 | **Vertical slice**: Unity↔server↔A*↔Unity | car routes to a goal via server A*, sync stable |
| 3 | Multi-vehicle + collision prediction + following | N cars, no collisions, following holds gaps |
| 4 | LKA/ADAS test track (Pure Pursuit / Stanley) | lateral_error logged; lane departures countable |
| 5 | Highway scenarios (merge / lane change / hazard) | merge reservation + lane change demo runs |
| 6 | Urban scenarios (intersections / signals / peds) | signalized + reservation intersection demo runs |
| 7 | A*/RRT/RRT* comparison + logging + report | CSV logs + comparison charts + written report |

---

## 2. Phase detail

Each phase lists: **goal**, **Claude Code tasks**, **human tasks**,
**exit gate**.

### Phase 0 — Foundation
**Goal:** repo runs end-to-end empty.
**Claude Code:**
- Verify the server stub boots and validates messages (write a tiny Python
  fake-Unity client that sends one schema-valid StateMessage and asserts a
  schema-valid CommandMessage comes back).
- Add `pytest` smoke tests: schema files load; round-trip a sample message.
- Confirm `.gitignore`/LFS are correct; produce the first commit plan.
**Human:**
- `git init`, `git lfs install`, create remote, first push.
**Exit gate:** `python main.py` + fake client round-trips cleanly; `pytest`
green.

### Phase 1 — Unity waypoint driving + lane graph
**Goal:** one car follows a hand-authored lane; lane/road data structures
exist on both sides.
**Claude Code:**
- `unity/.../Road/Lane.cs`, `RoadNetworkManager.cs`: lane = id, centerline
  waypoints, width, speed_limit, left/right/next lane ids (per plan §10.1).
- `unity/.../Vehicle/VehicleController.cs`: kinematic waypoint follower.
- Python `world_model.py`: lane-graph representation matching the
  `World` protocol in `planners/base.py`; a loader for lane data exported
  from Unity (define the export format).
**Human:**
- Build a minimal scene with one lane and place waypoints in the editor.
- Tune vehicle speed/turn feel.
**Exit gate:** car drives the authored lane smoothly in the editor.

### Phase 2 — A* server + comms (VERTICAL SLICE)
**Goal:** Unity sends start+goal, Python returns an A* route, car follows it.
**Claude Code:**
- Implement `planners/astar.py` against the `World` protocol (lane-graph A*,
  stitch centerlines into a waypoint path). Unit-test with synthetic graph.
- Flesh out `V2XClient` send/receive: real StateMessage from scene, apply
  returned `path` to the vehicle.
- Add a route-request path (how Unity asks for a goal) and visualize the
  returned route.
- Harden sync: tick echo, stale-command warning, reconnect handling.
**Human:**
- Editor wiring of V2XClient + route visualization.
- **Timestep-sync debugging** if Unity physics and server cadence drift.
**Exit gate:** pick a goal in-editor → car drives the A* route → no stale-
command or out-of-order warnings during a full run. **This is the project's
key risk-retirement moment.**

### Phase 3 — Multi-vehicle + collision prediction + following
**Goal:** many cars share lanes safely.
**Claude Code:**
- `MultiVehicleManager` (spawn/route N cars).
- `collision_predictor.py`: sample predicted trajectories
  (horizon 3–5 s, dt 0.1–0.2 s), compute TTC / min-distance (plan §12.3).
- Following behavior (ACC-like gap keeping) + FSM states (plan §12.2).
- Conflict resolution table (plan §12.4) for same-lane slowdowns.
**Human:** scene with multiple spawn points; sanity-watch behavior.
**Exit gate:** 5–20 cars to random goals, zero collisions, gaps maintained.

### Phase 4 — LKA / ADAS test track
**Goal:** measurable lateral control.
**Claude Code:**
- `controllers/`: Pure Pursuit, Stanley, PID lateral controllers behind one
  interface; simple ACC for longitudinal.
- lateral_error / heading_error / curvature computation from lane centerline
  (plan §10.1). CSV logging per the fixed schema (plan §21.1).
- Noise modes (Full V2X / Noisy V2X / Local) as a toggle (plan §11.4).
**Human:**
- Build straight + curved test track scenes.
- **Controller gain tuning** (lookahead, Stanley gain, PID) — iterative,
  human-in-the-loop. Claude Code provides a tuning harness + plots, not final
  gains.
**Exit gate:** lateral_error and lane-departure count logged across speeds
40/60/80/100; plots generated.

### Phase 5 — Highway scenarios
**Goal:** highway behaviors and central-control advantage.
**Claude Code:**
- Merge reservation system (plan §13.3), lane-change with gap acceptance +
  collision prediction, hazard events (falling object, sudden stop) with
  replanning, emergency-vehicle yielding.
- Highway metrics (plan §6.4): avg speed, gaps, TTC min, lane departures,
  lane-change success, merge success, replan time.
**Human:** highway map (mainline + ramp) in editor; tune merge feel.
**Exit gate:** merge + lane-change + one hazard event run and log metrics.

### Phase 6 — Urban scenarios
**Goal:** intersections, signals, pedestrians.
**Claude Code:**
- `TrafficLightManager`, reservation-based `IntersectionManager`
  (plan §13.1–13.2), pedestrian spawner + predicted-trajectory yield/stop,
  sudden-crossing event, stopped-vehicle + construction replanning.
- Urban metrics (plan §7.4): arrival time, intersection wait, near-misses,
  pedestrian risk events, signal violations, throughput.
**Human:** urban map with intersections/crosswalks; pedestrian paths.
**Exit gate:** signalized AND reservation intersection demos run with peds.

### Phase 7 — Algorithm comparison + report
**Goal:** the actual research deliverable.
**Claude Code:**
- `planners/rrt.py`, `planners/rrt_star.py` behind the same interface.
- `experiments/` runners (plan §20): A* vs RRT vs RRT*; highway LKA;
  highway merge; urban intersection; hazard response. CSV logging + variance
  across seeds.
- `analysis.ipynb`: charts (plan §21.3) + `docs/experiment_results.md`.
**Human:** run the experiment matrix; review the report.
**Exit gate:** comparison charts + written results doc produced from logged
CSVs.

---

## 3. Cross-cutting requirements

- **Logging schema is frozen** (plan §21.1). Keep CSV columns stable so
  notebooks don't break:
  `time,vehicle_id,scenario,position_x,position_z,speed,lane_id,behavior_state,lateral_error,heading_error,target_speed,collision_risk,ttc,event_type`
- **Planner interface is fixed** (`planners/base.py`): `plan(start, goal,
  world) -> list[vec3]`. RRT/RRT* must conform so the runner can swap them.
- **Controller interface is fixed**: one lateral controller interface so
  Pure Pursuit / Stanley / PID are interchangeable in experiments.
- **Every Python module gets a unit test** that runs without Unity.

---

## 4. Risk register

| Risk | Phase | Mitigation |
|------|-------|-----------|
| Unity↔Python timestep drift | 2 | tick echo + stale warnings; retire risk in the vertical slice before anything else |
| Controller tuning eats time | 4 | Claude Code supplies tuning harness + plots; human tunes; don't block other phases on perfect gains |
| Scope creep (perception, MPC) | all | perception out of scope; MPC is optional/last |
| Schema drift between sides | all | single source of truth + same-commit rule |
| Multi-vehicle deadlocks at intersections | 3,6 | reservation logic + timeout/yield fallback |

---

## 5. How to delegate this to Claude Code

Recommended working rhythm, one phase per working session:

1. Start the session by pointing Claude Code at `CLAUDE.md` and this file.
2. Ask it to implement the **current phase's Claude Code tasks only**, with
   unit tests, and to run them.
3. When it hits a **human task** (editor work, tuning), it stops and gives
   you a precise, minimal instruction.
4. Close the phase only when the **exit gate** passes; commit.
5. Move to the next phase.

Suggested first prompt to Claude Code:
> "Read CLAUDE.md and IMPLEMENTATION_PLAN.md. Execute Phase 0: verify the
> server stub, write the fake-Unity round-trip client, add pytest smoke
> tests, and run everything. Stop at the Phase 0 exit gate and report."
