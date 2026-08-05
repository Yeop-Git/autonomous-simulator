# EmergencyAvoidance scene implementation plan

## Goal

Add a separate Unity scene that demonstrates two centralized V2X reactions:

1. an unexpected obstacle causes an online A* -> RRT/RRT* local-planner switch,
   lateral avoidance, and safe lane re-entry;
2. an emergency vehicle approaching from behind causes an active right-side
   pull-over, a yielding hold, and a safe return after it passes.

The existing `Highway.unity` and its normal A*/ACC behaviour remain unchanged.
The new scene is named `EmergencyAvoidance.unity` and reports the scenario id
`emergency_avoidance`.

## Safety and scope rules

- A* remains the global road-route planner.
- RRT is the default real-time local avoidance planner.
- RRT* is an experiment/quality mode and must have a strict time or iteration
  budget. It must not block the V2X tick loop indefinitely.
- Sampling is restricted to a drivable corridor made from travel lanes and the
  right shoulder. It may not use the opposite carriageway or leave the road.
- Obstacles are inflated by vehicle half-width plus a configurable safety margin.
- A candidate path must pass collision, road-boundary, curvature, and steering
  feasibility checks before Unity receives it.
- Planning failure degrades to controlled stopping and then emergency braking;
  it never falls back to an unchecked path.

## Target scene layout

Start from a copy of `Assets/Scenes/Highway.unity`.

- three forward travel lanes, reusing the Highway lane geometry where possible;
- one right-side shoulder represented as a real `Lane`/escape corridor but not
  part of ordinary A* routing;
- ego vehicle in the middle or right travel lane;
- two optional background vehicles for front/rear gap constraints;
- one obstacle spawn zone 35-55 m ahead of the ego;
- one emergency-vehicle spawn zone 45-70 m behind the ego;
- a scenario controller with buttons/hotkeys for:
  - spawn falling object;
  - spawn stopped vehicle;
  - dispatch emergency vehicle;
  - select RRT or RRT*;
  - reset scenario;
- a debug panel showing behaviour state, selected planner, planning time,
  replans, minimum clearance, TTC, and emergency-vehicle pass status;
- path visualization with distinct colours for global A*, candidate local path,
  accepted avoidance path, and rejoin path.

## Central decision architecture

Introduce a hybrid local-planning layer in the Python server:

```text
global A* route
  -> hazard/emergency trigger
  -> escape corridor selection
  -> local RRT or budgeted RRT*
  -> feasibility and dynamic-conflict validation
  -> path smoothing/resampling
  -> Unity path command
  -> receding-horizon revalidation/replan
  -> A* route rejoin
```

Recommended new server modules:

- `server/local_avoidance.py`
  - trigger classification;
  - local goal and corridor selection;
  - planner dispatch and time budget;
  - accepted-plan cache and replan policy.
- `server/planners/avoidance_world.py`
  - drivable bounds;
  - inflated static obstacles;
  - predicted dynamic occupancy samples;
  - `is_blocked()` adapter for RRT/RRT*.
- `server/path_postprocess.py`
  - collision-safe shortcutting;
  - waypoint resampling;
  - curvature/steering feasibility checks.

The first version remains spatial and receding-horizon: moving objects are
represented by inflated samples of their predicted 0-4 s trajectories and the
plan is revalidated every tick. A full space-time RRT is intentionally deferred.

## Behaviour state machine

Add the following command behaviours to the shared protocol and both endpoints:

```text
LaneKeeping
  -> HazardDetected
  -> EscapePlanning
  -> LateralEvading
  -> Yielding
  -> RejoinPlanning
  -> LaneRejoining
  -> LaneKeeping
```

Failure path:

```text
EscapePlanning -> ControlledStopping -> EmergencyBraking
```

Emergency-vehicle policy:

1. detect an approaching `emergency_vehicle` within the V2X trigger radius;
2. rank right travel lane and shoulder escape targets;
3. reject targets with unsafe predicted front/rear occupancy;
4. plan and execute a lateral pull-over;
5. hold at crawl speed or zero while the emergency vehicle passes;
6. require both longitudinal clearance and a short stable-clear timer;
7. plan a return to the original A* route.

## Unity runtime additions

Recommended scripts:

- `Assets/Scripts/Sim/EmergencyAvoidanceScenarioController.cs`
  - deterministic scenario triggers, reset, planner-mode selection;
  - registers spawned `DynamicObjectAgent` instances with `SimulationManager`.
- `Assets/Scripts/Sim/EmergencyVehicleMover.cs`
  - moves the emergency vehicle along its lane at a configured speed;
  - reports motion through `DynamicObjectAgent` with type `emergency_vehicle`.
- `Assets/Scripts/UI/EmergencyAvoidanceDashboard.cs`
  - presents planner/FSM/timing/safety diagnostics.

The scenario controller should use fixed seeds and repeatable spawn positions so
RRT/RRT* comparisons are reproducible.

## Protocol additions

Extend `command_message.schema.json` and `Messages.cs` together with:

- the new behaviour enum values;
- optional `planner` (`astar`, `rrt`, `rrt_star`);
- optional `plan_status`;
- optional `planning_time_ms`;
- optional `minimum_clearance`.

Keep these diagnostics optional so existing scenes remain compatible.

## Unity MCP execution workflow

The implementation should use Unity MCP in this order:

1. `Unity.GetUserGuidelines`, `Unity.GetProjectData`, `Unity.ManageEditor(GetState)`.
2. Stop Play Mode before editing.
3. Inspect `Highway` hierarchy and relevant component serialization.
4. Use `Unity.RunCommand` with `AssetDatabase.CopyAsset` to copy Highway to
   `Assets/Scenes/EmergencyAvoidance.unity`; never overwrite Highway.
5. Load the new scene and save it explicitly.
6. Create/position roots and GameObjects using `Unity.ManageGameObject` or a
   bounded `Unity.RunCommand`; register Undo/result changes.
7. Assign `Lane`, `SimulationManager`, V2X, camera, UI, spawn-controller, and
   object references through serialized component properties.
8. Export the lane network non-interactively with
   `LaneNetworkExporter.ExportToDefaultLocation()`.
9. Add the new scene to Build Settings with a bounded editor command.
10. Validate scripts, wait for compilation, and inspect all Console errors.
11. Capture a multi-angle Scene view to verify lane/shoulder/object placement.
12. Start Play Mode, trigger each deterministic scenario, inspect the Console,
    and capture the Game camera/debug panel.
13. Stop Play Mode and verify that scene/prefab changes were saved.

## Implementation phases and gates

### Phase A - server-only vertical slice

- Add avoidance world, RRT dispatch, path validation, and FSM.
- Use a synthetic straight-road test with one obstacle and shoulder.
- Gate: RRT returns a collision-free, in-corridor path within the configured
  budget; failure yields `ControlledStopping`.

### Phase B - unexpected obstacle Unity slice

- Build the new scene with Unity MCP and add deterministic obstacle spawning.
- Gate: ego transitions through avoidance and rejoin without collision or road
  departure; the existing Highway scene still runs normally.

### Phase C - emergency pull-over

- Add moving emergency vehicle, target-corridor selection, pass detection, and
  safe rejoin.
- Gate: emergency vehicle passes without TTC entering the emergency threshold,
  and ego does not return before the clearance timer expires.

### Phase D - RRT/RRT* comparison

- Add planner toggle and repeatable seeds.
- Log success, planning time, path length, minimum clearance, maximum lateral
  acceleration, and replan count.
- Gate: RRT respects the real-time budget; RRT* either returns within its budget
  or safely reports timeout without stalling control.

### Phase E - integrated regression

- Run Python planner/FSM/protocol tests.
- Compile Unity scripts and check Console errors.
- Play-test obstacle and emergency scenarios using Unity MCP.
- Re-open Main, Highway, Urban, and LKA_Test to check protocol compatibility.

## Acceptance criteria

- No edit or overwrite of the original Highway scene.
- No path point outside the configured drivable/shoulder corridor.
- No accepted path intersects an inflated static or predicted dynamic obstacle.
- Planner timeout results in controlled stopping, not stale-path continuation.
- Emergency pull-over includes lateral movement, hold, pass detection, and rejoin.
- All command messages echo the source tick/time and remain schema-valid.
- The scenario can be reset and replayed deterministically.
- Unity Console contains no compile or runtime errors during both demonstrations.
- Visual captures show the scene layout and both completed manoeuvres.

## Deliberately deferred work

- full space-time/kinodynamic RRT;
- learned intent prediction;
- automatic controller-gain optimisation;
- physically detailed tyre/suspension modelling;
- arbitrary-map shoulder inference beyond explicitly authored escape corridors.
