# Unity Setup & Scene-Assembly Guide (Unity 6 / 6000.0.42f1)

This document describes the Unity side of the simulator: what Claude Code has
already written (C# scripts + project config), and the **human tasks** —
editor scene construction, prefab wiring, and gain tuning — that finish each
phase. It is matched to **Unity 6000.0.42f1**.

> Division of labor (per `CLAUDE.md`): Claude writes C# scripts and the Python
> server; the human builds scenes, places road/lane geometry, tunes controller
> gains, and does final timestep-sync debugging.

---

## 1. What's already provided (code)

Pinned project config:

- `unity/ProjectSettings/ProjectVersion.txt` → `6000.0.42f1`
- `unity/Packages/manifest.json` → adds **Newtonsoft.Json**
  (`com.unity.nuget.newtonsoft-json`) plus UGUI / Input System / core modules.

Scripts under `unity/Assets/Scripts/`:

| Script | Role |
|---|---|
| `Communication/Messages.cs` | Wire DTOs mirroring `shared/protocol/*.schema.json` |
| `Communication/V2XClient.cs` | WebSocket loop; sends StateMessage, applies CommandMessage; sync warnings |
| `Road/Lane.cs` | A lane: centerline (child waypoints), width, speed limit, neighbour links |
| `Road/RoadNetworkManager.cs` | Lane registry; nearest-lane lookup |
| `Vehicle/VehicleController.cs` | Kinematic bicycle vehicle; applies commands; integrates motion |
| `Vehicle/LKAController.cs` | Lateral laws (Pure Pursuit / Stanley) + lateral/heading error |
| `Sim/SimulationManager.cs` | Gathers world → StateMessage; dispatches commands (the bridge) |
| `Sim/DynamicObjectAgent.cs` | Reports pedestrians / obstacles to the server |
| `Visualization/PathVisualizer.cs` | Draws the server route with a LineRenderer |
| `UI/DebugDashboard.cs` | On-screen HUD: connection, tick, lag, per-vehicle state |
| `Editor/LaneNetworkExporter.cs` | Menu **V2X ▸ Export Lane Network…** → server JSON |

### Why Newtonsoft and not JsonUtility

The protocol uses jagged arrays (`path = [[x,y,z], …]`, vec3 lists).
Unity's built-in `JsonUtility` **cannot serialize jagged/nested arrays**, so
the client uses Newtonsoft.Json. Nothing else is needed — it resolves
automatically from `manifest.json` on first open.

---

## 2. First open (one-time)

1. **Add the project in Unity Hub** → *Add ▸ Add project from disk* → pick the
   `unity/` folder. Hub should show editor version **6000.0.42f1**. Open it.
2. Unity generates the rest of `Library/`, `ProjectSettings/`, and the package
   lock on first import (this is normal; those are git-ignored).
3. Confirm Newtonsoft resolved: **Window ▸ Package Manager ▸ In Project** lists
   *Newtonsoft Json*. If not, *Add package by name* →
   `com.unity.nuget.newtonsoft-json`.
4. Check the Console compiles with no errors. The scripts target the default
   `Assembly-CSharp`; the exporter lives in an `Editor/` folder so it compiles
   into the editor assembly automatically.

---

## 3. Data flow (how a frame works)

```
FixedUpdate (Unity)
  └─ V2XClient asks SimulationManager.CollectState(tick, time)
        → builds StateMessage (vehicles + objects, current_lane tagged)
        → JSON (Newtonsoft) → WebSocket → Python server
Python server
  └─ validates, runs central control (A* route, collision pred, ACC)
        → CommandMessage (target_speed, behavior, path) → back over WebSocket
V2XClient.ReceiveLoop → queue → DrainCommands (main thread)
  └─ SimulationManager.Apply(cmd) → VehicleController.ApplyCommand
        → VehicleController integrates motion (LKA steers along path)
```

`tick`/`time` ride on every message both ways. The HUD shows lag; the server
logs out-of-order / gap / duplicate ticks. **Watch these — silent clock drift
is the project's #1 risk.**

---

## 4. Build the vertical-slice scene (Phase 1–2, human task)

Goal: one car drives a hand-authored lane to a goal via the server's A*.

### 4.1 Road & lanes
1. New scene `Assets/Scenes/LKATestTrack.unity` (or HighwayScene).
2. Create an empty `RoadNetwork` GameObject; add **RoadNetworkManager**.
3. For each lane:
   - Empty GameObject named e.g. `lane_0`; add the **Lane** component.
   - Add empty child GameObjects as **waypoints**, ordered start→end, placed
     along the lane centerline. (Lane auto-uses children if `waypoints` is
     empty.) The cyan gizmo line shows the centerline.
   - Set `width`, `speedLimit` (m/s: 13.9≈50, 27.8≈100 km/h).
   - Link `nextLanes` (forward successors) and `leftLane`/`rightLane`
     (adjacent, for later lane changes).
4. For a first test a single straight lane with 2 segments (`lane_0_a` →
   `lane_0_b`) is enough to exercise A*.

### 4.2 Export the lane graph to the server
- Menu **V2X ▸ Export Lane Network…** → save to
  `server/scenarios/<scene>_lanes.json`. This matches
  `shared/protocol/lane_network.schema.json` and is what the server loads.

### 4.3 Vehicle
1. Create a `Vehicle` GameObject (a stretched cube is fine to start); face it
   along the lane (its +Z is forward).
2. Add **VehicleController**; set `vehicleId` (e.g. `car_01`), `wheelBase`,
   `maxSpeed`. Assign a `goal` Transform (an empty placed at the destination)
   — this sets `has_goal=true`.
3. (Optional) Add **PathVisualizer** to draw the returned route.

### 4.4 Bridge + client
1. On a `SimulationManager` GameObject add **SimulationManager**; set
   `scenario` (`highway`/`urban`/`lka_test`); assign `road`. Leave vehicle /
   object lists empty to auto-collect at Start.
2. Add **V2XClient**; set `serverUrl` (`ws://localhost:8765`). Assign **the
   SimulationManager** to BOTH `stateProviderSource` and `commandSinkSource`.
3. (Optional) Add **DebugDashboard**, assign the V2XClient.

### 4.5 Run
1. Start the server with the exported network:
   ```
   cd server
   python main.py --network scenarios/<scene>_lanes.json
   ```
2. Press **Play**. Expect `[V2XClient] connected`, the car routing to its goal
   along the A* path, and the HUD showing matching ticks with low lag.

**Exit gate (Phase 2):** pick a goal → car drives the A* route → no stale /
out-of-order warnings for a full run.

---

## 5. Later phases (what the human adds)

- **Phase 3 (multi-vehicle):** duplicate the Vehicle prefab to several cars
  with distinct `vehicleId`s and goals; a multi-spawn layout. Server already
  does following / collision prediction (`headless_sim` validates the logic).
- **Phase 4 (LKA track):** straight + curved test-track scenes; tune
  `lookaheadBase`/`lookaheadK` (Pure Pursuit) or `stanleyGain` (Stanley).
  Gains are iterative human work; Claude provides plots/harness, not final
  numbers.
- **Phase 5 (highway):** mainline + ramp geometry, link lanes for merges.
- **Phase 6 (urban):** intersections/crosswalks; place **DynamicObjectAgent**
  on pedestrians; add traffic-light geometry.

---

## 6. Gain-tuning notes (human-in-the-loop)

The controllers expose tunables in the inspector. Start points:

| Param | Where | Start | Notes |
|---|---|---|---|
| `lookaheadBase` | VehicleController | 4 m | larger = smoother, lazier turn-in |
| `lookaheadK` | VehicleController | 0.4 | scales lookahead with speed |
| `stanleyGain` | LKAController | 1.5 | higher = tighter tracking, can oscillate |
| `wheelBase` | VehicleController | 2.7 m | match the visual car length |
| `maxAccel/maxDecel` | VehicleController | 3 / 6 | ride feel vs. responsiveness |

Tune at the test speeds 40/60/80/100 km/h (11.1 / 16.7 / 22.2 / 27.8 m/s) and
watch `latErr` on the HUD.

---

## 7. Constraints & gotchas

- **Transport:** `ClientWebSocket` works in the Editor and standalone builds,
  **not WebGL**. A WebGL build needs a JS WebSocket bridge (out of scope now).
- **Heading convention:** yaw degrees, `0 = +Z`, clockwise — this is Unity's
  native `transform.eulerAngles.y`, so no conversion is needed.
- **Schema lockstep:** any message change updates
  `shared/protocol/*.schema.json`, `Messages.cs`, AND the Python server in the
  **same commit**.
- **Send cadence:** `V2XClient.sendEveryNTicks` throttles state sends if
  FixedUpdate is faster than you want server traffic; keep it `1` for the
  slice.
- **.meta files:** commit the `.meta` files Unity generates for the new
  scripts (the `.gitignore` already tracks `Assets/**/*.meta`).
