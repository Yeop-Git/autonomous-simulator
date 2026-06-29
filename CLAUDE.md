# CLAUDE.md — Project Context for Claude Code

This file gives Claude Code the standing context for this repository.
Read it before making changes.

## What this project is

A **centralized V2X autonomous-driving simulator**. We assume a perfect
V2X world: a central server knows the exact position/velocity/acceleration
of every vehicle, pedestrian, obstacle, and emergency vehicle in real time.
Camera/LiDAR perception is intentionally **out of scope**. The focus is:

1. Central global situation awareness
2. Multi-vehicle path planning & collision avoidance
3. Highway vs. urban driving strategy comparison
4. Path-search algorithm comparison (A*, RRT, RRT*)
5. LKA / ADAS lateral & longitudinal control
6. Event-driven extensions (pedestrians, obstacles, emergency vehicles)

Pipeline (simplified, no perception):
`central state collection -> risk prediction -> (re)planning -> behavior decision -> vehicle control -> Unity`

## Architecture

- **unity/** — Unity (C#). Visualization + vehicle kinematics/physics.
  Sends world state every tick, applies control commands. The Unity editor
  scene work (roads, lanes, intersections) is done by a human, NOT by code.
- **server/** — Python central control server. Owns the world model,
  planning, collision prediction, traffic control, behavior decisions.
- **experiments/** — experiment runners + analysis notebooks.
- **shared/protocol/** — **single source of truth** for the wire format.
  JSON Schemas for the two message types. If you change a message shape,
  update the schema here AND both sides (Unity V2XClient + Python server).
- **docs/** — design plan and protocol notes.

## Communication contract (critical)

Two messages, defined in `shared/protocol/`:

- **Unity -> Python**: `state_message.schema.json` (full world snapshot per tick)
- **Python -> Unity**: `command_message.schema.json` (per-vehicle commands)

Transport: WebSocket (recommended). Both messages carry `time` and `tick`
so each side can detect lag / dropped frames. **Never let the two sides
drift out of sync silently** — this is the project's #1 risk.

Coordinates: Unity world space, meters. `heading` is yaw in degrees,
0 = +Z axis, increasing clockwise (Unity convention).

## Build order (follow this; do not jump ahead)

The single most important early goal is a **vertical slice**: one car going
`Unity -> server -> A* -> Unity` end to end. Get the WebSocket loop and
time-sync solid BEFORE adding features.

- Phase 1: Unity waypoint driving + road/lane graph
- Phase 2: Python A* server + Unity-Python comms  <-- prove the vertical slice here
- Phase 3: multi-vehicle + collision prediction + following
- Phase 4: LKA/ADAS test track (Pure Pursuit / Stanley)
- Phase 5: highway scenarios (merge / lane change / falling object)
- Phase 6: urban scenarios (intersections / signals / pedestrians)
- Phase 7: A*/RRT/RRT* comparison experiments + logging + report

## Conventions

- Python: keep planners pure/stateless where possible (input graph+endpoints
  -> path). Side-effecting world state lives in `world_model.py`.
- Each planner in `server/planners/` exposes the same interface so the
  experiment runner can swap them: `plan(start, goal, world) -> list[vec3]`.
- Each controller in `server/controllers/` is independently testable with
  synthetic lane data (no Unity needed).
- Logging schema is fixed — see `docs/` and keep the CSV columns stable so
  analysis notebooks don't break.

## What Claude Code is good for here

Python server (planners, collision prediction, reservation, merge logic),
Unity C# scripts (VehicleController, V2XClient, FSM, logging), experiment
runners, analysis notebooks. These are well-specified and fast to generate.

## What needs a human (do not try to automate)

Unity editor scene construction, road/lane geometry placement, controller
gain tuning (Pure Pursuit lookahead, Stanley gain, PID), and Unity-Python
timestep sync debugging. Flag these to the user rather than guessing.
