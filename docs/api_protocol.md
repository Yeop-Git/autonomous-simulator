# API / Communication Protocol

The Unity client and Python server exchange two JSON messages over a
WebSocket. The authoritative definitions are the JSON Schemas in
`shared/protocol/`. This document is the human-readable companion.

## Transport

- WebSocket, default `ws://localhost:8765`.
- One **StateMessage** per simulation tick: Unity -> Python.
- One **CommandMessage** in response: Python -> Unity.
- Both carry `time` (seconds) and `tick` (monotonic int) for sync.

## Coordinate & unit conventions

- Positions/velocities are Unity world space, **meters** (and m/s).
- Vectors are `[x, y, z]`.
- `heading` is yaw in **degrees**, 0 = +Z axis, increasing clockwise.
- `target_speed` is m/s; `0` means full stop.

## StateMessage (Unity -> Python)

Full snapshot of every dynamic object plus any events this tick.
See `shared/protocol/state_message.schema.json`. Top-level keys:
`time`, `tick`, `scenario`, `vehicles[]`, `objects[]`, `events[]`.

## CommandMessage (Python -> Unity)

Per-vehicle control output. See
`shared/protocol/command_message.schema.json`. Each command has
`vehicle_id`, `target_speed`, `target_lane`, `behavior`, optional `path`,
and `lka_enabled`.

## Sync rules (do not skip)

1. Server echoes the `time`/`tick` it is responding to.
2. Unity warns if a returned command lags more than N ticks behind.
3. Server warns on out-of-order or duplicate ticks.

These checks are deliberately loud — silent drift between the two clocks is
the single biggest source of "it worked yesterday" bugs in this project.

## Changing the protocol

Edit the schema in `shared/protocol/` first, then update **both**:
- Python: message construction/validation in `server/`,
- Unity: `Assets/Scripts/Communication/Messages.cs`.

Keep them in lockstep within a single commit.
