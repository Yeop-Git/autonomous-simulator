# Autonomous V2X Simulation

Centralized V2X autonomous-driving simulator: a Unity front-end for
visualization & vehicle motion, driven by a Python central control server
that knows the full world state (no camera/LiDAR perception). Used to study
multi-vehicle planning, highway vs. urban strategies, A*/RRT/RRT*
comparison, and LKA/ADAS control.

See `docs/project_plan.md` for the full design and `CLAUDE.md` for the
working context Claude Code should read first.

## Layout

```
autonomous-v2x-sim/
├─ unity/              Unity project (C#) — visualization + vehicle motion
│  └─ Assets/Scripts/
│     ├─ Communication/  V2XClient + message classes
│     ├─ Vehicle/        (Phase 1+) controllers, LKA
│     └─ Road/           (Phase 1+) lane graph, road network
├─ server/             Python central control server
│  ├─ main.py            WebSocket server (vertical-slice ready)
│  ├─ planners/          A* (Phase 2), RRT/RRT* (Phase 7)
│  ├─ controllers/       LKA / ACC / behavior (Phase 4+)
│  └─ scenarios/         highway / urban / lka_test
├─ experiments/        experiment runners + analysis notebooks
├─ shared/protocol/    JSON Schemas — single source of truth for messages
└─ docs/               design plan + protocol notes
```

## Quick start

### 1. Python server

```bash
cd server
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python main.py                   # listens on ws://localhost:8765
```

The server validates incoming state against the schema and replies with a
trivial "hold lane, keep speed" command. That is intentional — it exists to
close the loop first.

### 2. Unity

1. Open the `unity/` folder in Unity (recent LTS).
2. Add a `V2XClient` component to a scene GameObject.
3. Press Play. With the Python server running, you should see
   `[V2XClient] connected` and matching tick traffic on both sides.

## Build order

Follow the phases in `CLAUDE.md`. The first milestone is the **vertical
slice**: one car going `Unity -> server -> A* -> Unity` end to end with
stable time-sync. Do not add features until that loop is solid.

## Git notes

- `.gitignore` already excludes Unity `Library/`, `Temp/`, build output, and
  Python `__pycache__/` / `.venv/`.
- Binary Unity assets are routed through **Git LFS** (`.gitattributes`).
  Run `git lfs install` once before your first commit.
