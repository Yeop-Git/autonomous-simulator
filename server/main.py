"""Central V2X control server — minimal vertical-slice version.

Phase 2 starting point. This server:
  1. accepts a WebSocket connection from Unity,
  2. receives a StateMessage every tick,
  3. validates it against shared/protocol/state_message.schema.json,
  4. returns a trivial CommandMessage (hold lane, keep current speed).

The point of this file is to PROVE the Unity <-> Python loop and time-sync
before any real planning exists. Replace the dummy logic in decide() with
calls into world_model / planners / controllers as later phases land.

Run:
    pip install -r requirements.txt
    python main.py
"""
from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path

import jsonschema
import websockets

PROTOCOL_DIR = Path(__file__).resolve().parents[1] / "shared" / "protocol"
HOST = "localhost"
PORT = 8765


def _load_schema(name: str) -> dict:
    with open(PROTOCOL_DIR / name, "r", encoding="utf-8") as f:
        return json.load(f)


STATE_SCHEMA = _load_schema("state_message.schema.json")
COMMAND_SCHEMA = _load_schema("command_message.schema.json")


def decide(state: dict) -> dict:
    """Dummy policy: every vehicle holds its lane at its current speed.

    Replace with real central control. This exists only to close the loop.
    """
    commands = []
    for v in state.get("vehicles", []):
        vel = v.get("velocity", [0.0, 0.0, 0.0])
        speed = math.sqrt(sum(c * c for c in vel))
        commands.append(
            {
                "vehicle_id": v["id"],
                "target_speed": round(speed, 3),
                "target_lane": v.get("current_lane"),
                "behavior": "LaneKeeping",
                "lka_enabled": True,
            }
        )
    return {
        "time": state.get("time", 0.0),
        "tick": state.get("tick", 0),
        "commands": commands,
    }


async def handle(ws):
    print(f"[server] Unity connected: {ws.remote_address}")
    last_tick = -1
    try:
        async for raw in ws:
            try:
                state = json.loads(raw)
                jsonschema.validate(state, STATE_SCHEMA)
            except (json.JSONDecodeError, jsonschema.ValidationError) as e:
                print(f"[server] bad state message: {e}")
                continue

            # crude lag / ordering check — surfaces sync problems early
            tick = state.get("tick", last_tick + 1)
            if tick <= last_tick:
                print(f"[server] WARNING out-of-order tick {tick} <= {last_tick}")
            last_tick = tick

            command = decide(state)
            jsonschema.validate(command, COMMAND_SCHEMA)
            await ws.send(json.dumps(command))
    except websockets.ConnectionClosed:
        print("[server] Unity disconnected")


async def main():
    print(f"[server] listening on ws://{HOST}:{PORT}")
    async with websockets.serve(handle, HOST, PORT, max_size=2 ** 22):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
