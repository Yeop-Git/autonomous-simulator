"""Central V2X control server — WebSocket front-end for the vertical slice.

This file is now thin: all decision logic lives in ``central_control``.
Here we only:
  1. load the lane network (from a JSON export, or a synthetic fallback),
  2. accept a WebSocket connection from Unity,
  3. validate each StateMessage against the schema,
  4. run sync checks (out-of-order / duplicate tick warnings, tick echo),
  5. hand the state to ``CentralController`` and return its CommandMessage.

Run:
    pip install -r requirements.txt
    python main.py                      # synthetic highway network
    python main.py --network net.json   # network exported from Unity
    python main.py --scenario urban_grid
"""
from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

import jsonschema
import websockets

from central_control import CentralController
from world_model import LaneNetwork

PROTOCOL_DIR = Path(__file__).resolve().parents[1] / "shared" / "protocol"
HOST = "localhost"
PORT = 8765


def _load_schema(name: str) -> dict:
    with open(PROTOCOL_DIR / name, "r", encoding="utf-8") as f:
        return json.load(f)


STATE_SCHEMA = _load_schema("state_message.schema.json")
COMMAND_SCHEMA = _load_schema("command_message.schema.json")


def load_network(network_path: str | None, scenario: str) -> LaneNetwork:
    if network_path:
        print(f"[server] loading lane network from {network_path}")
        return LaneNetwork.from_json(network_path)
    # Synthetic fallback so the server is runnable with no Unity export yet.
    from scenarios import networks

    print(f"[server] no --network given; using synthetic '{scenario}'")
    return networks.build(scenario)


async def handle(ws, controller: CentralController):
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

            # --- sync checks: surface drift loudly (project's #1 risk) ---
            tick = state.get("tick", last_tick + 1)
            if tick == last_tick:
                print(f"[server] WARNING duplicate tick {tick}")
            elif tick < last_tick:
                print(f"[server] WARNING out-of-order tick {tick} < {last_tick}")
            elif tick > last_tick + 1:
                print(f"[server] WARNING gap: jumped {last_tick} -> {tick}")
            last_tick = max(last_tick, tick)

            command = controller.step(state)  # echoes time/tick
            jsonschema.validate(command, COMMAND_SCHEMA)
            await ws.send(json.dumps(command))
    except websockets.ConnectionClosed:
        print("[server] Unity disconnected")


async def main_async(args):
    network = load_network(args.network, args.scenario)
    controller = CentralController(network)
    print(f"[server] network '{network.name or args.scenario}' "
          f"with {len(network.all_lane_ids())} lanes")
    print(f"[server] listening on ws://{args.host}:{args.port}")

    async def _handler(ws):
        await handle(ws, controller)

    async with websockets.serve(_handler, args.host, args.port, max_size=2 ** 22):
        await asyncio.Future()  # run forever


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Central V2X control server")
    p.add_argument("--network", default=None, help="lane-network JSON export")
    p.add_argument("--scenario", default="highway_straight",
                   help="synthetic network name when --network is omitted")
    p.add_argument("--host", default=HOST)
    p.add_argument("--port", type=int, default=PORT)
    return p.parse_args(argv)


if __name__ == "__main__":
    asyncio.run(main_async(parse_args()))
