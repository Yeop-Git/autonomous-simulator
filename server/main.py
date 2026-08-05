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
# Compiling a validator for every physics snapshot costs enough to put the
# WebSocket loop several Unity ticks behind.  Both schemas are immutable for a
# server run, so build their Draft-07 validators once at startup.
STATE_VALIDATOR = jsonschema.Draft7Validator(STATE_SCHEMA)
COMMAND_VALIDATOR = jsonschema.Draft7Validator(COMMAND_SCHEMA)


def load_network(network_path: str | None, scenario: str) -> LaneNetwork:
    if network_path:
        print(f"[server] loading lane network from {network_path}")
        return LaneNetwork.from_json(network_path)
    # Synthetic fallback so the server is runnable with no Unity export yet.
    from scenarios import networks

    print(f"[server] no --network given; using synthetic '{scenario}'")
    return networks.build(scenario)


async def handle(ws, controller: CentralController,
                 expected_tick_stride: int = 2):
    print(f"[server] Unity connected: {ws.remote_address}")
    last_tick = -1
    last_left_phases: dict[str, str] = {}
    last_green_block_cycle: dict[str, int] = {}
    try:
        async for raw in ws:
            try:
                state = json.loads(raw)
                STATE_VALIDATOR.validate(state)
            except (json.JSONDecodeError, jsonschema.ValidationError) as e:
                print(f"[server] bad state message: {e}")
                continue

            # --- sync checks: surface drift loudly (project's #1 risk) ---
            tick = state.get("tick", last_tick + 1)
            if tick == last_tick:
                print(f"[server] WARNING duplicate tick {tick}")
            elif tick < last_tick:
                print(f"[server] WARNING out-of-order tick {tick} < {last_tick}")
            elif last_tick >= 0 and tick > last_tick + expected_tick_stride:
                print(f"[server] WARNING gap: jumped {last_tick} -> {tick}")
            last_tick = max(last_tick, tick)

            command = controller.step(state)  # echoes time/tick
            COMMAND_VALIDATOR.validate(command)
            state_by_id = {v.get("id"): v for v in state.get("vehicles", [])}
            for vehicle_command in command.get("commands", []):
                phase = vehicle_command.get("left_turn_phase")
                vehicle_id = vehicle_command.get("vehicle_id")
                diagnostic = controller.left_turn_diagnostics.get(vehicle_id, {})
                if (phase == "SignalWaiting"
                        and not diagnostic.get("signal_requires_stop", True)):
                    cycle = int(float(state.get("time", 0.0)) // 54.0)
                    if last_green_block_cycle.get(vehicle_id) != cycle:
                        last_green_block_cycle[vehicle_id] = cycle
                        print(
                            "[server][LeftTurnBlocked] "
                            f"tick={tick} time={state.get('time', 0):.2f} "
                            f"vehicle={vehicle_id} diagnostic={diagnostic}",
                            flush=True,
                        )
                if not phase or last_left_phases.get(vehicle_id) == phase:
                    continue
                last_left_phases[vehicle_id] = phase
                source = state_by_id.get(vehicle_id, {})
                print(
                    "[server][LeftTurn] "
                    f"tick={tick} time={state.get('time', 0):.2f} "
                    f"vehicle={vehicle_id} phase={phase} "
                    f"lane={source.get('current_lane')} "
                    f"pos={source.get('position')} "
                    f"speed={vehicle_command.get('target_speed')}",
                    flush=True,
                )
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
        await handle(ws, controller, args.expected_tick_stride)

    async with websockets.serve(_handler, args.host, args.port, max_size=2 ** 22):
        await asyncio.Future()  # run forever


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Central V2X control server")
    p.add_argument("--network", default=None, help="lane-network JSON export")
    p.add_argument("--scenario", default="highway_straight",
                   help="synthetic network name when --network is omitted")
    p.add_argument("--host", default=HOST)
    p.add_argument("--port", type=int, default=PORT)
    p.add_argument(
        "--expected-tick-stride", type=int, default=2,
        help="largest normal Unity tick delta before reporting a dropped snapshot",
    )
    return p.parse_args(argv)


if __name__ == "__main__":
    asyncio.run(main_async(parse_args()))
