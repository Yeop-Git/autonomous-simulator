"""Protocol + end-to-end loop tests.

Covers:
  * the two JSON Schemas load and a sample message validates,
  * the CentralController output validates against the command schema,
  * a full in-process WebSocket round-trip (server + client, no Unity),
    asserting tick echo / no sync drift.
"""
import asyncio
import json
from pathlib import Path

import jsonschema
import pytest
import websockets

from central_control import CentralController
from scenarios import networks

PROTOCOL_DIR = Path(__file__).resolve().parents[2] / "shared" / "protocol"


def _schema(name):
    with open(PROTOCOL_DIR / name, "r", encoding="utf-8") as f:
        return json.load(f)


STATE_SCHEMA = _schema("state_message.schema.json")
COMMAND_SCHEMA = _schema("command_message.schema.json")
LANE_SCHEMA = _schema("lane_network.schema.json")


def sample_state(tick=0, z=5.0, with_goal=True):
    return {
        "time": tick * 0.1,
        "tick": tick,
        "scenario": "highway",
        "vehicles": [
            {
                "id": "car_01",
                "position": [0.0, 0.0, z],
                "velocity": [0.0, 0.0, 20.0],
                "heading": 0.0,
                "current_lane": "hw_l0_a",
                "has_goal": with_goal,
                "goal": [0.0, 0.0, 290.0],
            }
        ],
        "objects": [],
        "events": [],
    }


def test_schemas_load_and_sample_validates():
    jsonschema.validate(sample_state(), STATE_SCHEMA)


def test_lane_network_export_validates():
    net = networks.highway_straight(lanes=2, length=100.0)
    jsonschema.validate(net.to_dict(), LANE_SCHEMA)


def test_controller_output_validates_against_command_schema():
    net = networks.highway_straight(lanes=1, length=300.0)
    ctrl = CentralController(net)
    cmd = ctrl.step(sample_state())
    jsonschema.validate(cmd, COMMAND_SCHEMA)


def test_controller_echoes_time_and_tick():
    net = networks.highway_straight(lanes=1, length=300.0)
    ctrl = CentralController(net)
    cmd = ctrl.step(sample_state(tick=7))
    assert cmd["tick"] == 7
    assert cmd["time"] == pytest.approx(0.7)


def test_controller_routes_when_goal_present():
    net = networks.highway_straight(lanes=1, length=300.0)
    ctrl = CentralController(net)
    cmd = ctrl.step(sample_state(z=5.0))
    c0 = cmd["commands"][0]
    assert c0["behavior"] == "LaneKeeping"
    assert c0["path"], "expected a planned path"
    assert c0["target_speed"] > 0


def test_controller_arrives_near_goal():
    net = networks.highway_straight(lanes=1, length=300.0)
    ctrl = CentralController(net)
    cmd = ctrl.step(sample_state(z=289.0))
    c0 = cmd["commands"][0]
    assert c0["behavior"] == "Arrived"
    assert c0["target_speed"] == 0.0


def test_controller_caches_route():
    net = networks.highway_straight(lanes=1, length=300.0)
    ctrl = CentralController(net)
    ctrl.step(sample_state(tick=0, z=5.0))
    ctrl.step(sample_state(tick=1, z=10.0))
    assert ctrl.replans == 1  # same goal => no second plan


def test_no_goal_cruises_toward_lane_speed():
    net = networks.highway_straight(lanes=1, length=300.0)
    ctrl = CentralController(net)
    cmd = ctrl.step(sample_state(with_goal=False))
    c0 = cmd["commands"][0]
    assert c0["behavior"] == "LaneKeeping"
    # ACC ramps up from ego speed (20) toward the 27.8 limit, capped by accel.
    assert 20.0 <= c0["target_speed"] <= 27.8
    assert "path" not in c0


# --------------------------------------------------------------------------- #
# Full in-process WebSocket round-trip (mirrors main.handle without argparse).
# --------------------------------------------------------------------------- #
def test_websocket_round_trip():
    async def scenario():
        net = networks.highway_straight(lanes=1, length=300.0)
        ctrl = CentralController(net)
        import main

        async def _handler(ws):
            await main.handle(ws, ctrl)

        async with websockets.serve(_handler, "localhost", 8799):
            async with websockets.connect("ws://localhost:8799") as ws:
                for tick in range(5):
                    await ws.send(json.dumps(sample_state(tick=tick, z=tick * 20.0)))
                    reply = json.loads(await ws.recv())
                    jsonschema.validate(reply, COMMAND_SCHEMA)
                    assert reply["tick"] == tick  # tick echo, no drift
        return True

    assert asyncio.run(scenario())
