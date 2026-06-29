"""Fake-Unity client — drives the server with no Unity in the loop.

Connects to a running ``main.py``, streams synthetic StateMessages for one
vehicle with a goal, and prints the commands it gets back (including the
planned path length and any sync warnings). Use it to prove the
``Unity -> server -> A* -> Unity`` loop end to end from the terminal.

Run (in two terminals):
    python main.py --scenario highway_straight
    python tools/fake_unity_client.py --ticks 30
"""
from __future__ import annotations

import argparse
import asyncio
import json
import sys
from pathlib import Path

# allow running from server/ or repo root
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import websockets  # noqa: E402


def make_state(tick: int, z: float) -> dict:
    return {
        "time": tick * 0.1,
        "tick": tick,
        "scenario": "highway",
        "vehicles": [
            {
                "id": "car_01",
                "type": "car",
                "position": [0.0, 0.0, z],
                "velocity": [0.0, 0.0, 20.0],
                "heading": 0.0,
                "current_lane": "hw_l0_a",
                "has_goal": True,
                "goal": [0.0, 0.0, 290.0],
            }
        ],
        "objects": [],
        "events": [],
    }


async def run(url: str, ticks: int):
    async with websockets.connect(url) as ws:
        print(f"[fake-unity] connected to {url}")
        z = 0.0
        ok = True
        for tick in range(ticks):
            await ws.send(json.dumps(make_state(tick, z)))
            reply = json.loads(await ws.recv())

            if reply.get("tick") != tick:
                print(f"[fake-unity] SYNC MISMATCH sent tick {tick}, "
                      f"got {reply.get('tick')}")
                ok = False
            cmds = reply.get("commands", [])
            cmd = cmds[0] if cmds else {}
            path_len = len(cmd.get("path", []))
            print(f"  tick {tick:3d}  z={z:6.1f}  "
                  f"behavior={cmd.get('behavior'):<12} "
                  f"target_speed={cmd.get('target_speed', 0):5.1f}  "
                  f"path_pts={path_len}")
            # advance the car roughly along its velocity
            z = min(z + 20.0 * 0.1, 290.0)
            await asyncio.sleep(0.02)

        print("[fake-unity] DONE", "OK" if ok else "WITH SYNC ERRORS")
        return ok


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="ws://localhost:8765")
    p.add_argument("--ticks", type=int, default=30)
    args = p.parse_args()
    ok = asyncio.run(run(args.url, args.ticks))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
