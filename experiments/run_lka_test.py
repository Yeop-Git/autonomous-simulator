"""Phase 4 experiment: LKA lateral-error sweep (plan §11.2, §20.2).

Drives a kinematic bicycle along a curved lane with each lateral controller
(Pure Pursuit / Stanley / PID) at several speeds, logging per-step rows in the
frozen CSV schema and summarizing lateral error + lane-departure counts.

No Unity required — this is the headless LKA test bed. Output goes to
``experiments/results/``.

Run:
    python experiments/run_lka_test.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

# make the server package importable
REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "server"))

from controllers import lateral          # noqa: E402
from logging_csv import DriveLogger, LogRow  # noqa: E402
from scenarios import networks           # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
SPEEDS_KMH = [40, 60, 80, 100]
CONTROLLERS = ["pure_pursuit", "stanley", "pid"]


def kmh(v: int) -> float:
    return v / 3.6


def simulate(ctrl, centerline, scenario, speed, lane_width, logger,
             vehicle_id, dt=0.02, wheel_base=2.7, max_steps=4000):
    """Track the centerline; log every step. Returns (rms, max_abs, departures)."""
    x = centerline[0][0]
    z = centerline[0][2]
    heading = 0.0  # +Z, aligned with the start of these networks
    if hasattr(ctrl, "reset"):
        ctrl.reset()

    sq_sum = 0.0
    max_abs = 0.0
    departures = 0
    in_departure = False
    n = 0
    t = 0.0
    for _ in range(max_steps):
        lat, herr, curv = lateral.frenet_errors(x, z, heading, centerline)
        delta = ctrl.steer(x, z, heading, speed, centerline, wheel_base, dt=dt)
        logger.log(LogRow(
            time=round(t, 3), vehicle_id=vehicle_id, scenario=scenario,
            position_x=x, position_z=z, speed=speed, lane_id="curve_0",
            behavior_state="LaneKeeping", lateral_error=lat, heading_error=herr,
            target_speed=speed,
        ))
        # departure = drifted past half the lane width
        if abs(lat) > lane_width / 2.0:
            if not in_departure:
                departures += 1
                in_departure = True
        else:
            in_departure = False

        sq_sum += lat * lat
        max_abs = max(max_abs, abs(lat))
        n += 1

        # integrate kinematic bicycle
        yaw_rate = speed / wheel_base * math.tan(delta)
        heading = (heading + math.degrees(yaw_rate * dt)) % 360.0
        rad = math.radians(heading)
        x += math.sin(rad) * speed * dt
        z += math.cos(rad) * speed * dt
        t += dt
        if z >= centerline[-1][2] - 2.0:
            break

    rms = math.sqrt(sq_sum / n) if n else 0.0
    return rms, max_abs, departures


def main():
    RESULTS.mkdir(parents=True, exist_ok=True)
    net = networks.highway_curve(length=240.0, radius=140.0)
    lane = net.lane("curve_0")
    centerline = list(lane.centerline)

    summary_path = RESULTS / "lka_summary.csv"
    log_path = RESULTS / "lka_drive_log.csv"

    summary_rows = []
    with DriveLogger(log_path) as logger:
        for name in CONTROLLERS:
            for v_kmh in SPEEDS_KMH:
                speed = kmh(v_kmh)
                ctrl = lateral.make(name)
                rms, max_abs, dep = simulate(
                    ctrl, centerline, net.scenario, speed, lane.width, logger,
                    vehicle_id=f"{name}_{v_kmh}")
                summary_rows.append((name, v_kmh, rms, max_abs, dep))
                print(f"{name:13s} {v_kmh:3d} km/h  "
                      f"RMS={rms:5.3f}m  max={max_abs:5.3f}m  departures={dep}")

    with open(summary_path, "w", encoding="utf-8") as f:
        f.write("controller,speed_kmh,rms_lateral_m,max_lateral_m,departures\n")
        for name, v, rms, mx, dep in summary_rows:
            f.write(f"{name},{v},{rms:.4f},{mx:.4f},{dep}\n")

    print(f"\nwrote {summary_path}")
    print(f"wrote {log_path}")


if __name__ == "__main__":
    main()
