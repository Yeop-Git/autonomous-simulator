"""Per-scene load measurement on the headless sim (plan §21.2).

`run_algorithm_compare.py` measures a planner in isolation and `run_lka_test.py`
measures a lateral law in isolation. This runner measures the thing those two
leave out: the **whole central controller** — routing, signals, merge
reservation, left-turn policy, collision prediction and ACC together — driving
each authored scene with the same traffic the regression suite uses.

Reported per scenario:

  * ``step_p50_ms`` / ``step_p95_ms`` / ``step_max_ms`` — wall-clock cost of one
    ``CentralController.step()`` for the whole scene. This harness advances in
    0.1 s synthetic steps; it does not measure the Unity/WebSocket loop. The
    authored Unity scenes nominally send every 0.04 s (25 Hz).
  * ``min_same_lane_gap_m`` — closest centre-to-centre approach of two vehicles
    that were on the *same* lane. Adjacent-lane pairs are excluded on purpose:
    a shoulder taper legitimately runs centrelines ~2 m apart.
  * ``peak_decel_mps2`` / ``hard_brake_episodes`` — the largest deceleration any
    vehicle applied, and the number of *episodes* (rising edges, not ticks) past
    ``metrics.HARD_BRAKE_DECEL`` = 4 m/s². Read them together: 4 m/s² is also
    the ACC's own comfort limit, so an episode barely over the line is the
    controller working at its limit, not an emergency stop.
  * ``min_ttc_s`` — smallest time-to-safety-distance the collision predictor
    reported **for a pair involving a controlled vehicle**. Object-object pairs
    are excluded: scripted props are not steered by the server, so an ambulance
    driving through the fallen cargo is a harness artefact, not a near miss.
    ``inf`` means no such pair ever entered the 4 s horizon.
  * ``max_behavior_changes_10s`` — behaviour flips inside any 10 s window, i.e.
    a decision-layer limit cycle.

No Unity required. The kinematic states and scenario outcomes are deterministic;
wall-clock timing is machine- and run-dependent and will not reproduce exactly.

Run:
    python experiments/run_scene_stats.py
"""
from __future__ import annotations

import math
import statistics
import sys
import time
from pathlib import Path

# make the server package importable
REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "server"))

from central_control import CentralController   # noqa: E402
from headless_sim import HeadlessSim            # noqa: E402
from metrics import HARD_BRAKE_DECEL            # noqa: E402
from world_model import LaneNetwork             # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
SCENARIO_DIR = REPO / "server" / "scenarios"
DT = 0.1
WINDOW = int(10.0 / DT)   # behaviour-flip window, in ticks

COLUMNS = [
    "scenario", "vehicles", "ticks", "sim_seconds",
    "step_p50_ms", "step_p95_ms", "step_max_ms",
    "mean_speed_mps", "min_same_lane_gap_m", "min_ttc_s",
    "peak_decel_mps2", "hard_brake_episodes", "max_behavior_changes_10s",
    "arrivals",
]


def network(scene: str) -> LaneNetwork:
    return LaneNetwork.from_json(SCENARIO_DIR / f"{scene}_lanes.json")


def new_sim(scene: str) -> tuple[LaneNetwork, HeadlessSim]:
    net = network(scene)
    return net, HeadlessSim(net, CentralController(net, dt=DT), dt=DT,
                            scenario=net.scenario)


def measure(name: str, sim: HeadlessSim, steps: int, hook=None) -> dict:
    """Drive ``sim`` for ``steps`` ticks and return one summary row."""
    step_ms: list[float] = []
    speeds: list[float] = []
    closest = math.inf
    min_ttc = math.inf
    hard_brake_episodes = 0
    peak_decel = 0.0
    braking: set[str] = set()          # vehicles inside a hard-brake episode
    last_speed: dict[str, float] = {}
    window: dict[str, list[str]] = {v: [] for v in sim.vehicles}
    worst_flips = 0
    arrived: set[str] = set()

    for tick in range(steps):
        if hook:
            hook(sim, tick)

        started = time.perf_counter()
        command = sim.step()
        step_ms.append((time.perf_counter() - started) * 1000.0)

        commands = {c["vehicle_id"]: c for c in command["commands"]}
        for conflict in sim.controller.last_conflicts:
            if conflict.a_id in sim.vehicles or conflict.b_id in sim.vehicles:
                min_ttc = min(min_ttc, conflict.ttc)

        by_lane: dict[str, list] = {}
        for vehicle in sim.vehicles.values():
            if vehicle.arrived:
                arrived.add(vehicle.id)
                continue
            speeds.append(vehicle.speed)
            by_lane.setdefault(vehicle.lane, []).append(vehicle)

            previous = last_speed.get(vehicle.id)
            decel = (previous - vehicle.speed) / DT if previous is not None else 0.0
            peak_decel = max(peak_decel, decel)
            hard = decel > HARD_BRAKE_DECEL
            if hard and vehicle.id not in braking:
                hard_brake_episodes += 1
                braking.add(vehicle.id)
            elif not hard:
                braking.discard(vehicle.id)
            last_speed[vehicle.id] = vehicle.speed

            behaviour = commands.get(vehicle.id, {}).get("behavior", "")
            seen = (window.setdefault(vehicle.id, []) + [behaviour])[-WINDOW:]
            window[vehicle.id] = seen
            if len(seen) == WINDOW:
                worst_flips = max(worst_flips, sum(
                    1 for a, b in zip(seen, seen[1:]) if a != b))

        for group in by_lane.values():
            for i in range(len(group)):
                for j in range(i + 1, len(group)):
                    closest = min(closest, math.hypot(
                        group[i].position[0] - group[j].position[0],
                        group[i].position[2] - group[j].position[2]))

    step_ms.sort()
    return {
        "scenario": name,
        "vehicles": len(sim.vehicles),
        "ticks": steps,
        "sim_seconds": round(steps * DT, 1),
        "step_p50_ms": round(statistics.median(step_ms), 3),
        "step_p95_ms": round(step_ms[min(int(len(step_ms) * 0.95), len(step_ms) - 1)], 3),
        "step_max_ms": round(step_ms[-1], 3),
        "mean_speed_mps": round(statistics.fmean(speeds), 2) if speeds else 0.0,
        "min_same_lane_gap_m": round(closest, 2) if math.isfinite(closest) else "n/a",
        "min_ttc_s": round(min_ttc, 2) if math.isfinite(min_ttc) else "inf",
        "peak_decel_mps2": round(peak_decel, 2),
        "hard_brake_episodes": hard_brake_episodes,
        "max_behavior_changes_10s": worst_flips,
        "arrivals": len(arrived),
    }


# --------------------------------------------------------------------------- #
# Scenarios — the same traffic the regression suite drives.
# --------------------------------------------------------------------------- #
def case_lka() -> dict:
    net, sim = new_sim("LKA_Test")
    lane = net.lane("lka_curve")
    sim.add_vehicle("lka", list(lane.start), "lka_curve", speed=20.0,
                    goal=list(lane.end))
    return measure("LKA_Test", sim, 400)


def case_highway() -> dict:
    net, sim = new_sim("Highway")
    sim.add_vehicle("ramp", net.lane("hw_ramp").start, "hw_ramp", speed=0.0,
                    goal=list(net.lane("hw_l2").end))
    for i, z in enumerate((95.0, 60.0, 25.0)):
        sim.add_vehicle(f"main_{i}", [3.5, 0.0, z], "hw_l2", speed=25.0,
                        goal=list(net.lane("hw_l2").end))
    return measure("Highway (ramp merge)", sim, 400)


def case_urban() -> dict:
    traffic = [
        ("urban_nb_0_in", [5.4, 0.0, -55.0], "urban_wb_0_out", "left",
         "urban_nb_1_in"),
        ("urban_nb_0_in", [5.4, 0.0, -70.0], "urban_nb_0_out", "straight", None),
        ("urban_sb_0_in", [-5.4, 0.0, 70.0], "urban_sb_0_out", "straight", None),
        ("urban_sb_1_in", [-1.8, 0.0, 62.0], "urban_sb_1_out", "straight", None),
        ("urban_eb_0_in", [-70.0, 0.0, -5.4], "urban_eb_0_out", "straight", None),
        ("urban_eb_1_in", [-70.0, 0.0, -1.8], "urban_eb_1_out", "straight", None),
        ("urban_wb_0_in", [70.0, 0.0, 5.4], "urban_wb_0_out", "straight", None),
        ("urban_wb_1_in", [70.0, 0.0, 1.8], "urban_wb_1_out", "straight", None),
    ]
    net, sim = new_sim("Urban")
    for i, (lane, start, goal, manoeuvre, target) in enumerate(traffic):
        sim.add_vehicle(f"u{i}", start, lane, speed=9.0,
                        goal=list(net.lane(goal).end),
                        maneuver=manoeuvre, target_lane=target)
    sim.add_object("ped", "pedestrian", [9.0, 0.0, 13.0], [0.0, 0.0, 0.0],
                   radius=0.4)

    def walk_on_the_signal(s, _tick):
        pedestrian = s.objects["ped"]
        phase = s.time % 60.0
        walking = (13.0 <= phase < 21.0) or (47.0 <= phase < 55.0)
        if walking and pedestrian.position[0] <= -9.0:
            pedestrian.position[0] = 9.0
        pedestrian.velocity[0] = -2.5 if walking else 0.0

    return measure("Urban (8 approaches + pedestrian)", sim, 1200,
                   walk_on_the_signal)


def case_emergency() -> dict:
    net, sim = new_sim("EmergencyAvoidance")
    sim.add_vehicle("ego", [0.0, 0.0, 30.0], "ea_center", speed=20.0,
                    goal=[0.0, 0.0, 320.0])
    sim.add_vehicle("follow", [0.0, 0.0, 0.0], "ea_center", speed=20.0,
                    goal=[0.0, 0.0, 320.0])

    def hazards(s, tick):
        ego = s.vehicles["ego"]
        if tick == 30:
            s.add_object("cargo", "unexpected_obstacle",
                         [0.0, 0.0, ego.position[2] + 48.0],
                         [0.0, 0.0, 0.0], radius=1.25)
        if tick == 90:
            s.add_object("ambulance", "emergency_vehicle",
                         [0.0, 0.0, ego.position[2] - 45.0],
                         [0.0, 0.0, 31.0], radius=1.2)

    return measure("EmergencyAvoidance (cargo + ambulance)", sim, 500, hazards)


def case_integrated_city() -> dict:
    net, sim = new_sim("IntegratedCity")
    goal = list(net.lane("city_south").centerline[10])
    sim.add_vehicle("shoulder", [9.0, 0.0, 300.0], "city_boulevard_escape",
                    speed=8.0, goal=goal)
    sim.add_vehicle("main", [5.4, 0.0, 190.0], "city_boulevard_main",
                    speed=20.0, goal=goal)
    sim.add_vehicle("urban", [5.4, 0.0, -70.0], "urban_nb_0_in", speed=12.0,
                    goal=[5.4, 0.0, 60.0])
    sim.add_object("rock", "unexpected_obstacle", [5.4, 0.0, -30.0],
                   [0.0, 0.0, 0.0], radius=1.2)
    return measure("IntegratedCity (shoulder merge + obstacle)", sim, 500)


CASES = [case_lka, case_highway, case_urban, case_emergency,
         case_integrated_city]


def main() -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    rows = []
    for case in CASES:
        row = case()
        rows.append(row)
        print(f"{row['scenario']:44s} "
              f"n={row['vehicles']:2d}  "
              f"step p50/p95/max = {row['step_p50_ms']:6.2f} / "
              f"{row['step_p95_ms']:6.2f} / {row['step_max_ms']:6.2f} ms  "
              f"min same-lane gap = {row['min_same_lane_gap_m']} m  "
              f"min TTC = {row['min_ttc_s']} s  "
              f"peak decel = {row['peak_decel_mps2']} m/s^2  "
              f"hard-brake episodes = {row['hard_brake_episodes']}")

    out = RESULTS / "scene_stats.csv"
    with open(out, "w", encoding="utf-8", newline="") as f:
        f.write(",".join(COLUMNS) + "\n")
        for row in rows:
            f.write(",".join(str(row[c]) for c in COLUMNS) + "\n")
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
