"""Drive every scene with real traffic and assert what must never happen again.

The per-module tests each pin one defect. This one is the standing check: put
plausible traffic and events into each authored scene, run it, and assert the
three properties that seven rounds of audit kept violating in new ways.

  * **no rear-end** — two vehicles on the *same* lane never end up within a car
    length of each other. Same-lane is deliberate: cars in adjacent lanes are
    legitimately close, and a raw pairwise minimum flags the ramp taper (where
    the centrelines run 1.98 m apart) as a collision.
  * **no wedged vehicle** — nothing is commanded to move while it fails to move.
    That is the signature of a stale path, a lost route, or a manoeuvre stuck in
    a phase it cannot leave.
  * **no flapping** — a settled vehicle does not change reported behaviour every
    tick. Two different limit cycles produced that: a conflict released the
    instant braking stopped the vehicle, and a 4 s-horizon prediction that flips
    between creep and stop.
"""
import math

import pytest

from central_control import CentralController
from headless_sim import HeadlessSim
from world_model import LaneNetwork

from test_scene_networks import SCENARIO_DIR

CAR_LENGTH = 4.0        # m; closer than this on one lane is a rear-end
WEDGED_TICKS = 20       # commanded to move while stationary for 2 s
FLAP_LIMIT = 20         # behaviour changes within any 10 s window


def network(scene):
    return LaneNetwork.from_json(SCENARIO_DIR / f"{scene}_lanes.json")


def drive(sim, steps, hook=None):
    """Run the sim and return a list of human-readable violations."""
    wedged = {v: 0 for v in sim.vehicles}
    worst_wedged = dict(wedged)
    previous = {v: None for v in sim.vehicles}
    window = {v: [] for v in sim.vehicles}
    worst_flaps = dict(wedged)
    closest, closest_at = math.inf, None

    for tick in range(steps):
        if hook:
            hook(sim, tick)
        commands = {c["vehicle_id"]: c for c in sim.step()["commands"]}

        by_lane: dict[str, list] = {}
        for vehicle in sim.vehicles.values():
            if not vehicle.arrived:
                by_lane.setdefault(vehicle.lane, []).append(vehicle)
        for group in by_lane.values():
            for i in range(len(group)):
                for j in range(i + 1, len(group)):
                    gap = math.hypot(
                        group[i].position[0] - group[j].position[0],
                        group[i].position[2] - group[j].position[2])
                    if gap < closest:
                        closest = gap
                        closest_at = (group[i].id, group[j].id, group[i].lane,
                                      round(sim.time, 1))

        for vehicle in sim.vehicles.values():
            if vehicle.arrived:
                continue
            command = commands.get(vehicle.id, {})
            still = (previous[vehicle.id] is not None
                     and math.dist(vehicle.position, previous[vehicle.id]) < 1e-6)
            told_to_move = float(command.get("target_speed", 0.0)) > 0.5
            wedged[vehicle.id] = (wedged[vehicle.id] + 1
                                  if (still and told_to_move) else 0)
            worst_wedged[vehicle.id] = max(worst_wedged[vehicle.id],
                                           wedged[vehicle.id])
            previous[vehicle.id] = list(vehicle.position)

            window[vehicle.id] = (window[vehicle.id]
                                  + [command.get("behavior", "")])[-100:]
            if len(window[vehicle.id]) == 100:
                flaps = sum(1 for a, b in zip(window[vehicle.id],
                                              window[vehicle.id][1:]) if a != b)
                worst_flaps[vehicle.id] = max(worst_flaps[vehicle.id], flaps)

    problems = []
    if closest < CAR_LENGTH:
        problems.append(
            f"same-lane separation fell to {closest:.2f} m {closest_at}")
    for vid in sim.vehicles:
        if worst_wedged[vid] > WEDGED_TICKS:
            problems.append(f"{vid} was told to move while stationary for "
                            f"{worst_wedged[vid] * 0.1:.1f} s")
        if worst_flaps[vid] > FLAP_LIMIT:
            problems.append(f"{vid} changed behaviour {worst_flaps[vid]} times "
                            "within 10 s")
    return problems


def _sim(scene):
    net = network(scene)
    return net, HeadlessSim(net, CentralController(net, dt=0.1), dt=0.1,
                            scenario=net.scenario)


def test_highway_ramp_merges_into_a_mainline_stream():
    net, sim = _sim("Highway")
    sim.add_vehicle("ramp", net.lane("hw_ramp").start, "hw_ramp", speed=0.0,
                    goal=list(net.lane("hw_l2").end))
    for i, z in enumerate((95.0, 60.0, 25.0)):
        sim.add_vehicle(f"main_{i}", [3.5, 0.0, z], "hw_l2", speed=25.0,
                        goal=list(net.lane("hw_l2").end))
    assert not drive(sim, 400)
    assert sim.vehicles["ramp"].lane == "hw_l2", "the ramp car never merged"


def test_highway_merge_with_the_mainline_blocked_past_the_join():
    net, sim = _sim("Highway")
    sim.add_vehicle("ramp", net.lane("hw_ramp").start, "hw_ramp", speed=0.0,
                    goal=list(net.lane("hw_l2").end))
    for i, z in enumerate((95.0, 55.0)):
        sim.add_vehicle(f"main_{i}", [3.5, 0.0, z], "hw_l2", speed=25.0,
                        goal=list(net.lane("hw_l2").end))

    def drop_cargo(s, tick):
        if tick == 60:
            s.add_object("cargo", "unexpected_obstacle", [3.5, 0.0, 185.0],
                         [0.0, 0.0, 0.0], radius=1.25)

    assert not drive(sim, 400, drop_cargo)


URBAN_TRAFFIC = [
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


def test_urban_intersection_under_full_load():
    """All eight approaches, one protected left, and a pedestrian on the
    scene's own northern crosswalk, walking only on the walk phase."""
    net, sim = _sim("Urban")
    for i, (lane, start, goal, manoeuvre, target) in enumerate(URBAN_TRAFFIC):
        sim.add_vehicle(f"u{i}", start, lane, speed=9.0,
                        goal=list(net.lane(goal).end),
                        maneuver=manoeuvre, target_lane=target)
    # The route V2XSceneBuilder authors: (9, 13) -> (-9, 13), across the
    # northbound exits and finishing clear of the roadway.
    sim.add_object("ped", "pedestrian", [9.0, 0.0, 13.0], [0.0, 0.0, 0.0],
                   radius=0.4)

    def walk_on_the_signal(s, _tick):
        pedestrian = s.objects["ped"]
        phase = s.time % 60.0
        walking = (13.0 <= phase < 21.0) or (47.0 <= phase < 55.0)
        if walking and pedestrian.position[0] <= -9.0:
            pedestrian.position[0] = 9.0        # next wave starts over
        pedestrian.velocity[0] = -2.5 if walking else 0.0

    assert not drive(sim, 1200, walk_on_the_signal)
    assert sim.vehicles["u0"].lane == "urban_wb_0_out", "the left turn never completed"


def test_emergency_avoidance_with_a_follower_and_an_ambulance():
    net, sim = _sim("EmergencyAvoidance")
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

    assert not drive(sim, 500, hazards)
    assert sim.vehicles["ego"].position[2] > 200.0, "the ego never got past the cargo"


def test_integrated_city_shoulder_merge_and_an_urban_obstacle():
    net, sim = _sim("IntegratedCity")
    goal = list(net.lane("city_south").centerline[10])
    sim.add_vehicle("shoulder", [9.0, 0.0, 300.0], "city_boulevard_escape",
                    speed=8.0, goal=goal)
    sim.add_vehicle("main", [5.4, 0.0, 190.0], "city_boulevard_main",
                    speed=20.0, goal=goal)
    sim.add_vehicle("urban", [5.4, 0.0, -70.0], "urban_nb_0_in", speed=12.0,
                    goal=[5.4, 0.0, 60.0])
    sim.add_object("rock", "unexpected_obstacle", [5.4, 0.0, -30.0],
                   [0.0, 0.0, 0.0], radius=1.2)
    assert not drive(sim, 500)


def test_lka_track_is_driven_without_incident():
    net, sim = _sim("LKA_Test")
    lane = net.lane("lka_curve")
    sim.add_vehicle("lka", list(lane.start), "lka_curve", speed=20.0,
                    goal=list(lane.end))
    assert not drive(sim, 400)
