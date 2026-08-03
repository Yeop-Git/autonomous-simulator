"""Phase 6 — urban: traffic lights, intersection reservation, pedestrian yield."""
import math

import pytest

from headless_sim import HeadlessSim
from central_control import CentralController
from intersection import IntersectionManager
from traffic import GREEN, RED, YELLOW, TrafficLight, TrafficLightManager
from world_model import DynamicVehicle, Lane, LaneNetwork


def veh(vid, pos, vel, lane="L"):
    return DynamicVehicle(id=vid, position=list(pos), velocity=list(vel),
                          heading=0.0, current_lane=lane)


# ---- traffic lights ------------------------------------------------------ #
def test_light_cycle():
    light = TrafficLight("l", stop_line=[0, 0, 100], green_time=10, yellow_time=3,
                         red_time=12)
    assert light.state(0) == GREEN
    assert light.state(9.9) == GREEN
    assert light.state(11) == YELLOW
    assert light.state(20) == RED
    assert light.state(25 + 0.0) == GREEN  # 25 == period -> wraps to green


def test_should_stop_on_red():
    mgr = TrafficLightManager()
    mgr.add(TrafficLight("l", stop_line=[0, 0, 100], approach_heading=0.0,
                         green_time=0, yellow_time=0, red_time=10))  # always red
    assert mgr.should_stop([0, 0, 80], speed=10, light_id="l", t=1.0)
    # past the line -> no stop
    assert not mgr.should_stop([0, 0, 110], speed=10, light_id="l", t=1.0)


def test_no_stop_on_green():
    mgr = TrafficLightManager()
    mgr.add(TrafficLight("l", stop_line=[0, 0, 100], green_time=10, yellow_time=3,
                         red_time=10))
    assert not mgr.should_stop([0, 0, 80], speed=10, light_id="l", t=1.0)


def test_yellow_stops_only_if_stoppable():
    mgr = TrafficLightManager()
    # yellow at t in [10,13)
    mgr.add(TrafficLight("l", stop_line=[0, 0, 100], green_time=10, yellow_time=3,
                         red_time=10))
    # slow + far => can stop => should stop
    assert mgr.should_stop([0, 0, 80], speed=5, light_id="l", t=11.0)
    # fast + close => can't stop comfortably => proceed
    assert not mgr.should_stop([0, 0, 98], speed=20, light_id="l", t=11.0)


def test_signal_priority_prevents_crossing_deadlock():
    north = Lane("urban_north", [[-1.8, 0, -70], [-1.8, 0, 70]])
    east = Lane("urban_east", [[-70, 0, 1.8], [70, 0, 1.8]])
    controller = CentralController(LaneNetwork(
        [north, east], scenario="urban"))

    def vehicle(vid, pos, velocity, lane, goal, heading):
        return {
            "id": vid, "type": "car", "position": pos,
            "velocity": velocity, "acceleration": [0, 0, 0],
            "heading": heading, "current_lane": lane, "target_lane": None,
            "has_goal": True, "goal": goal, "behavior_state": "LaneKeeping",
        }
    state = {
        "time": 0.0, "tick": 1, "scenario": "urban",
        "vehicles": [
            vehicle("north", [-1.8, 0, -20], [0, 0, 10],
                    "urban_north", [-1.8, 0, 60], 0),
            vehicle("east", [-20, 0, 1.8], [10, 0, 0],
                    "urban_east", [60, 0, 1.8], 90),
        ],
        "objects": [], "events": [],
    }
    commands = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}
    assert commands["north"]["target_speed"] > 0
    assert commands["north"]["behavior"] != "EmergencyBraking"
    assert 0 < commands["east"]["target_speed"] < 13.9
    assert commands["east"]["behavior"] == "WaitingAtIntersection"


def test_protected_left_phase_holds_oncoming_traffic():
    left = Lane("urban_nb_1_in", [[1.8, 0, -70], [1.8, 0, -11]])
    south = Lane("urban_sb_0_in", [[-5.4, 0, 70], [-5.4, 0, 11]])
    controller = CentralController(LaneNetwork([left, south], scenario="urban"))

    def state_vehicle(vid, pos, velocity, lane, heading):
        return {
            "id": vid, "type": "car", "position": pos,
            "velocity": velocity, "acceleration": [0, 0, 0],
            "heading": heading, "current_lane": lane, "target_lane": None,
            "has_goal": False, "goal": pos, "behavior_state": "LaneKeeping",
        }
    state = {
        "time": 36.0, "tick": 1, "scenario": "urban",
        "vehicles": [
            state_vehicle("left", [1.8, 0, -20], [0, 0, 8], "urban_nb_1_in", 0),
            state_vehicle("oncoming", [-5.4, 0, 20], [0, 0, -8], "urban_sb_0_in", 180),
        ],
        "objects": [], "events": [],
    }
    commands = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}
    assert commands["left"]["target_speed"] > 0
    assert commands["left"]["behavior"] != "EmergencyBraking"
    assert 0 < commands["oncoming"]["target_speed"] < 13.9
    assert commands["oncoming"]["behavior"] == "WaitingAtIntersection"


def test_real_urban_cycle_starts_with_perpendicular_traffic_green():
    controller = CentralController(LaneNetwork([
        Lane("urban_nb_0_in", [[5.4, 0, -70], [5.4, 0, -11]]),
        Lane("urban_eb_0_in", [[-70, 0, -5.4], [-11, 0, -5.4]]),
    ], scenario="urban"))
    assert controller.traffic.state("urban_eb_0_in", 0.0) == "Green"
    assert controller.traffic.state("urban_nb_0_in", 0.0) == "Red"
    assert controller.traffic.lights["urban_nb_0_in"].stop_line[2] == -16.0


def test_red_signal_approach_speed_stops_before_painted_line():
    lane = Lane("urban_nb_0_in", [[5.4, 0, -70], [5.4, 0, -11]])
    controller = CentralController(LaneNetwork([lane], scenario="urban"))

    def command_at(z):
        state = {
            "time": 5.0, "tick": 1, "scenario": "urban",
            "vehicles": [{
                "id": "ego", "type": "car", "position": [5.4, 0, z],
                "velocity": [0, 0, 10], "acceleration": [0, 0, 0],
                "heading": 0, "current_lane": "urban_nb_0_in",
                "target_lane": None, "has_goal": False,
                "goal": [5.4, 0, 60], "behavior_state": "LaneKeeping",
            }],
            "objects": [], "events": [],
        }
        return controller.step(state)["commands"][0]

    approaching = command_at(-30.0)
    at_line_buffer = command_at(-17.0)
    assert 0.0 < approaching["target_speed"] < 13.9
    assert at_line_buffer["target_speed"] == 0.0
    assert at_line_buffer["behavior"] == "WaitingAtIntersection"


# ---- intersection reservation -------------------------------------------- #
def test_single_vehicle_no_yield():
    mgr = IntersectionManager(center=[0, 0, 0], radius=6.0)
    v = veh("a", [0, 0, -40], [0, 0, 10])
    grants = mgr.reserve([v], t_now=0.0)
    assert "a" in grants
    assert not grants["a"].must_yield


def test_crossing_vehicles_one_yields():
    mgr = IntersectionManager(center=[0, 0, 0], radius=6.0, buffer=1.5)
    a = veh("a", [0, 0, -40], [0, 0, 10])   # from -Z
    b = veh("b", [-40, 0, 0], [10, 0, 0])   # from -X, same ETA
    grants = mgr.reserve([a, b], t_now=0.0)
    yields = [g.must_yield for g in grants.values()]
    assert sum(yields) == 1, "exactly one of the two should yield"
    # granted windows must not overlap: later enter >= earlier enter + occupancy
    enters = sorted(g.enter_time for g in grants.values())
    assert enters[1] - enters[0] >= 1.0


def test_approaching_filters_inside_and_receding():
    mgr = IntersectionManager(center=[0, 0, 0], radius=6.0)
    inside = veh("inside", [0, 0, 0], [0, 0, 10])      # already in zone
    receding = veh("rec", [0, 0, 40], [0, 0, 10])      # past, moving away
    incoming = veh("inc", [0, 0, -30], [0, 0, 10])     # approaching
    got = {v.id for v in mgr.approaching([inside, receding, incoming])}
    assert got == {"inc"}


# ---- pedestrian crossing (generic conflict prediction) ------------------- #
def test_vehicle_stops_for_crossing_pedestrian():
    cl = [[0, 0, z] for z in range(0, 201, 5)]
    net = LaneNetwork([Lane(id="L", centerline=cl, next_lane_ids=[])],
                      scenario="urban")
    sim = HeadlessSim(net, dt=0.1, scenario="urban")
    sim.add_vehicle("car", [0, 0, 0], "L", speed=12, goal=[0, 0, 190])
    # pedestrian walking across the lane ~80 m ahead
    sim.add_object("ped", "pedestrian", [-6, 0, 80], [1.0, 0, 0], radius=0.4)
    min_dist = math.inf
    for _ in range(120):
        sim.step()
        car = sim.vehicles["car"]
        ped = sim.objects["ped"]
        min_dist = min(min_dist, math.hypot(car.position[0] - ped.position[0],
                                            car.position[2] - ped.position[2]))
    assert min_dist > 2.0, f"car got too close to pedestrian ({min_dist:.2f} m)"
