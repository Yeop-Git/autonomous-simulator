"""Phase 5 — highway: lane change, merge reservation, hazard replan, metrics."""
import math

import pytest

import lane_change
import merge
import metrics
from central_control import CentralController
from headless_sim import HeadlessSim
from scenarios import networks
from world_model import DynamicVehicle, Lane, LaneNetwork


def veh(vid, pos, vel, lane):
    return DynamicVehicle(id=vid, position=list(pos), velocity=list(vel),
                          heading=0.0, current_lane=lane)


# ---- lane change gap acceptance ------------------------------------------ #
def test_lane_change_into_empty_lane_accepted():
    net = networks.highway_straight(lanes=2, length=300.0)
    ego = veh("ego", [0, 0, 50], [0, 0, 25], "hw_l0_a")
    d = lane_change.evaluate(ego, "hw_l1_a", [], net)
    assert d.accept


def test_lane_change_blocked_by_side_vehicle():
    net = networks.highway_straight(lanes=2, length=300.0)
    ego = veh("ego", [0, 0, 50], [0, 0, 25], "hw_l0_a")
    # a car right beside the ego in the target lane
    beside = veh("beside", [3.5, 0, 51], [0, 0, 25], "hw_l1_a")
    d = lane_change.evaluate(ego, "hw_l1_a", [beside], net)
    assert not d.accept
    assert "gap" in d.reason


def test_lane_change_far_traffic_accepted():
    net = networks.highway_straight(lanes=2, length=400.0)
    ego = veh("ego", [0, 0, 50], [0, 0, 25], "hw_l0_a")
    ahead = veh("ahead", [3.5, 0, 200], [0, 0, 25], "hw_l1_a")  # 150 m ahead
    behind = veh("behind", [3.5, 0, 5], [0, 0, 20], "hw_l1_a")  # 45 m behind, slower
    d = lane_change.evaluate(ego, "hw_l1_a", [ahead, behind], net)
    assert d.accept


def test_central_controller_applies_requested_safe_lane_change():
    net = networks.highway_straight(lanes=2, length=300.0)
    controller = CentralController(net)
    state = {
        "time": 1.0, "tick": 1, "scenario": "highway",
        "vehicles": [{
            "id": "ego", "type": "car", "position": [0, 0, 40],
            "velocity": [0, 0, 20], "acceleration": [0, 0, 0],
            "heading": 0, "current_lane": "hw_l0_a",
            "target_lane": "hw_l1_a", "has_goal": True,
            "goal": [3.5, 0, 280], "behavior_state": "LaneKeeping",
        }],
        "objects": [], "events": [],
    }
    cmd = controller.step(state)["commands"][0]
    assert cmd["target_lane"] == "hw_l1_a"
    assert cmd["behavior"] == "LaneChanging"
    assert len(cmd["path"]) >= 5
    assert cmd["path"][1][0] > cmd["path"][0][0]


def test_central_controller_rejects_unsafe_lane_change_request():
    net = networks.highway_straight(lanes=2, length=300.0)
    controller = CentralController(net)
    def vehicle(vid, x, z, lane, target=None):
        return {
            "id": vid, "type": "car", "position": [x, 0, z],
            "velocity": [0, 0, 20], "acceleration": [0, 0, 0],
            "heading": 0, "current_lane": lane, "target_lane": target,
            "has_goal": True, "goal": [x, 0, 280],
            "behavior_state": "LaneKeeping",
        }
    state = {
        "time": 1.0, "tick": 1, "scenario": "highway",
        "vehicles": [
            vehicle("ego", 0, 40, "hw_l0_a", "hw_l1_a"),
            vehicle("beside", 3.5, 41, "hw_l1_a"),
        ],
        "objects": [], "events": [],
    }
    cmd = next(c for c in controller.step(state)["commands"]
               if c["vehicle_id"] == "ego")
    assert cmd["target_lane"] == "hw_l0_a"
    assert cmd["behavior"] != "LaneChanging"


# ---- merge reservation --------------------------------------------------- #
def test_merge_clear_mainline():
    ramp = veh("ramp", [0, 0, 0], [0, 0, 20], "ramp")
    plan = merge.plan_merge(ramp, [], merge_point=[0, 0, 100])
    assert plan.feasible
    assert plan.ramp_target_speed == pytest.approx(20.0)


def test_merge_fits_existing_gap():
    ramp = veh("ramp", [0, 0, 0], [0, 0, 20], "ramp")  # ETA 5 s to z=100
    # mainline cars arriving at 1 s and 9 s -> big gap around 5 s
    m1 = veh("m1", [0, 0, 80], [0, 0, 20], "main")   # ETA 1 s
    m2 = veh("m2", [0, 0, -80], [0, 0, 20], "main")  # ETA 9 s
    plan = merge.plan_merge(ramp, [m1, m2], merge_point=[0, 0, 100])
    assert plan.feasible
    assert plan.reason == "fits existing gap"


def test_merge_opens_gap_when_packed():
    ramp = veh("ramp", [0, 0, 0], [0, 0, 20], "ramp")  # ETA 5 s
    # a dense stream all arriving near 5 s -> must retime or yield
    main = [veh(f"m{i}", [0, 0, 100 - 20 * (4.5 + 0.3 * i)], [0, 0, 20], "main")
            for i in range(4)]
    plan = merge.plan_merge(ramp, main, merge_point=[0, 0, 100])
    assert plan.feasible  # central control always finds a way (retime or yield)


# ---- hazard event -> replan / stop --------------------------------------- #
def test_hazard_blocks_lane_and_vehicle_stops():
    cl = [[0, 0, z] for z in range(0, 201, 5)]
    net = LaneNetwork([Lane(id="L", centerline=cl, next_lane_ids=[])],
                      scenario="highway")
    sim = HeadlessSim(net, dt=0.1)
    sim.add_vehicle("car", [0, 0, 0], "L", speed=15, goal=[0, 0, 190])
    # drive a bit, then drop an obstacle on the lane ahead
    for _ in range(20):
        sim.step()
    sim.events = [{"type": "FallingObject", "position": [0, 0, 90]}]
    stopped_before_hazard = False
    for _ in range(120):
        sim.step()
        sim.events = [{"type": "FallingObject", "position": [0, 0, 90]}]  # persists
        if sim.vehicles["car"].speed < 0.5 and sim.vehicles["car"].position[2] < 90:
            stopped_before_hazard = True
            break
    assert stopped_before_hazard, "car did not stop for the blocked lane"


# ---- metrics ------------------------------------------------------------- #
def test_metrics_from_rows():
    rows = [
        {"time": 0.0, "vehicle_id": "a", "speed": 20, "lateral_error": 0.1, "ttc": "",
         "behavior_state": "LaneKeeping", "collision_risk": 0},
        {"time": 1.0, "vehicle_id": "a", "speed": 10, "lateral_error": 2.0, "ttc": 2.5,
         "behavior_state": "Stopping", "collision_risk": 1},
        {"time": 2.0, "vehicle_id": "a", "speed": 0, "lateral_error": 0.1, "ttc": "",
         "behavior_state": "Arrived", "collision_risk": 0},
    ]
    m = metrics.from_rows(rows, lane_width=3.5)
    assert m.samples == 3
    assert m.avg_speed == pytest.approx(10.0)
    assert m.min_ttc == pytest.approx(2.5)
    assert m.lane_departures == 1            # |2.0| > 1.75
    assert m.hard_brakes == 2                # 20->10 and 10->0, both > 4 m/s^2
    assert m.arrivals == 1
    assert m.collision_risk_events == 1
