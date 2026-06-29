"""Regression tests for bugs found in the deep review pass.

Each test fails on the pre-fix code and passes after the fix. Named by the
issue so a future regression points straight at the cause.
"""
import math

import pytest

from behavior import find_leader
from collision_predictor import CollisionPredictor, time_to_breach
from central_control import CentralController
from planners import AStarPlanner
from scenarios import networks
from world_model import DynamicVehicle, Lane, LaneNetwork


def veh(vid, pos, vel, lane="L"):
    return DynamicVehicle(id=vid, position=list(pos), velocity=list(vel),
                          heading=0.0, current_lane=lane)


# 1) collision predictor sample-tunneling --------------------------------- #
def test_fast_crosser_not_tunneled():
    # obstacle sweeps across the ego at 200 m/s — would pass between 0.2 s samples
    ego = veh("ego", [0, 0, 0], [0, 0, 0])
    fast = veh("fast", [-20, 0, 2], [200, 0, 0])
    ttc = time_to_breach(ego, fast, safety_distance=5.0, horizon=4.0)
    assert math.isfinite(ttc), "fast crosser tunneled through the sampling"
    assert 0.0 <= ttc <= 0.2

    pred = CollisionPredictor(safety_distance=5.0)
    c = pred.pair_conflict(ego, fast)
    assert c is not None and math.isfinite(c.ttc)


def test_diverging_pair_has_no_breach():
    a = veh("a", [0, 0, 0], [0, 0, -10])
    b = veh("b", [0, 0, 5], [0, 0, 10])  # already moving apart
    assert time_to_breach(a, b, safety_distance=4.0, horizon=5.0) == math.inf


def test_already_inside_safety_breaches_now():
    a = veh("a", [0, 0, 0], [0, 0, 0])
    b = veh("b", [0, 0, 2], [0, 0, 0])  # 2 m apart, safety 5
    assert time_to_breach(a, b, safety_distance=5.0) == 0.0


# 2) find_leader across the next lane segment ----------------------------- #
def test_find_leader_on_next_segment():
    a = Lane(id="L1", centerline=[[0, 0, 0], [0, 0, 100]], next_lane_ids=["L2"])
    b = Lane(id="L2", centerline=[[0, 0, 100], [0, 0, 200]], next_lane_ids=[])
    net = LaneNetwork([a, b])
    ego = veh("ego", [0, 0, 90], [0, 0, 15], lane="L1")
    leader = veh("lead", [0, 0, 110], [0, 0, 10], lane="L2")  # 20 m ahead, next lane
    res = find_leader(ego, [leader], net)
    assert res is not None and res.vehicle.id == "lead"
    assert res.gap == pytest.approx(20.0 - 4.5, abs=0.5)


# 3) urban_grid edge continuity (no teleport edges) ----------------------- #
def test_urban_grid_edges_are_continuous():
    net = networks.urban_grid(rows=2, cols=2, block=60.0)
    for lid in net.all_lane_ids():
        lane = net.lane(lid)
        for nxt in lane.next_lane_ids:
            nl = net.lane(nxt)
            d = math.hypot(lane.end[0] - nl.start[0], lane.end[2] - nl.start[2])
            assert d < 1e-6, f"teleport edge {lid}->{nxt}, gap {d:.1f} m"


def test_urban_grid_route_has_no_backward_jump():
    net = networks.urban_grid(rows=2, cols=2, block=60.0)
    planner = AStarPlanner()
    path = planner.plan([2, 0, 2], [118, 0, 118], net)
    assert path
    for i in range(len(path) - 1):
        step = math.hypot(path[i + 1][0] - path[i][0], path[i + 1][2] - path[i][2])
        assert step < 40.0, f"path jump of {step:.1f} m at index {i}"


# 4) A* single-lane: goal behind start must not be dropped ----------------- #
def test_single_lane_goal_behind_start_keeps_goal():
    cl = [[0, 0, z] for z in range(0, 31, 5)]
    net = LaneNetwork([Lane(id="L", centerline=cl, next_lane_ids=[])])
    planner = AStarPlanner()
    path = planner.plan([0, 0, 25], [0, 0, 5], net)  # goal behind start
    assert len(path) >= 2
    assert path[-1][2] == pytest.approx(5.0, abs=1.0)


def test_single_lane_forward_goal_ok():
    cl = [[0, 0, z] for z in range(0, 31, 5)]
    net = LaneNetwork([Lane(id="L", centerline=cl, next_lane_ids=[])])
    planner = AStarPlanner()
    path = planner.plan([0, 0, 5], [0, 0, 25], net)
    assert path[0][2] == pytest.approx(5.0, abs=1.0)
    assert path[-1][2] == pytest.approx(25.0, abs=1.0)


# 5) route cache invalidates when the vehicle leaves its path -------------- #
def test_route_replans_when_off_path():
    cl = [[0, 0, z] for z in range(0, 301, 5)]
    net = LaneNetwork([Lane(id="L", centerline=cl, next_lane_ids=[])],
                      scenario="highway")
    ctrl = CentralController(net)
    state = {
        "time": 0.0, "tick": 0, "scenario": "highway",
        "vehicles": [{
            "id": "car", "position": [0, 0, 5], "velocity": [0, 0, 15],
            "heading": 0.0, "current_lane": "L",
            "has_goal": True, "goal": [0, 0, 290]}],
        "objects": [], "events": [],
    }
    ctrl.step(state)
    assert ctrl.replans == 1
    # nudge the car along its path (stays on route) — must NOT replan
    state["tick"] = 1
    state["vehicles"][0]["position"] = [0, 0, 25]
    ctrl.step(state)
    assert ctrl.replans == 1
    # shove the car 4 m off its path laterally — must replan
    state["tick"] = 2
    state["vehicles"][0]["position"] = [4.0, 0, 40]
    ctrl.step(state)
    assert ctrl.replans == 2
