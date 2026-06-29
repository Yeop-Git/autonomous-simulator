"""Unit tests for the A* lane-graph planner (synthetic graphs, no Unity)."""
import math

import pytest

from planners import AStarPlanner
from scenarios import networks
from world_model import Lane, LaneNetwork, polyline_length


def _path_len(path):
    return polyline_length(path)


def test_plan_single_lane_straight():
    net = networks.highway_straight(lanes=1, length=100.0)
    planner = AStarPlanner()
    path = planner.plan([0.0, 0.0, 5.0], [0.0, 0.0, 90.0], net)
    assert path, "expected a non-empty path"
    # starts near start, ends near goal
    assert path[0][2] == pytest.approx(5.0, abs=1.0)
    assert path[-1][2] == pytest.approx(90.0, abs=1.0)
    # roughly the straight-line distance
    assert _path_len(path) == pytest.approx(85.0, abs=2.0)


def test_plan_spans_two_segments():
    net = networks.highway_straight(lanes=1, length=200.0)
    planner = AStarPlanner()
    path = planner.plan([0.0, 0.0, 10.0], [0.0, 0.0, 190.0], net)
    assert path
    assert planner.last_lane_route == ["hw_l0_a", "hw_l0_b"]
    assert path[-1][2] == pytest.approx(190.0, abs=1.0)


def test_plan_routes_through_grid():
    net = networks.urban_grid(rows=2, cols=2, block=60.0)
    planner = AStarPlanner()
    # bottom-left origin to somewhere up-right
    path = planner.plan([0.0, 0.0, 2.0], [120.0, 0.0, 118.0], net)
    assert path
    assert len(planner.last_lane_route) >= 2
    # path should make monotone-ish progress toward the goal corner
    assert path[-1][0] == pytest.approx(120.0, abs=3.0)


def test_no_path_when_goal_unreachable():
    # two disconnected lanes
    a = Lane(id="a", centerline=[[0, 0, 0], [0, 0, 50]], next_lane_ids=[])
    b = Lane(id="b", centerline=[[1000, 0, 0], [1000, 0, 50]], next_lane_ids=[])
    net = LaneNetwork([a, b])
    planner = AStarPlanner()
    path = planner.plan([0, 0, 5], [1000, 0, 45], net)
    assert path == []


def test_plan_avoids_blocked_lane_entry():
    # straight chain a -> b -> c; block entry of b so search must fail
    a = Lane(id="a", centerline=[[0, 0, 0], [0, 0, 50]], next_lane_ids=["b"])
    b = Lane(id="b", centerline=[[0, 0, 50], [0, 0, 100]], next_lane_ids=["c"])
    c = Lane(id="c", centerline=[[0, 0, 100], [0, 0, 150]], next_lane_ids=[])
    net = LaneNetwork([a, b, c])
    net.block([0, 0, 50], radius=2.0)  # blocks b's entry
    planner = AStarPlanner()
    path = planner.plan([0, 0, 5], [0, 0, 145], net)
    assert path == []


def test_chooses_shorter_of_two_routes():
    # diamond: s -> {short, longA->longB} -> g
    s = Lane(id="s", centerline=[[0, 0, 0], [0, 0, 10]], next_lane_ids=["short", "longA"])
    short = Lane(id="short", centerline=[[0, 0, 10], [0, 0, 60]], next_lane_ids=["g"])
    longA = Lane(id="longA", centerline=[[0, 0, 10], [40, 0, 35]], next_lane_ids=["longB"])
    longB = Lane(id="longB", centerline=[[40, 0, 35], [0, 0, 60]], next_lane_ids=["g"])
    g = Lane(id="g", centerline=[[0, 0, 60], [0, 0, 80]], next_lane_ids=[])
    net = LaneNetwork([s, short, longA, longB, g])
    planner = AStarPlanner()
    planner.plan([0, 0, 1], [0, 0, 78], net)
    assert "short" in planner.last_lane_route
    assert "longA" not in planner.last_lane_route


def test_empty_when_no_lanes():
    net = LaneNetwork([])
    planner = AStarPlanner()
    assert planner.plan([0, 0, 0], [1, 0, 1], net) == []
