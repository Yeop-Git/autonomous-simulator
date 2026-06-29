"""Unit tests for the static lane graph and dynamic world model."""
import math

import pytest

from scenarios import networks
from world_model import Lane, LaneNetwork, WorldModel, dist_xz, polyline_length


def test_lane_length_and_endpoints():
    lane = Lane(id="l", centerline=[[0, 0, 0], [0, 0, 10], [0, 0, 30]])
    assert lane.length == pytest.approx(30.0)
    assert lane.start == [0, 0, 0]
    assert lane.end == [0, 0, 30]


def test_closest_point_projects_onto_segment():
    lane = Lane(id="l", centerline=[[0, 0, 0], [0, 0, 100]])
    point, lat, arc = lane.closest_point([3.0, 0.0, 40.0])
    assert point[0] == pytest.approx(0.0)
    assert point[2] == pytest.approx(40.0)
    assert lat == pytest.approx(3.0)
    assert arc == pytest.approx(40.0)


def test_closest_point_clamps_before_start():
    lane = Lane(id="l", centerline=[[0, 0, 0], [0, 0, 100]])
    point, lat, arc = lane.closest_point([0.0, 0.0, -20.0])
    assert arc == pytest.approx(0.0)
    assert point[2] == pytest.approx(0.0)


def test_network_neighbors_and_centerline():
    net = networks.highway_straight(lanes=2, length=100.0)
    assert "hw_l0_a" in net.all_lane_ids()
    assert net.neighbors("hw_l0_a") == ["hw_l0_b"]
    assert net.neighbors("hw_l0_b") == []
    cl = net.lane_centerline("hw_l0_a")
    assert len(cl) >= 2


def test_nearest_lane():
    net = networks.highway_straight(lanes=3, length=100.0, lane_width=3.5)
    # a point right on lane 1's line (x = 3.5)
    assert net.nearest_lane([3.5, 0.0, 20.0]) in ("hw_l1_a", "hw_l1_b")
    # a point near lane 0
    assert net.nearest_lane([0.1, 0.0, 10.0]) == "hw_l0_a"


def test_blocking_marks_positions():
    net = networks.highway_straight(lanes=1, length=50.0)
    assert not net.is_blocked([0.0, 0.0, 25.0])
    net.block([0.0, 0.0, 25.0], radius=2.0)
    assert net.is_blocked([1.0, 0.0, 25.0])
    assert not net.is_blocked([10.0, 0.0, 25.0])
    net.clear_blocks()
    assert not net.is_blocked([0.0, 0.0, 25.0])


def test_world_model_ingests_state_message():
    net = networks.highway_straight(lanes=2, length=100.0)
    wm = WorldModel(net)
    state = {
        "time": 1.5,
        "tick": 3,
        "scenario": "highway",
        "vehicles": [
            {
                "id": "car_01",
                "position": [0.0, 0.0, 10.0],
                "velocity": [0.0, 0.0, 15.0],
                "heading": 0.0,
                "current_lane": "hw_l0_a",
            }
        ],
        "objects": [],
        "events": [],
    }
    wm.update_from_state(state)
    assert wm.tick == 3
    assert wm.time == pytest.approx(1.5)
    v = wm.vehicle("car_01")
    assert v is not None
    assert v.speed == pytest.approx(15.0)


def test_world_model_applies_hazard_events():
    net = networks.highway_straight(lanes=1, length=100.0)
    wm = WorldModel(net)
    state = {
        "time": 0.0,
        "tick": 0,
        "vehicles": [],
        "objects": [],
        "events": [{"type": "FallingObject", "position": [0.0, 0.0, 50.0]}],
    }
    wm.update_from_state(state)
    assert net.is_blocked([0.0, 0.0, 50.0])


def test_json_round_trip(tmp_path):
    net = networks.urban_grid(rows=2, cols=2)
    p = tmp_path / "net.json"
    import json

    p.write_text(json.dumps(net.to_dict()), encoding="utf-8")
    loaded = LaneNetwork.from_json(p)
    assert set(loaded.all_lane_ids()) == set(net.all_lane_ids())
    for lid in net.all_lane_ids():
        assert loaded.neighbors(lid) == net.neighbors(lid)


def test_helpers():
    assert dist_xz([0, 5, 0], [3, 99, 4]) == pytest.approx(5.0)
    assert polyline_length([[0, 0, 0], [0, 0, 3], [0, 0, 7]]) == pytest.approx(7.0)
