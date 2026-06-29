"""Unit tests for kinematic collision prediction."""
import math

import pytest

from collision_predictor import (CollisionPredictor, closest_approach,
                                  predict_position, time_to_breach)
from world_model import DynamicObject, DynamicVehicle


def veh(vid, pos, vel):
    return DynamicVehicle(id=vid, position=list(pos), velocity=list(vel),
                          heading=0.0, current_lane="l")


def test_predict_position_constant_velocity():
    v = veh("a", [0, 0, 0], [0, 0, 10])
    assert predict_position(v, 2.0) == [0, 0, 20]


def test_closest_approach_head_on():
    # two cars closing along z, 100 m apart, 10 m/s each
    a = veh("a", [0, 0, 0], [0, 0, 10])
    b = veh("b", [0, 0, 100], [0, 0, -10])
    t, d = closest_approach(a, b, horizon=10.0)
    assert t == pytest.approx(5.0, abs=0.01)
    assert d == pytest.approx(0.0, abs=0.01)


def test_closest_approach_parallel_constant_gap():
    a = veh("a", [0, 0, 0], [0, 0, 10])
    b = veh("b", [4, 0, 0], [0, 0, 10])  # parallel lane, same speed
    t, d = closest_approach(a, b, horizon=5.0)
    assert d == pytest.approx(4.0)


def test_time_to_breach_detects_crossing():
    a = veh("a", [0, 0, 0], [0, 0, 10])
    b = veh("b", [0, 0, 50], [0, 0, -10])
    ttc = time_to_breach(a, b, safety_distance=5.0, horizon=5.0, dt=0.1)
    # they meet at t=2.5s; breach a bit before
    assert 2.0 < ttc <= 2.5


def test_no_conflict_when_far_and_diverging():
    a = veh("a", [0, 0, 0], [0, 0, -10])
    b = veh("b", [0, 0, 100], [0, 0, 10])  # moving apart
    pred = CollisionPredictor(safety_distance=5.0)
    assert pred.pair_conflict(a, b) is None


def test_conflicts_sorted_by_ttc():
    ego = veh("ego", [0, 0, 0], [0, 0, 10])
    near = veh("near", [0, 0, 20], [0, 0, -10])   # meet ~1s
    far = veh("far", [0, 0, 80], [0, 0, -10])     # meet ~4s
    pred = CollisionPredictor(horizon=6.0, safety_distance=5.0)
    cs = pred.conflicts([ego, near, far])
    assert cs, "expected conflicts"
    assert cs[0].ttc <= cs[-1].ttc


def test_object_type_participates():
    a = veh("car", [0, 0, 0], [0, 0, 5])
    ped = DynamicObject(id="ped", type="pedestrian", position=[0, 0, 10],
                        velocity=[0, 0, 0], radius=0.4)
    pred = CollisionPredictor(safety_distance=3.0)
    c = pred.pair_conflict(a, ped)
    assert c is not None
    assert c.a_id == "car" and c.b_id == "ped"
