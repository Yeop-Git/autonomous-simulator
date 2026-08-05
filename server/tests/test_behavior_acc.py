"""Tests for the ACC longitudinal controller, leader detection, and FSM."""
import math

import pytest

from behavior import (ARRIVED, EMERGENCY_BRAKING, FOLLOWING, LANE_KEEPING,
                      STOPPING, BehaviorInputs, Leader, find_leader, next_behavior)
from controllers.acc import ACCController, ACCParams
from scenarios import networks
from world_model import DynamicVehicle, Lane, LaneNetwork


def veh(vid, z, vz=10.0, lane="hw_l0_a"):
    return DynamicVehicle(id=vid, position=[0, 0, z], velocity=[0, 0, vz],
                          heading=0.0, current_lane=lane)


# ---- ACC ------------------------------------------------------------------ #
def test_acc_free_flow_accelerates_toward_free_speed():
    acc = ACCController()
    out = acc.target_speed(ego_speed=10.0, free_speed=20.0,
                           leader_gap=None, leader_speed=None, dt=0.1)
    assert 10.0 < out <= 10.0 + acc.p.max_accel * 0.1 + 1e-9


def test_acc_caps_at_free_speed():
    acc = ACCController()
    out = acc.target_speed(ego_speed=20.0, free_speed=20.0,
                           leader_gap=None, leader_speed=None, dt=1.0)
    assert out == pytest.approx(20.0)


def test_acc_slows_for_close_leader():
    acc = ACCController()
    # leader very close and slow -> ego must decelerate
    out = acc.target_speed(ego_speed=20.0, free_speed=25.0,
                           leader_gap=2.0, leader_speed=5.0, dt=0.1)
    assert out < 20.0


def test_acc_emergency_decel_when_inside_standstill_gap():
    acc = ACCController(ACCParams(standstill_gap=4.0))
    out = acc.target_speed(ego_speed=20.0, free_speed=25.0,
                           leader_gap=1.0, leader_speed=0.0, dt=0.1)
    # Command a full stop; Unity applies its physical deceleration limit.
    assert out == 0.0


def test_acc_settles_behind_stopped_queue_instead_of_creeping_forever():
    acc = ACCController()
    out = acc.target_speed(ego_speed=0.08, free_speed=13.9,
                           leader_gap=6.4, leader_speed=0.0, dt=0.1)
    assert out == 0.0


def test_acc_resumes_when_stopped_queue_moves():
    acc = ACCController()
    out = acc.target_speed(ego_speed=0.0, free_speed=13.9,
                           leader_gap=6.4, leader_speed=1.0, dt=0.1)
    assert out > 0.0


def test_acc_keeps_gap_at_steady_state():
    acc = ACCController()
    gap = acc.desired_gap(15.0)
    # at exactly the desired gap and matching speed, hold leader speed
    out = acc.target_speed(ego_speed=15.0, free_speed=30.0,
                           leader_gap=gap, leader_speed=15.0, dt=0.1)
    assert out == pytest.approx(15.0, abs=0.3)


# ---- leader detection ----------------------------------------------------- #
def test_find_leader_same_lane_ahead():
    net = networks.highway_straight(lanes=1, length=300.0)
    ego = veh("ego", 10.0)
    lead = veh("lead", 40.0)
    other = veh("behind", 5.0)
    res = find_leader(ego, [lead, other], net)
    assert res is not None
    assert res.vehicle.id == "lead"
    assert res.gap == pytest.approx(30.0 - 4.5, abs=0.5)


def test_find_leader_ignores_other_lane():
    net = networks.highway_straight(lanes=2, length=300.0)
    ego = veh("ego", 10.0, lane="hw_l0_a")
    lead = veh("lead", 40.0, lane="hw_l1_a")  # different lane
    assert find_leader(ego, [lead], net) is None


def test_find_leader_none_when_alone():
    net = networks.highway_straight(lanes=1, length=300.0)
    assert find_leader(veh("ego", 10.0), [], net) is None


def test_find_leader_survives_stale_lane_tag_at_intersection_boundary():
    incoming = Lane(id="in", centerline=[[0, 0, -20], [0, 0, 0]],
                    next_lane_ids=["connector"])
    connector = Lane(id="connector", centerline=[[0, 0, 0], [0, 0, 20]])
    net = LaneNetwork([incoming, connector])
    ego = DynamicVehicle(id="ego", position=[0, 0, 1], velocity=[0, 0, 3],
                         heading=0.0, current_lane="connector")
    leader = DynamicVehicle(id="lead", position=[0, 0, 8], velocity=[0, 0, 0],
                            heading=0.0, current_lane="in")

    result = find_leader(ego, [leader], net)

    assert result is not None and result.vehicle.id == "lead"
    assert result.gap == pytest.approx(2.5)


def test_find_leader_corridor_fallback_ignores_cross_traffic():
    north = Lane(id="north", centerline=[[0, 0, -20], [0, 0, 20]])
    east = Lane(id="east", centerline=[[-20, 0, 4], [20, 0, 4]])
    net = LaneNetwork([north, east])
    ego = DynamicVehicle(id="ego", position=[0, 0, 0], velocity=[0, 0, 3],
                         heading=0.0, current_lane="north")
    crossing = DynamicVehicle(id="cross", position=[0, 0, 4], velocity=[3, 0, 0],
                              heading=90.0, current_lane="east")

    assert find_leader(ego, [crossing], net) is None


# ---- FSM ------------------------------------------------------------------ #
def _inp(**kw):
    base = dict(has_goal=True, arrived=False, route_found=True,
                leader=None, min_ttc=math.inf)
    base.update(kw)
    return BehaviorInputs(**base)


def test_fsm_emergency_overrides_everything():
    assert next_behavior(_inp(min_ttc=1.0, arrived=True)) == EMERGENCY_BRAKING


def test_fsm_arrived():
    assert next_behavior(_inp(arrived=True)) == ARRIVED


def test_fsm_stops_when_no_route():
    assert next_behavior(_inp(route_found=False)) == STOPPING


def test_fsm_following_when_close_leader():
    lead = Leader(vehicle=veh("l", 30.0), gap=10.0, speed=8.0)
    assert next_behavior(_inp(leader=lead)) == FOLLOWING


def test_fsm_lane_keeping_default():
    lead = Leader(vehicle=veh("l", 200.0), gap=120.0, speed=20.0)
    assert next_behavior(_inp(leader=lead)) == LANE_KEEPING
