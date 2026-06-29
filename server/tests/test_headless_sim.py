"""Integration tests: the headless sim driven by the central controller.

These stand in for the Phase 3 exit gate (multi-vehicle following with zero
collisions and maintained gaps) without needing Unity.
"""
import pytest

from headless_sim import HeadlessSim
from world_model import Lane, LaneNetwork


def long_single_lane(length=600.0, speed_limit=20.0):
    cl = [[0.0, 0.0, z] for z in range(0, int(length) + 1, 5)]
    return LaneNetwork([Lane(id="L", centerline=cl, width=3.5,
                             speed_limit=speed_limit)], scenario="highway")


def test_single_car_reaches_goal():
    net = long_single_lane()
    sim = HeadlessSim(net, dt=0.1)
    sim.add_vehicle("car", [0, 0, 0], "L", speed=0.0, goal=[0, 0, 590])
    for _ in range(800):
        sim.step()
        if sim.vehicles["car"].arrived:
            break
    assert sim.vehicles["car"].arrived
    assert sim.vehicles["car"].position[2] == pytest.approx(590, abs=5)


def test_following_train_no_collision():
    net = long_single_lane(length=2000.0, speed_limit=20.0)
    sim = HeadlessSim(net, dt=0.1)
    # 6 cars, 25 m apart, all routing far down the lane (no pile-up at goal)
    for i in range(6):
        sim.add_vehicle(f"car_{i}", [0, 0, 150 - i * 25.0], "L",
                        speed=15.0, goal=[0, 0, 1900])
    min_gap_seen = float("inf")
    for _ in range(200):  # ~20 s; lead stays well short of the goal
        sim.step()
        min_gap_seen = min(min_gap_seen, sim.min_pairwise_distance())
    # never rear-end: center spacing stays above one vehicle length
    assert min_gap_seen > 4.5, f"vehicles collided, min gap {min_gap_seen:.2f}"


def test_follower_slows_for_slow_leader():
    net = long_single_lane(length=600.0, speed_limit=25.0)
    sim = HeadlessSim(net, dt=0.1)
    sim.add_vehicle("lead", [0, 0, 60], "L", speed=5.0, goal=[0, 0, 100])
    sim.add_vehicle("follow", [0, 0, 20], "L", speed=20.0, goal=[0, 0, 590])
    # run until the leader arrives & stops near z=100
    for _ in range(200):
        sim.step()
    # follower must not have driven through the (stopped) leader
    assert sim.min_pairwise_distance() > 4.5


def test_emergency_brake_for_crossing_object():
    net = long_single_lane(length=300.0, speed_limit=20.0)
    sim = HeadlessSim(net, dt=0.1)
    car = sim.add_vehicle("car", [0, 0, 0], "L", speed=18.0, goal=[0, 0, 290])
    # a static obstacle sitting on the lane ahead
    sim.add_object("rock", "static_obstacle", [0, 0, 80], [0, 0, 0], radius=1.0)
    braked = False
    for _ in range(100):
        cmd = sim.step()
        b = cmd["commands"][0]["behavior"]
        if b in ("EmergencyBraking", "Stopping"):
            braked = True
        # never reach/pass the obstacle position
        assert sim.vehicles["car"].position[2] < 80.0 or braked
    assert braked, "expected the car to brake for the obstacle"
