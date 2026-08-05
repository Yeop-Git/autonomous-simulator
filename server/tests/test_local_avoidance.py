import math

from central_control import CentralController
from local_avoidance import LocalAvoidanceManager
from planners.avoidance_world import AvoidanceWorld
from world_model import DynamicObject, DynamicVehicle, Lane, LaneNetwork, WorldModel


def network():
    lanes = []
    xs = [-3.5, 0.0, 3.5, 7.0]
    ids = ["ea_left", "ea_center", "ea_right", "ea_shoulder"]
    for i, (lane_id, x) in enumerate(zip(ids, xs)):
        lanes.append(Lane(
            lane_id, [[x, 0.0, 0.0], [x, 0.0, 120.0]],
            width=3.5, speed_limit=16.0,
            left_lane_id=ids[i - 1] if i > 0 else None,
            right_lane_id=ids[i + 1] if i + 1 < len(ids) else None))
    return LaneNetwork(lanes, name="EmergencyAvoidance",
                       scenario="emergency_avoidance")


def vehicle(lane="ea_center", x=0.0, z=10.0):
    return DynamicVehicle(
        id="ego", position=[x, 0.0, z], velocity=[0.0, 0.0, 10.0],
        heading=0.0, current_lane=lane, has_goal=True,
        goal=[0.0, 0.0, 110.0])


def state(tick=1, objects=None, lane="ea_center", x=0.0, z=10.0,
          planner="rrt"):
    return {
        "time": tick * 0.1, "tick": tick,
        "scenario": "emergency_avoidance", "planner_mode": planner,
        "vehicles": [{
            "id": "ego", "position": [x, 0.0, z],
            "velocity": [0.0, 0.0, 10.0], "heading": 0.0,
            "current_lane": lane, "has_goal": True,
            "goal": [0.0, 0.0, 110.0], "maneuver": "straight",
        }],
        "objects": objects or [], "events": [],
    }


def test_avoidance_world_blocks_obstacle_and_road_exterior():
    net = network()
    obstacle = DynamicObject(
        "box", "unexpected_obstacle", [0.0, 0.0, 35.0],
        [0.0, 0.0, 0.0], radius=1.0)
    world = AvoidanceWorld(net, [vehicle()], [obstacle], exclude_ids={"ego"})

    assert world.is_blocked([0.0, 0.0, 35.0])
    assert not world.is_blocked([3.5, 0.0, 35.0])
    assert world.is_blocked([14.0, 0.0, 35.0])


def test_obstacle_switches_astar_to_rrt_and_returns_valid_local_path():
    controller = CentralController(network())
    obstacle = [{
        "id": "fallen", "type": "unexpected_obstacle",
        "position": [0.0, 0.0, 42.0],
        "velocity": [0.0, 0.0, 0.0], "radius": 1.0,
    }]

    first = controller.step(state(1, obstacle))["commands"][0]
    second = controller.step(state(2, obstacle))["commands"][0]
    planned = controller.step(state(3, obstacle))["commands"][0]

    assert first["behavior"] == "HazardDetected"
    assert second["behavior"] == "EscapePlanning"
    assert planned["behavior"] == "LateralEvading"
    assert planned["planner"] == "rrt"
    assert planned["plan_status"] == "active"
    assert planned["target_lane"] == "ea_right"
    assert len(planned["path"]) >= 3
    assert planned["planning_time_ms"] >= 0.0
    assert planned["minimum_clearance"] >= -1e-6


def test_emergency_vehicle_targets_rightmost_shoulder():
    controller = CentralController(network())
    ambulance = [{
        "id": "ambulance", "type": "emergency_vehicle",
        "position": [0.0, 0.0, -35.0],
        "velocity": [0.0, 0.0, 24.0], "radius": 1.0,
    }]

    controller.step(state(1, ambulance))
    controller.step(state(2, ambulance))
    planned = controller.step(state(3, ambulance))["commands"][0]

    assert planned["behavior"] == "LateralEvading"
    assert planned["target_lane"] == "ea_shoulder"
    assert planned["turn_signal"] == "right"
    assert planned["target_speed"] <= 5.0


def test_rrt_star_mode_is_reported_and_budgeted():
    controller = CentralController(network())
    obstacle = [{
        "id": "fallen", "type": "unexpected_obstacle",
        "position": [0.0, 0.0, 42.0],
        "velocity": [0.0, 0.0, 0.0], "radius": 1.0,
    }]
    controller.step(state(1, obstacle, planner="rrt_star"))
    controller.step(state(2, obstacle, planner="rrt_star"))
    planned = controller.step(state(3, obstacle, planner="rrt_star"))["commands"][0]

    assert planned["planner"] == "rrt_star"
    assert planned["planning_time_ms"] < 500.0
    assert planned["behavior"] in {"LateralEvading", "ControlledStopping"}
