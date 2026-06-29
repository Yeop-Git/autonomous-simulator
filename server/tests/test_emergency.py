"""Emergency-vehicle yielding."""
import emergency
from central_control import CentralController
from scenarios import networks
from world_model import DynamicObject, DynamicVehicle


def veh(vid, pos, vel):
    return DynamicVehicle(id=vid, position=list(pos), velocity=list(vel),
                          heading=0.0, current_lane="hw_l0_a")


def ev(pos, vel):
    return DynamicObject(id="amb", type="emergency_vehicle",
                         position=list(pos), velocity=list(vel))


def test_yields_to_approaching_emergency_from_behind():
    ego = veh("ego", [0, 0, 100], [0, 0, 20])
    amb = ev([0, 0, 60], [0, 0, 35])  # behind, faster, catching up
    assert emergency.approaching_emergency(ego, [amb]) is amb
    assert emergency.yield_speed(ego, [amb]) == emergency.YIELD_SPEED


def test_no_yield_when_emergency_far():
    ego = veh("ego", [0, 0, 100], [0, 0, 20])
    amb = ev([0, 0, 0], [0, 0, 35])  # 100 m back, beyond radius
    assert emergency.yield_speed(ego, [amb]) is None


def test_no_yield_when_emergency_receding():
    ego = veh("ego", [0, 0, 100], [0, 0, 20])
    amb = ev([0, 0, 130], [0, 0, 35])  # ahead and pulling away
    assert emergency.approaching_emergency(ego, [amb]) is None


def test_controller_commands_yield_speed():
    net = networks.highway_straight(lanes=1, length=300.0)
    ctrl = CentralController(net)
    state = {
        "time": 0.0, "tick": 0, "scenario": "highway",
        "vehicles": [{"id": "ego", "position": [0, 0, 100], "velocity": [0, 0, 20],
                      "heading": 0.0, "current_lane": "hw_l0_a",
                      "has_goal": False, "goal": [0, 0, 0]}],
        "objects": [{"id": "amb", "type": "emergency_vehicle",
                     "position": [0, 0, 70], "velocity": [0, 0, 35], "radius": 1.0}],
        "events": [],
    }
    cmd = ctrl.step(state)["commands"][0]
    assert cmd["target_speed"] <= emergency.YIELD_SPEED
