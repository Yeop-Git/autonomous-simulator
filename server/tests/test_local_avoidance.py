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


def test_evading_extends_the_escape_instead_of_stalling_on_its_last_waypoint():
    """Regression, on the real EmergencyAvoidance export.

    The escape path only reaches ~42 m ahead of where the manoeuvre began. A
    hazard dropped just beyond that (the scene's own 48 m trigger distance)
    put the path's last waypoint short of the obstacle, so the ego reached the
    end of its path with the hazard still alongside: never rejoining, never
    stopping, pinned on that waypoint at a commanded 8 m/s for the whole run.
    """
    from pathlib import Path

    from headless_sim import HeadlessSim

    export = Path(__file__).resolve().parents[1] / "scenarios" / \
        "EmergencyAvoidance_lanes.json"
    net = LaneNetwork.from_json(export)
    controller = CentralController(net, dt=0.1)
    sim = HeadlessSim(net, controller, dt=0.1, scenario=net.scenario)
    lane = net.lane("ea_center")
    sim.add_vehicle("ego", [0.0, 0.0, 20.0], "ea_center", speed=16.0,
                    goal=list(lane.centerline[-1]))

    phases, positions = [], []
    for tick in range(600):
        ego = sim.vehicles["ego"]
        if tick == 40:  # drop the cargo at the scene's own trigger distance
            sim.add_object("cargo", "unexpected_obstacle",
                           [ego.position[0], 0.0, ego.position[2] + 45.0],
                           [0.0, 0.0, 0.0], radius=1.25)
        behavior = sim.step()["commands"][0]["behavior"]
        if behavior not in phases[-1:]:
            phases.append(behavior)
        positions.append(sim.vehicles["ego"].position[2])
        if sim.vehicles["ego"].arrived:
            break

    assert "LateralEvading" in phases
    assert "ControlledStopping" not in phases
    assert "LaneRejoining" in phases, f"never rejoined; phases were {phases}"
    assert sim.vehicles["ego"].arrived
    # and it never sat still while being commanded to move
    longest_stall = 0
    stall = 0
    for a, b in zip(positions, positions[1:]):
        stall = stall + 1 if abs(b - a) < 1e-6 else 0
        longest_stall = max(longest_stall, stall)
    assert longest_stall < 20, f"stalled for {longest_stall} ticks mid-manoeuvre"


def _export(name):
    from pathlib import Path

    return LaneNetwork.from_json(
        Path(__file__).resolve().parents[1] / "scenarios" / f"{name}_lanes.json")


def test_blocked_escape_extension_holds_instead_of_evading_with_no_path():
    """Two cars avoiding the same obstacle: the second one's escape is blocked.

    The follower reaches the end of its escape path while the leader's own
    predicted swerve still crosses the escape lane, so the extension fails and
    ``_plan`` empties the path. Continuing to report LateralEvading then pinned
    it there for good: it was commanded 8 m/s with nothing to follow, and an
    empty path can never again satisfy the "at path end" retry condition.
    """
    from headless_sim import HeadlessSim

    net = _export("EmergencyAvoidance")
    sim = HeadlessSim(net, CentralController(net, dt=0.1), dt=0.1,
                      scenario="emergency_avoidance")
    sim.add_vehicle("ego", [0.0, 0.0, 40.0], "ea_center", speed=20.0,
                    goal=[0.0, 0.0, 320.0])
    sim.add_vehicle("follow", [0.0, 0.0, 10.0], "ea_center", speed=20.0,
                    goal=[0.0, 0.0, 320.0])
    sim.add_object("rock", "unexpected_obstacle", [0.0, 0.0, 88.0],
                   [0.0, 0.0, 0.0], radius=1.2)

    worst_moving_stall = 0
    stall = 0
    previous = None
    for _ in range(400):
        command = next(c for c in sim.step()["commands"]
                       if c["vehicle_id"] == "follow")
        follower = sim.vehicles["follow"]
        still = previous is not None and abs(follower.position[2] - previous) < 1e-6
        # Standing still is fine; standing still *while told to drive* is the bug.
        stall = stall + 1 if (still and command["target_speed"] > 0.5) else 0
        worst_moving_stall = max(worst_moving_stall, stall)
        previous = follower.position[2]

    assert worst_moving_stall < 20, (
        f"commanded to move while stationary for {worst_moving_stall} ticks")
    assert sim.vehicles["follow"].position[2] > 150.0, (
        "follower never got past the obstacle "
        f"(z={sim.vehicles['follow'].position[2]:.1f})")


def test_blocked_rejoin_keeps_driving_the_escape_lane_instead_of_stuttering():
    """The home lane stays occupied for a while after the obstacle is passed.

    A failed rejoin used to fall into ControlledStopping, whose retry replans
    the *escape* — which succeeds, which returns to LateralEvading, which asks
    to rejoin again on the very next tick. The ego stop-and-goes from 8 m/s to
    a dead halt once per cycle for as long as the lane is busy.
    """
    from headless_sim import HeadlessSim

    net = _export("EmergencyAvoidance")
    sim = HeadlessSim(net, CentralController(net, dt=0.1), dt=0.1,
                      scenario="emergency_avoidance")
    sim.add_vehicle("ego", [0.0, 0.0, 40.0], "ea_center", speed=20.0,
                    goal=[0.0, 0.0, 320.0])
    sim.add_object("rock", "unexpected_obstacle", [0.0, 0.0, 88.0],
                   [0.0, 0.0, 0.0], radius=1.2)
    for i in range(4):  # slow wall in the home lane, just past the obstacle
        sim.add_vehicle(f"wall{i}", [0.0, 0.0, 110.0 + 9.0 * i], "ea_center",
                        speed=2.0, goal=[0.0, 0.0, 320.0])

    behaviors, speeds = [], []
    for _ in range(220):
        command = next(c for c in sim.step()["commands"]
                       if c["vehicle_id"] == "ego")
        behaviors.append(command["behavior"])
        speeds.append(sim.vehicles["ego"].speed)

    manoeuvring = [s for s, b in zip(speeds, behaviors)
                   if b in {"LateralEvading", "RejoinPlanning"}]
    assert manoeuvring, "the ego never evaded"
    assert min(manoeuvring) > 3.0, (
        f"stop-and-go through the blocked rejoin (min {min(manoeuvring):.2f} m/s)")
    assert "ControlledStopping" not in behaviors, (
        "a busy home lane is not a reason to stop dead in the escape lane")
    assert sim.vehicles["ego"].position[2] > 200.0, "never got past the wall"


def test_a_unity_scene_reset_abandons_the_in_progress_manoeuvre():
    """The avoidance scene's reset key reloads the scene.

    That rebuilds the V2XClient, so Unity's tick counter starts over while this
    server keeps running with everything it remembered. The ego reappears at
    the start line with the hazard gone, and used to be told it was still
    mid-evasion — handed an escape path beginning 20 m ahead of where it now
    stands, at a commanded 8 m/s.
    """
    net = _export("EmergencyAvoidance")
    controller = CentralController(net, dt=0.1)

    def snapshot(tick, z, objects):
        return {
            "time": tick * 0.1, "tick": tick, "scenario": "emergency_avoidance",
            "vehicles": [{
                "id": "ego", "type": "car", "position": [0.0, 0.0, z],
                "velocity": [0.0, 0.0, 18.0], "acceleration": [0.0, 0.0, 0.0],
                "heading": 0.0, "current_lane": "ea_center", "target_lane": None,
                "maneuver": "straight", "has_goal": True,
                "goal": [0.0, 0.0, 320.0], "behavior_state": "LaneKeeping",
            }],
            "objects": objects, "events": [],
        }

    rock = [{"id": "falling_cargo", "type": "unexpected_obstacle",
             "position": [0.0, 0.0, 90.0], "velocity": [0.0, 0.0, 0.0],
             "radius": 1.25}]
    z = 40.0
    for tick in range(1, 20):
        controller.step(snapshot(tick, z, rock))
        z += 1.8
    assert controller.local_avoidance._active, "fixture never started an evasion"

    # the player hits reset: tick restarts, ego is back at the start, no hazard
    after = controller.step(snapshot(0, 20.0, []))["commands"][0]

    assert not controller.local_avoidance._active
    assert after["behavior"] not in {
        "LateralEvading", "EscapePlanning", "HazardDetected", "Yielding",
        "RejoinPlanning", "LaneRejoining", "ControlledStopping"}, (
        f"still evading a hazard that no longer exists: {after['behavior']}")
    assert after["path"][0][2] <= 21.0, (
        "handed a path that starts ahead of the vehicle: "
        f"{[round(v, 1) for v in after['path'][0]]}")


def test_avoidance_corridor_covers_every_road_family_in_the_scene():
    """IntegratedCity mixes ``city_`` and ``urban_`` lanes.

    The escape corridor used to be selected by lane-id prefix, which left the
    ego's own lane out of the search space on the urban half of the very same
    scene: every sample read as off-road, so the only reachable outcome was a
    permanent ControlledStopping.
    """
    from headless_sim import HeadlessSim

    net = _export("IntegratedCity")
    sim = HeadlessSim(net, CentralController(net, dt=0.1), dt=0.1,
                      scenario="integrated_city")
    sim.add_vehicle("ego", [5.4, 0.0, -70.0], "urban_nb_0_in", speed=12.0,
                    goal=[5.4, 0.0, 60.0])
    sim.add_object("rock", "unexpected_obstacle", [5.4, 0.0, -30.0],
                   [0.0, 0.0, 0.0], radius=1.2)

    behaviors = set()
    for _ in range(300):
        behaviors.add(sim.step()["commands"][0]["behavior"])

    assert "LateralEvading" in behaviors, (
        f"never planned an escape on an urban_ lane; saw {sorted(behaviors)}")
    assert sim.vehicles["ego"].position[2] > 0.0, (
        "never got past the obstacle at z=-30 "
        f"(z={sim.vehicles['ego'].position[2]:.1f})")
