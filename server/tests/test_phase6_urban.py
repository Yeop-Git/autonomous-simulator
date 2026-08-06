"""Phase 6 — urban: traffic lights, intersection reservation, pedestrian yield."""
import math
from pathlib import Path

import pytest

from headless_sim import HeadlessSim
from central_control import CentralController
from intersection import IntersectionManager
from traffic import GREEN, RED, YELLOW, TrafficLight, TrafficLightManager
from world_model import DynamicVehicle, Lane, LaneNetwork


URBAN_NETWORK = Path(__file__).parents[1] / "scenarios" / "Urban_lanes.json"


def veh(vid, pos, vel, lane="L"):
    return DynamicVehicle(id=vid, position=list(pos), velocity=list(vel),
                          heading=0.0, current_lane=lane)


# ---- traffic lights ------------------------------------------------------ #
def test_light_cycle():
    light = TrafficLight("l", stop_line=[0, 0, 100], green_time=10, yellow_time=3,
                         red_time=12)
    assert light.state(0) == GREEN
    assert light.state(9.9) == GREEN
    assert light.state(11) == YELLOW
    assert light.state(20) == RED
    assert light.state(25 + 0.0) == GREEN  # 25 == period -> wraps to green


def test_should_stop_on_red():
    mgr = TrafficLightManager()
    mgr.add(TrafficLight("l", stop_line=[0, 0, 100], approach_heading=0.0,
                         green_time=0, yellow_time=0, red_time=10))  # always red
    assert mgr.should_stop([0, 0, 80], speed=10, light_id="l", t=1.0)
    # past the line -> no stop
    assert not mgr.should_stop([0, 0, 110], speed=10, light_id="l", t=1.0)


def test_no_stop_on_green():
    mgr = TrafficLightManager()
    mgr.add(TrafficLight("l", stop_line=[0, 0, 100], green_time=10, yellow_time=3,
                         red_time=10))
    assert not mgr.should_stop([0, 0, 80], speed=10, light_id="l", t=1.0)


def test_protected_left_has_clearance_before_pedestrian_phase():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    left = controller.traffic.lights["urban_nb_1_in"]

    assert left.state(41.9) == GREEN
    assert left.state(42.1) == YELLOW
    assert left.state(44.1) == RED
    assert not controller._is_pedestrian_phase(46.9)
    assert controller._is_pedestrian_phase(47.0)
    assert left.state(47.0) == RED


def test_yellow_stops_only_if_stoppable():
    mgr = TrafficLightManager()
    # yellow at t in [10,13)
    mgr.add(TrafficLight("l", stop_line=[0, 0, 100], green_time=10, yellow_time=3,
                         red_time=10))
    # slow + far => can stop => should stop
    assert mgr.should_stop([0, 0, 80], speed=5, light_id="l", t=11.0)
    # fast + close => can't stop comfortably => proceed
    assert not mgr.should_stop([0, 0, 98], speed=20, light_id="l", t=11.0)


def test_signal_priority_prevents_crossing_deadlock():
    north = Lane("urban_north", [[-1.8, 0, -70], [-1.8, 0, 70]])
    east = Lane("urban_east", [[-70, 0, 1.8], [70, 0, 1.8]])
    controller = CentralController(LaneNetwork(
        [north, east], scenario="urban"))

    def vehicle(vid, pos, velocity, lane, goal, heading):
        return {
            "id": vid, "type": "car", "position": pos,
            "velocity": velocity, "acceleration": [0, 0, 0],
            "heading": heading, "current_lane": lane, "target_lane": None,
            "has_goal": True, "goal": goal, "behavior_state": "LaneKeeping",
        }
    state = {
        "time": 0.0, "tick": 1, "scenario": "urban",
        "vehicles": [
            vehicle("north", [-1.8, 0, -20], [0, 0, 10],
                    "urban_north", [-1.8, 0, 60], 0),
            vehicle("east", [-20, 0, 1.8], [10, 0, 0],
                    "urban_east", [60, 0, 1.8], 90),
        ],
        "objects": [], "events": [],
    }
    commands = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}
    assert commands["north"]["target_speed"] > 0
    assert commands["north"]["behavior"] != "EmergencyBraking"
    assert 0 < commands["east"]["target_speed"] < 13.9
    assert commands["east"]["behavior"] == "WaitingAtIntersection"


def test_protected_left_phase_holds_oncoming_traffic():
    left = Lane("urban_nb_1_in", [[1.8, 0, -70], [1.8, 0, -11]])
    south = Lane("urban_sb_0_in", [[-5.4, 0, 70], [-5.4, 0, 11]])
    controller = CentralController(LaneNetwork([left, south], scenario="urban"))

    def state_vehicle(vid, pos, velocity, lane, heading):
        return {
            "id": vid, "type": "car", "position": pos,
            "velocity": velocity, "acceleration": [0, 0, 0],
            "heading": heading, "current_lane": lane, "target_lane": None,
            "has_goal": False, "goal": pos, "behavior_state": "LaneKeeping",
        }
    state = {
        "time": 36.0, "tick": 1, "scenario": "urban",
        "vehicles": [
            state_vehicle("left", [1.8, 0, -20], [0, 0, 8], "urban_nb_1_in", 0),
            state_vehicle("oncoming", [-5.4, 0, 20], [0, 0, -8], "urban_sb_0_in", 180),
        ],
        "objects": [], "events": [],
    }
    commands = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}
    assert commands["left"]["target_speed"] > 0
    assert commands["left"]["behavior"] != "EmergencyBraking"
    # Center is four metres from the line, inside the enlarged 5.5 m buffer.
    assert commands["oncoming"]["target_speed"] == 0.0
    assert commands["oncoming"]["behavior"] == "WaitingAtIntersection"


def _left_turn_state(time, position, lane, target_lane=None, speed=8.0):
    return {
        "time": time, "tick": 1, "scenario": "urban",
        "vehicles": [{
            "id": "ego", "type": "car", "position": position,
            "velocity": [0, 0, speed], "acceleration": [0, 0, 0],
            "heading": 0, "current_lane": lane, "target_lane": target_lane,
            "maneuver": "left", "has_goal": True,
            "goal": [-191, 0, 5.4], "behavior_state": "LaneKeeping",
        }],
        "objects": [], "events": [],
    }


def _queued_vehicle(vehicle_id, position, lane, speed=0.0):
    return {
        "id": vehicle_id, "type": "car", "position": position,
        "velocity": [0, 0, speed], "acceleration": [0, 0, 0],
        "heading": 0, "current_lane": lane, "target_lane": None,
        "maneuver": "left", "has_goal": True,
        "goal": [-191, 0, 5.4], "behavior_state": "WaitingAtIntersection",
    }


def test_left_strategy_changes_lane_then_uses_protected_turn_connector():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    cmd = controller.step(_left_turn_state(
        5.0, [5.4, 0, -64], "urban_nb_0_in", "urban_nb_1_in"))["commands"][0]

    assert cmd["behavior"] == "LaneChanging"
    assert cmd["target_lane"] == "urban_nb_1_in"
    assert cmd["left_turn_phase"] == "LaneChanging"
    assert cmd["turn_signal"] == "left"
    path = cmd["path"]
    assert path[1][0] < path[0][0], "first complete the move into the left lane"
    assert path[4][2] <= -30.0, "lane change must finish well before the stop line"
    assert len(path) == 5
    assert all(p[2] < -20 for p in path), \
        "lane-change phase must not expose intersection waypoints"


def test_left_change_accepts_runtime_moving_leader_gap_and_merges_behind():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        5.0, [5.4, 0, -61.08], "urban_nb_0_in",
        "urban_nb_1_in", speed=4.0)
    state["vehicles"].append(_queued_vehicle(
        "moving_left_leader", [1.8, 0, -45.08],
        "urban_nb_1_in", speed=4.0))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["left_turn_phase"] == "LaneChanging"
    assert cmd["target_lane"] == "urban_nb_1_in"
    assert 12.0 <= cmd["path"][-1][2] - state["vehicles"][0]["position"][2] < 20.0


def test_left_lane_change_ends_behind_stopped_target_lane_queue():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        5.0, [5.4, 0, -64], "urban_nb_0_in", "urban_nb_1_in")
    state["vehicles"].append(_queued_vehicle(
        "left_queue", [1.8, 0, -29], "urban_nb_1_in"))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["behavior"] == "LaneChanging"
    assert cmd["path"][4][2] <= -39.0, \
        "merge endpoint must retain the configured six-metre bumper gap"
    assert cmd["target_speed"] < 8.0, \
        "ego must already match the stopped target-lane queue"


def test_left_lane_change_waits_before_deadline_when_queue_gap_is_too_small():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        5.0, [5.4, 0, -64], "urban_nb_0_in", "urban_nb_1_in")
    state["vehicles"].append(_queued_vehicle(
        "left_queue", [1.8, 0, -48], "urban_nb_1_in"))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["target_lane"] == "urban_nb_0_in"
    assert cmd["behavior"] == "WaitingAtIntersection"
    assert cmd["target_speed"] < 8.0
    assert cmd["path"][-1][2] <= -35.0, \
        "do not let a rejected request retain a route through the intersection"


def test_left_lane_queue_is_followed_after_lane_change_completes():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [1.8, 0, -52], "urban_nb_1_in", speed=8.0)
    state["vehicles"].append(_queued_vehicle(
        "left_queue", [1.8, 0, -36], "urban_nb_1_in"))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["behavior"] == "Following"
    assert cmd["target_speed"] < 8.0


def test_left_strategy_waits_on_arrow_red_after_lane_change():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    cmd = controller.step(_left_turn_state(
        25.0, [1.8, 0, -21], "urban_nb_1_in", speed=6.0))["commands"][0]

    assert controller.traffic.state("urban_nb_0_in", 25.0) == GREEN
    assert controller.traffic.state("urban_nb_1_in", 25.0) == RED
    assert cmd["behavior"] == "WaitingAtIntersection"
    assert cmd["target_speed"] == 0.0
    assert cmd["left_turn_phase"] == "SignalWaiting"
    assert all(p[2] <= -16.0 for p in cmd["path"])


def test_left_strategy_does_not_force_a_late_change_across_red_arrow():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    cmd = controller.step(_left_turn_state(
        30.0, [5.4, 0, -21], "urban_nb_0_in",
        "urban_nb_1_in", speed=6.0))["commands"][0]

    assert cmd["left_turn_phase"] == "AbortedStraight"
    assert cmd["target_lane"] == "urban_nb_0_in"
    assert cmd["target_speed"] > 0.0, \
        "the straight-lane signal is green, so cancellation must continue straight"


def test_left_strategy_rejects_lane_change_that_would_end_in_intersection():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    cmd = controller.step(_left_turn_state(
        25.0, [5.4, 0, -30], "urban_nb_0_in",
        "urban_nb_1_in", speed=6.0))["commands"][0]

    assert cmd["target_lane"] == "urban_nb_0_in"
    assert cmd["left_turn_phase"] == "AbortedStraight"
    assert cmd["target_speed"] > 0.0
    assert cmd["path"][-1][2] > 50.0


def test_left_strategy_aborts_when_gap_stays_closed_until_deadline():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [5.4, 0, -35], "urban_nb_0_in", "urban_nb_1_in", speed=4.0)
    state["vehicles"].append(_queued_vehicle(
        "left_queue", [1.8, 0, -27], "urban_nb_1_in"))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["behavior"] == "LeftTurnAborted"
    assert cmd["target_lane"] == "urban_nb_0_in"
    assert cmd["path"][-1][2] > 50.0


def test_left_arrow_green_still_waits_for_stopped_leader():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [1.8, 0, -22], "urban_nb_1_in", speed=0.0)
    state["vehicles"].append(_queued_vehicle(
        "left_queue", [1.8, 0, -12], "urban_nb_1_in"))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["behavior"] == "WaitingAtIntersection"
    assert cmd["target_speed"] == 0.0


def test_left_arrow_green_waits_when_turn_exit_is_blocked():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [1.8, 0, -21], "urban_nb_1_in", speed=0.0)
    state["vehicles"].append(_queued_vehicle(
        "blocked_exit", [-13, 0, 5.4], "urban_wb_0_out"))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["behavior"] == "WaitingAtIntersection"
    assert cmd["target_speed"] == 0.0


def test_active_left_lane_change_brakes_without_reversing_path():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    first = _left_turn_state(
        5.0, [5.4, 0, -64], "urban_nb_0_in", "urban_nb_1_in", speed=8.0)
    controller.step(first)
    second = _left_turn_state(
        5.1, [4.9, 0, -62], "urban_nb_0_in", "urban_nb_1_in", speed=8.0)
    second["vehicles"].append(_queued_vehicle(
        "new_hazard", [1.8, 0, -61], "urban_nb_1_in"))

    cmd = {c["vehicle_id"]: c for c in controller.step(second)["commands"]}["ego"]

    assert cmd["left_turn_phase"] == "LaneChanging"
    assert cmd["target_speed"] == 0.0
    assert cmd["path"][1][0] < cmd["path"][0][0], \
        "an active change may brake but must not suddenly steer back right"


def test_active_left_lane_change_keeps_moving_left_after_halfway_point():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    controller.step(_left_turn_state(
        5.0, [5.4, 0, -64], "urban_nb_0_in", "urban_nb_1_in", speed=4.0))

    halfway = _left_turn_state(
        5.1, [3.6, 0, -35.5], "urban_nb_0_in", "urban_nb_1_in", speed=1.0)
    cmd = {c["vehicle_id"]: c for c in controller.step(halfway)["commands"]}["ego"]

    assert cmd["left_turn_phase"] == "LaneChanging"
    assert cmd["path"][1][0] < cmd["path"][0][0], \
        "per-tick replanning must preserve completed lateral progress"
    assert all(cmd["path"][i + 1][0] <= cmd["path"][i][0]
               for i in range(len(cmd["path"]) - 1))


def test_current_lane_leader_inside_safe_gap_commands_full_stop():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [1.8, 0, -40], "urban_nb_1_in", speed=6.0)
    state["vehicles"].append(_queued_vehicle(
        "leader", [1.8, 0, -30], "urban_nb_1_in"))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["target_speed"] == 0.0


def test_current_lane_emergency_prevents_left_change_from_starting():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        5.0, [5.4, 0, -60], "urban_nb_0_in", "urban_nb_1_in", speed=8.0)
    state["vehicles"].append(_queued_vehicle(
        "current_leader", [5.4, 0, -52], "urban_nb_0_in"))

    cmd = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}["ego"]

    assert cmd["target_lane"] == "urban_nb_0_in"
    assert cmd["left_turn_phase"] == "LaneChangeWaiting"
    assert cmd["target_speed"] == 0.0


def test_left_turn_completion_centers_lane_and_cancels_indicator():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [-20, 0, 5.4], "urban_wb_0_out", speed=5.0)
    state["vehicles"][0]["velocity"] = [-5, 0, 0]
    state["vehicles"][0]["heading"] = 270.0
    state["vehicles"][0]["target_lane"] = None

    cmd = controller.step(state)["commands"][0]

    assert cmd["left_turn_phase"] == "Completed"
    assert cmd["turn_signal"] == "none"


def test_left_turn_policy_uses_lane_roles_not_urban_lane_ids():
    lanes = [
        Lane("ordinary_approach", [[0, 0, -60], [0, 0, -10]],
             left_lane_id="pocket_alpha"),
        Lane("pocket_alpha", [[-3.5, 0, -60], [-3.5, 0, -10]],
             right_lane_id="ordinary_approach",
             next_lane_ids=["continue_beta", "curve_gamma"]),
        Lane("continue_beta", [[-3.5, 0, -10], [-3.5, 0, 30]],
             next_lane_ids=["north_exit"]),
        Lane("north_exit", [[-3.5, 0, 30], [-3.5, 0, 70]]),
        Lane("curve_gamma", [[-3.5, 0, -10], [-12, 0, -2], [-20, 0, 0]],
             next_lane_ids=["departure_delta"]),
        Lane("departure_delta", [[-20, 0, 0], [-70, 0, 0]]),
    ]
    controller = CentralController(LaneNetwork(lanes, scenario="custom_city"))
    controller.traffic.add(TrafficLight(
        "pocket_alpha", stop_line=[-3.5, 0, -10], approach_heading=0,
        green_time=20, yellow_time=3, red_time=20))
    state = _left_turn_state(
        5.0, [0, 0, -55], "ordinary_approach", "pocket_alpha", speed=6.0)
    state["vehicles"][0]["goal"] = [-65, 0, 0]

    cmd = controller.step(state)["commands"][0]
    context = controller._left_turn_contexts["ego"]

    assert context.source_lane == "ordinary_approach"
    assert context.target_lane == "pocket_alpha"
    assert context.connector_lane == "curve_gamma"
    assert context.exit_lane == "departure_delta"
    assert cmd["left_turn_phase"] == "LaneChanging"


def test_left_strategy_enters_turn_only_on_protected_arrow_green():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    cmd = controller.step(_left_turn_state(
        36.0, [1.8, 0, -21], "urban_nb_1_in", speed=0.0))["commands"][0]

    assert cmd["behavior"] != "WaitingAtIntersection"
    assert cmd["target_speed"] > 0.0
    assert cmd["left_turn_phase"] == "IntersectionEntry"
    assert any(p[0] < -5 and p[2] > 4 for p in cmd["path"])


def test_headless_urban_left_turn_runs_lane_change_wait_and_turn_sequence():
    """Exercise the same continuous state/command loop used with Unity."""
    network = LaneNetwork.from_json(URBAN_NETWORK)
    sim = HeadlessSim(network, dt=0.1, scenario="urban")
    sim.time = 5.0
    goal = [-191, 0, 5.4]
    sim.add_vehicle(
        "urban_left_turn", [1.8, 0, -45.08], "urban_nb_1_in",
        speed=4.0, goal=goal, maneuver="left")
    ego = sim.add_vehicle(
        "urban_ego", [5.4, 0, -61.08], "urban_nb_0_in",
        speed=4.0, goal=goal, maneuver="left",
        target_lane="urban_nb_1_in")

    phases = []
    lanes = []
    min_distance = math.inf
    for _ in range(650):
        command = sim.step()
        ego_cmd = next(c for c in command["commands"]
                       if c["vehicle_id"] == "urban_ego")
        phase = ego_cmd.get("left_turn_phase")
        if phase and (not phases or phases[-1] != phase):
            phases.append(phase)
        lanes.append(ego.lane)
        min_distance = min(min_distance, sim.min_pairwise_distance())
        if phase == "Completed":
            break

    assert "LaneChanging" in phases, phases
    assert "urban_nb_1_in" in lanes, (phases, ego.position, ego.lane)
    assert "SignalWaiting" in phases, phases
    assert "IntersectionEntry" in phases, phases
    assert "IntersectionCrossing" in phases, phases
    assert phases.index("LaneChanging") < phases.index("SignalWaiting")
    assert phases.index("SignalWaiting") < phases.index("IntersectionEntry")
    assert min_distance >= 10.0, \
        "six-metre bumper clearance must be retained between 4.5 m vehicles"


def test_default_straight_cruise_can_switch_to_left_while_moving():
    network = LaneNetwork.from_json(URBAN_NETWORK)
    sim = HeadlessSim(network, dt=0.1, scenario="urban")
    sim.time = 5.0
    ego = sim.add_vehicle(
        "urban_ego", [5.4, 0, -64], "urban_nb_0_in",
        speed=0.0, goal=[5.4, 0, 70], maneuver="straight")

    for _ in range(20):
        sim.step()

    assert ego.position[2] > -64.0
    assert ego.lane == "urban_nb_0_in"
    ego.maneuver = "left"
    ego.target_lane = "urban_nb_1_in"
    ego.goal = [-191, 0, 5.4]

    phases = []
    for _ in range(220):
        command = sim.step()
        ego_cmd = next(c for c in command["commands"]
                       if c["vehicle_id"] == "urban_ego")
        phases.append(ego_cmd.get("left_turn_phase"))
        if ego.lane == "urban_nb_1_in":
            break

    assert "LaneChanging" in phases
    assert "AbortedStraight" not in phases
    assert ego.lane == "urban_nb_1_in"


def test_stopped_left_queue_leader_enters_on_green_with_follower_behind():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [1.8, 0, -21.495], "urban_nb_1_in", speed=0.0)
    state["vehicles"][0]["id"] = "leader"
    follower = _left_turn_state(
        36.0, [2.04, 0, -32.11], "urban_nb_1_in", speed=0.0)["vehicles"][0]
    follower["id"] = "follower"
    state["vehicles"].append(follower)

    commands = {c["vehicle_id"]: c for c in controller.step(state)["commands"]}

    assert commands["leader"]["left_turn_phase"] == "IntersectionEntry"
    assert commands["leader"]["target_speed"] > 0.0
    assert commands["follower"]["left_turn_phase"] in {
        "ApproachStopLine", "SignalWaiting"}


def test_left_green_ignores_pedestrian_waiting_safely_on_curb():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [1.8, 0, -21.5], "urban_nb_1_in", speed=0.0)
    state["objects"].append({
        "id": "waiting_pedestrian", "type": "pedestrian",
        "position": [-9.0, 0.0, -13.0],
        "velocity": [0.0, 0.0, 0.0], "radius": 0.5,
    })

    cmd = controller.step(state)["commands"][0]

    assert cmd["left_turn_phase"] == "IntersectionEntry"
    assert cmd["target_speed"] > 0.0


def test_left_green_waits_for_pedestrian_inside_turn_corridor():
    controller = CentralController(LaneNetwork.from_json(URBAN_NETWORK))
    state = _left_turn_state(
        36.0, [1.8, 0, -21.5], "urban_nb_1_in", speed=0.0)
    state["objects"].append({
        "id": "crossing_pedestrian", "type": "pedestrian",
        "position": [1.8, 0.0, -13.0],
        "velocity": [1.5, 0.0, 0.0], "radius": 0.5,
    })

    cmd = controller.step(state)["commands"][0]

    assert cmd["left_turn_phase"] == "SignalWaiting"
    assert cmd["target_speed"] == 0.0


def test_real_urban_cycle_starts_with_perpendicular_traffic_green():
    controller = CentralController(LaneNetwork([
        Lane("urban_nb_0_in", [[5.4, 0, -70], [5.4, 0, -11]]),
        Lane("urban_eb_0_in", [[-70, 0, -5.4], [-11, 0, -5.4]]),
    ], scenario="urban"))
    assert controller.traffic.state("urban_eb_0_in", 0.0) == "Green"
    assert controller.traffic.state("urban_nb_0_in", 0.0) == "Red"
    assert controller.traffic.lights["urban_nb_0_in"].stop_line[2] == -16.0


def test_red_signal_approach_speed_stops_before_painted_line():
    lane = Lane("urban_nb_0_in", [[5.4, 0, -70], [5.4, 0, -11]])
    controller = CentralController(LaneNetwork([lane], scenario="urban"))

    def command_at(z, speed=10.0):
        state = {
            "time": 5.0, "tick": 1, "scenario": "urban",
            "vehicles": [{
                "id": "ego", "type": "car", "position": [5.4, 0, z],
                "velocity": [0, 0, speed], "acceleration": [0, 0, 0],
                "heading": 0, "current_lane": "urban_nb_0_in",
                "target_lane": None, "has_goal": False,
                "goal": [5.4, 0, 60], "behavior_state": "LaneKeeping",
            }],
            "objects": [], "events": [],
        }
        return controller.step(state)["commands"][0]

    early_approach = command_at(-65.0, 13.9)
    approaching = command_at(-30.0)
    at_line_buffer = command_at(-17.0)
    assert 0.0 < early_approach["target_speed"] < 13.9
    assert early_approach["behavior"] == "WaitingAtIntersection"
    assert 0.0 < approaching["target_speed"] < 13.9
    assert at_line_buffer["target_speed"] == 0.0
    assert at_line_buffer["behavior"] == "WaitingAtIntersection"


# ---- right turn on red (Korean rule) ------------------------------------- #
def _right_turn_sim(lane, start, goal_lane, objects=()):
    net = LaneNetwork.from_json(URBAN_NETWORK)
    controller = CentralController(net, dt=0.1)
    sim = HeadlessSim(net, controller, dt=0.1, scenario="urban")
    sim.add_vehicle("rt", start, lane, speed=10.0,
                    goal=list(net.lane(goal_lane).end), maneuver="right")
    for oid, otype, position in objects:
        sim.add_object(oid, otype, list(position), [0.0, 0.0, 0.0], radius=0.4)
    return net, controller, sim


def test_right_on_red_makes_a_full_stop_then_completes_the_turn():
    """The rule is stop-then-go, not go: neither a red-light run nor a wait."""
    net, controller, sim = _right_turn_sim(
        "urban_nb_0_in", [5.4, 0.0, -60.0], "urban_eb_0_out")
    light = controller.traffic.lights["urban_nb_0_in"]
    assert light.state(0.0) == RED, "fixture assumes the approach starts red"

    stopped = False
    entered_on_red = False
    for _ in range(400):
        sim.step()
        v = sim.vehicles["rt"]
        if v.position[2] < -15.0 and v.speed <= 0.2:
            stopped = True
        if v.lane == "urban_nb_right" and light.state(sim.time) == RED:
            entered_on_red = True

    assert stopped, "never made the required full stop before the line"
    assert entered_on_red, "waited out a red it is allowed to turn right on"
    assert sim.vehicles["rt"].arrived


def test_right_on_red_holds_for_a_pedestrian():
    net, controller, sim = _right_turn_sim(
        "urban_nb_0_in", [5.4, 0.0, -60.0], "urban_eb_0_out",
        objects=[("ped", "pedestrian", [9.0, 0.0, -8.0])])
    light = controller.traffic.lights["urban_nb_0_in"]

    for _ in range(200):
        sim.step()
        if light.state(sim.time) != RED:
            break
        assert sim.vehicles["rt"].lane != "urban_nb_right", (
            f"turned across a pedestrian on red at t={sim.time:.1f}")


def test_right_on_red_exemption_needs_an_actual_right_turn_connector():
    """Regression: the exemption used to be a literal list of four lane ids.

    Three of them — the southbound, eastbound and westbound approaches — have
    no right-turn connector in the export at all. A vehicle there that reported
    ``maneuver="right"`` still got the stop-then-go exemption and drove
    *straight* through the red light.
    """
    net = LaneNetwork.from_json(URBAN_NETWORK)
    controller = CentralController(net, dt=0.1)
    assert controller._turn_successor("urban_nb_0_in", want_left=False) \
        == "urban_nb_right"
    assert controller._turn_successor("urban_sb_0_in", want_left=False) is None

    sim = HeadlessSim(net, controller, dt=0.1, scenario="urban")
    sim.add_vehicle("sb", [-5.4, 0.0, 60.0], "urban_sb_0_in", speed=10.0,
                    goal=list(net.lane("urban_sb_0_out").end), maneuver="right")
    light = controller.traffic.lights["urban_sb_0_in"]
    assert light.state(0.0) == RED

    for _ in range(300):
        sim.step()
        if light.state(sim.time) != RED:
            break
        assert sim.vehicles["sb"].position[2] > 10.0, (
            f"ran the red at t={sim.time:.1f}, "
            f"z={sim.vehicles['sb'].position[2]:.1f}")


# ---- intersection reservation -------------------------------------------- #
def test_single_vehicle_no_yield():
    mgr = IntersectionManager(center=[0, 0, 0], radius=6.0)
    v = veh("a", [0, 0, -40], [0, 0, 10])
    grants = mgr.reserve([v], t_now=0.0)
    assert "a" in grants
    assert not grants["a"].must_yield


def test_crossing_vehicles_one_yields():
    mgr = IntersectionManager(center=[0, 0, 0], radius=6.0, buffer=1.5)
    a = veh("a", [0, 0, -40], [0, 0, 10])   # from -Z
    b = veh("b", [-40, 0, 0], [10, 0, 0])   # from -X, same ETA
    grants = mgr.reserve([a, b], t_now=0.0)
    yields = [g.must_yield for g in grants.values()]
    assert sum(yields) == 1, "exactly one of the two should yield"
    # granted windows must not overlap: later enter >= earlier enter + occupancy
    enters = sorted(g.enter_time for g in grants.values())
    assert enters[1] - enters[0] >= 1.0


def test_approaching_filters_inside_and_receding():
    mgr = IntersectionManager(center=[0, 0, 0], radius=6.0)
    inside = veh("inside", [0, 0, 0], [0, 0, 10])      # already in zone
    receding = veh("rec", [0, 0, 40], [0, 0, 10])      # past, moving away
    incoming = veh("inc", [0, 0, -30], [0, 0, 10])     # approaching
    got = {v.id for v in mgr.approaching([inside, receding, incoming])}
    assert got == {"inc"}


# ---- pedestrian crossing (generic conflict prediction) ------------------- #
def test_vehicle_stops_for_crossing_pedestrian():
    cl = [[0, 0, z] for z in range(0, 201, 5)]
    net = LaneNetwork([Lane(id="L", centerline=cl, next_lane_ids=[])],
                      scenario="urban")
    sim = HeadlessSim(net, dt=0.1, scenario="urban")
    sim.add_vehicle("car", [0, 0, 0], "L", speed=12, goal=[0, 0, 190])
    # pedestrian walking across the lane ~80 m ahead
    sim.add_object("ped", "pedestrian", [-6, 0, 80], [1.0, 0, 0], radius=0.4)
    min_dist = math.inf
    for _ in range(120):
        sim.step()
        car = sim.vehicles["car"]
        ped = sim.objects["ped"]
        min_dist = min(min_dist, math.hypot(car.position[0] - ped.position[0],
                                            car.position[2] - ped.position[2]))
    assert min_dist > 2.0, f"car got too close to pedestrian ({min_dist:.2f} m)"
