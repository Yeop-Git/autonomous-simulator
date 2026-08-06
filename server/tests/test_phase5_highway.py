"""Phase 5 — highway: lane change, merge reservation, hazard replan, metrics."""
import math

import pytest

import lane_change
import merge
import metrics
from central_control import CentralController
from headless_sim import HeadlessSim, _point_at_arc
from scenarios import networks
from world_model import DynamicVehicle, Lane, LaneNetwork


def veh(vid, pos, vel, lane):
    return DynamicVehicle(id=vid, position=list(pos), velocity=list(vel),
                          heading=0.0, current_lane=lane)


# ---- lane change gap acceptance ------------------------------------------ #
def test_lane_change_into_empty_lane_accepted():
    net = networks.highway_straight(lanes=2, length=300.0)
    ego = veh("ego", [0, 0, 50], [0, 0, 25], "hw_l0_a")
    d = lane_change.evaluate(ego, "hw_l1_a", [], net)
    assert d.accept


def test_lane_change_blocked_by_side_vehicle():
    net = networks.highway_straight(lanes=2, length=300.0)
    ego = veh("ego", [0, 0, 50], [0, 0, 25], "hw_l0_a")
    # a car right beside the ego in the target lane
    beside = veh("beside", [3.5, 0, 51], [0, 0, 25], "hw_l1_a")
    d = lane_change.evaluate(ego, "hw_l1_a", [beside], net)
    assert not d.accept
    assert "gap" in d.reason


def test_lane_change_far_traffic_accepted():
    net = networks.highway_straight(lanes=2, length=400.0)
    ego = veh("ego", [0, 0, 50], [0, 0, 25], "hw_l0_a")
    ahead = veh("ahead", [3.5, 0, 200], [0, 0, 25], "hw_l1_a")  # 150 m ahead
    behind = veh("behind", [3.5, 0, 5], [0, 0, 20], "hw_l1_a")  # 45 m behind, slower
    d = lane_change.evaluate(ego, "hw_l1_a", [ahead, behind], net)
    assert d.accept


def test_lane_change_uses_acc_for_accepted_front_gap():
    net = networks.highway_straight(lanes=2, length=400.0)
    ego = veh("ego", [0, 0, 50], [0, 0, 20], "hw_l0_a")
    # 36 m bumper gap satisfies the lane-change headway. A raw
    # constant-velocity predictor sees a collision because it ignores the ACC
    # deceleration that is applied throughout the lateral blend.
    stopped_leader = veh(
        "leader", [3.5, 0, 90.5], [0, 0, 0], "hw_l1_a")

    decision = lane_change.evaluate(ego, "hw_l1_a", [stopped_leader], net)

    assert decision.accept
    assert decision.lead_id == "leader"


def test_lane_change_waits_for_fast_rear_vehicle():
    net = networks.highway_straight(lanes=2, length=400.0)
    ego = veh("ego", [0, 0, 100], [0, 0, 12], "hw_l0_a")
    fast_rear = veh("rear", [3.5, 0, 65], [0, 0, 30], "hw_l1_a")

    decision = lane_change.evaluate(ego, "hw_l1_a", [fast_rear], net)

    assert not decision.accept
    assert decision.lag_id == "rear"


def test_central_controller_applies_requested_safe_lane_change():
    net = networks.highway_straight(lanes=2, length=300.0)
    controller = CentralController(net)
    state = {
        "time": 1.0, "tick": 1, "scenario": "highway",
        "vehicles": [{
            "id": "ego", "type": "car", "position": [0, 0, 40],
            "velocity": [0, 0, 20], "acceleration": [0, 0, 0],
            "heading": 0, "current_lane": "hw_l0_a",
            "target_lane": "hw_l1_a", "has_goal": True,
            "goal": [3.5, 0, 280], "behavior_state": "LaneKeeping",
        }],
        "objects": [], "events": [],
    }
    cmd = controller.step(state)["commands"][0]
    assert cmd["target_lane"] == "hw_l1_a"
    assert cmd["behavior"] == "LaneChanging"
    assert len(cmd["path"]) >= 5
    assert cmd["path"][1][0] > cmd["path"][0][0]


def test_central_controller_rejects_unsafe_lane_change_request():
    net = networks.highway_straight(lanes=2, length=300.0)
    controller = CentralController(net)
    def vehicle(vid, x, z, lane, target=None):
        return {
            "id": vid, "type": "car", "position": [x, 0, z],
            "velocity": [0, 0, 20], "acceleration": [0, 0, 0],
            "heading": 0, "current_lane": lane, "target_lane": target,
            "has_goal": True, "goal": [x, 0, 280],
            "behavior_state": "LaneKeeping",
        }
    state = {
        "time": 1.0, "tick": 1, "scenario": "highway",
        "vehicles": [
            vehicle("ego", 0, 40, "hw_l0_a", "hw_l1_a"),
            vehicle("beside", 3.5, 41, "hw_l1_a"),
        ],
        "objects": [], "events": [],
    }
    cmd = next(c for c in controller.step(state)["commands"]
               if c["vehicle_id"] == "ego")
    assert cmd["target_lane"] == "hw_l0_a"
    assert cmd["behavior"] != "LaneChanging"


# ---- merge reservation --------------------------------------------------- #
def test_merge_clear_mainline():
    ramp = veh("ramp", [0, 0, 0], [0, 0, 20], "ramp")
    plan = merge.plan_merge(ramp, [], merge_point=[0, 0, 100])
    assert plan.feasible
    assert plan.ramp_target_speed == pytest.approx(20.0)


def test_merge_fits_existing_gap():
    ramp = veh("ramp", [0, 0, 0], [0, 0, 20], "ramp")  # ETA 5 s to z=100
    # mainline cars arriving at 1 s and 9 s -> big gap around 5 s
    m1 = veh("m1", [0, 0, 80], [0, 0, 20], "main")   # ETA 1 s
    m2 = veh("m2", [0, 0, -80], [0, 0, 20], "main")  # ETA 9 s
    plan = merge.plan_merge(ramp, [m1, m2], merge_point=[0, 0, 100])
    assert plan.feasible
    assert plan.reason == "fits existing gap"


def test_merge_opens_gap_when_packed():
    ramp = veh("ramp", [0, 0, 0], [0, 0, 20], "ramp")  # ETA 5 s
    # a dense stream all arriving near 5 s -> must retime or yield
    main = [veh(f"m{i}", [0, 0, 100 - 20 * (4.5 + 0.3 * i)], [0, 0, 20], "main")
            for i in range(4)]
    plan = merge.plan_merge(ramp, main, merge_point=[0, 0, 100],
                            desired_speed=20.0)
    assert plan.feasible  # central control always finds a way (retime or yield)
    # ...and "a way" must be an actual change of plan. Asserting only
    # ``feasible`` passed even when the slot search handed the ramp car back its
    # own ETA — i.e. drove it straight into the middle of the platoon.
    last_arrival = 4.5 + 0.3 * 3
    assert plan.slot_eta >= last_arrival + merge.REQUIRED_GAP_S / 2, (
        f"slot at {plan.slot_eta:.2f}s lands inside the platoon "
        f"(last car arrives at {last_arrival:.2f}s)")
    assert plan.ramp_target_speed < 20.0, "ramp car was never retimed"


def test_merge_does_not_pin_a_ramp_car_that_is_stopped():
    """Regression: the plan used to return the ramp car's *current* speed as its
    target. A car waiting at the ramp head was therefore told to keep doing
    0 m/s, which kept its ETA infinite, which kept the plan telling it to
    hold — it never moved."""
    stopped = veh("ramp", [0, 0, 0], [0, 0, 0], "ramp")
    plan = merge.plan_merge(stopped, [], merge_point=[0, 0, 100],
                            desired_speed=18.0)
    assert plan.feasible
    assert plan.ramp_target_speed == pytest.approx(18.0)

    # ...and the same with traffic on the mainline.
    main = [veh("m0", [0, 0, 20], [0, 0, 20], "main")]
    plan = merge.plan_merge(stopped, main, merge_point=[0, 0, 100],
                            desired_speed=18.0)
    assert plan.ramp_target_speed >= merge.MIN_MERGE_SPEED


def test_merge_ignores_mainline_traffic_already_past_the_merge_point():
    """ETA from unsigned distance cannot tell a car 20 m short of the join from
    one 20 m beyond it, so departed traffic used to book phantom slots."""
    ramp = veh("ramp", [0, 0, 0], [0, 0, 20], "ramp")
    departed = veh("gone", [0, 0, 120], [0, 0, 20], "main")  # 20 m downstream
    plan = merge.plan_merge(ramp, [departed], merge_point=[0, 0, 100],
                            desired_speed=20.0)
    assert plan.reason == "clear mainline"


def test_merge_asks_the_mainline_to_yield_when_no_gap_is_reachable():
    """The headline central-control move. It was unreachable: the slot after
    the last car is unbounded, so the search always "found" a gap there no
    matter how absurdly slow arriving in it would be."""
    ramp = veh("ramp", [0, 0, 70], [0, 0, 15], "ramp")   # only 30 m from the join
    stream = [veh(f"m{i}", [0, 0, 100 - 20 * (1.0 + 0.5 * i)], [0, 0, 20], "main")
              for i in range(18)]                        # bumper to bumper, no 2 s gap
    plan = merge.plan_merge(ramp, stream, merge_point=[0, 0, 100],
                            desired_speed=18.0)
    assert plan.feasible
    assert plan.reason == "opened gap via mainline yield"
    assert plan.yield_vehicle is not None
    assert plan.yield_target_speed < 20.0, "the 'yield' is not a slowdown"
    assert plan.ramp_target_speed >= merge.MIN_MERGE_SPEED


def _segmented_ramp_network():
    """A mainline split into two segments with an on-ramp joining the second.

    ``highway_straight`` splits every lane in half, which is what a real export
    looks like. The join is on ``hw_l2_b``, so approaching traffic is still on
    ``hw_l2_a`` — a *predecessor* of the joined lane.
    """
    base = networks.highway_straight(lanes=3, length=300.0)
    lanes = list(base.lanes.values()) + [
        Lane(id="on_ramp",
             centerline=[[13.0, 0.0, 15.0], [8.0, 0.0, 90.0], [7.0, 0.0, 150.0]],
             speed_limit=18.0, next_lane_ids=["hw_l2_b"])]
    return LaneNetwork(lanes, scenario="highway")


def test_merge_mainline_selection_is_not_scene_specific():
    """Regression: the mainline was ``successors + literal "hw_l2"``. The
    hardcoded id happened to be redundant in the Highway export, so it silently
    contributed nothing there while being simply wrong everywhere else."""
    from pathlib import Path

    scenarios = Path(__file__).resolve().parents[1] / "scenarios"
    highway = LaneNetwork.from_json(scenarios / "Highway_lanes.json")
    controller = CentralController(highway)
    assert controller._mainline_lanes(highway.lane("hw_ramp")) == {"hw_l2"}

    # A scene that names nothing "hw_l2" still resolves its own mainline.
    net = _segmented_ramp_network()
    mainline = CentralController(net)._mainline_lanes(net.lane("on_ramp"))
    assert "hw_l2_b" in mainline           # the lane joined
    assert "on_ramp" not in mainline       # the ramp is not its own mainline


def test_merge_sees_traffic_on_the_upstream_mainline_segment():
    """A car seconds from the join, but still on the segment *before* it, used
    to be invisible: the reservation only looked at the ramp's direct
    successors, so it reported a clear mainline and merged at full speed."""
    net = _segmented_ramp_network()
    controller = CentralController(net)

    def vehicle(vid, x, z, speed, lane):
        return {"id": vid, "type": "car", "position": [x, 0.0, z],
                "velocity": [0.0, 0.0, speed], "acceleration": [0, 0, 0],
                "heading": 0, "current_lane": lane, "target_lane": None,
                "has_goal": True, "goal": [7.0, 0.0, 300.0],
                "behavior_state": "LaneKeeping"}

    controller.step({
        "time": 1.0, "tick": 1, "scenario": "highway",
        "vehicles": [
            # 60 m of ramp left; at its 18 m/s limit it arrives in ~3.3 s
            vehicle("ramp_01", 8.0, 90.0, 15.0, "on_ramp"),
            # upstream segment, arriving at the join at the same moment
            vehicle("up_01", 7.0, 66.7, 25.0, "hw_l2_a"),
        ],
        "objects": [], "events": [],
    })

    reserved = controller._merge_speed_overrides["ramp_01"]
    assert reserved < 18.0, (
        f"ramp car reserved {reserved:.1f} m/s — the full ramp speed limit, "
        "i.e. it never saw the upstream car")
    assert reserved >= merge.MIN_MERGE_SPEED


def test_ramp_vehicle_starting_at_rest_reaches_the_mainline():
    """End to end on the real Highway export: the ramp car must actually get
    onto hw_l2, not sit at the ramp head."""
    from pathlib import Path

    from world_model import LaneNetwork

    scenarios = Path(__file__).resolve().parents[1] / "scenarios"
    net = LaneNetwork.from_json(scenarios / "Highway_lanes.json")
    ramp_lane, mainline = net.lane("hw_ramp"), net.lane("hw_l2")

    sim = HeadlessSim(net, CentralController(net, dt=0.1), dt=0.1,
                      scenario="highway")
    sim.add_vehicle("ramp_01", ramp_lane.start, "hw_ramp", speed=0.0,
                    goal=mainline.end)
    # Mainline stream, spaced along hw_l2 ahead of the join at z=115.
    for i, z in enumerate((80.0, 35.0, 0.0)):
        sim.add_vehicle(f"main_{i}", [3.5, 0.0, z], "hw_l2", speed=25.0,
                        goal=mainline.end)

    # Only the merge itself is under test — past the join the cars queue at
    # their shared goal, which is ACC's business, not the reservation's.
    closest = math.inf
    for _ in range(200):
        sim.step()
        if sim.vehicles["ramp_01"].position[2] < 150.0:
            closest = min(closest, sim.min_pairwise_distance())

    ramp_car = sim.vehicles["ramp_01"]
    assert ramp_car.lane == "hw_l2", (
        f"ramp car never merged (still on {ramp_car.lane} at "
        f"z={ramp_car.position[2]:.1f})")
    assert closest > 5.0, f"merged unsafely: closest approach {closest:.1f} m"


def _highway_export():
    from pathlib import Path

    scenarios = Path(__file__).resolve().parents[1] / "scenarios"
    return LaneNetwork.from_json(scenarios / "Highway_lanes.json")


def test_leader_search_measures_a_ramp_join_from_the_merge_point():
    """``hw_ramp`` joins the *middle* of ``hw_l2`` (arc 115, not 0).

    The downstream leader search used to add the successor's arc measured from
    that lane's own start, so a car sitting on the join read as 132 m away
    instead of 17 m — past ``max_range``, no leader, no ACC, and the ramp car
    merged into it at the full ramp speed limit.
    """
    from behavior import find_leader

    net = _highway_export()
    ramp = veh("ramp", [5.08, 0.0, 98.3], [-1.7, 0.0, 17.9], "hw_ramp")

    on_join = veh("stalled", [3.5, 0.0, 115.0], [0.0, 0.0, 5.0], "hw_l2")
    leader = find_leader(ramp, [on_join], net)
    assert leader is not None and leader.vehicle.id == "stalled"
    assert leader.gap < 20.0, f"gap {leader.gap:.1f} m spans the whole mainline"

    # ...and mainline traffic still upstream of the join is behind us, not ahead
    upstream = veh("upstream", [3.5, 0.0, 80.0], [0.0, 0.0, 25.0], "hw_l2")
    assert find_leader(ramp, [upstream], net) is None


def test_ramp_car_does_not_merge_into_a_stream_stalled_across_the_join():
    """A dense stream brakes itself to a crawl right on the merge point.

    The reservation ignores traffic that has already passed the join — correct
    for slot timing, but it leaves ACC solely responsible for the car sitting
    *in* the join. With the leader search blind to a mid-lane merge, nothing
    was: the ramp car drove through the mainline at 18 m/s (0.12 m closest).
    """
    net = _highway_export()
    mainline = net.lane("hw_l2")
    sim = HeadlessSim(net, CentralController(net, dt=0.1), dt=0.1,
                      scenario="highway")
    sim.add_vehicle("ramp_01", net.lane("hw_ramp").start, "hw_ramp", speed=0.0,
                    goal=mainline.end)
    for i in range(6):  # 20 m apart at 25 m/s: 0.8 s headway, no natural gap
        sim.add_vehicle(f"main_{i}", [3.5, 0.0, 100.0 - 20.0 * i], "hw_l2",
                        speed=25.0, goal=mainline.end)

    closest = math.inf
    for _ in range(400):
        sim.step()
        if sim.vehicles["ramp_01"].position[2] < 170.0:
            closest = min(closest, sim.min_pairwise_distance())

    ramp_car = sim.vehicles["ramp_01"]
    assert ramp_car.lane == "hw_l2", "ramp car never merged"
    assert closest > 5.0, f"merged unsafely: closest approach {closest:.2f} m"


def test_ramp_car_stopped_beside_the_mainline_does_not_deadlock_the_highway():
    """The ramp taper puts the two centrelines under 2 m apart.

    A ramp car that halts there sits inside the mainline car's safety radius.
    Both then read ``ttc = 0`` and emergency-brake, both are commanded to zero,
    the separation never changes, and neither ever moves again — taking the
    whole queue behind them with it.
    """
    net = _highway_export()
    mainline = net.lane("hw_l2")
    sim = HeadlessSim(net, CentralController(net, dt=0.1), dt=0.1,
                      scenario="highway")
    ramp_lane = net.lane("hw_ramp")
    sim.add_vehicle("ramp_01", _point_at_arc(ramp_lane.centerline, 60.0),
                    "hw_ramp", speed=16.0, goal=mainline.end)
    for i in range(6):
        sim.add_vehicle(f"main_{i}", [3.5, 0.0, 100.0 - 20.0 * i], "hw_l2",
                        speed=25.0, goal=mainline.end)

    for _ in range(400):
        sim.step()

    assert sim.vehicles["ramp_01"].lane == "hw_l2", (
        "ramp car stuck on the ramp at "
        f"z={sim.vehicles['ramp_01'].position[2]:.1f}")
    # nobody is left standing still short of the merge point
    stranded = [v.id for v in sim.vehicles.values()
                if v.speed < 0.1 and v.position[2] < 150.0]
    assert not stranded, f"deadlocked short of the join: {stranded}"


@pytest.mark.parametrize("scene,expected", [
    ("Highway", {"hw_ramp"}),                       # joins hw_l2 mid-lane
    ("IntegratedCity", {"city_boulevard_escape"}),  # slower strip beside the main
    ("Urban", set()),                               # turn connectors are not merges
    ("EmergencyAvoidance", set()),                  # parallel lanes, no successors
    ("LKA_Test", set()),
    ("Main", set()),
])
def test_merge_reservation_targets_are_topological_not_named(scene, expected):
    """Which lanes get a slot reservation, per authored scene.

    This used to be ``"ramp" in lane_id``, so the whole feature only existed in
    the one scene that happened to use that word — IntegratedCity's shoulder
    merges into the boulevard with no reservation at all. The replacement is
    geometric, so it must also *not* over-trigger: an intersection turn
    connector shares its exit lane with the through movement but is separated
    by signals, not by a merge slot.
    """
    from pathlib import Path

    scenarios = Path(__file__).resolve().parents[1] / "scenarios"
    net = LaneNetwork.from_json(scenarios / f"{scene}_lanes.json")
    assert CentralController(net).merging_lane_ids() == expected


def test_merging_sibling_lane_is_visible_as_a_leader():
    """Two lanes ending into one, on the IntegratedCity boulevard.

    ``city_boulevard_escape`` and ``city_boulevard_main`` both feed
    ``city_turn_east``. Neither is named "ramp", so no reservation runs, and
    neither car is on the other's lane or downstream of it — so nothing saw
    anything until both were already on the connector. A 20 m/s main-lane car
    drove into the 8 m/s shoulder car merging ahead of it (0.01 m apart).
    """
    from pathlib import Path

    scenarios = Path(__file__).resolve().parents[1] / "scenarios"
    net = LaneNetwork.from_json(scenarios / "IntegratedCity_lanes.json")
    goal = list(net.lane("city_south").centerline[10])

    sim = HeadlessSim(net, CentralController(net, dt=0.1), dt=0.1,
                      scenario="integrated_city")
    sim.add_vehicle("shoulder", [9.0, 0.0, 300.0], "city_boulevard_escape",
                    speed=8.0, goal=goal)
    sim.add_vehicle("main", [5.4, 0.0, 180.0], "city_boulevard_main",
                    speed=20.0, goal=goal)

    closest = math.inf
    for _ in range(300):
        sim.step()
        if 280.0 < sim.vehicles["shoulder"].position[2] < 460.0:
            closest = min(closest, sim.min_pairwise_distance())

    assert closest > 5.0, (
        f"main-lane car ran into the merging shoulder car ({closest:.2f} m)")


# ---- hazard event -> replan / stop --------------------------------------- #
def test_hazard_blocks_lane_and_vehicle_stops():
    cl = [[0, 0, z] for z in range(0, 201, 5)]
    net = LaneNetwork([Lane(id="L", centerline=cl, next_lane_ids=[])],
                      scenario="highway")
    sim = HeadlessSim(net, dt=0.1)
    sim.add_vehicle("car", [0, 0, 0], "L", speed=15, goal=[0, 0, 190])
    # drive a bit, then drop an obstacle on the lane ahead
    for _ in range(20):
        sim.step()
    sim.events = [{"type": "FallingObject", "position": [0, 0, 90]}]
    stopped_before_hazard = False
    for _ in range(120):
        sim.step()
        sim.events = [{"type": "FallingObject", "position": [0, 0, 90]}]  # persists
        if sim.vehicles["car"].speed < 0.5 and sim.vehicles["car"].position[2] < 90:
            stopped_before_hazard = True
            break
    assert stopped_before_hazard, "car did not stop for the blocked lane"


# ---- metrics ------------------------------------------------------------- #
def test_metrics_from_rows():
    rows = [
        {"time": 0.0, "vehicle_id": "a", "speed": 20, "lateral_error": 0.1, "ttc": "",
         "behavior_state": "LaneKeeping", "collision_risk": 0},
        {"time": 1.0, "vehicle_id": "a", "speed": 10, "lateral_error": 2.0, "ttc": 2.5,
         "behavior_state": "Stopping", "collision_risk": 1},
        {"time": 2.0, "vehicle_id": "a", "speed": 0, "lateral_error": 0.1, "ttc": "",
         "behavior_state": "Arrived", "collision_risk": 0},
    ]
    m = metrics.from_rows(rows, lane_width=3.5)
    assert m.samples == 3
    assert m.avg_speed == pytest.approx(10.0)
    assert m.min_ttc == pytest.approx(2.5)
    assert m.lane_departures == 1            # |2.0| > 1.75
    assert m.hard_brakes == 2                # 20->10 and 10->0, both > 4 m/s^2
    assert m.arrivals == 1
    assert m.collision_risk_events == 1
