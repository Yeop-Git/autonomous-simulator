"""Integrity of every committed scene lane-network export.

``server/scenarios/*_lanes.json`` is what the server actually plans on when a
Unity scene connects, but until now only ``Urban_lanes.json`` was loaded by any
test. These checks run over all of them so a broken export cannot reach a live
run: schema validity, referential integrity, and — the one that bit us — that
every lane-graph edge is geometrically drivable.
"""
import json
import math
import re
from pathlib import Path

import jsonschema
import pytest

import central_control
from central_control import CentralController
from headless_sim import HeadlessSim
from planners.astar import AStarPlanner
from tools import unity_scene
from world_model import LaneNetwork

SCENARIO_DIR = Path(__file__).resolve().parents[1] / "scenarios"
PROTOCOL_DIR = Path(__file__).resolve().parents[2] / "shared" / "protocol"
LANE_SCHEMA = json.loads(
    (PROTOCOL_DIR / "lane_network.schema.json").read_text(encoding="utf-8"))
STATE_SCHEMA = json.loads(
    (PROTOCOL_DIR / "state_message.schema.json").read_text(encoding="utf-8"))
COMMAND_SCHEMA = json.loads(
    (PROTOCOL_DIR / "command_message.schema.json").read_text(encoding="utf-8"))

SCENE_DIR = Path(__file__).resolve().parents[2] / "unity" / "Assets" / "Scenes"
EXPORTS = sorted(SCENARIO_DIR.glob("*_lanes.json"))
EXPORT_IDS = [p.stem for p in EXPORTS]

# Scenes authored in the Unity editor, each with a scene-named export.
SCENES = ["Main", "LKA_Test", "Highway", "Urban", "EmergencyAvoidance",
          "IntegratedCity"]


def _load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def _lateral_to(point, centerline):
    """Planar distance from ``point`` to a polyline."""
    best = math.hypot(point[0] - centerline[0][0], point[2] - centerline[0][2])
    for a, b in zip(centerline, centerline[1:]):
        dx, dz = b[0] - a[0], b[2] - a[2]
        seg_sq = dx * dx + dz * dz
        if seg_sq == 0.0:
            continue
        t = ((point[0] - a[0]) * dx + (point[2] - a[2]) * dz) / seg_sq
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(point[0] - (a[0] + t * dx),
                                    point[2] - (a[2] + t * dz)))
    return best


def _scene_lane_ids(scene: str) -> set[str]:
    return set(unity_scene.SceneLaneNetwork(SCENE_DIR / f"{scene}.unity").lanes())


def test_every_scene_with_a_road_has_an_export():
    """Only driving scenes need a lane export.

    ``Main`` is a hub: a menu that lists the other scenes and loads one. It has
    no Lane components, so it must have no export either — a leftover file
    would describe a road that scene no longer contains.
    """
    exported = {p.stem.removesuffix("_lanes") for p in EXPORTS}
    for scene in SCENES:
        has_road = bool(_scene_lane_ids(scene))
        assert has_road == (scene in exported), (
            f"{scene}.unity {'has' if has_road else 'has no'} lanes but "
            f"{'no' if has_road else 'a stale'} {scene}_lanes.json export")


@pytest.mark.parametrize("path", EXPORTS, ids=EXPORT_IDS)
def test_export_validates_against_schema(path):
    jsonschema.validate(_load(path), LANE_SCHEMA)


def test_lane_network_scenarios_match_the_state_message_enum():
    """A network whose scenario Unity cannot report is a wiring mistake."""
    lane_enum = set(LANE_SCHEMA["properties"]["scenario"]["enum"])
    state_enum = set(STATE_SCHEMA["properties"]["scenario"]["enum"])
    assert lane_enum == state_enum


@pytest.mark.parametrize("path", EXPORTS, ids=EXPORT_IDS)
def test_lane_references_resolve(path):
    lanes = {l["id"]: l for l in _load(path)["lanes"]}
    assert len(lanes) == len(_load(path)["lanes"]), "duplicate lane ids"
    for lid, lane in lanes.items():
        for nid in lane.get("next_lane_ids", []):
            assert nid in lanes, f"{lid}: dangling successor {nid}"
        for side in ("left_lane_id", "right_lane_id"):
            sid = lane.get(side)
            assert sid is None or sid in lanes, f"{lid}: dangling {side} {sid}"


@pytest.mark.parametrize("path", EXPORTS, ids=EXPORT_IDS)
def test_lateral_adjacency_is_reciprocal(path):
    lanes = {l["id"]: l for l in _load(path)["lanes"]}
    for lid, lane in lanes.items():
        left = lane.get("left_lane_id")
        if left:
            assert lanes[left].get("right_lane_id") == lid, (
                f"{lid}.left={left} but {left}.right="
                f"{lanes[left].get('right_lane_id')}")
        right = lane.get("right_lane_id")
        if right:
            assert lanes[right].get("left_lane_id") == lid, (
                f"{lid}.right={right} but {right}.left="
                f"{lanes[right].get('left_lane_id')}")


@pytest.mark.parametrize("path", EXPORTS, ids=EXPORT_IDS)
def test_successor_edges_are_drivable(path):
    """A successor may be joined mid-lane (an on-ramp merging into the
    mainline), but the predecessor's end must lie ON it — otherwise the
    stitched route steps sideways off the road at the join."""
    lanes = {l["id"]: l for l in _load(path)["lanes"]}
    for lid, lane in lanes.items():
        for nid in lane.get("next_lane_ids", []):
            lateral = _lateral_to(lane["centerline"][-1],
                                  lanes[nid]["centerline"])
            assert lateral <= 1.0, (
                f"{lid} -> {nid}: join is {lateral:.2f} m off the successor "
                "centerline")


@pytest.mark.parametrize("path", EXPORTS, ids=EXPORT_IDS)
def test_centerlines_have_no_degenerate_segments(path):
    for lane in _load(path)["lanes"]:
        cl = lane["centerline"]
        assert len(cl) >= 2, f"{lane['id']}: centerline too short"
        for i, (a, b) in enumerate(zip(cl, cl[1:])):
            assert math.hypot(a[0] - b[0], a[2] - b[2]) > 1e-6, (
                f"{lane['id']}: duplicate centerline point at {i}")


@pytest.mark.parametrize("path", EXPORTS, ids=EXPORT_IDS)
def test_every_lane_is_routable_to_its_own_end(path):
    """Smoke the planner on each export: a route along a single lane must
    exist and start/end where asked."""
    network = LaneNetwork.from_json(path)
    planner = AStarPlanner()
    for lane in network.lanes.values():
        start, goal = lane.centerline[0], lane.centerline[-1]
        route = planner.plan(start, goal, network)
        assert route, f"{lane.id}: no route along its own centerline"
        assert math.hypot(route[0][0] - start[0],
                          route[0][2] - start[2]) <= 1.0
        assert math.hypot(route[-1][0] - goal[0],
                          route[-1][2] - goal[2]) <= 1.0


@pytest.mark.parametrize("path", EXPORTS, ids=EXPORT_IDS)
def test_planned_routes_never_jump(path):
    """Regression: the highway on-ramp merges into the middle of hw_l2, and
    stitching used to append the successor from ITS start — teleporting the
    route 115 m back down the mainline.

    A stitched route only ever walks along centerlines, so no leg of it may be
    longer than the longest centerline segment the network contains.
    """
    network = LaneNetwork.from_json(path)
    longest_segment = max(
        math.hypot(b[0] - a[0], b[2] - a[2])
        for lane in network.lanes.values()
        for a, b in zip(lane.centerline, lane.centerline[1:]))
    planner = AStarPlanner()
    for lane in network.lanes.values():
        for nid in lane.next_lane_ids:
            route = planner.plan(lane.centerline[0], network.lane(nid).end,
                                 network)
            if not route:
                continue
            worst = max(math.hypot(b[0] - a[0], b[2] - a[2])
                        for a, b in zip(route, route[1:]))
            assert worst <= longest_segment + 0.5, (
                f"{lane.id} -> {nid}: route leg of {worst:.1f} m exceeds the "
                f"network's longest centerline segment ({longest_segment:.1f} m)")


SIGNALIZED = ["Urban", "IntegratedCity"]


@pytest.mark.parametrize("scene", SIGNALIZED)
def test_traffic_lights_sit_on_the_lanes_they_govern(scene):
    """A stop line off its own lane never stops anybody at the right place."""
    network = LaneNetwork.from_json(SCENARIO_DIR / f"{scene}_lanes.json")
    controller = CentralController(network)
    governed = 0
    for light_id, light in controller.traffic.lights.items():
        lane = network.lane(light_id)
        if lane is None:
            continue  # legacy synthetic-test ids, no lane in this scene
        governed += 1
        _, lateral, arc = lane.closest_point(light.stop_line)
        assert lateral <= 1.0, f"{light_id}: stop line {lateral:.2f} m off lane"
        assert 0.0 < arc < lane.length, (
            f"{light_id}: stop line at arc {arc:.1f} of {lane.length:.1f}")
        heading_error = abs(
            (light.approach_heading - lane.heading_at_arc(arc) + 180.0) % 360.0
            - 180.0)
        assert heading_error <= 10.0, (
            f"{light_id}: approach heading off by {heading_error:.1f} deg")
    assert governed >= 8, f"{scene}: only {governed} approaches are signalized"


@pytest.mark.parametrize("scene", SIGNALIZED)
def test_conflicting_approaches_are_never_green_together(scene):
    """Sweep a whole cycle: north/south and east/west must never both be
    showing anything other than red."""
    network = LaneNetwork.from_json(SCENARIO_DIR / f"{scene}_lanes.json")
    traffic = CentralController(network).traffic
    live = [lid for lid in traffic.lights if network.lane(lid) is not None]
    north_south = [l for l in live if "_nb_" in l or "_sb_" in l]
    east_west = [l for l in live if "_eb_" in l or "_wb_" in l]
    assert north_south and east_west

    period = max(traffic.lights[l].period for l in live)
    t = 0.0
    while t < period:
        running = [l for l in live if traffic.state(l, t) != "Red"]
        assert not ({*running} & {*north_south} and {*running} & {*east_west}), (
            f"t={t:.1f}s: conflicting approaches green together: {running}")
        t += 0.25


@pytest.mark.parametrize("scene", SIGNALIZED)
def test_pedestrian_phases_fall_in_all_red_windows(scene):
    """Pedestrians only cross while every vehicle approach is red."""
    network = LaneNetwork.from_json(SCENARIO_DIR / f"{scene}_lanes.json")
    traffic = CentralController(network).traffic
    live = [lid for lid in traffic.lights if network.lane(lid) is not None]
    for start, end in central_control.PEDESTRIAN_PHASES:
        t = start
        while t < end:
            running = [l for l in live if traffic.state(l, t) != "Red"]
            assert not running, (
                f"t={t:.1f}s is a pedestrian phase but {running} is not red")
            t += 0.25


@pytest.mark.parametrize("scene", SCENES)
def test_scene_drives_a_protocol_valid_loop(scene):
    """Run each scene's own network through the real controller and check both
    wire messages against their schemas — including the ``scenario`` string,
    which is how the stale lane-network enum went unnoticed."""
    network = LaneNetwork.from_json(SCENARIO_DIR / f"{scene}_lanes.json")
    controller = CentralController(network)
    sim = HeadlessSim(network, controller, scenario=network.scenario)

    has_predecessor = {nid for lane in network.lanes.values()
                       for nid in lane.next_lane_ids}
    source = next((l for l in network.lanes.values()
                   if l.id not in has_predecessor),
                  next(iter(network.lanes.values())))
    sim.add_vehicle("car_01", source.start, source.id, speed=5.0,
                    goal=source.end)

    for _ in range(30):
        jsonschema.validate(sim.build_state(), STATE_SCHEMA)
        jsonschema.validate(sim.step(), COMMAND_SCHEMA)


def test_route_survives_an_ambiguous_intersection_entry():
    """At the northbound stop line the straight and left connectors both start
    at (1.8, -11): position alone cannot say which lane the car is on. Picking
    the left connector for a car going straight used to yield no route at all,
    which the FSM reads as 'no route' and stops the car in the intersection."""
    network = LaneNetwork.from_json(SCENARIO_DIR / "Urban_lanes.json")
    goal = network.lane("urban_nb_1_out").end
    planner = AStarPlanner()
    for z in (-11.5, -11.0, -10.5, -9.0, 0.0):
        assert planner.plan([1.8, 0.0, z], goal, network), (
            f"no route northbound from z={z}")
        assert planner.last_lane_route[-1] == "urban_nb_1_out"
        assert "urban_nb_left" not in planner.last_lane_route


def test_ramp_route_merges_forward_into_the_mainline():
    """The concrete case, spelled out: a car entering at the ramp head must
    reach the mainline goal by driving forward the whole way."""
    network = LaneNetwork.from_json(SCENARIO_DIR / "Highway_lanes.json")
    ramp = network.lane("hw_ramp")
    mainline = network.lane("hw_l2")
    route = AStarPlanner().plan(ramp.start, mainline.end, network)
    assert route
    forward = [b[2] - a[2] for a, b in zip(route, route[1:])]
    assert all(step > -0.01 for step in forward), (
        "route travels backwards along the highway")


# --- Unity <-> Python signal-plan drift ----------------------------------- #
UNITY_SIGNALS = (Path(__file__).resolve().parents[2] / "unity" / "Assets" /
                 "Scripts" / "UI" / "TrafficLightSystem.cs")

# Which Python approach lights each Unity signal head drives.
SIGNAL_GROUPS = {
    "EastWestState": ["urban_eb_0_in", "urban_eb_1_in",
                      "urban_wb_0_in", "urban_wb_1_in"],
    "NorthSouthState": ["urban_nb_0_in", "urban_sb_0_in", "urban_sb_1_in"],
    "ProtectedLeftState": ["urban_nb_1_in"],
}


def _unity_windows():
    """(start, green, yellow) per signal head, read out of the C# source."""
    source = UNITY_SIGNALS.read_text(encoding="utf-8")
    pattern = (r"public SignalState (\w+) => WindowState\(Time\.fixedTime, "
               r"([\d.]+)f, ([\d.]+)f, ([\d.]+)f\)")
    return {name: (float(start), float(green), float(yellow))
            for name, start, green, yellow in re.findall(pattern, source)}


def _unity_pedestrian_phases():
    source = UNITY_SIGNALS.read_text(encoding="utf-8")
    body = re.search(r"PedestriansMayCross(.+?)\n        }\n", source,
                     re.S).group(1)
    return tuple((float(a), float(b)) for a, b in re.findall(
        r"t >= ([\d.]+)f && t < ([\d.]+)f", body))


def test_unity_signal_plan_matches_the_servers():
    """Unity draws the lights; Python enforces them. Two copies of one plan.

    Nothing links them, so a retimed phase on either side would show green to a
    driver the server is holding at red — the project's stated #1 risk, in the
    one place where both sides own the same numbers. Compare them directly.
    """
    controller = CentralController(
        LaneNetwork.from_json(SCENARIO_DIR / "Urban_lanes.json"))
    windows = _unity_windows()
    assert set(windows) == set(SIGNAL_GROUPS), (
        f"Unity signal heads changed: {sorted(windows)}")

    for head, lane_ids in SIGNAL_GROUPS.items():
        start, green, yellow = windows[head]
        for lane_id in lane_ids:
            light = controller.traffic.lights[lane_id]
            python_start = (light.period - light.offset) % light.period
            assert (python_start, light.green_time, light.yellow_time) == \
                (start, green, yellow), (
                    f"{head} in Unity is (start={start}, green={green}, "
                    f"yellow={yellow}) but {lane_id} in Python is "
                    f"(start={python_start}, green={light.green_time}, "
                    f"yellow={light.yellow_time})")


def test_unity_pedestrian_phases_match_the_servers():
    assert _unity_pedestrian_phases() == central_control.PEDESTRIAN_PHASES


def test_unity_signal_cycle_length_matches_the_servers():
    source = UNITY_SIGNALS.read_text(encoding="utf-8")
    cycle = float(re.search(r"CycleTime = ([\d.]+)f", source).group(1))
    assert cycle == central_control.URBAN_SIGNAL_PERIOD


# --- Unity scene <-> lane export drift ------------------------------------ #
@pytest.mark.parametrize("scene", SCENES)
def test_export_still_matches_the_authored_scene(scene):
    """The scene is the source of truth; the export is a copy of it.

    Nothing in the pipeline forces a re-export after a scene edit, so the
    server can end up planning confidently on a road Unity no longer draws.
    Read the lane graph back out of the ``.unity`` file and diff every field
    the exporter writes: ids, waypoints, width, speed limit, and the
    left/right/next links.
    """
    if not _scene_lane_ids(scene):
        pytest.skip(f"{scene} has no road (hub/menu scene)")
    export = _load(SCENARIO_DIR / f"{scene}_lanes.json")
    problems = unity_scene.diff_against_export(SCENE_DIR / f"{scene}.unity", export)
    assert not problems, (
        f"{scene}.unity and {scene}_lanes.json have drifted apart "
        f"({len(problems)} differences); re-run V2X > Export Lane Network.\n  "
        + "\n  ".join(problems[:15]))
