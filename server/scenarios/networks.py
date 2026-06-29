"""Synthetic lane networks for tests and experiments (no Unity required).

These mirror what a human would author in the Unity editor, but are built
programmatically so planners/controllers can be exercised headless. Geometry
follows the project convention: meters, Unity world space, [x, y, z] with y
the (flat) height axis, heading 0 = +Z.
"""
from __future__ import annotations

import math

from world_model import Lane, LaneNetwork

Vec3 = list[float]


def _straight_centerline(
    start: Vec3, heading_deg: float, length: float, step: float = 5.0
) -> list[Vec3]:
    """Sample a straight centerline of ``length`` m from ``start`` along
    ``heading_deg`` (0 = +Z, clockwise)."""
    rad = math.radians(heading_deg)
    dx, dz = math.sin(rad), math.cos(rad)
    n = max(1, int(round(length / step)))
    pts = []
    for i in range(n + 1):
        d = i * (length / n)
        pts.append([start[0] + dx * d, start[1], start[2] + dz * d])
    return pts


def highway_straight(lanes: int = 3, length: float = 300.0, lane_width: float = 3.5) -> LaneNetwork:
    """A multi-lane straight highway running along +Z.

    Lanes are indexed left(0) .. right(N-1) along +X. Each is split into two
    consecutive segments so the lane graph has real successor edges to search.
    """
    lane_objs: list[Lane] = []
    half = length / 2.0
    for li in range(lanes):
        x = li * lane_width
        # two segments per lane: seg a -> seg b
        seg_a = _straight_centerline([x, 0.0, 0.0], 0.0, half)
        seg_b = _straight_centerline([x, 0.0, half], 0.0, half)
        id_a, id_b = f"hw_l{li}_a", f"hw_l{li}_b"
        lane_objs.append(
            Lane(id=id_a, centerline=seg_a, width=lane_width, speed_limit=27.8,
                 next_lane_ids=[id_b])
        )
        lane_objs.append(
            Lane(id=id_b, centerline=seg_b, width=lane_width, speed_limit=27.8,
                 next_lane_ids=[])
        )
    # left/right adjacency between same-segment lanes
    by_id = {l.id: l for l in lane_objs}
    for li in range(lanes):
        for seg in ("a", "b"):
            cur = by_id[f"hw_l{li}_{seg}"]
            if li > 0:
                cur.left_lane_id = f"hw_l{li-1}_{seg}"
            if li < lanes - 1:
                cur.right_lane_id = f"hw_l{li+1}_{seg}"
    return LaneNetwork(lane_objs, name="highway_straight", scenario="highway")


def highway_curve(length: float = 200.0, radius: float = 150.0, lane_width: float = 3.5) -> LaneNetwork:
    """A single-lane gentle constant-radius curve, for LKA curve tests."""
    n = max(2, int(length / 4.0))
    arc = length / radius  # total swept angle (rad)
    pts: list[Vec3] = []
    for i in range(n + 1):
        theta = arc * (i / n)
        x = radius * (1 - math.cos(theta))
        z = radius * math.sin(theta)
        pts.append([x, 0.0, z])
    lane = Lane(id="curve_0", centerline=pts, width=lane_width, speed_limit=27.8)
    return LaneNetwork([lane], name="highway_curve", scenario="lka_test")


def urban_grid(rows: int = 2, cols: int = 2, block: float = 60.0, lane_width: float = 3.5) -> LaneNetwork:
    """A simple one-way grid: horizontal lanes run +X, vertical run +Z.

    Intersections are where a horizontal lane's end meets a vertical lane's
    start (and vice versa). Enough structure for A* routing and intersection
    reservation experiments.
    """
    lanes: list[Lane] = []

    def node(r: int, c: int) -> Vec3:
        return [c * block, 0.0, r * block]

    # horizontal lanes: from (r,c) -> (r,c+1)
    h_ids: dict[tuple[int, int], str] = {}
    for r in range(rows + 1):
        for c in range(cols):
            lid = f"h_{r}_{c}"
            h_ids[(r, c)] = lid
            cl = _straight_centerline(node(r, c), 90.0, block)
            lanes.append(Lane(id=lid, centerline=cl, width=lane_width, speed_limit=13.9))
    # vertical lanes: from (r,c) -> (r+1,c)
    v_ids: dict[tuple[int, int], str] = {}
    for r in range(rows):
        for c in range(cols + 1):
            lid = f"v_{r}_{c}"
            v_ids[(r, c)] = lid
            cl = _straight_centerline(node(r, c), 0.0, block)
            lanes.append(Lane(id=lid, centerline=cl, width=lane_width, speed_limit=13.9))

    by_id = {l.id: l for l in lanes}
    # Wire successors at shared grid nodes. This is a ONE-WAY grid (horizontals
    # run +X, verticals run +Z), so a lane arriving at a node may only continue
    # straight or turn onto the lane that STARTS at that exact node — never a
    # lane offset by a block (that would be a geometric teleport edge).
    for r in range(rows + 1):
        for c in range(cols):
            lid = h_ids[(r, c)]
            # arrives at node (r, c+1)
            succ = []
            if (r, c + 1) in h_ids:           # continue +X
                succ.append(h_ids[(r, c + 1)])
            if (r, c + 1) in v_ids:           # turn onto +Z vertical at this node
                succ.append(v_ids[(r, c + 1)])
            by_id[lid].next_lane_ids = succ
    for r in range(rows):
        for c in range(cols + 1):
            lid = v_ids[(r, c)]
            # arrives at node (r+1, c)
            succ = []
            if (r + 1, c) in v_ids:           # continue +Z
                succ.append(v_ids[(r + 1, c)])
            if (r + 1, c) in h_ids:           # turn onto +X horizontal at this node
                succ.append(h_ids[(r + 1, c)])
            by_id[lid].next_lane_ids = succ

    return LaneNetwork(lanes, name="urban_grid", scenario="urban")


REGISTRY = {
    "highway_straight": highway_straight,
    "highway_curve": highway_curve,
    "urban_grid": urban_grid,
}


def build(name: str, **kwargs) -> LaneNetwork:
    if name not in REGISTRY:
        raise KeyError(f"unknown network '{name}', have {list(REGISTRY)}")
    return REGISTRY[name](**kwargs)
