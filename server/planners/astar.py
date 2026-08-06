"""A* path planning over the lane graph.

Phase 2 deliverable. Pure and stateless: given start/goal world positions
and a ``World`` (the lane graph), it
  1. locates the start and goal lanes (nearest centerline),
  2. runs A* over the lane graph using lane lengths as edge cost and the
     straight-line distance from a lane's end to the goal as the heuristic,
  3. stitches the chosen lanes' centerlines into one waypoint path, trimmed
     to start at the projected start point and end at the projected goal.

No Unity, no global state — unit-tested on a synthetic graph.
"""
from __future__ import annotations

import heapq
import math

from .base import Path, Vec3, World


def _dist_xz(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[2] - b[2])


def _polyline_length(points: Path) -> float:
    return sum(_dist_xz(points[i], points[i + 1]) for i in range(len(points) - 1))


def _project(position: Vec3, centerline: Path) -> tuple[int, Vec3, float]:
    """Project ``position`` onto a centerline.

    Returns (segment_index, closest_point, lateral_distance). The point is
    the nearest location on the polyline; segment_index is the index of the
    segment start vertex it falls on.
    """
    best = (0, list(centerline[0]), _dist_xz(position, centerline[0]))
    for i in range(len(centerline) - 1):
        a, b = centerline[i], centerline[i + 1]
        dx, dz = b[0] - a[0], b[2] - a[2]
        seg_sq = dx * dx + dz * dz
        if seg_sq == 0.0:
            continue
        t = ((position[0] - a[0]) * dx + (position[2] - a[2]) * dz) / seg_sq
        t = max(0.0, min(1.0, t))
        point = [a[0] + t * dx, a[1] + t * (b[1] - a[1]), a[2] + t * dz]
        lat = _dist_xz(position, point)
        if lat < best[2]:
            best = (i, point, lat)
    return best


LANE_TIE_BAND = 1.0   # m; lanes this close to the best one count as tied
MAX_LANE_CANDIDATES = 4


def _candidate_lanes(position: Vec3, world: World) -> list[str]:
    """Lanes that could plausibly own ``position``, closest first.

    Only lanes within ``LANE_TIE_BAND`` of the closest one are returned, so on
    an ordinary stretch of road this is just ``[nearest_lane]``.
    """
    all_ids = getattr(world, "all_lane_ids", None)
    if all_ids is None:
        nearest = world.nearest_lane(position)
        return [nearest] if nearest else []

    scored: list[tuple[float, str]] = []
    for lane_id in all_ids():
        centerline = world.lane_centerline(lane_id)
        if len(centerline) < 2:
            continue
        scored.append((_project(position, centerline)[2], lane_id))
    if not scored:
        return []
    scored.sort()
    best = scored[0][0]
    return [lane_id for lateral, lane_id in scored[:MAX_LANE_CANDIDATES]
            if lateral <= best + LANE_TIE_BAND]


class AStarPlanner:
    name = "astar"

    def __init__(self, heuristic: str = "euclidean") -> None:
        self.heuristic = heuristic
        # Filled in after each plan() for experiment logging / inspection.
        self.last_lane_route: list[str] = []
        self.last_expanded: int = 0

    # ---- public API ------------------------------------------------------- #
    def plan(self, start: Vec3, goal: Vec3, world: World) -> Path:
        start = list(start)
        goal = list(goal)
        self.last_lane_route = []
        self.last_expanded = 0

        # At an intersection several connectors leave the same point, so "the"
        # nearest lane to a car sitting on the stop line is a coin flip: pick
        # the left-turn connector for a car going straight and the search
        # returns no route at all. Try the tied candidates in order instead.
        starts = _candidate_lanes(start, world)
        goals = _candidate_lanes(goal, world)
        if not starts or not goals:
            return []

        for start_lane in starts:
            for goal_lane in goals:
                lane_route = self._search_lane_graph(
                    start_lane, goal_lane, goal, world)
                if lane_route:
                    self.last_lane_route = lane_route
                    return self._stitch(lane_route, start, goal, world)
        return []

    # ---- lane-graph A* ---------------------------------------------------- #
    def _search_lane_graph(
        self, start_lane: str, goal_lane: str, goal: Vec3, world: World
    ) -> list[str]:
        def h(lane_id: str) -> float:
            cl = world.lane_centerline(lane_id)
            return _dist_xz(cl[-1], goal) if cl else 0.0

        open_heap: list[tuple[float, str]] = [(h(start_lane), start_lane)]
        g_score: dict[str, float] = {start_lane: 0.0}
        came_from: dict[str, str] = {}
        closed: set[str] = set()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)
            self.last_expanded += 1

            if current == goal_lane:
                return self._reconstruct(came_from, current)

            cur_cl = world.lane_centerline(current)
            cur_len = _polyline_length(cur_cl) if cur_cl else 0.0
            for nxt in world.neighbors(current):
                nxt_cl = world.lane_centerline(nxt)
                if not nxt_cl:
                    continue
                # Skip lanes whose entry is blocked by a hazard.
                if world.is_blocked(nxt_cl[0]):
                    continue
                tentative = g_score[current] + cur_len
                if tentative < g_score.get(nxt, math.inf):
                    came_from[nxt] = current
                    g_score[nxt] = tentative
                    heapq.heappush(open_heap, (tentative + h(nxt), nxt))
        return []

    @staticmethod
    def _reconstruct(came_from: dict[str, str], current: str) -> list[str]:
        route = [current]
        while current in came_from:
            current = came_from[current]
            route.append(current)
        route.reverse()
        return route

    # ---- centerline stitching --------------------------------------------- #
    def _stitch(self, lane_route: list[str], start: Vec3, goal: Vec3, world: World) -> Path:
        """Concatenate lane centerlines into one path, trimmed to start/goal.

        The first lane is entered at the start projection and the last lane is
        exited at the goal projection. Every intermediate lane is entered at
        the projection of the point where the *previous* lane ended: a
        successor need not begin where its predecessor ends (an on-ramp merges
        into the middle of the mainline lane), and taking such a lane from its
        own start would teleport the path backwards to the mainline origin.
        Consecutive duplicate points (lane joins that share a vertex) collapse.
        """
        # Single-lane route: build the sub-polyline between the two projections
        # directly (forward or backward along the lane) so the goal is never
        # lost when it projects behind the start.
        if len(lane_route) == 1:
            cl = world.lane_centerline(lane_route[0])
            return _dedupe(_sub_polyline(cl, start, goal)) if cl else []

        path: Path = []
        entry: Vec3 = list(start)
        for idx, lane_id in enumerate(lane_route):
            cl = world.lane_centerline(lane_id)
            if not cl:
                continue
            last = idx == len(lane_route) - 1
            path.extend(_sub_polyline(cl, entry, goal if last else None))
            entry = list(cl[-1])  # we leave this lane where it ends

        return _dedupe(path)


def _sub_polyline(cl: Path, start: Vec3 | None = None,
                  end: Vec3 | None = None) -> Path:
    """Portion of ``cl`` between the projections of ``start`` and ``end``.

    ``None`` means "the lane's own start/end". A goal that projects behind the
    entry point yields the reversed interior rather than dropping the goal.
    """
    si, sp = (0, list(cl[0])) if start is None else _project(start, cl)[:2]
    gi, gp = ((len(cl) - 2, list(cl[-1])) if end is None
              else _project(end, cl)[:2])
    if gi >= si:
        interior = [list(p) for p in cl[si + 1: gi + 1]]
    else:
        interior = [list(p) for p in cl[gi + 1: si + 1]]
        interior.reverse()
    return [sp] + interior + [gp]


def _dedupe(points: Path, eps: float = 1e-6) -> Path:
    out: Path = []
    for p in points:
        if not out or _dist_xz(out[-1], p) > eps:
            out.append([float(p[0]), float(p[1]), float(p[2])])
    return out
