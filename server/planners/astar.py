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
        start_lane = world.nearest_lane(start)
        goal_lane = world.nearest_lane(goal)
        self.last_lane_route = []
        self.last_expanded = 0
        if start_lane is None or goal_lane is None:
            return []

        lane_route = self._search_lane_graph(start_lane, goal_lane, goal, world)
        if not lane_route:
            return []
        self.last_lane_route = lane_route
        return self._stitch(lane_route, start, goal, world)

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

        The first lane is entered at the start projection; the last lane is
        exited at the goal projection. Consecutive duplicate points (lane
        joins that share a vertex) are collapsed.
        """
        # Single-lane route: build the sub-polyline between the two projections
        # directly (forward or backward along the lane) so the goal is never
        # lost when it projects behind the start.
        if len(lane_route) == 1:
            cl = world.lane_centerline(lane_route[0])
            return _stitch_single(cl, start, goal) if cl else []

        path: Path = []
        for idx, lane_id in enumerate(lane_route):
            cl = world.lane_centerline(lane_id)
            if not cl:
                continue
            first = idx == 0
            last = idx == len(lane_route) - 1

            if first:
                seg_i, proj, _ = _project(start, cl)
                segment: Path = [proj] + [list(p) for p in cl[seg_i + 1:]]
            else:
                segment = [list(p) for p in cl]

            if last:
                # Trim everything past the goal projection on this lane.
                seg_i, proj, _ = _project(goal, cl)
                segment = segment[: seg_i + 1] + [proj]

            path.extend(segment)

        return _dedupe(path)


def _stitch_single(cl: Path, start: Vec3, goal: Vec3) -> Path:
    """Sub-polyline of one lane between the start and goal projections, in
    travel order (handles goal-behind-start without dropping the goal)."""
    si, sp, _ = _project(start, cl)
    gi, gp, _ = _project(goal, cl)
    if gi >= si:
        interior = [list(p) for p in cl[si + 1: gi + 1]]
        pts = [sp] + interior + [gp]
    else:
        interior = [list(p) for p in cl[gi + 1: si + 1]]
        interior.reverse()
        pts = [sp] + interior + [gp]
    return _dedupe(pts)


def _dedupe(points: Path, eps: float = 1e-6) -> Path:
    out: Path = []
    for p in points:
        if not out or _dist_xz(out[-1], p) > eps:
            out.append([float(p[0]), float(p[1]), float(p[2])])
    return out
