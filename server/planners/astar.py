"""A* path planning over the lane graph.

Phase 2 deliverable. Implement plan() to do A* on the lane graph exposed by
the World protocol. Keep it pure: no Unity, no global state.
"""
from __future__ import annotations

from .base import Path, Vec3, World


class AStarPlanner:
    def __init__(self, heuristic: str = "euclidean") -> None:
        self.heuristic = heuristic

    def plan(self, start: Vec3, goal: Vec3, world: World) -> Path:
        # TODO(Phase 2): A* over world.neighbors() + lane centerlines.
        # 1. locate start/goal lanes
        # 2. A* on lane graph using lane lengths as cost
        # 3. stitch lane centerlines into a single waypoint path
        raise NotImplementedError("A* planner not yet implemented (Phase 2).")
