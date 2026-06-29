"""Common planner interface.

Every planner in this package implements `plan(start, goal, world)` and
returns a list of [x, y, z] waypoints. Keeping the signature identical lets
the experiment runner swap A* / RRT / RRT* without touching call sites.

Implementations are intentionally left as stubs — fill them in per Phase 2
(A*) and Phase 7 (RRT / RRT*).
"""
from __future__ import annotations

from typing import Protocol, Sequence

Vec3 = Sequence[float]  # [x, y, z]
Path = list[list[float]]


class World(Protocol):
    """Minimal view a planner needs of the world.

    Concrete implementation lives in server/world_model.py. Planners should
    depend only on this protocol so they stay testable without Unity.
    """

    def neighbors(self, lane_id: str) -> list[str]:
        """Lane-graph successors of a lane."""
        ...

    def lane_centerline(self, lane_id: str) -> Path:
        """Ordered centerline waypoints for a lane."""
        ...

    def is_blocked(self, position: Vec3) -> bool:
        """Whether a world position is currently obstructed."""
        ...


class Planner(Protocol):
    def plan(self, start: Vec3, goal: Vec3, world: World) -> Path:
        """Return a list of waypoints from start to goal. Empty list = no path."""
        ...
