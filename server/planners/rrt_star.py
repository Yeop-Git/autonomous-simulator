"""RRT* path planning in the continuous xz plane (plan §15.3).

RRT* extends RRT with two improvements that trade compute for path quality:

  1. **Choose-parent**: a new node is attached to the neighbour that minimizes
     its cost-to-come (not just the nearest node).
  2. **Rewire**: existing neighbours are re-parented through the new node when
     that lowers their cost.

Unlike RRT it does not stop at the first connection — it keeps refining for the
full iteration budget, then returns the lowest-cost collision-free path to the
goal found. This is the "better path quality, more compute" arm of the A* / RRT
/ RRT* comparison (plan §15.4, experiment §20.1).

Pure and deterministic given a seed. Conforms to the ``Planner`` interface.
"""
from __future__ import annotations

import math
import random

from . import _rrt_common as rc
from .base import Path, Vec3, World
from .rrt import _nearest


class RRTStarPlanner:
    name = "rrt_star"

    def __init__(self, config: rc.RRTConfig | None = None,
                 rewire_radius: float = 12.0, **overrides) -> None:
        self.config = config or rc.RRTConfig(**overrides)
        self.rewire_radius = rewire_radius
        self.last_nodes: int = 0
        self.last_iters: int = 0
        self.last_cost: float = 0.0
        self.last_lane_route: list[str] = []

    def plan(self, start: Vec3, goal: Vec3, world: World) -> Path:
        cfg = self.config
        rng = random.Random(cfg.seed)
        start = [float(start[0]), float(start[1]), float(start[2])]
        goal = [float(goal[0]), float(goal[1]), float(goal[2])]
        y = start[1]

        self.last_nodes = 1
        self.last_iters = 0
        self.last_cost = 0.0

        if world.is_blocked(start) or world.is_blocked(goal):
            return []

        nodes: list[Vec3] = [start]
        parents: dict[int, int | None] = {0: None}
        cost: dict[int, float] = {0: 0.0}
        bnds = rc.bounds(start, goal, cfg.margin)

        for _ in range(cfg.max_iters):
            self.last_iters += 1
            target = rc.sample(rng, goal, bnds, cfg.goal_sample_rate, y)
            ni = _nearest(nodes, target)
            new = rc.steer(nodes[ni], target, cfg.step_size)
            if not rc.collision_free(nodes[ni], new, world, cfg.edge_resolution):
                continue

            # Neighbours within the rewire radius that connect collision-free.
            near = [
                i for i in range(len(nodes))
                if rc.dist_xz(nodes[i], new) <= self.rewire_radius
                and rc.collision_free(nodes[i], new, world, cfg.edge_resolution)
            ]

            # Choose the parent that minimizes cost-to-come.
            best_parent, best_cost = ni, cost[ni] + rc.dist_xz(nodes[ni], new)
            for i in near:
                c = cost[i] + rc.dist_xz(nodes[i], new)
                if c < best_cost:
                    best_parent, best_cost = i, c

            nodes.append(new)
            new_idx = len(nodes) - 1
            parents[new_idx] = best_parent
            cost[new_idx] = best_cost
            self.last_nodes += 1

            # Rewire neighbours through the new node when it lowers their cost.
            for i in near:
                c = best_cost + rc.dist_xz(new, nodes[i])
                if c < cost[i]:
                    parents[i] = new_idx
                    cost[i] = c

        return self._best_goal_path(nodes, parents, cost, goal, world, cfg)

    def _best_goal_path(self, nodes, parents, cost, goal, world, cfg) -> Path:
        """Pick the lowest total-cost node that can connect to the goal."""
        best_idx, best_total = None, math.inf
        for i in range(len(nodes)):
            d = rc.dist_xz(nodes[i], goal)
            if d > cfg.goal_radius:
                continue
            if not rc.collision_free(nodes[i], goal, world, cfg.edge_resolution):
                continue
            total = cost[i] + d
            if total < best_total:
                best_idx, best_total = i, total
        if best_idx is None:
            return []
        path = rc.reconstruct(nodes, parents, best_idx)
        if rc.dist_xz(path[-1], goal) > 1e-6:
            path.append(list(goal))
        self.last_cost = rc.polyline_length(path)
        return path
