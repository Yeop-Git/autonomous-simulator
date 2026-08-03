"""RRT path planning in the continuous xz plane (plan §15.2).

Complementary to the lane-graph A*: where A* searches a fixed road graph, RRT
grows a random tree through free space and avoids obstacles via
``world.is_blocked``. This is the planner meant for "비정형 장애물 회피" —
irregular obstacle avoidance, stalled-car detours, parking-lot / free-space
navigation — the cases the plan flags as A*'s weakness (§15.4).

Pure and deterministic given a seed. No Unity, no global state. Conforms to the
``Planner`` interface (``plan(start, goal, world) -> Path``) so the experiment
runner can swap it for A* / RRT* freely.
"""
from __future__ import annotations

import random

from . import _rrt_common as rc
from .base import Path, Vec3, World


class RRTPlanner:
    name = "rrt"

    def __init__(self, config: rc.RRTConfig | None = None, **overrides) -> None:
        self.config = config or rc.RRTConfig(**overrides)
        # Filled after each plan() for experiment logging / inspection.
        self.last_nodes: int = 0
        self.last_iters: int = 0
        self.last_cost: float = 0.0
        self.last_lane_route: list[str] = []  # RRT ignores the lane graph

    def plan(self, start: Vec3, goal: Vec3, world: World) -> Path:
        cfg = self.config
        rng = random.Random(cfg.seed)
        start = [float(start[0]), float(start[1]), float(start[2])]
        goal = [float(goal[0]), float(goal[1]), float(goal[2])]
        y = start[1]

        self.last_nodes = 1
        self.last_iters = 0
        self.last_cost = 0.0

        # Degenerate cases: a blocked endpoint has no feasible path.
        if world.is_blocked(start) or world.is_blocked(goal):
            return []
        # Trivial straight shot.
        if rc.collision_free(start, goal, world, cfg.edge_resolution):
            self.last_cost = rc.dist_xz(start, goal)
            return [start, goal] if self.last_cost > 0 else [start]

        nodes: list[Vec3] = [start]
        parents: dict[int, int | None] = {0: None}
        bnds = rc.bounds(start, goal, cfg.margin)

        for _ in range(cfg.max_iters):
            self.last_iters += 1
            target = rc.sample(rng, goal, bnds, cfg.goal_sample_rate, y)
            ni = _nearest(nodes, target)
            new = rc.steer(nodes[ni], target, cfg.step_size)
            if not rc.collision_free(nodes[ni], new, world, cfg.edge_resolution):
                continue
            nodes.append(new)
            new_idx = len(nodes) - 1
            parents[new_idx] = ni
            self.last_nodes += 1

            if rc.dist_xz(new, goal) <= cfg.goal_radius and \
                    rc.collision_free(new, goal, world, cfg.edge_resolution):
                nodes.append(goal)
                parents[len(nodes) - 1] = new_idx
                path = rc.reconstruct(nodes, parents, len(nodes) - 1)
                self.last_cost = rc.polyline_length(path)
                return path

        return []  # failed within the iteration budget


def _nearest(nodes: list[Vec3], target: Vec3) -> int:
    best_i, best_d = 0, rc.dist_xz(nodes[0], target)
    for i in range(1, len(nodes)):
        d = rc.dist_xz(nodes[i], target)
        if d < best_d:
            best_i, best_d = i, d
    return best_i
