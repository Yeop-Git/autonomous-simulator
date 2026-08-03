"""Shared sampling-based planning primitives for RRT / RRT* (plan §15.2-15.3).

Continuous xz-plane planners that avoid obstacles via ``world.is_blocked`` —
complementary to the lane-graph A*. Used for "비정형 장애물 회피" (irregular
obstacle avoidance): road blockage, stalled-car detour, parking-lot style free
space. Deterministic given a seed.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

Vec3 = list[float]


@dataclass
class RRTConfig:
    step_size: float = 4.0
    goal_sample_rate: float = 0.1   # P(sample the goal directly)
    max_iters: int = 2000
    goal_radius: float = 4.0
    edge_resolution: float = 1.0    # m between collision checks along an edge
    margin: float = 30.0            # bounds padding around the start/goal bbox
    seed: int = 0


def dist_xz(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[2] - b[2])


def polyline_length(points: list[Vec3]) -> float:
    return sum(dist_xz(points[i], points[i + 1]) for i in range(len(points) - 1))


def bounds(start: Vec3, goal: Vec3, margin: float):
    minx = min(start[0], goal[0]) - margin
    maxx = max(start[0], goal[0]) + margin
    minz = min(start[2], goal[2]) - margin
    maxz = max(start[2], goal[2]) + margin
    return minx, maxx, minz, maxz


def steer(src: Vec3, dst: Vec3, step: float) -> Vec3:
    d = dist_xz(src, dst)
    if d <= step:
        return [dst[0], src[1], dst[2]]
    t = step / d
    return [src[0] + (dst[0] - src[0]) * t, src[1], src[2] + (dst[2] - src[2]) * t]


def collision_free(a: Vec3, b: Vec3, world, resolution: float) -> bool:
    """True if the segment a->b is clear of blocked positions."""
    d = dist_xz(a, b)
    n = max(1, int(d / resolution))
    for i in range(n + 1):
        t = i / n
        p = [a[0] + (b[0] - a[0]) * t, a[1], a[2] + (b[2] - a[2]) * t]
        if world.is_blocked(p):
            return False
    return True


def sample(rng: random.Random, goal: Vec3, bnds, rate: float, y: float) -> Vec3:
    if rng.random() < rate:
        return list(goal)
    minx, maxx, minz, maxz = bnds
    return [rng.uniform(minx, maxx), y, rng.uniform(minz, maxz)]


def reconstruct(nodes: list[Vec3], parents: dict[int, int], idx: int) -> list[Vec3]:
    path = []
    while idx is not None:
        path.append(list(nodes[idx]))
        idx = parents[idx]
    path.reverse()
    return path
