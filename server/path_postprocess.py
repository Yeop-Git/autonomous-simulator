"""Collision-preserving cleanup for sampling-planner vehicle paths."""
from __future__ import annotations

import math

from planners import _rrt_common as rc
from planners.base import Path, Vec3


def shortcut(path: Path, world, resolution: float = 0.5) -> Path:
    """Greedily remove RRT vertices while preserving collision clearance."""
    if len(path) <= 2:
        return [list(p) for p in path]
    out: Path = [list(path[0])]
    i = 0
    while i < len(path) - 1:
        nxt = len(path) - 1
        while nxt > i + 1:
            if rc.collision_free(path[i], path[nxt], world, resolution):
                break
            nxt -= 1
        out.append(list(path[nxt]))
        i = nxt
    return out


def resample(path: Path, spacing: float = 2.0) -> Path:
    """Return evenly spaced waypoints suitable for Unity path following."""
    if len(path) < 2:
        return [list(p) for p in path]
    out: Path = [list(path[0])]
    carry = spacing
    for a, b in zip(path, path[1:]):
        length = rc.dist_xz(a, b)
        if length <= 1e-9:
            continue
        distance = carry
        while distance < length:
            t = distance / length
            out.append([a[j] + (b[j] - a[j]) * t for j in range(3)])
            distance += spacing
        carry = distance - length
    if rc.dist_xz(out[-1], path[-1]) > 0.05:
        out.append(list(path[-1]))
    return out


def max_turn_angle(path: Path) -> float:
    best = 0.0
    for a, b, c in zip(path, path[1:], path[2:]):
        ux, uz = b[0] - a[0], b[2] - a[2]
        vx, vz = c[0] - b[0], c[2] - b[2]
        un, vn = math.hypot(ux, uz), math.hypot(vx, vz)
        if un <= 1e-9 or vn <= 1e-9:
            continue
        dot = max(-1.0, min(1.0, (ux * vx + uz * vz) / (un * vn)))
        best = max(best, math.degrees(math.acos(dot)))
    return best


def prepare(path: Path, world, *, max_angle_deg: float = 78.0) -> Path:
    cleaned = shortcut(path, world)
    if len(cleaned) < 2 or max_turn_angle(cleaned) > max_angle_deg:
        return []
    sampled = resample(cleaned)
    if any(world.is_blocked(point) for point in sampled):
        return []
    return sampled
