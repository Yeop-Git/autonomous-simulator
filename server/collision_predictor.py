"""Collision prediction over the central world model.

Per plan §12.3: sample predicted trajectories on a short horizon and compute
TTC / minimum separation between every pair of dynamic objects. Because the
V2X world gives us exact states, prediction is just kinematic extrapolation —
constant velocity (optionally constant acceleration) in the xz plane.

Pure and Unity-free: feed it ``DynamicVehicle`` / ``DynamicObject`` (or any
object exposing ``position`` and ``velocity``), get back ``Conflict`` records.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Protocol, Sequence

Vec3 = Sequence[float]

DEFAULT_HORIZON = 4.0       # s  (plan: 3–5 s)
DEFAULT_DT = 0.2            # s  (plan: 0.1–0.2 s)
DEFAULT_SAFETY_DISTANCE = 5.0  # m, center-to-center


class HasMotion(Protocol):
    id: str
    position: Vec3
    velocity: Vec3


@dataclass
class Conflict:
    a_id: str
    b_id: str
    ttc: float          # time (s) until the pair first breaches safety distance
    min_distance: float  # minimum predicted separation (m) over the horizon
    t_at_min: float      # time (s) of that minimum separation

    def __repr__(self) -> str:  # pragma: no cover - cosmetic
        return (f"Conflict({self.a_id}<->{self.b_id} "
                f"ttc={self.ttc:.2f}s min={self.min_distance:.2f}m)")


def _xz(p: Vec3) -> tuple[float, float]:
    return p[0], p[2]


def predict_position(obj: HasMotion, t: float, use_accel: bool = False,
                     acceleration: Optional[Vec3] = None) -> list[float]:
    """Constant-velocity (optionally constant-accel) extrapolation."""
    px, py, pz = obj.position
    vx, vy, vz = obj.velocity
    if use_accel and acceleration is not None:
        ax, ay, az = acceleration
        return [px + vx * t + 0.5 * ax * t * t,
                py + vy * t + 0.5 * ay * t * t,
                pz + vz * t + 0.5 * az * t * t]
    return [px + vx * t, py + vy * t, pz + vz * t]


def predict_trajectory(obj: HasMotion, horizon: float = DEFAULT_HORIZON,
                       dt: float = DEFAULT_DT) -> list[tuple[float, list[float]]]:
    """Sampled ``(t, position)`` trajectory over ``[0, horizon]``."""
    if dt <= 0.0:
        raise ValueError("dt must be > 0")
    steps = int(round(horizon / dt))
    return [(i * dt, predict_position(obj, i * dt)) for i in range(steps + 1)]


def closest_approach(a: HasMotion, b: HasMotion,
                     horizon: float = DEFAULT_HORIZON) -> tuple[float, float]:
    """Analytic closest approach of two constant-velocity points in xz.

    Returns ``(t_at_min, min_distance)`` with ``t_at_min`` clamped to
    ``[0, horizon]``. Exact, so it never misses a fast crossing the way a
    coarse time sampling can.
    """
    ax, az = _xz(a.position)
    bx, bz = _xz(b.position)
    avx, avz = a.velocity[0], a.velocity[2]
    bvx, bvz = b.velocity[0], b.velocity[2]

    rpx, rpz = ax - bx, az - bz        # relative position
    rvx, rvz = avx - bvx, avz - bvz    # relative velocity
    rv2 = rvx * rvx + rvz * rvz

    if rv2 == 0.0:
        t_star = 0.0  # parallel / static: distance is constant
    else:
        t_star = -(rpx * rvx + rpz * rvz) / rv2
        t_star = max(0.0, min(horizon, t_star))

    dx = rpx + rvx * t_star
    dz = rpz + rvz * t_star
    return t_star, math.hypot(dx, dz)


def time_to_breach(a: HasMotion, b: HasMotion, safety_distance: float,
                   horizon: float = DEFAULT_HORIZON, dt: float = DEFAULT_DT) -> float:
    """First time in ``[0, horizon]`` the pair is within ``safety_distance``.

    ``inf`` if they never breach within the horizon. Solved analytically:
    ``|rp + rv*t|^2 = safety^2`` is a quadratic in t, so we take its smallest
    non-negative root. This never tunnels through a fast crossing the way a
    coarse time sampling can (``dt`` is kept only for signature stability).
    """
    rpx = a.position[0] - b.position[0]
    rpz = a.position[2] - b.position[2]
    rvx = a.velocity[0] - b.velocity[0]
    rvz = a.velocity[2] - b.velocity[2]

    A = rvx * rvx + rvz * rvz
    B = 2.0 * (rpx * rvx + rpz * rvz)
    C = rpx * rpx + rpz * rpz - safety_distance * safety_distance

    if C <= 0.0:
        return 0.0  # already within safety distance now
    if A == 0.0:
        return math.inf  # no relative motion and currently apart
    disc = B * B - 4.0 * A * C
    if disc < 0.0:
        return math.inf  # closest approach never reaches safety distance
    sq = math.sqrt(disc)
    t_enter = (-B - sq) / (2.0 * A)  # smaller root = first crossing inward
    if t_enter < 0.0:
        return math.inf  # crossing is in the past; pair is now diverging
    return t_enter if t_enter <= horizon else math.inf


class CollisionPredictor:
    def __init__(self, horizon: float = DEFAULT_HORIZON, dt: float = DEFAULT_DT,
                 safety_distance: float = DEFAULT_SAFETY_DISTANCE):
        self.horizon = horizon
        self.dt = dt
        self.safety_distance = safety_distance

    def pair_conflict(self, a: HasMotion, b: HasMotion,
                      safety_distance: Optional[float] = None) -> Optional[Conflict]:
        sd = self.safety_distance if safety_distance is None else safety_distance
        t_min, d_min = closest_approach(a, b, self.horizon)
        if d_min > sd:
            return None
        ttc = time_to_breach(a, b, sd, self.horizon, self.dt)
        return Conflict(a_id=a.id, b_id=b.id, ttc=ttc,
                        min_distance=d_min, t_at_min=t_min)

    def conflicts(self, objects: Iterable[HasMotion],
                  safety_distance: Optional[float] = None) -> list[Conflict]:
        """All breaching pairs, sorted by soonest TTC."""
        objs = list(objects)
        out: list[Conflict] = []
        for i in range(len(objs)):
            for j in range(i + 1, len(objs)):
                c = self.pair_conflict(objs[i], objs[j], safety_distance)
                if c is not None:
                    out.append(c)
        out.sort(key=lambda c: c.ttc)
        return out
