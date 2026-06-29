"""Server-side lateral controllers for headless LKA experiments (Phase 4).

Unity runs its own lateral control (C# LKAController) for the live demo, but
to study LKA *quantitatively* without the editor — lateral error vs. speed vs.
curvature (plan §11) — we need the same laws in Python, driving the headless
sim. All three implement one interface so the experiment runner can swap them:

    steer(x, z, heading_deg, speed, centerline, wheel_base) -> steering_rad

plus a shared ``frenet_errors`` that yields the logged diagnostics
(lateral_error, heading_error, curvature) from a lane/path centerline.

Conventions match the rest of the project: positions in meters, ``heading`` is
yaw in degrees with 0 = +Z and increasing clockwise (Unity), so the forward
unit vector is ``(sin h, cos h)`` in the (x, z) plane.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

Point = list[float]  # [x, y, z]


# --------------------------------------------------------------------------- #
# Shared geometry
# --------------------------------------------------------------------------- #
def _delta_angle_deg(a: float, b: float) -> float:
    """Smallest signed difference b-a folded into [-180, 180] degrees."""
    d = (b - a + 180.0) % 360.0 - 180.0
    return d


def _nearest_index(x: float, z: float, centerline: list[Point]) -> int:
    best, best_d = 0, math.inf
    for i, p in enumerate(centerline):
        d = (p[0] - x) ** 2 + (p[2] - z) ** 2
        if d < best_d:
            best_d, best = d, i
    return best


def _segment_dir(centerline: list[Point], idx: int) -> tuple[float, float]:
    a = min(idx, len(centerline) - 2)
    dx = centerline[a + 1][0] - centerline[a][0]
    dz = centerline[a + 1][2] - centerline[a][2]
    n = math.hypot(dx, dz)
    if n < 1e-9:
        return 0.0, 1.0
    return dx / n, dz / n


def _menger_curvature(centerline: list[Point], idx: int) -> float:
    """Curvature (1/m) estimated from the triangle of three nearby points."""
    if idx <= 0 or idx >= len(centerline) - 1:
        return 0.0
    a, b, c = centerline[idx - 1], centerline[idx], centerline[idx + 1]
    ax, az = a[0], a[2]
    bx, bz = b[0], b[2]
    cx, cz = c[0], c[2]
    area2 = abs((bx - ax) * (cz - az) - (cx - ax) * (bz - az))  # 2*area
    la = math.hypot(bx - ax, bz - az)
    lb = math.hypot(cx - bx, cz - bz)
    lc = math.hypot(cx - ax, cz - az)
    denom = la * lb * lc
    if denom < 1e-9:
        return 0.0
    return 2.0 * area2 / denom  # = 4*area / (la*lb*lc); area2=2*area


def frenet_errors(x: float, z: float, heading_deg: float,
                  centerline: list[Point]) -> tuple[float, float, float]:
    """Return ``(lateral_error, heading_error_deg, curvature)``.

    ``lateral_error`` is signed: positive = vehicle is to the LEFT of the path
    direction (consistent with a left-handed steering correction).
    """
    if len(centerline) < 2:
        return 0.0, 0.0, 0.0
    idx = _nearest_index(x, z, centerline)
    dirx, dirz = _segment_dir(centerline, idx)
    near = centerline[min(idx, len(centerline) - 1)]
    offx, offz = x - near[0], z - near[2]
    # signed cross-track: cross(dir, off).y in Unity's left-handed xz frame
    lateral = dirz * offx - dirx * offz
    path_heading = math.degrees(math.atan2(dirx, dirz))
    heading_err = _delta_angle_deg(heading_deg, path_heading)
    curvature = _menger_curvature(centerline, idx)
    return lateral, heading_err, curvature


# --------------------------------------------------------------------------- #
# Controllers
# --------------------------------------------------------------------------- #
@dataclass
class PurePursuit:
    name: str = "pure_pursuit"
    lookahead_base: float = 4.0
    lookahead_k: float = 0.4
    max_steer: float = 0.6

    def steer(self, x, z, heading_deg, speed, centerline, wheel_base,
              dt: float = 0.1) -> float:
        if len(centerline) < 2:
            return 0.0
        ld = self.lookahead_base + self.lookahead_k * speed
        tx, tz = self._lookahead(x, z, centerline, ld)
        target_angle = math.degrees(math.atan2(tx - x, tz - z))
        alpha = math.radians(_delta_angle_deg(heading_deg, target_angle))
        delta = math.atan2(2.0 * wheel_base * math.sin(alpha), max(ld, 0.1))
        return _clamp(delta, self.max_steer)

    @staticmethod
    def _lookahead(x, z, centerline, ld) -> tuple[float, float]:
        near = _nearest_index(x, z, centerline)
        for i in range(near, len(centerline)):
            if math.hypot(centerline[i][0] - x, centerline[i][2] - z) >= ld:
                return centerline[i][0], centerline[i][2]
        return centerline[-1][0], centerline[-1][2]


@dataclass
class Stanley:
    name: str = "stanley"
    gain: float = 1.5
    softening: float = 1.0
    max_steer: float = 0.6

    def steer(self, x, z, heading_deg, speed, centerline, wheel_base,
              dt: float = 0.1) -> float:
        lateral, heading_err, _ = frenet_errors(x, z, heading_deg, centerline)
        heading_term = math.radians(heading_err)
        cross_term = math.atan2(self.gain * (-lateral), self.softening + speed)
        return _clamp(heading_term + cross_term, self.max_steer)


@dataclass
class PIDLateral:
    name: str = "pid"
    kp: float = 0.12
    ki: float = 0.0
    kd: float = 0.4
    max_steer: float = 0.6
    _integral: float = 0.0
    _prev: float = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._prev = 0.0

    def steer(self, x, z, heading_deg, speed, centerline, wheel_base,
              dt: float = 0.1) -> float:
        lateral, _, _ = frenet_errors(x, z, heading_deg, centerline)
        err = -lateral  # drive lateral error to zero
        self._integral += err * dt
        deriv = (err - self._prev) / dt if dt > 0 else 0.0
        self._prev = err
        out = self.kp * err + self.ki * self._integral + self.kd * deriv
        return _clamp(out, self.max_steer)


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


REGISTRY = {
    "pure_pursuit": PurePursuit,
    "stanley": Stanley,
    "pid": PIDLateral,
}


def make(name: str, **kwargs):
    if name not in REGISTRY:
        raise KeyError(f"unknown lateral controller '{name}', have {list(REGISTRY)}")
    return REGISTRY[name](**kwargs)
