"""Emergency-vehicle priority / yielding (plan §6.2, §12.4, §14.2).

A V2X-world advantage: the server knows an emergency vehicle's exact state, so
surrounding cars can yield *before* it's a conflict. Policy: if an
``emergency_vehicle`` object is within ``radius`` and approaching the ego
(closing distance and roughly heading toward it), the ego yields by dropping
to a low ``yield_speed`` (and would pull aside where geometry allows — lane
selection is left to the lane-change layer).

Pure helper used by the central controller.
"""
from __future__ import annotations

import math
from typing import Iterable, Optional

from world_model import DynamicObject, DynamicVehicle

YIELD_RADIUS = 60.0     # m, start yielding within this range
YIELD_SPEED = 3.0       # m/s, crawl speed while yielding


def approaching_emergency(ego: DynamicVehicle, objects: Iterable[DynamicObject],
                          radius: float = YIELD_RADIUS) -> Optional[DynamicObject]:
    """Return the nearest approaching emergency vehicle, or None."""
    best, best_d = None, math.inf
    for o in objects:
        if o.type != "emergency_vehicle":
            continue
        rx = ego.position[0] - o.position[0]
        rz = ego.position[2] - o.position[2]
        d = math.hypot(rx, rz)
        if d > radius:
            continue
        # closing if the emergency vehicle's velocity has a component toward ego
        closing = o.velocity[0] * rx + o.velocity[2] * rz
        if closing > 0.0 and d < best_d:
            best, best_d = o, d
    return best


def yield_speed(ego: DynamicVehicle, objects: Iterable[DynamicObject],
                radius: float = YIELD_RADIUS,
                crawl: float = YIELD_SPEED) -> Optional[float]:
    """Recommended yield speed (m/s) if an emergency vehicle is approaching,
    else None (no yield needed)."""
    ev = approaching_emergency(ego, objects, radius)
    return crawl if ev is not None else None
