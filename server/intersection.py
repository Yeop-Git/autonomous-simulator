"""Reservation-based unsignalized intersection (plan §13.2).

The project's flagship central-control idea: instead of a traffic light, the
server reserves a time slot in the intersection's conflict zone for each
approaching vehicle, so they interleave without stopping when possible.

Policy: vehicles are served in ETA order (first to arrive gets first slot).
Each is granted the earliest zone-entry time >= its free ETA that keeps a
``buffer`` from every already-granted occupancy window. A vehicle granted a
later slot than its free ETA must slow to hit it — that's the reservation
"yield". Recomputed each tick from current states (no stale reservations).

Pure & testable; the controller/experiment feeds it the approaching vehicles.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from world_model import DynamicVehicle

MIN_SPEED = 0.5
VEHICLE_LENGTH = 4.5


@dataclass
class Grant:
    vehicle_id: str
    enter_time: float    # absolute time the vehicle may enter the zone (s)
    must_yield: bool     # granted later than its free ETA
    target_speed: float  # speed to arrive exactly at enter_time (0 => stop)


@dataclass
class _Approach:
    vehicle_id: str
    dist: float          # distance to the zone entry (m)
    eta: float           # free arrival time at the zone (s, absolute)
    occupancy: float     # time spent inside the zone (s)


class IntersectionManager:
    def __init__(self, center, radius: float = 6.0, buffer: float = 1.5):
        self.center = list(center)
        self.radius = radius
        self.buffer = buffer

    # ------------------------------------------------------------------ #
    def approaching(self, vehicles: Iterable[DynamicVehicle],
                    max_dist: float = 60.0) -> list[DynamicVehicle]:
        """Vehicles heading toward the zone and within ``max_dist`` of it."""
        out = []
        for v in vehicles:
            rx = self.center[0] - v.position[0]
            rz = self.center[2] - v.position[2]
            d = math.hypot(rx, rz)
            if d - self.radius > max_dist or d <= self.radius:
                continue
            # heading toward the zone => velocity component along r is positive
            if v.velocity[0] * rx + v.velocity[2] * rz > 0.0:
                out.append(v)
        return out

    def reserve(self, vehicles: Iterable[DynamicVehicle], t_now: float = 0.0,
                max_dist: float = 60.0) -> dict[str, Grant]:
        approaches = []
        for v in self.approaching(vehicles, max_dist):
            rx = self.center[0] - v.position[0]
            rz = self.center[2] - v.position[2]
            d = max(0.0, math.hypot(rx, rz) - self.radius)
            speed = max(v.speed, MIN_SPEED)
            eta = t_now + d / speed
            occupancy = (2 * self.radius + VEHICLE_LENGTH) / speed
            approaches.append(_Approach(v.id, d, eta, occupancy))

        approaches.sort(key=lambda a: a.eta)  # first-come (by ETA) priority

        grants: dict[str, Grant] = {}
        windows: list[tuple[float, float]] = []  # granted (enter, exit), in time
        for a in approaches:
            enter = a.eta
            # push past any window it would overlap (with buffer), repeat until free
            changed = True
            while changed:
                changed = False
                for (w_in, w_out) in windows:
                    if enter < w_out + self.buffer and enter + a.occupancy > w_in - self.buffer:
                        enter = w_out + self.buffer
                        changed = True
            must_yield = enter > a.eta + 1e-6
            rel = enter - t_now
            target = a.dist / rel if rel > 1e-6 else a.dist  # slow to hit the slot
            grants[a.vehicle_id] = Grant(a.vehicle_id, enter, must_yield, max(0.0, target))
            windows.append((enter, enter + a.occupancy))
        return grants
