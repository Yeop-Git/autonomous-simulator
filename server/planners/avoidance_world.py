"""Local RRT search space constrained to authored drivable corridors.

The lane graph remains the source of road geometry.  This adapter turns the
union of selected lane strips into free space and adds inflated snapshots of
static and predicted dynamic obstacles.  RRT/RRT* only need ``is_blocked``;
the remaining world methods delegate to the lane network for interface
compatibility and diagnostics.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from planners.base import Vec3
from world_model import DynamicObject, DynamicVehicle, LaneNetwork, dist_xz


@dataclass(frozen=True)
class Occupancy:
    position: Vec3
    radius: float
    source_id: str


class AvoidanceWorld:
    def __init__(self, network: LaneNetwork,
                 vehicles: Iterable[DynamicVehicle] = (),
                 objects: Iterable[DynamicObject] = (),
                 *, allowed_lane_ids: Iterable[str] | None = None,
                 exclude_ids: Iterable[str] = (),
                 vehicle_clearance: float = 1.45,
                 prediction_horizon: float = 3.0,
                 prediction_dt: float = 0.5):
        self.network = network
        ids = list(allowed_lane_ids or network.all_lane_ids())
        self.allowed_lane_ids = [lane_id for lane_id in ids
                                 if network.lane(lane_id) is not None]
        excluded = set(exclude_ids)
        self.occupancies: list[Occupancy] = []

        for vehicle in vehicles:
            if vehicle.id in excluded:
                continue
            self._add_prediction(
                vehicle.id, vehicle.position, vehicle.velocity,
                radius=vehicle_clearance + 1.0,
                horizon=prediction_horizon, dt=prediction_dt)

        for obj in objects:
            if obj.id in excluded:
                continue
            speed = math.hypot(obj.velocity[0], obj.velocity[2])
            radius = max(0.2, obj.radius) + vehicle_clearance
            if speed <= 0.05:
                self.occupancies.append(
                    Occupancy(list(obj.position), radius, obj.id))
            else:
                self._add_prediction(
                    obj.id, obj.position, obj.velocity, radius,
                    prediction_horizon, prediction_dt)

    def _add_prediction(self, source_id: str, position: Vec3, velocity: Vec3,
                        radius: float, horizon: float, dt: float) -> None:
        steps = max(0, int(round(horizon / max(dt, 1e-3))))
        for i in range(steps + 1):
            t = i * dt
            self.occupancies.append(Occupancy(
                [position[0] + velocity[0] * t,
                 position[1] + velocity[1] * t,
                 position[2] + velocity[2] * t],
                radius, source_id))

    def is_blocked(self, position: Vec3) -> bool:
        if not self._inside_drivable_corridor(position):
            return True
        return any(dist_xz(position, occ.position) <= occ.radius
                   for occ in self.occupancies)

    def _inside_drivable_corridor(self, position: Vec3) -> bool:
        for lane_id in self.allowed_lane_ids:
            lane = self.network.lane(lane_id)
            if lane is None:
                continue
            _, lateral, _ = lane.closest_point(position)
            # Keep the vehicle centre inside the authored strip while allowing
            # a small overlap between adjacent strips for lane transitions.
            if lateral <= lane.width * 0.5 + 0.35:
                return True
        return False

    def minimum_clearance(self, path: list[Vec3]) -> float:
        if not path or not self.occupancies:
            return math.inf
        best = math.inf
        for point in path:
            for occ in self.occupancies:
                best = min(best, dist_xz(point, occ.position) - occ.radius)
        return best

    def neighbors(self, lane_id: str) -> list[str]:
        return self.network.neighbors(lane_id)

    def lane_centerline(self, lane_id: str) -> list[Vec3]:
        return self.network.lane_centerline(lane_id)

    def nearest_lane(self, position: Vec3) -> str | None:
        return self.network.nearest_lane(position)
