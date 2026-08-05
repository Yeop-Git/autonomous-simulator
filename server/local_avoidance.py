"""Stateful local avoidance and emergency pull-over policy.

Global routing remains A*.  In the dedicated emergency-avoidance scenario this
manager temporarily owns the commanded path, runs a corridor-constrained RRT or
RRT*, and hands control back after a validated lane rejoin.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import emergency
import path_postprocess
from planners import RRTConfig, RRTPlanner, RRTStarPlanner
from planners.avoidance_world import AvoidanceWorld
from planners.base import Path, Vec3
from world_model import DynamicObject, DynamicVehicle, Lane, WorldModel, dist_xz

ACTIVE_SCENARIOS = {"emergency_avoidance", "integrated_city"}
HAZARD_TYPES = {"unexpected_obstacle", "static_obstacle"}


@dataclass
class AvoidanceDecision:
    behavior: str
    path: Path = field(default_factory=list)
    target_lane: str | None = None
    target_speed: float = 0.0
    turn_signal: str = "hazard"
    planner: str = "astar"
    plan_status: str = "idle"
    planning_time_ms: float = 0.0
    minimum_clearance: float = math.inf


@dataclass
class _Maneuver:
    cause: str
    source_id: str
    original_lane: str
    target_lane: str
    phase: str = "HazardDetected"
    path: Path = field(default_factory=list)
    planner: str = "rrt"
    planning_time_ms: float = 0.0
    minimum_clearance: float = math.inf
    last_plan_time: float = -math.inf
    clear_since: float | None = None


class LocalAvoidanceManager:
    def __init__(self):
        self._active: dict[str, _Maneuver] = {}

    def update(self, vehicle: DynamicVehicle, world: WorldModel) \
            -> AvoidanceDecision | None:
        if world.scenario not in ACTIVE_SCENARIOS:
            self._active.pop(vehicle.id, None)
            return None

        maneuver = self._active.get(vehicle.id)
        if maneuver is None:
            trigger = self._trigger(vehicle, world)
            if trigger is None:
                return None
            cause, source, target_lane = trigger
            maneuver = _Maneuver(
                cause=cause, source_id=source.id,
                original_lane=vehicle.current_lane,
                target_lane=target_lane,
                planner=self._planner_name(world))
            self._active[vehicle.id] = maneuver
            return self._decision(maneuver, vehicle, "HazardDetected", 5.0)

        if maneuver.phase == "HazardDetected":
            maneuver.phase = "EscapePlanning"
            return self._decision(maneuver, vehicle, "EscapePlanning", 3.0)

        if maneuver.phase in {"EscapePlanning", "ControlledStopping"}:
            if (maneuver.phase == "ControlledStopping"
                    and world.time - maneuver.last_plan_time < 0.75):
                return self._decision(
                    maneuver, vehicle, "ControlledStopping", 0.0)
            if self._plan(vehicle, world, maneuver, maneuver.target_lane):
                maneuver.phase = "LateralEvading"
                return self._decision(
                    maneuver, vehicle, "LateralEvading",
                    5.0 if maneuver.cause == "emergency" else 8.0)
            maneuver.phase = "ControlledStopping"
            return self._decision(maneuver, vehicle, "ControlledStopping", 0.0)

        if maneuver.phase == "LateralEvading":
            reached = (vehicle.current_lane == maneuver.target_lane
                       or self._near_path_end(vehicle, maneuver.path))
            if reached:
                if maneuver.cause == "emergency":
                    maneuver.phase = "Yielding"
                    maneuver.path = []
                    return self._decision(maneuver, vehicle, "Yielding", 0.5)
                if self._source_is_behind(vehicle, world, maneuver.source_id):
                    maneuver.phase = "RejoinPlanning"
                    maneuver.path = []
                    return self._decision(
                        maneuver, vehicle, "RejoinPlanning", 4.0)
            return self._decision(
                maneuver, vehicle, "LateralEvading",
                5.0 if maneuver.cause == "emergency" else 8.0)

        if maneuver.phase == "Yielding":
            source = world.objects.get(maneuver.source_id)
            passed = (source is None
                      or self._longitudinal(vehicle, source.position) >= 18.0)
            if passed:
                if maneuver.clear_since is None:
                    maneuver.clear_since = world.time
                elif world.time - maneuver.clear_since >= 1.0:
                    maneuver.phase = "RejoinPlanning"
                    return self._decision(
                        maneuver, vehicle, "RejoinPlanning", 3.0)
            else:
                maneuver.clear_since = None
            return self._decision(maneuver, vehicle, "Yielding", 0.5)

        if maneuver.phase == "RejoinPlanning":
            if self._plan(vehicle, world, maneuver, maneuver.original_lane,
                          exclude_source=True):
                maneuver.phase = "LaneRejoining"
                return self._decision(maneuver, vehicle, "LaneRejoining", 7.0)
            maneuver.phase = "ControlledStopping"
            return self._decision(maneuver, vehicle, "ControlledStopping", 0.0)

        if maneuver.phase == "LaneRejoining":
            if (vehicle.current_lane == maneuver.original_lane
                    or self._near_path_end(vehicle, maneuver.path)):
                self._active.pop(vehicle.id, None)
                return None
            return self._decision(maneuver, vehicle, "LaneRejoining", 7.0)

        return None

    def _trigger(self, vehicle: DynamicVehicle, world: WorldModel):
        ev = emergency.approaching_emergency(
            vehicle, world.objects.values(), radius=70.0)
        if ev is not None:
            target = self._rightmost_escape_lane(
                world.network.lane(vehicle.current_lane), world)
            if target and target != vehicle.current_lane:
                return "emergency", ev, target

        obstacle = self._nearest_obstacle_ahead(vehicle, world)
        if obstacle is not None:
            target = self._adjacent_escape_lane(vehicle, world)
            if target:
                return "obstacle", obstacle, target
        return None

    def _nearest_obstacle_ahead(self, vehicle: DynamicVehicle,
                                world: WorldModel) -> DynamicObject | None:
        best, best_long = None, math.inf
        for obj in world.objects.values():
            if obj.type not in HAZARD_TYPES:
                continue
            longitudinal = self._longitudinal(vehicle, obj.position)
            lateral = abs(self._lateral(vehicle, obj.position))
            if 5.0 <= longitudinal <= 65.0 and lateral <= 4.0:
                if longitudinal < best_long:
                    best, best_long = obj, longitudinal
        return best

    @staticmethod
    def _adjacent_escape_lane(vehicle: DynamicVehicle,
                              world: WorldModel) -> str | None:
        lane = world.network.lane(vehicle.current_lane)
        if lane is None:
            return None
        # Prefer the right side/shoulder, then the left travel lane.
        return lane.right_lane_id or lane.left_lane_id

    @staticmethod
    def _rightmost_escape_lane(lane: Lane | None,
                               world: WorldModel) -> str | None:
        if lane is None:
            return None
        current = lane
        visited = {current.id}
        while current.right_lane_id and current.right_lane_id not in visited:
            nxt = world.network.lane(current.right_lane_id)
            if nxt is None:
                break
            current = nxt
            visited.add(current.id)
        return current.id

    def _plan(self, vehicle: DynamicVehicle, world: WorldModel,
              maneuver: _Maneuver, target_lane_id: str,
              exclude_source: bool = False) -> bool:
        target_lane = world.network.lane(target_lane_id)
        if target_lane is None:
            return False
        _, _, arc = target_lane.closest_point(vehicle.position)
        goal = self._point_at_arc(target_lane, arc + 42.0)
        excluded = {vehicle.id}
        if maneuver.cause == "emergency" or exclude_source:
            excluded.add(maneuver.source_id)
        allowed = [lane_id for lane_id in world.network.all_lane_ids()
                   if lane_id.startswith("ea_") or lane_id.startswith("city_")]
        if not allowed:
            allowed = world.network.all_lane_ids()
        search_world = AvoidanceWorld(
            world.network, world.vehicles.values(), world.objects.values(),
            allowed_lane_ids=allowed, exclude_ids=excluded)

        planner_name = self._planner_name(world)
        config = RRTConfig(
            step_size=3.0, goal_sample_rate=0.22,
            max_iters=1800 if planner_name == "rrt" else 1200,
            goal_radius=3.5, edge_resolution=0.5, margin=12.0,
            seed=(world.tick + sum(ord(c) for c in vehicle.id)) % 100000,
            max_time_ms=45.0 if planner_name == "rrt" else 140.0)
        planner = (RRTStarPlanner(config, rewire_radius=8.0)
                   if planner_name == "rrt_star" else RRTPlanner(config))
        started = time.perf_counter()
        raw = planner.plan(vehicle.position, goal, search_world)
        elapsed = (time.perf_counter() - started) * 1000.0
        path = path_postprocess.prepare(raw, search_world)

        maneuver.last_plan_time = world.time
        maneuver.planner = planner_name
        maneuver.planning_time_ms = elapsed
        if not path:
            maneuver.path = []
            maneuver.minimum_clearance = 0.0
            return False
        maneuver.path = path
        maneuver.minimum_clearance = search_world.minimum_clearance(path)
        maneuver.target_lane = target_lane_id
        return True

    @staticmethod
    def _planner_name(world: WorldModel) -> str:
        return "rrt_star" if world.planner_mode == "rrt_star" else "rrt"

    @staticmethod
    def _point_at_arc(lane: Lane, arc: float) -> Vec3:
        remaining = max(0.0, min(arc, lane.length))
        for a, b in zip(lane.centerline, lane.centerline[1:]):
            length = dist_xz(a, b)
            if length > 0.0 and remaining <= length:
                t = remaining / length
                return [a[j] + (b[j] - a[j]) * t for j in range(3)]
            remaining -= length
        return list(lane.end)

    @staticmethod
    def _near_path_end(vehicle: DynamicVehicle, path: Path) -> bool:
        return bool(path) and dist_xz(vehicle.position, path[-1]) <= 4.0

    def _source_is_behind(self, vehicle: DynamicVehicle, world: WorldModel,
                          source_id: str) -> bool:
        source = world.objects.get(source_id)
        return source is None or self._longitudinal(vehicle, source.position) < -6.0

    @staticmethod
    def _longitudinal(vehicle: DynamicVehicle, point: Vec3) -> float:
        rad = math.radians(vehicle.heading)
        fx, fz = math.sin(rad), math.cos(rad)
        return ((point[0] - vehicle.position[0]) * fx
                + (point[2] - vehicle.position[2]) * fz)

    @staticmethod
    def _lateral(vehicle: DynamicVehicle, point: Vec3) -> float:
        rad = math.radians(vehicle.heading)
        rx, rz = math.cos(rad), -math.sin(rad)
        return ((point[0] - vehicle.position[0]) * rx
                + (point[2] - vehicle.position[2]) * rz)

    @staticmethod
    def _decision(maneuver: _Maneuver, vehicle: DynamicVehicle,
                  behavior: str, speed: float) -> AvoidanceDecision:
        signal = "hazard" if behavior in {
            "HazardDetected", "EscapePlanning", "Yielding",
            "ControlledStopping"} else "right"
        current_lane = vehicle.current_lane
        if (behavior == "LaneRejoining"
                and maneuver.original_lane != current_lane):
            signal = "left"
        return AvoidanceDecision(
            behavior=behavior, path=[list(p) for p in maneuver.path],
            target_lane=(maneuver.original_lane if behavior == "LaneRejoining"
                         else maneuver.target_lane),
            target_speed=speed, turn_signal=signal,
            planner=maneuver.planner,
            plan_status=("failed" if behavior == "ControlledStopping"
                         else "active"),
            planning_time_ms=round(maneuver.planning_time_ms, 3),
            minimum_clearance=(round(maneuver.minimum_clearance, 3)
                               if math.isfinite(maneuver.minimum_clearance)
                               else 999.0))
