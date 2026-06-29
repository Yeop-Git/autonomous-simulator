"""Central control policy — turns a StateMessage into a CommandMessage.

The brain of the simulation, kept free of any transport concern so it can be
unit-tested directly (feed it a dict, assert on the dict it returns).
``main.py`` wraps it with WebSocket I/O and sync checks.

Pipeline per tick (plan §2.1):
    ingest state -> collision prediction -> per-vehicle routing
                 -> behavior FSM -> longitudinal (ACC) command

Lateral control (Pure Pursuit / Stanley / PID) runs Unity-side off the
returned ``path``; the server commands speed + behavior + path.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import emergency
from behavior import (ARRIVED, EMERGENCY_BRAKING, STOPPING, BehaviorInputs,
                      Leader, find_leader, next_behavior)
from collision_predictor import CollisionPredictor
from controllers.acc import ACCController
from planners import AStarPlanner
from planners.base import Path, Vec3
from world_model import LaneNetwork, WorldModel, dist_xz

ARRIVAL_RADIUS = 3.0   # m; within this of the goal the vehicle is "Arrived"
REPLAN_GOAL_EPS = 1.0  # m; goal moved more than this => replan
OFFROUTE_EPS = 3.0     # m; vehicle this far off its cached path => replan
                       # (below a lane width, so a completed lane change reroutes)


@dataclass
class _RouteCache:
    goal: Vec3
    path: Path


def _project_path(path: Path, pos: Vec3) -> tuple[float, float, float]:
    """Project ``pos`` onto a path polyline (xz).

    Returns ``(arc_at_projection, lateral_distance, total_length)``.
    """
    best_arc, best_lat, acc = 0.0, math.inf, 0.0
    total = 0.0
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        dx, dz = b[0] - a[0], b[2] - a[2]
        seg_sq = dx * dx + dz * dz
        seg = math.sqrt(seg_sq)
        if seg_sq > 0.0:
            t = ((pos[0] - a[0]) * dx + (pos[2] - a[2]) * dz) / seg_sq
            t = max(0.0, min(1.0, t))
            px, pz = a[0] + t * dx, a[2] + t * dz
            lat = math.hypot(pos[0] - px, pos[2] - pz)
            if lat < best_lat:
                best_lat, best_arc = lat, total + t * seg
        total += seg
    return best_arc, best_lat, total


class CentralController:
    def __init__(self, network: LaneNetwork, planner=None,
                 predictor: CollisionPredictor | None = None,
                 acc: ACCController | None = None,
                 noise=None, dt: float = 0.1):
        self.world = WorldModel(network)
        self.planner = planner or AStarPlanner()
        self.predictor = predictor or CollisionPredictor()
        self.acc = acc or ACCController()
        self.noise = noise  # optional NoiseModel applied to incoming state
        self.dt = dt
        self._routes: dict[str, _RouteCache] = {}
        # stats for logging / experiments
        self.replans = 0
        self.last_conflicts = []

    # ------------------------------------------------------------------ #
    def step(self, state: dict) -> dict:
        """Ingest one StateMessage dict, return one CommandMessage dict."""
        if self.noise is not None:
            state = self.noise.apply(state)
        self.world.update_from_state(state)

        # Central global situation awareness: predict conflicts across every
        # dynamic entity — vehicles AND objects (pedestrians, obstacles, ...).
        movers = list(self.world.vehicles.values()) + list(self.world.objects.values())
        self.last_conflicts = self.predictor.conflicts(movers)

        commands = [self._command_for(vid) for vid in self.world.vehicles]
        return {
            "time": self.world.time,
            "tick": self.world.tick,
            "commands": commands,
        }

    # ------------------------------------------------------------------ #
    def _command_for(self, vehicle_id: str) -> dict:
        v = self.world.vehicles[vehicle_id]
        others = [o for o in self.world.vehicles.values() if o.id != vehicle_id]
        leader = find_leader(v, others, self.world.network)

        # The leader's gap is managed by ACC (longitudinal). The TTC that
        # escalates the FSM to Stopping/EmergencyBraking should reflect only
        # *other* conflicts — cross traffic, pedestrians, obstacles — not the
        # car we're already following.
        leader_id = leader.vehicle.id if leader else None
        min_ttc = self._min_ttc_for(vehicle_id, exclude_id=leader_id)
        min_ttc_emergency = self._min_ttc_for(vehicle_id)  # includes leader

        cmd = {
            "vehicle_id": vehicle_id,
            "target_speed": 0.0,
            "target_lane": v.current_lane or None,
            "behavior": "LaneKeeping",
            "lka_enabled": True,
        }

        # --- routing -------------------------------------------------- #
        # Use distance ALONG the cached path (not straight-line, which can
        # under-brake when the route curves around an obstacle).
        remaining = self._remaining_to_goal(vehicle_id, v)
        arrived = bool(v.has_goal and v.goal is not None
                       and remaining <= ARRIVAL_RADIUS)
        path: Path = []
        route_found = True
        if v.has_goal and v.goal is not None and not arrived:
            path = self._route(vehicle_id, v.position, v.goal)
            route_found = bool(path)
            if path:
                cmd["path"] = path
            remaining = self._remaining_to_goal(vehicle_id, v)  # along new path
        elif arrived:
            self._routes.pop(vehicle_id, None)

        # --- behavior FSM --------------------------------------------- #
        behavior = next_behavior(BehaviorInputs(
            has_goal=v.has_goal,
            arrived=arrived,
            route_found=route_found,
            leader=leader,
            min_ttc=min_ttc,
            min_ttc_emergency=min_ttc_emergency,
        ))
        cmd["behavior"] = behavior

        # --- longitudinal command (ACC) ------------------------------- #
        free_speed = self._lane_speed(v.current_lane)
        speed = self._target_speed(behavior, v, leader, free_speed, remaining)

        # --- emergency-vehicle yielding (V2X priority) ---------------- #
        yld = emergency.yield_speed(v, self.world.objects.values())
        if yld is not None and behavior not in (ARRIVED,):
            speed = min(speed, yld)
            if behavior == "LaneKeeping":
                cmd["behavior"] = "Stopping"  # visibly yielding
        cmd["target_speed"] = round(speed, 3)
        return cmd

    # ------------------------------------------------------------------ #
    def _target_speed(self, behavior: str, v, leader: Optional[Leader],
                      free_speed: float, remaining: float) -> float:
        if behavior in (ARRIVED, EMERGENCY_BRAKING, STOPPING):
            return 0.0
        leader_gap = leader.gap if leader else None
        leader_speed = leader.speed if leader else None
        speed = self.acc.target_speed(
            ego_speed=v.speed, free_speed=free_speed,
            leader_gap=leader_gap, leader_speed=leader_speed, dt=self.dt)
        # Ease to a stop at the goal: treat it as a stationary point so the
        # vehicle decelerates smoothly instead of halting abruptly on arrival
        # (an abrupt halt would defeat any follower's safe-following margin).
        if v.has_goal and v.goal is not None and math.isfinite(remaining):
            goal_speed = math.sqrt(max(0.0,
                2.0 * self.acc.p.max_decel * (remaining - 1.0)))
            speed = min(speed, goal_speed)
        return round(max(0.0, speed), 3)

    def _remaining_to_goal(self, vehicle_id: str, v) -> float:
        """Distance left to the goal along the cached path; straight-line if
        no path is cached yet."""
        cached = self._routes.get(vehicle_id)
        if cached and cached.path and len(cached.path) >= 2:
            arc, _, total = _project_path(cached.path, v.position)
            return max(0.0, total - arc)
        return dist_xz(v.position, v.goal) if v.goal is not None else math.inf

    def _min_ttc_for(self, vehicle_id: str, exclude_id: str | None = None) -> float:
        """Soonest conflict TTC involving ``vehicle_id``, ignoring conflicts
        with ``exclude_id`` (typically the ACC-managed leader)."""
        best = math.inf
        for c in self.last_conflicts:
            if vehicle_id not in (c.a_id, c.b_id):
                continue
            other = c.b_id if c.a_id == vehicle_id else c.a_id
            if exclude_id is not None and other == exclude_id:
                continue
            best = min(best, c.ttc)
        return best

    def _route(self, vehicle_id: str, start: Vec3, goal: Vec3) -> Path:
        cached = self._routes.get(vehicle_id)
        if cached and dist_xz(cached.goal, goal) <= REPLAN_GOAL_EPS and cached.path:
            # reuse only while the vehicle is still on its route; if it has
            # been pushed off (lane change, shove), replan from where it is.
            _, lateral, _ = _project_path(cached.path, start)
            if lateral <= OFFROUTE_EPS:
                return cached.path
        path = self.planner.plan(start, goal, self.world.network)
        if path:
            self._routes[vehicle_id] = _RouteCache(goal=list(goal), path=path)
            self.replans += 1
        return path

    def _lane_speed(self, lane_id: Optional[str]) -> float:
        lane = self.world.network.lane(lane_id) if lane_id else None
        return lane.speed_limit if lane else 0.0
