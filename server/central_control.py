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
import lane_change
import merge
from traffic import TrafficLight, TrafficLightManager
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
        self._merge_speed_overrides: dict[str, float] = {}
        self._right_turn_stopped: set[str] = set()
        self.traffic = TrafficLightManager()
        if network.scenario == "urban":
            # Legacy two-lane ids remain supported for synthetic tests.
            self.traffic.add(TrafficLight(
                "urban_north", stop_line=[-1.8, 0.0, -10.0],
                approach_heading=0.0, green_time=12.0,
                yellow_time=3.0, red_time=31.0))
            self.traffic.add(TrafficLight(
                "urban_east", stop_line=[-10.0, 0.0, 1.8],
                approach_heading=90.0, green_time=12.0,
                yellow_time=3.0, red_time=31.0, offset=23.0))
            self.traffic.add(TrafficLight(
                "urban_nb_0_in", stop_line=[5.4, 0.0, -16.0],
                approach_heading=0.0, green_time=10.0,
                yellow_time=3.0, red_time=41.0, offset=33.0))
            self.traffic.add(TrafficLight(
                "urban_nb_1_in", stop_line=[1.8, 0.0, -16.0],
                approach_heading=0.0, green_time=6.0,
                yellow_time=2.0, red_time=46.0, offset=19.0))
            for lane_id, x in (("urban_sb_0_in", -5.4), ("urban_sb_1_in", -1.8)):
                self.traffic.add(TrafficLight(
                    lane_id, stop_line=[x, 0.0, 16.0],
                    approach_heading=180.0, green_time=10.0,
                    yellow_time=3.0, red_time=41.0, offset=33.0))
            for lane_id, z in (("urban_eb_0_in", -5.4), ("urban_eb_1_in", -1.8)):
                self.traffic.add(TrafficLight(
                    lane_id, stop_line=[-16.0, 0.0, z],
                    approach_heading=90.0, green_time=10.0,
                    yellow_time=3.0, red_time=41.0, offset=0.0))
            for lane_id, z in (("urban_wb_0_in", 5.4), ("urban_wb_1_in", 1.8)):
                self.traffic.add(TrafficLight(
                    lane_id, stop_line=[16.0, 0.0, z],
                    approach_heading=270.0, green_time=10.0,
                    yellow_time=3.0, red_time=41.0, offset=0.0))

    # ------------------------------------------------------------------ #
    def step(self, state: dict) -> dict:
        """Ingest one StateMessage dict, return one CommandMessage dict."""
        if self.noise is not None:
            state = self.noise.apply(state)
        self.world.update_from_state(state)
        self._update_merge_reservations()

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

        # --- requested V2X lane change -------------------------------- #
        # Unity only expresses intent.  The centralized world model checks
        # adjacency, front/rear gaps, and predicted conflicts before issuing
        # a lateral path.
        lane_change_accepted = False
        if v.target_lane and v.target_lane != v.current_lane:
            current = self.world.network.lane(v.current_lane)
            adjacent = current is not None and v.target_lane in (
                current.left_lane_id, current.right_lane_id)
            if adjacent:
                decision = lane_change.evaluate(
                    v, v.target_lane, others, self.world.network, self.predictor)
                if decision.accept:
                    change_path = self._lane_change_path(v, v.target_lane)
                    if change_path:
                        path = change_path
                        cmd["path"] = path
                        cmd["target_lane"] = v.target_lane
                        lane_change_accepted = True
                        if v.goal is not None:
                            self._routes[vehicle_id] = _RouteCache(
                                goal=list(v.goal), path=path)
                        if behavior not in (ARRIVED, EMERGENCY_BRAKING):
                            behavior = "LaneChanging"
                            cmd["behavior"] = behavior

        # --- longitudinal command (ACC) ------------------------------- #
        free_speed = self._lane_speed(v.current_lane)
        speed = self._target_speed(behavior, v, leader, free_speed, remaining)
        if vehicle_id in self._merge_speed_overrides:
            reserved_speed = self._merge_speed_overrides[vehicle_id]
            if reserved_speed < speed:
                speed = max(0.0, reserved_speed)
                if cmd["behavior"] == "LaneKeeping":
                    cmd["behavior"] = "Following"

        # --- signalized urban intersection (same simulation time as Unity) #
        if self._must_wait_for_signal(v, min_ttc) and behavior != ARRIVED:
            # Approach the painted stop line under comfortable braking instead
            # of commanding zero as soon as the 40 m detection zone begins.
            # The 1.5 m buffer keeps the vehicle nose behind the line.
            speed = min(speed, self._signal_approach_speed(v))
            cmd["behavior"] = "WaitingAtIntersection"

        # --- emergency-vehicle yielding (V2X priority) ---------------- #
        yld = emergency.yield_speed(v, self.world.objects.values())
        if yld is not None and behavior not in (ARRIVED,):
            speed = min(speed, yld)
            if behavior == "LaneKeeping":
                cmd["behavior"] = "Stopping"  # visibly yielding
        cmd["target_speed"] = round(speed, 3)
        return cmd

    def _must_wait_for_signal(self, v, min_ttc: float) -> bool:
        if v.current_lane not in self.traffic.lights:
            return False
        regular_stop = self.traffic.should_stop(
            v.position, v.speed, v.current_lane, self.world.time)

        if v.maneuver == "left":
            # The scene's protected left lane has its own arrow phase.
            if v.current_lane != "urban_nb_1_in":
                return regular_stop
            return regular_stop

        if v.maneuver != "right":
            return regular_stop

        rightmost = {
            "urban_nb_0_in", "urban_sb_0_in",
            "urban_eb_0_in", "urban_wb_0_in",
        }
        if v.current_lane not in rightmost:
            return regular_stop
        if not regular_stop:
            self._right_turn_stopped.discard(v.id)
            return False

        # Korean right-turn rule: on a red vehicle signal, first make a full
        # stop.  Afterwards proceed only while yielding to pedestrians and
        # traffic already moving through the intersection.
        if v.id not in self._right_turn_stopped:
            if v.speed <= 0.2:
                self._right_turn_stopped.add(v.id)
            return True
        pedestrian_present = any(
            o.type == "pedestrian" for o in self.world.objects.values())
        pedestrian_phase = self._is_pedestrian_phase(self.world.time)
        return pedestrian_present or pedestrian_phase or min_ttc <= 3.0

    def _signal_approach_speed(self, v) -> float:
        light = self.traffic.lights.get(v.current_lane)
        if light is None:
            return 0.0
        rad = math.radians(light.approach_heading)
        forward_x, forward_z = math.sin(rad), math.cos(rad)
        dx = light.stop_line[0] - v.position[0]
        dz = light.stop_line[2] - v.position[2]
        distance = dx * forward_x + dz * forward_z
        braking_distance = max(0.0, distance - 1.5)
        return math.sqrt(2.0 * 3.0 * braking_distance)

    @staticmethod
    def _is_pedestrian_phase(time: float) -> bool:
        phase = time % 54.0
        return 13.0 <= phase < 21.0 or 43.0 <= phase < 51.0

    def _update_merge_reservations(self) -> None:
        """Reserve a V2X time slot for every active on-ramp vehicle."""
        self._merge_speed_overrides = {}
        ramps = [v for v in self.world.vehicles.values()
                 if "ramp" in (v.current_lane or "").lower()]
        for ramp_vehicle in ramps:
            ramp_lane = self.world.network.lane(ramp_vehicle.current_lane)
            if ramp_lane is None:
                continue
            merge_point = list(ramp_lane.end)
            successor_ids = set(ramp_lane.next_lane_ids)
            mainline = [v for v in self.world.vehicles.values()
                        if v.id != ramp_vehicle.id
                        and (v.current_lane in successor_ids
                             or v.current_lane == "hw_l2")]
            plan = merge.plan_merge(ramp_vehicle, mainline, merge_point)
            self._merge_speed_overrides[ramp_vehicle.id] = max(
                0.0, plan.ramp_target_speed)
            if plan.yield_vehicle and plan.yield_target_speed is not None:
                self._merge_speed_overrides[plan.yield_vehicle] = max(
                    0.0, plan.yield_target_speed)

    def _lane_change_path(self, v, target_lane_id: str,
                          distance: float = 32.0) -> Path:
        """Smoothly blend from the current lane to an adjacent centerline."""
        current = self.world.network.lane(v.current_lane)
        target = self.world.network.lane(target_lane_id)
        if current is None or target is None:
            return []
        _, _, current_arc = current.closest_point(v.position)
        _, _, target_arc = target.closest_point(v.position)
        path: Path = [list(v.position)]
        for i in range(1, 5):
            f = i / 4.0
            a = self._point_at_arc(current.centerline, current_arc + distance * f)
            b = self._point_at_arc(target.centerline, target_arc + distance * f)
            path.append([
                a[0] + (b[0] - a[0]) * f,
                a[1] + (b[1] - a[1]) * f,
                a[2] + (b[2] - a[2]) * f,
            ])
        merge_arc = target_arc + distance
        for point in target.centerline:
            _, _, arc = target.closest_point(point)
            if arc > merge_arc + 1.0:
                path.append(list(point))
        if v.goal is not None and dist_xz(path[-1], v.goal) > 1.0:
            path.append(list(v.goal))
        return path

    @staticmethod
    def _point_at_arc(centerline: Path, arc: float) -> Vec3:
        if not centerline:
            return [0.0, 0.0, 0.0]
        remaining = max(0.0, arc)
        for i in range(len(centerline) - 1):
            a, b = centerline[i], centerline[i + 1]
            length = dist_xz(a, b)
            if length > 0.0 and remaining <= length:
                t = remaining / length
                return [a[j] + (b[j] - a[j]) * t for j in range(3)]
            remaining -= length
        return list(centerline[-1])

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
            if self._traffic_conflict_is_managed(vehicle_id, other):
                continue
            best = min(best, c.ttc)
        return best

    def _traffic_conflict_is_managed(self, ego_id: str, other_id: str) -> bool:
        """A green approach need not emergency-brake for a red approach that
        is still upstream of its enforced stop line."""
        ego = self.world.vehicles.get(ego_id)
        other = self.world.vehicles.get(other_id)
        if ego is None or other is None:
            return False
        ego_in_intersection = any(token in (ego.current_lane or "")
                                  for token in ("_straight", "_left", "_right"))
        if ego.current_lane not in self.traffic.lights and not ego_in_intersection:
            return False
        if other.current_lane not in self.traffic.lights:
            return False
        ego_stops = False if ego_in_intersection else self.traffic.should_stop(
            ego.position, ego.speed, ego.current_lane, self.world.time)
        other_stops = self.traffic.should_stop(
            other.position, other.speed, other.current_lane, self.world.time)
        return other_stops and not ego_stops

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
