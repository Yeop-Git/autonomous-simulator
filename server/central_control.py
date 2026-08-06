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
import left_turn
from local_avoidance import LocalAvoidanceManager
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
STOP_LINE_BUFFER = 5.5  # m from vehicle center to painted stop line
SIGNAL_APPROACH_DECEL = 1.8  # m/s^2; early, comfortable signal braking
LANE_CHANGE_MIN_LENGTH = 12.0
LEFT_LANE_CHANGE_READY_DISTANCE = 14.0  # before the painted stop line
LANE_CHANGE_STOP_MARGIN = 2.0
LEFT_TURN_ABORT_EPS = 1.0
LEFT_TURN_PEDESTRIAN_CORRIDOR = 3.0
LEFT_TURN_CONFLICT_HORIZON = 35.0
LEFT_TURN_ACCEPT_TIME_GAP = 1.25
LANE_CHANGE_LEAD_PREVIEW = 2.0
MERGE_UPSTREAM_HORIZON = 250.0  # m of mainline to watch upstream of a join
MERGE_JOIN_EPS = 1.0            # m; beyond this a join is mid-lane, not end-to-start
MERGE_PARALLEL_PROBE = 15.0     # m back from a shared junction to compare approaches
MERGE_PARALLEL_SPAN = 8.0       # m; wider apart there and it is not one stream
MERGE_PARALLEL_ANGLE = 30.0     # deg of heading disagreement allowed there
URBAN_SIGNAL_PERIOD = 60.0
PEDESTRIAN_PHASES = ((13.0, 21.0), (47.0, 55.0))


@dataclass
class _RouteCache:
    goal: Vec3
    path: Path


@dataclass(frozen=True)
class _LeftTurnContext:
    source_lane: str
    target_lane: str
    connector_lane: str
    exit_lane: str


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
        self.left_turn_diagnostics: dict[str, dict] = {}
        self._merge_speed_overrides: dict[str, float] = {}
        self._merging_lanes: set[str] | None = None
        self._right_turn_stopped: set[str] = set()
        self._left_turn_commitments: dict[str, left_turn.LeftTurnCommitment] = {}
        self._left_turn_contexts: dict[str, _LeftTurnContext] = {}
        self.local_avoidance = LocalAvoidanceManager()
        self.traffic = TrafficLightManager()
        if network.scenario in ("urban", "integrated_city"):
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
                yellow_time=3.0, red_time=47.0, offset=39.0))
            self.traffic.add(TrafficLight(
                "urban_nb_1_in", stop_line=[1.8, 0.0, -16.0],
                approach_heading=0.0, green_time=6.0,
                yellow_time=2.0, red_time=52.0, offset=24.0))
            for lane_id, x in (("urban_sb_0_in", -5.4), ("urban_sb_1_in", -1.8)):
                self.traffic.add(TrafficLight(
                    lane_id, stop_line=[x, 0.0, 16.0],
                    approach_heading=180.0, green_time=10.0,
                    yellow_time=3.0, red_time=47.0, offset=39.0))
            for lane_id, z in (("urban_eb_0_in", -5.4), ("urban_eb_1_in", -1.8)):
                self.traffic.add(TrafficLight(
                    lane_id, stop_line=[-16.0, 0.0, z],
                    approach_heading=90.0, green_time=10.0,
                    yellow_time=3.0, red_time=47.0, offset=0.0))
            for lane_id, z in (("urban_wb_0_in", 5.4), ("urban_wb_1_in", 1.8)):
                self.traffic.add(TrafficLight(
                    lane_id, stop_line=[16.0, 0.0, z],
                    approach_heading=270.0, green_time=10.0,
                    yellow_time=3.0, red_time=47.0, offset=0.0))

    # ------------------------------------------------------------------ #
    def step(self, state: dict) -> dict:
        """Ingest one StateMessage dict, return one CommandMessage dict."""
        if self.noise is not None:
            state = self.noise.apply(state)
        incoming_tick = state.get("tick")
        if (incoming_tick is not None and self.world.tick >= 0
                and incoming_tick < self.world.tick):
            self._forget_vehicle_state()
        self.world.update_from_state(state)
        self.left_turn_diagnostics = {}
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

    def _forget_vehicle_state(self) -> None:
        """Drop everything remembered about individual vehicles.

        A tick that goes backwards means Unity restarted its clock — the
        avoidance scene's reset key reloads the scene, which rebuilds the
        V2XClient and starts its counter over. Every per-vehicle memory here
        then describes a world that no longer exists: the ego is back at the
        start line with the hazard gone while this side still believes it is
        mid-evasion, and hands it an escape path beginning 20 m ahead of where
        it now stands. The lane network is static and stays.
        """
        self._routes.clear()
        self._merge_speed_overrides.clear()
        self._right_turn_stopped.clear()
        self._left_turn_commitments.clear()
        self._left_turn_contexts.clear()
        self.local_avoidance.forget_all()

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
            "turn_signal": "none",
        }
        if v.maneuver != "left":
            self._left_turn_commitments.pop(vehicle_id, None)
            self._left_turn_contexts.pop(vehicle_id, None)

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

        # --- per-tick manoeuvre arbitration ---------------------------- #
        # Left turn uses the same priority-style evaluation as the general
        # behaviour layer.  Its phase is an output/path guard, not a state
        # that blindly dictates the next tick.
        lane_change_accepted = False
        lane_change_pending = False
        target_lane_leader: Optional[Leader] = None
        left_context = (self._get_left_turn_context(vehicle_id, v)
                        if v.maneuver == "left" else None)
        managed_left = left_context is not None
        left_decision: Optional[left_turn.LeftTurnDecision] = None
        left_turn_aborted = False
        left_signal_stop = False
        left_entry_clear = True

        current = self.world.network.lane(v.current_lane)
        adjacent = bool(current is not None and v.target_lane and
                        v.target_lane in (current.left_lane_id,
                                          current.right_lane_id))
        geometric_distance = (self._lane_change_distance(v, v.target_lane)
                              if adjacent else None)
        gap_decision = None
        change_distance = None
        if adjacent and geometric_distance is not None:
            gap_decision = lane_change.evaluate(
                v, v.target_lane, others, self.world.network, self.predictor,
                time_gap=(LEFT_TURN_ACCEPT_TIME_GAP
                          if managed_left else lane_change.TIME_GAP))
            target_vehicle = None
            if gap_decision.lead_id:
                target_vehicle = next(
                    (o for o in others if o.id == gap_decision.lead_id), None)
                if target_vehicle is not None:
                    target_lane_leader = Leader(
                        vehicle=target_vehicle,
                        gap=max(0.0, gap_decision.lead_gap),
                        speed=target_vehicle.speed,
                    )
            change_distance = self._queue_safe_lane_change_distance(
                geometric_distance, gap_decision,
                target_vehicle.speed if target_vehicle is not None else 0.0)

        if managed_left:
            memory = self._left_turn_commitments.get(
                vehicle_id, left_turn.LeftTurnCommitment())
            left_signal_stop = self._must_wait_for_signal(
                v, min_ttc, left_context.target_lane)
            left_entry_clear = self._left_turn_entry_clear(
                v, leader, left_context)
            deadline = (v.current_lane == left_context.source_lane
                        and v.target_lane == left_context.target_lane
                        and (geometric_distance is None
                             or self._left_turn_change_deadline_reached(
                                 v, left_context.target_lane)))
            inputs = left_turn.LeftTurnInputs(
                location=self._left_turn_location(v, left_context),
                emergency=behavior == EMERGENCY_BRAKING,
                lane_change_safe=bool(
                    gap_decision and gap_decision.accept
                    and change_distance is not None),
                lane_change_deadline_reached=deadline,
                signal_requires_stop=left_signal_stop,
                entry_clear=left_entry_clear,
                at_entry_gate=self._left_turn_at_entry_gate(
                    v, left_context.target_lane),
                exit_aligned=self._left_turn_exit_aligned(
                    v, left_context.exit_lane),
            )
            left_decision, next_memory = left_turn.evaluate(inputs, memory)
            self.left_turn_diagnostics[vehicle_id] = {
                "location": inputs.location,
                "emergency": inputs.emergency,
                "signal_requires_stop": inputs.signal_requires_stop,
                "entry_clear": inputs.entry_clear,
                "at_entry_gate": inputs.at_entry_gate,
                "leader_id": leader.vehicle.id if leader else None,
                "leader_gap": leader.gap if leader else None,
                "leader_speed": leader.speed if leader else None,
            }
            if next_memory == left_turn.LeftTurnCommitment():
                self._left_turn_commitments.pop(vehicle_id, None)
            else:
                self._left_turn_commitments[vehicle_id] = next_memory
            cmd["left_turn_phase"] = left_decision.phase
            cmd["turn_signal"] = left_decision.turn_signal

            if left_decision.action in (
                    left_turn.START_LANE_CHANGE,
                    left_turn.CONTINUE_LANE_CHANGE):
                # A committed change retains its lateral direction even when a
                # new hazard invalidates the gap; longitudinal priority then
                # commands a stop.
                effective_distance = change_distance or geometric_distance
                if effective_distance is not None and v.target_lane:
                    change_path = self._lane_change_path(
                        v, v.target_lane, effective_distance,
                        include_route_tail=False)
                    if change_path:
                        path = change_path
                        cmd["path"] = path
                        cmd["target_lane"] = v.target_lane
                        lane_change_accepted = True
                        if behavior not in (ARRIVED, EMERGENCY_BRAKING):
                            behavior = "LaneChanging"
                            cmd["behavior"] = behavior
            elif left_decision.action == left_turn.WAIT_LANE_CHANGE:
                lane_change_pending = bool(v.target_lane)
                if behavior not in (ARRIVED, EMERGENCY_BRAKING):
                    behavior = "Following" if target_lane_leader else "LaneKeeping"
                    cmd["behavior"] = behavior
                if v.target_lane:
                    wait_path = self._lane_change_wait_path(
                        v, left_context.target_lane)
                    if wait_path:
                        path = wait_path
                        cmd["path"] = path
            elif left_decision.action == left_turn.ABORT_STRAIGHT:
                left_turn_aborted = True
                abort_path = self._aborted_left_path(v, left_context)
                if abort_path:
                    path = abort_path
                    cmd["path"] = path
                cmd["target_lane"] = v.current_lane or None
                cmd["behavior"] = "LeftTurnAborted"
                behavior = "LaneKeeping"

        elif v.target_lane and v.target_lane != v.current_lane:
            # Non-left manoeuvres retain the generic per-tick gap arbitration.
            if (gap_decision is not None and gap_decision.accept
                    and change_distance is not None):
                change_path = self._lane_change_path(
                    v, v.target_lane, change_distance)
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
        if target_lane_leader is not None:
            target_lane_speed = self.acc.target_speed(
                ego_speed=v.speed,
                free_speed=min(free_speed, self._lane_speed(v.target_lane)),
                leader_gap=target_lane_leader.gap,
                leader_speed=target_lane_leader.speed,
                dt=self.dt,
            )
            speed = min(speed, target_lane_speed)
        if lane_change_pending:
            speed = min(speed, self._lane_change_wait_speed(v, v.target_lane))
        if vehicle_id in self._merge_speed_overrides:
            reserved_speed = self._merge_speed_overrides[vehicle_id]
            if reserved_speed < speed:
                speed = max(0.0, reserved_speed)
                if cmd["behavior"] == "LaneKeeping":
                    cmd["behavior"] = "Following"

        # --- signalized intersection (same simulation time as Unity) --- #
        if (managed_left and left_decision is not None
                and left_decision.action != left_turn.ABORT_STRAIGHT):
            if left_signal_stop and behavior != ARRIVED:
                speed = min(speed, self._signal_approach_speed(
                    v, left_context.target_lane))
                if not lane_change_accepted:
                    cmd["behavior"] = "WaitingAtIntersection"
            if left_decision.stop_now:
                speed = 0.0
                if left_decision.action != left_turn.CONTINUE_LANE_CHANGE:
                    cmd["behavior"] = "WaitingAtIntersection"
            if left_decision.phase == "SignalWaiting":
                wait_path = self._left_turn_wait_path(
                    v, left_context.target_lane)
                if wait_path:
                    cmd["path"] = wait_path
        else:
            # General traffic and an aborted left use their current lane's
            # ordinary signal instead of the protected-left arrow.
            signal_lane = v.current_lane
            must_wait_for_signal = self._must_wait_for_signal(
                v, min_ttc, signal_lane,
                ignore_left_maneuver=left_turn_aborted)
            if must_wait_for_signal and behavior != ARRIVED:
                speed = min(speed, self._signal_approach_speed(v, signal_lane))
                if not lane_change_accepted and not left_turn_aborted:
                    cmd["behavior"] = "WaitingAtIntersection"

        # --- emergency-vehicle yielding (V2X priority) ---------------- #
        yld = emergency.yield_speed(v, self.world.objects.values())
        if yld is not None and behavior not in (ARRIVED,):
            speed = min(speed, yld)
            if behavior == "LaneKeeping":
                cmd["behavior"] = "Stopping"  # visibly yielding

        # --- RRT/RRT* local avoidance and active emergency pull-over ---- #
        # Only active in dedicated scenarios.  It overrides the local path
        # after ordinary routing/traffic decisions but never defeats an
        # already-required emergency brake.
        avoidance = self.local_avoidance.update(v, self.world)
        if avoidance is not None:
            if avoidance.path:
                cmd["path"] = avoidance.path
            if avoidance.target_lane:
                cmd["target_lane"] = avoidance.target_lane
            cmd["planner"] = avoidance.planner
            cmd["plan_status"] = avoidance.plan_status
            cmd["planning_time_ms"] = avoidance.planning_time_ms
            cmd["minimum_clearance"] = avoidance.minimum_clearance
            cmd["turn_signal"] = avoidance.turn_signal
            if behavior != EMERGENCY_BRAKING:
                cmd["behavior"] = avoidance.behavior
                speed = avoidance.target_speed
        cmd["target_speed"] = round(speed, 3)
        return cmd

    def _must_wait_for_signal(self, v, min_ttc: float,
                              signal_lane: str | None = None,
                              ignore_left_maneuver: bool = False) -> bool:
        signal_lane = signal_lane or v.current_lane
        if signal_lane not in self.traffic.lights:
            return False
        regular_stop = self.traffic.should_stop(
            v.position, v.speed, signal_lane, self.world.time)

        if v.maneuver == "left" and not ignore_left_maneuver:
            return regular_stop

        if v.maneuver != "right":
            return regular_stop

        # The rule belongs to a lane you can actually turn right *from*. That
        # was a literal list of four Urban lane ids — three of which have no
        # right-turn connector at all, while any other scene that authored one
        # was silently excluded. Ask the lane graph instead.
        if self._turn_successor(v.current_lane, want_left=False) is None:
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

    def _get_left_turn_context(self, vehicle_id: str, v
                               ) -> _LeftTurnContext | None:
        """Resolve route roles from topology, never from scenario lane names."""
        cached = self._left_turn_contexts.get(vehicle_id)
        if cached is not None:
            return cached

        target_id = (v.target_lane if self.world.network.lane(v.target_lane)
                     else v.current_lane)
        target = self.world.network.lane(target_id)
        if target is None:
            return None
        connector_id = self._turn_successor(target_id, want_left=True)
        connector = self.world.network.lane(connector_id)
        if connector is None or not connector.next_lane_ids:
            inferred = self._infer_left_context_from_route_lane(v.current_lane)
            if inferred is not None:
                self._left_turn_contexts[vehicle_id] = inferred
            return inferred
        exit_id = connector.next_lane_ids[0]
        if self.world.network.lane(exit_id) is None:
            return None
        source_id = (v.current_lane if v.current_lane != target_id
                     else (target.right_lane_id or target_id))
        context = _LeftTurnContext(source_id, target_id, connector_id, exit_id)
        self._left_turn_contexts[vehicle_id] = context
        return context

    def _infer_left_context_from_route_lane(
            self, current_lane_id: str) -> _LeftTurnContext | None:
        """Reconstruct roles when a vehicle first appears mid-turn or on exit."""
        for target in self.world.network.lanes.values():
            connector_id = self._turn_successor(target.id, want_left=True)
            connector = self.world.network.lane(connector_id)
            if connector is None or not connector.next_lane_ids:
                continue
            exit_id = connector.next_lane_ids[0]
            if current_lane_id not in (target.id, connector.id, exit_id):
                continue
            source_id = target.right_lane_id or target.id
            return _LeftTurnContext(
                source_id, target.id, connector.id, exit_id)
        return None

    def _turn_successor(self, lane_id: str, want_left: bool) -> str | None:
        lane = self.world.network.lane(lane_id)
        if lane is None:
            return None
        approach = self._lane_heading(lane)
        candidates: list[tuple[float, str]] = []
        for successor_id in lane.next_lane_ids:
            successor = self.world.network.lane(successor_id)
            if successor is None:
                continue
            delta = self._heading_delta(self._lane_heading(successor), approach)
            if (want_left and delta < -15.0) or (not want_left and delta > 15.0):
                candidates.append((abs(abs(delta) - 90.0), successor_id))
        return min(candidates, default=(math.inf, None))[1]

    def _straight_successor(self, lane_id: str) -> str | None:
        lane = self.world.network.lane(lane_id)
        if lane is None:
            return None
        approach = self._lane_heading(lane)
        candidates = []
        for successor_id in lane.next_lane_ids:
            successor = self.world.network.lane(successor_id)
            if successor is not None:
                delta = abs(self._heading_delta(
                    self._lane_heading(successor), approach))
                candidates.append((delta, successor_id))
        return min(candidates, default=(math.inf, None))[1]

    @staticmethod
    def _lane_heading(lane) -> float:
        a, b = lane.centerline[0], lane.centerline[-1]
        return math.degrees(math.atan2(b[0] - a[0], b[2] - a[2])) % 360.0

    @staticmethod
    def _heading_delta(heading: float, reference: float) -> float:
        return (heading - reference + 180.0) % 360.0 - 180.0

    @staticmethod
    def _left_turn_location(v, context: _LeftTurnContext) -> str:
        if v.current_lane == context.source_lane and context.source_lane != context.target_lane:
            return left_turn.SOURCE_LANE
        if v.current_lane == context.target_lane:
            return left_turn.TARGET_APPROACH
        if v.current_lane == context.connector_lane:
            return left_turn.TURN_CONNECTOR
        if v.current_lane == context.exit_lane:
            return left_turn.EXIT_LANE
        return left_turn.OTHER

    def _left_turn_change_deadline_reached(self, v,
                                           target_lane_id: str) -> bool:
        target = self.world.network.lane(target_lane_id)
        deadline = self._lane_change_latest_start_arc(target_lane_id)
        if target is None or deadline is None:
            return False
        return target.closest_point(v.position)[2] >= deadline - LEFT_TURN_ABORT_EPS

    def _aborted_left_path(self, v, context: _LeftTurnContext) -> Path:
        """Safe straight-through route used after a late left cancellation."""
        if v.current_lane != context.source_lane:
            return []
        straight_id = self._straight_successor(context.source_lane)
        if straight_id is None:
            return []
        lane_ids = [context.source_lane, straight_id]
        straight = self.world.network.lane(straight_id)
        if straight is not None and straight.next_lane_ids:
            lane_ids.append(straight.next_lane_ids[0])
        path: Path = [list(v.position)]
        for index, lane_id in enumerate(lane_ids):
            lane = self.world.network.lane(lane_id)
            if lane is None:
                continue
            current_arc = lane.closest_point(v.position)[2] if index == 0 else -1.0
            for point in lane.centerline:
                if index == 0 and lane.closest_point(point)[2] <= current_arc + 0.05:
                    continue
                if dist_xz(path[-1], point) > 0.05:
                    path.append(list(point))
        return path

    def _left_turn_wait_path(self, v, target_lane_id: str) -> Path:
        """Expose only the approach lane while intersection entry is denied."""
        lane = self.world.network.lane(target_lane_id)
        light = self.traffic.lights.get(target_lane_id)
        if lane is None or light is None:
            return []
        rad = math.radians(light.approach_heading)
        fx, fz = math.sin(rad), math.cos(rad)
        stop = [
            light.stop_line[0] - fx * STOP_LINE_BUFFER,
            light.stop_line[1],
            light.stop_line[2] - fz * STOP_LINE_BUFFER,
        ]
        current_arc = lane.closest_point(v.position)[2]
        stop_arc = lane.closest_point(stop)[2]
        midpoint = self._point_at_arc(
            lane.centerline, current_arc + max(0.0, stop_arc - current_arc) * 0.5)
        return [list(v.position), midpoint, stop]

    def _left_turn_at_entry_gate(self, v, target_lane_id: str) -> bool:
        if v.current_lane != target_lane_id:
            return False
        light = self.traffic.lights.get(target_lane_id)
        if light is None:
            return True
        rad = math.radians(light.approach_heading)
        fx, fz = math.sin(rad), math.cos(rad)
        distance = ((light.stop_line[0] - v.position[0]) * fx
                    + (light.stop_line[2] - v.position[2]) * fz)
        return distance <= STOP_LINE_BUFFER + 2.0

    def _left_turn_exit_aligned(self, v, exit_lane_id: str) -> bool:
        if v.current_lane != exit_lane_id:
            return False
        lane = self.world.network.lane(exit_lane_id)
        if lane is None:
            return False
        lateral = lane.closest_point(v.position)[1]
        error = abs(self._heading_delta(v.heading, self._lane_heading(lane)))
        return lateral <= 0.5 and error <= 8.0

    def _left_turn_entry_clear(self, v, leader: Optional[Leader],
                               context: _LeftTurnContext) -> bool:
        """Hold at the line for a stopped queue, pedestrian, or blocked exit."""
        if v.current_lane != context.target_lane:
            return True
        light = self.traffic.lights.get(context.target_lane)
        if light is None:
            return True
        rad = math.radians(light.approach_heading)
        fx, fz = math.sin(rad), math.cos(rad)
        distance = ((light.stop_line[0] - v.position[0]) * fx
                    + (light.stop_line[2] - v.position[2]) * fz)
        if distance > STOP_LINE_BUFFER + 2.0:
            return True

        if (leader is not None and leader.speed <= 0.2
                and leader.gap <= 30.0):
            return False
        if any(o.type == "pedestrian"
               and self._pedestrian_blocks_left_path(v, o.position, context)
               for o in self.world.objects.values()):
            return False
        blocked_lanes = {context.connector_lane, context.exit_lane}
        return not any(
            other.current_lane in blocked_lanes
            and dist_xz(v.position, other.position) <= 35.0
            and other.speed <= 1.0
            for other in self.world.vehicles.values() if other.id != v.id)

    def _pedestrian_blocks_left_path(
            self, v, pedestrian_position: Vec3,
            context: _LeftTurnContext) -> bool:
        """Check the driven corridor, not a circle that includes sidewalks."""
        target = self.world.network.lane(context.target_lane)
        connector = self.world.network.lane(context.connector_lane)
        exit_lane = self.world.network.lane(context.exit_lane)
        if target is None or connector is None or exit_lane is None:
            return True

        current_arc = target.closest_point(v.position)[2]
        route: Path = [list(v.position)]
        for point in target.centerline:
            if target.closest_point(point)[2] > current_arc + 0.05:
                route.append(list(point))
        for lane in (connector, exit_lane):
            for point in lane.centerline:
                if dist_xz(route[-1], point) > 0.05:
                    route.append(list(point))

        arc, lateral, _ = _project_path(route, pedestrian_position)
        return (arc <= LEFT_TURN_CONFLICT_HORIZON
                and lateral <= LEFT_TURN_PEDESTRIAN_CORRIDOR)

    def _signal_approach_speed(self, v, signal_lane: str | None = None) -> float:
        light = self.traffic.lights.get(signal_lane or v.current_lane)
        if light is None:
            return 0.0
        rad = math.radians(light.approach_heading)
        forward_x, forward_z = math.sin(rad), math.cos(rad)
        dx = light.stop_line[0] - v.position[0]
        dz = light.stop_line[2] - v.position[2]
        distance = dx * forward_x + dz * forward_z
        braking_distance = max(0.0, distance - STOP_LINE_BUFFER)
        return math.sqrt(2.0 * SIGNAL_APPROACH_DECEL * braking_distance)

    @staticmethod
    def _is_pedestrian_phase(time: float) -> bool:
        phase = time % URBAN_SIGNAL_PERIOD
        return any(start <= phase < end for start, end in PEDESTRIAN_PHASES)

    def merging_lane_ids(self) -> set[str]:
        """Lanes whose traffic has to buy its way into another lane's stream.

        Two topologies qualify, and neither is discoverable from a lane's name
        — which is what this used to key off (``"ramp" in lane_id``), quietly
        confining the whole reservation feature to the one scene that happened
        to use that word:

          * **mid-lane join** — the lane ends part-way along its successor, so
            what it merges into is that successor's own upstream traffic. This
            is an on-ramp (``hw_ramp`` joins ``hw_l2`` at arc 115).
          * **side-by-side join** — a slower lane that runs alongside a faster
            one and ends into the same point of the same successor: a shoulder
            rejoining a boulevard (``city_boulevard_escape`` into
            ``city_turn_east``), or a ramp meeting the mainline at a segment
            boundary. Only the slower lane reserves; the faster one is the
            mainline being merged into.

        Intersection turn connectors also share an exit lane, but they arrive
        from somewhere else entirely — a short way back they are neither close
        to nor aligned with the through lane, which is what ``_runs_alongside``
        tests. Signals, not reservations, separate those.
        """
        if self._merging_lanes is not None:
            return self._merging_lanes
        network = self.world.network
        merging: set[str] = set()
        for lane in network.lanes.values():
            for successor_id in lane.next_lane_ids:
                successor = network.lane(successor_id)
                if successor is None:
                    continue
                entry = successor.closest_point(lane.end)[2]
                if entry > MERGE_JOIN_EPS:
                    merging.add(lane.id)
                    break
                siblings = (network.lane(pid)
                            for pid in network.predecessors(successor_id)
                            if pid != lane.id)
                if any(s is not None
                       and s.speed_limit > lane.speed_limit
                       and abs(successor.closest_point(s.end)[2] - entry)
                       <= MERGE_JOIN_EPS
                       and self._runs_alongside(lane, s)
                       for s in siblings):
                    merging.add(lane.id)
                    break
        self._merging_lanes = merging
        return merging

    @classmethod
    def _runs_alongside(cls, lane, other) -> bool:
        """Do two lanes approach their shared junction as parallel streams?

        Sampled a short way back from both ends: a merging pair is close and
        aligned there (that is what makes it a merge), a turn connector and the
        through lane it exits onto are neither.
        """
        back = min(MERGE_PARALLEL_PROBE, lane.length, other.length)
        if back <= 0.0:
            return False
        a = cls._point_at_arc(lane.centerline, lane.length - back)
        b = cls._point_at_arc(other.centerline, other.length - back)
        if dist_xz(a, b) > MERGE_PARALLEL_SPAN:
            return False
        delta = cls._heading_delta(lane.heading_at_arc(lane.length - back),
                                   other.heading_at_arc(other.length - back))
        return abs(delta) <= MERGE_PARALLEL_ANGLE

    def _update_merge_reservations(self) -> None:
        """Reserve a V2X time slot for every vehicle on a merging lane."""
        self._merge_speed_overrides = {}
        merging = self.merging_lane_ids()
        ramps = [v for v in self.world.vehicles.values()
                 if v.current_lane in merging]
        for ramp_vehicle in ramps:
            ramp_lane = self.world.network.lane(ramp_vehicle.current_lane)
            if ramp_lane is None:
                continue
            merge_point = list(ramp_lane.end)
            mainline_ids = self._mainline_lanes(ramp_lane)
            mainline = [v for v in self.world.vehicles.values()
                        if v.id != ramp_vehicle.id
                        and v.current_lane in mainline_ids]
            # Plan against the speed the ramp car is *meant* to hold, not the
            # one it happens to have: a reservation derived from the current
            # speed pins a stopped ramp car at 0 forever.
            plan = merge.plan_merge(
                ramp_vehicle, mainline, merge_point,
                desired_speed=self._lane_speed(ramp_vehicle.current_lane))
            self._merge_speed_overrides[ramp_vehicle.id] = max(
                0.0, plan.ramp_target_speed)
            if plan.yield_vehicle and plan.yield_target_speed is not None:
                self._merge_speed_overrides[plan.yield_vehicle] = max(
                    0.0, plan.yield_target_speed)

    def _mainline_lanes(self, ramp_lane) -> set[str]:
        """Lanes whose traffic contends for ``ramp_lane``'s merge point.

        The lanes the ramp feeds into, plus however much road drains into them
        from upstream — a mainline split into segments puts the cars that are
        seconds from the join on a *predecessor* of the joined lane, and those
        are exactly the ones the reservation must see. Walking back stops at
        ``MERGE_UPSTREAM_HORIZON`` so a road that merely shares a distant
        ancestor is not mistaken for approaching traffic, and at any other
        merging lane, whose cars are merging rather than being merged into.
        """
        network = self.world.network
        merging = self.merging_lane_ids()
        mainline: set[str] = set()
        # (lane id, metres of road between that lane's end and the merge point)
        frontier = [(lid, 0.0) for lid in ramp_lane.next_lane_ids]
        while frontier:
            lane_id, upstream = frontier.pop()
            if lane_id in mainline or lane_id == ramp_lane.id:
                continue
            if lane_id in merging:
                continue
            mainline.add(lane_id)
            if upstream >= MERGE_UPSTREAM_HORIZON:
                continue
            for pid in network.predecessors(lane_id):
                frontier.append((pid, upstream + network.lane_length(pid)))
        return mainline

    def _lane_change_path(self, v, target_lane_id: str,
                          distance: float = 32.0,
                          include_route_tail: bool = True) -> Path:
        """Blend to an adjacent lane, then route normally from that lane.

        The tail must be planned through the lane graph.  Appending the final
        goal directly would cut diagonally across an intersection and bypass
        protected turn connectors selected from the lane graph.
        """
        current = self.world.network.lane(v.current_lane)
        target = self.world.network.lane(target_lane_id)
        if current is None or target is None:
            return []
        current_point, _, current_arc = current.closest_point(v.position)
        _, _, target_arc = target.closest_point(v.position)
        # Preserve the lateral progress already made since the manoeuvre was
        # committed.  Rebuilding from the source-lane centre every tick would
        # put already-passed waypoints back in front of the vehicle and make a
        # half-completed left change steer right again.
        lateral_offset = [
            v.position[0] - current_point[0],
            v.position[1] - current_point[1],
            v.position[2] - current_point[2],
        ]
        path: Path = [list(v.position)]
        for i in range(1, 5):
            f = i / 4.0
            a = self._point_at_arc(current.centerline, current_arc + distance * f)
            b = self._point_at_arc(target.centerline, target_arc + distance * f)
            a = [a[j] + lateral_offset[j] for j in range(3)]
            path.append([
                a[0] + (b[0] - a[0]) * f,
                a[1] + (b[1] - a[1]) * f,
                a[2] + (b[2] - a[2]) * f,
            ])
        if include_route_tail and v.goal is not None:
            tail = self.planner.plan(path[-1], v.goal, self.world.network)
            if not tail:
                return []
            for point in tail[1:]:
                if dist_xz(path[-1], point) > 0.05:
                    path.append(list(point))
        return path

    def _lane_change_distance(self, v, target_lane_id: str) -> float | None:
        """Choose a merge end before an urban turn's stop line.

        A fixed 32 m change started late can end inside an intersection. Any
        requested left approach that has a configured signal is therefore
        settled roughly 14 m before its line, independent of lane naming.
        """
        if v.maneuver != "left" or target_lane_id not in self.traffic.lights:
            return 32.0

        target = self.world.network.lane(target_lane_id)
        light = self.traffic.lights.get(target_lane_id)
        if target is None or light is None:
            return None

        _, _, current_arc = target.closest_point(v.position)
        rad = math.radians(light.approach_heading)
        fx, fz = math.sin(rad), math.cos(rad)

        def arc_before_line(distance_before: float) -> float:
            point = [
                light.stop_line[0] - fx * distance_before,
                light.stop_line[1],
                light.stop_line[2] - fz * distance_before,
            ]
            return target.closest_point(point)[2]

        preferred_arc = arc_before_line(LEFT_LANE_CHANGE_READY_DISTANCE)
        latest_arc = arc_before_line(STOP_LINE_BUFFER + LANE_CHANGE_STOP_MARGIN)
        if latest_arc - current_arc < LANE_CHANGE_MIN_LENGTH:
            return None
        return min(32.0, max(
            LANE_CHANGE_MIN_LENGTH, preferred_arc - current_arc))

    def _queue_safe_lane_change_distance(
            self, geometric_distance: float,
            decision: lane_change.LaneChangeDecision,
            lead_speed: float = 0.0) -> float | None:
        """Leave a full stopped ACC gap at the end of the lateral blend.

        Gap acceptance alone only proves that the target lane is free *now*.
        A stopped queue may still occupy the originally selected merge end.
        Capping forward travel reserves a point behind that queue instead of
        drawing a path through its last vehicle.
        """
        if not math.isfinite(decision.lead_gap):
            return geometric_distance
        # A moving target-lane leader keeps opening the merge slot during the
        # lateral blend. Use only a short preview so a future signal stop is
        # not treated as unlimited space; ACC remains active throughout.
        predicted_opening = max(0.0, lead_speed) * LANE_CHANGE_LEAD_PREVIEW
        max_forward = (decision.lead_gap + predicted_opening
                       - self.acc.p.standstill_gap)
        safe_distance = min(geometric_distance, max_forward)
        if safe_distance < LANE_CHANGE_MIN_LENGTH:
            return None
        return safe_distance

    def _lane_change_latest_start_arc(self, target_lane_id: str) -> float | None:
        target = self.world.network.lane(target_lane_id)
        light = self.traffic.lights.get(target_lane_id)
        if target is None or light is None:
            return None
        rad = math.radians(light.approach_heading)
        fx, fz = math.sin(rad), math.cos(rad)
        latest_end = [
            light.stop_line[0] - fx * (STOP_LINE_BUFFER + LANE_CHANGE_STOP_MARGIN),
            light.stop_line[1],
            light.stop_line[2] - fz * (STOP_LINE_BUFFER + LANE_CHANGE_STOP_MARGIN),
        ]
        return target.closest_point(latest_end)[2] - LANE_CHANGE_MIN_LENGTH

    def _lane_change_wait_speed(self, v, target_lane_id: str) -> float:
        target = self.world.network.lane(target_lane_id)
        latest_start = self._lane_change_latest_start_arc(target_lane_id)
        if target is None or latest_start is None:
            return 0.0
        current_arc = target.closest_point(v.position)[2]
        distance = max(0.0, latest_start - current_arc)
        return math.sqrt(2.0 * SIGNAL_APPROACH_DECEL * distance)

    def _lane_change_wait_path(self, v, target_lane_id: str) -> Path:
        """Short current-lane path ending where a safe merge can still start."""
        current = self.world.network.lane(v.current_lane)
        target = self.world.network.lane(target_lane_id)
        latest_start = self._lane_change_latest_start_arc(target_lane_id)
        if current is None or target is None or latest_start is None:
            return []
        current_arc = current.closest_point(v.position)[2]
        target_arc = target.closest_point(v.position)[2]
        forward = max(0.5, latest_start - target_arc)
        end_arc = current_arc + forward
        return [
            list(v.position),
            self._point_at_arc(current.centerline, current_arc + forward * 0.5),
            self._point_at_arc(current.centerline, end_arc),
        ]

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
            if self._breach_is_standing(c):
                continue
            best = min(best, c.ttc)
        return best

    def _mover(self, mover_id: str):
        return (self.world.vehicles.get(mover_id)
                or self.world.objects.get(mover_id))

    def _breach_is_standing(self, conflict) -> bool:
        """True for a safety breach that braking cannot improve.

        ``time_to_breach`` reports 0 s for a pair already inside the safety
        radius, which is right while they are closing. For a pair that is *not*
        closing — both at rest, or running parallel — that zero is a trap: both
        vehicles read it as EmergencyBraking, both are commanded to zero, the
        separation never changes, and the emergency never clears. On the Highway
        export a ramp car that stops short of the join sits ~2 m from the
        mainline car beside it in the taper, and the pair (plus everything
        queued behind it) stands there for the rest of the run.
        """
        if conflict.ttc > 0.0:
            return False
        a, b = self._mover(conflict.a_id), self._mover(conflict.b_id)
        if a is None or b is None:
            return False
        rpx, rpz = a.position[0] - b.position[0], a.position[2] - b.position[2]
        rvx = a.velocity[0] - b.velocity[0]
        rvz = a.velocity[2] - b.velocity[2]
        return rpx * rvx + rpz * rvz >= 0.0

    def _is_intersection_lane(self, lane_id: str | None) -> bool:
        """Is ``lane_id`` past a stop line, i.e. inside the junction box?

        Any lane a signalized approach feeds into is. This was a substring test
        for ``_straight`` / ``_left`` / ``_right``, which reads the scene's
        naming rather than its shape: on the EmergencyAvoidance export it
        classified the plain travel lanes ``ea_left`` and ``ea_right`` as
        intersection interior.
        """
        if not lane_id:
            return False
        return any(lane_id in lane.next_lane_ids
                   for approach_id in self.traffic.lights
                   if (lane := self.world.network.lane(approach_id)) is not None)

    def _traffic_conflict_is_managed(self, ego_id: str, other_id: str) -> bool:
        """A green approach need not emergency-brake for a red approach that
        is still upstream of its enforced stop line."""
        ego = self.world.vehicles.get(ego_id)
        other = self.world.vehicles.get(other_id)
        if ego is None or other is None:
            return False
        ego_in_intersection = self._is_intersection_lane(ego.current_lane)
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
