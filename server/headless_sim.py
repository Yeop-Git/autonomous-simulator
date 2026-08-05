"""Headless kinematic simulator — a Unity stand-in for tests & experiments.

Unity owns vehicle motion in production, but we need to exercise the central
controller (routing, following, collision avoidance) without the editor. This
module plays Unity's role:

  * holds sim vehicles (position / speed / lane / commanded path),
  * each step builds a schema-shaped StateMessage, runs it through the
    ``CentralController``, then applies the returned commands by moving each
    vehicle along its path at the commanded speed (simple first-order speed
    tracking + arc-length advance along the path polyline),
  * optionally writes CSV rows in the frozen logging schema (plan §21.1).

It is intentionally kinematic (no tyre/engine dynamics) — that level of
fidelity is Unity's job. This is for logic, not ride feel.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from central_control import CentralController
from controllers import lateral as lat
from logging_csv import DriveLogger, LogRow
from world_model import LaneNetwork

Vec3 = list[float]

VEHICLE_ACCEL = 3.0   # m/s^2 how fast sim speed tracks the commanded speed
HEADING_FROM_VEL_EPS = 0.05


@dataclass
class SimVehicle:
    id: str
    position: Vec3
    speed: float
    lane: str
    goal: Optional[Vec3] = None
    has_goal: bool = False
    type: str = "car"
    heading: float = 0.0
    velocity: Vec3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    path: list[Vec3] = field(default_factory=list)
    path_arc: float = 0.0  # monotone arc position along the current path
    behavior: str = "LaneKeeping"
    arrived: bool = False
    maneuver: str = "straight"
    target_lane: Optional[str] = None


@dataclass
class SimObject:
    id: str
    type: str
    position: Vec3
    velocity: Vec3
    radius: float = 0.4


class HeadlessSim:
    def __init__(self, network: LaneNetwork, controller: CentralController | None = None,
                 dt: float = 0.1, scenario: str = "highway",
                 logger: DriveLogger | None = None):
        self.network = network
        self.controller = controller or CentralController(network, dt=dt)
        self.dt = dt
        self.scenario = scenario
        self.logger = logger
        self.time = 0.0
        self.tick = 0
        self.vehicles: dict[str, SimVehicle] = {}
        self.objects: dict[str, SimObject] = {}
        self.events: list[dict] = []

    # ------------------------------------------------------------------ #
    def add_vehicle(self, vid: str, position: Vec3, lane: str, speed: float = 0.0,
                    goal: Optional[Vec3] = None, maneuver: str = "straight",
                    target_lane: Optional[str] = None) -> SimVehicle:
        v = SimVehicle(id=vid, position=list(position), speed=speed, lane=lane,
                       goal=list(goal) if goal else None, has_goal=goal is not None,
                       maneuver=maneuver, target_lane=target_lane)
        self._set_velocity_from_lane(v)
        self.vehicles[vid] = v
        return v

    def add_object(self, oid: str, otype: str, position: Vec3, velocity: Vec3,
                   radius: float = 0.4) -> SimObject:
        o = SimObject(id=oid, type=otype, position=list(position),
                      velocity=list(velocity), radius=radius)
        self.objects[oid] = o
        return o

    # ------------------------------------------------------------------ #
    def build_state(self) -> dict:
        return {
            "time": self.time,
            "tick": self.tick,
            "scenario": self.scenario,
            "vehicles": [
                {
                    "id": v.id, "type": v.type,
                    "position": v.position, "velocity": v.velocity,
                    "heading": v.heading, "current_lane": v.lane,
                    "target_lane": v.target_lane, "maneuver": v.maneuver,
                    "has_goal": v.has_goal, "goal": v.goal or [0.0, 0.0, 0.0],
                    "behavior_state": v.behavior,
                }
                for v in self.vehicles.values()
            ],
            "objects": [
                {"id": o.id, "type": o.type, "position": o.position,
                 "velocity": o.velocity, "radius": o.radius}
                for o in self.objects.values()
            ],
            "events": list(self.events),
        }

    def step(self) -> dict:
        """Advance one tick; return the CommandMessage the controller produced."""
        state = self.build_state()
        command = self.controller.step(state)
        cmd_by_id = {c["vehicle_id"]: c for c in command["commands"]}

        for v in self.vehicles.values():
            cmd = cmd_by_id.get(v.id)
            if cmd is None:
                continue
            v.behavior = cmd.get("behavior", v.behavior)
            if cmd.get("path"):
                next_path = [list(p) for p in cmd["path"]]
                # Unity replaces its commanded path on every message and
                # projects from the current transform.  A stale arc from the
                # previous polyline can otherwise clamp a headless vehicle to
                # the end of a newly generated lane-change path.
                v.path_arc = _project_arc(
                    next_path, v.position, min_arc=0.0, window=math.inf)
                v.path = next_path
            self._apply(v, cmd)

        if self.logger is not None:
            self._log(cmd_by_id)

        # objects move at constant velocity
        for o in self.objects.values():
            o.position = [o.position[i] + o.velocity[i] * self.dt for i in range(3)]

        self.events = []  # events are one-shot per tick unless re-injected
        self.time += self.dt
        self.tick += 1
        return command

    def _log(self, cmd_by_id: dict) -> None:
        min_ttc = {}
        for c in self.controller.last_conflicts:
            for vid in (c.a_id, c.b_id):
                min_ttc[vid] = min(min_ttc.get(vid, math.inf), c.ttc)
        for v in self.vehicles.values():
            lane = self.network.lane(v.lane)
            lateral_err = heading_err = 0.0
            if lane is not None:
                lateral_err, heading_err, _ = lat.frenet_errors(
                    v.position[0], v.position[2], v.heading, list(lane.centerline))
            cmd = cmd_by_id.get(v.id, {})
            ttc = min_ttc.get(v.id, math.inf)
            self.logger.log(LogRow(
                time=round(self.time, 3), vehicle_id=v.id, scenario=self.scenario,
                position_x=v.position[0], position_z=v.position[2], speed=v.speed,
                lane_id=v.lane, behavior_state=v.behavior,
                lateral_error=lateral_err, heading_error=heading_err,
                target_speed=float(cmd.get("target_speed", 0.0)),
                collision_risk=1.0 if math.isfinite(ttc) else 0.0, ttc=ttc,
            ))

    # ------------------------------------------------------------------ #
    def _apply(self, v: SimVehicle, cmd: dict) -> None:
        target_speed = float(cmd.get("target_speed", 0.0))
        # first-order speed tracking toward the commanded speed
        dv = target_speed - v.speed
        max_dv = VEHICLE_ACCEL * self.dt
        v.speed += max(-max_dv * 2, min(max_dv, dv))  # allow faster braking
        v.speed = max(0.0, v.speed)

        if v.behavior == "Arrived":
            v.arrived = True
            v.speed = 0.0
            return

        advance = v.speed * self.dt
        if v.path:
            self._advance_along_path(v, advance)
        else:
            self._advance_along_lane(v, advance)

        candidates = {v.lane}
        current = self.network.lane(v.lane)
        if current is not None:
            candidates.update(current.next_lane_ids)
            if current.left_lane_id:
                candidates.add(current.left_lane_id)
            if current.right_lane_id:
                candidates.add(current.right_lane_id)
        if v.target_lane:
            candidates.add(v.target_lane)
            target = self.network.lane(v.target_lane)
            if target is not None:
                candidates.update(target.next_lane_ids)
        lane = self.network.nearest_lane(
            v.position, heading=v.heading, candidate_ids=candidates)
        if lane:
            v.lane = lane
        if v.target_lane == v.lane:
            v.target_lane = None

    def _advance_along_path(self, v: SimVehicle, distance: float) -> None:
        """Move ``distance`` m forward along the commanded path polyline.

        The path may begin behind the vehicle (the controller routes from the
        original start and caches it), so we project the vehicle onto the path
        first and advance from that arc position — never snapping back to the
        path's start point.
        """
        path = v.path
        if len(path) < 2:
            return
        # Project within a forward window of the last arc position so the
        # vehicle never snaps onto a far leg of a self-approaching path.
        proj_arc = _project_arc(path, v.position, min_arc=v.path_arc)
        new_arc = proj_arc + distance
        v.position = _point_at_arc(path, new_arc)
        v.path_arc = new_arc
        ahead = _point_at_arc(path, new_arc + 1.0)
        self._set_heading_velocity(v, ahead)

    def _advance_along_lane(self, v: SimVehicle, distance: float) -> None:
        lane = self.network.lane(v.lane)
        if lane is None:
            return
        # project, move along centerline direction
        point, _, arc = lane.closest_point(v.position)
        target_arc = arc + distance
        new_point = _point_at_arc(lane.centerline, target_arc)
        v.position = new_point
        # heading toward a slightly-ahead point
        ahead = _point_at_arc(lane.centerline, target_arc + 1.0)
        self._set_heading_velocity(v, ahead)

    def _set_heading_velocity(self, v: SimVehicle, toward: Optional[Vec3]) -> None:
        if toward is not None:
            dx = toward[0] - v.position[0]
            dz = toward[2] - v.position[2]
            norm = math.hypot(dx, dz)
            if norm > HEADING_FROM_VEL_EPS:
                v.heading = math.degrees(math.atan2(dx, dz)) % 360.0
                v.velocity = [dx / norm * v.speed, 0.0, dz / norm * v.speed]
                return
        # No usable look-ahead: keep heading, but rescale velocity to the new
        # speed so the reported velocity never desyncs from v.speed.
        rad = math.radians(v.heading)
        v.velocity = [math.sin(rad) * v.speed, 0.0, math.cos(rad) * v.speed]

    def _set_velocity_from_lane(self, v: SimVehicle) -> None:
        lane = self.network.lane(v.lane)
        if lane is None:
            return
        _, _, arc = lane.closest_point(v.position)
        ahead = _point_at_arc(lane.centerline, arc + 1.0)
        self._set_heading_velocity(v, ahead)

    # ------------------------------------------------------------------ #
    def min_pairwise_distance(self) -> float:
        """Smallest center-to-center distance between any two vehicles now."""
        vs = list(self.vehicles.values())
        best = math.inf
        for i in range(len(vs)):
            for j in range(i + 1, len(vs)):
                best = min(best, _dist_xz(vs[i].position, vs[j].position))
        return best

    def all_arrived(self) -> bool:
        return all(v.arrived for v in self.vehicles.values()) and bool(self.vehicles)


# --------------------------------------------------------------------------- #
def _dist_xz(a: Vec3, b: Vec3) -> float:
    return math.hypot(a[0] - b[0], a[2] - b[2])


def _project_arc(polyline: list[Vec3], position: Vec3, min_arc: float = 0.0,
                 back_tol: float = 2.0, window: float = 80.0) -> float:
    """Arc-length of the closest point on ``polyline`` to ``position`` (xz).

    Restricted to segments within ``[min_arc - back_tol, min_arc + window]`` so
    the projection stays local and monotone — on a path that loops back near
    itself, the global lateral minimum could otherwise jump to a far leg and
    teleport the vehicle. ``min_arc=0`` with a large window reproduces a plain
    global projection.
    """
    best_arc, best_lat, acc = min_arc, math.inf, 0.0
    lo, hi = min_arc - back_tol, min_arc + window
    found = False
    for i in range(len(polyline) - 1):
        a, b = polyline[i], polyline[i + 1]
        dx, dz = b[0] - a[0], b[2] - a[2]
        seg_sq = dx * dx + dz * dz
        seg = math.sqrt(seg_sq)
        seg_start, seg_end = acc, acc + seg
        acc = seg_end
        if seg_sq == 0.0 or seg_end < lo or seg_start > hi:
            continue
        t = ((position[0] - a[0]) * dx + (position[2] - a[2]) * dz) / seg_sq
        t = max(0.0, min(1.0, t))
        px, pz = a[0] + t * dx, a[2] + t * dz
        lat = math.hypot(position[0] - px, position[2] - pz)
        if lat < best_lat:
            best_lat = lat
            best_arc = seg_start + t * seg
            found = True
    return best_arc if found else min_arc


def _point_at_arc(centerline: list[Vec3], arc: float) -> Vec3:
    """Point at arc-length ``arc`` along a polyline (clamped to its ends)."""
    if arc <= 0:
        return list(centerline[0])
    acc = 0.0
    for i in range(len(centerline) - 1):
        a, b = centerline[i], centerline[i + 1]
        seg = _dist_xz(a, b)
        if acc + seg >= arc:
            t = (arc - acc) / seg if seg > 0 else 0.0
            return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t,
                    a[2] + (b[2] - a[2]) * t]
        acc += seg
    return list(centerline[-1])
