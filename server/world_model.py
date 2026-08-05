"""World model — the server's authoritative view of the static road graph
and the dynamic objects reported by Unity each tick.

Two concerns live here, deliberately separated:

  * ``LaneNetwork`` — the *static* lane graph. It implements the ``World``
    protocol the planners depend on (``neighbors`` / ``lane_centerline`` /
    ``is_blocked``). Pure geometry + graph; no Unity, no per-tick state.
  * ``WorldModel`` — the *dynamic* world: the latest StateMessage decoded
    into vehicles/objects, plus the lane network and any blocked positions
    (from hazard events). This is the side-effecting part the planners are
    kept away from.

Loading: ``LaneNetwork.from_json`` consumes the lane-network export format
defined in ``shared/protocol/lane_network.schema.json``.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path as FsPath
from typing import Iterable, Optional

Vec3 = list[float]

DEFAULT_WIDTH = 3.5
DEFAULT_SPEED_LIMIT = 13.9  # m/s, ~50 km/h


# --------------------------------------------------------------------------- #
# Small vector helpers (no numpy dependency so this stays import-cheap and
# trivially testable; planners that want numpy can convert at their edge).
# --------------------------------------------------------------------------- #
def dist(a: Vec3, b: Vec3) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def dist_xz(a: Vec3, b: Vec3) -> float:
    """Planar distance ignoring height (y). Road geometry is effectively 2D."""
    return math.hypot(a[0] - b[0], a[2] - b[2])


def polyline_length(points: list[Vec3]) -> float:
    return sum(dist(points[i], points[i + 1]) for i in range(len(points) - 1))


# --------------------------------------------------------------------------- #
# Static lane graph
# --------------------------------------------------------------------------- #
@dataclass
class Lane:
    id: str
    centerline: list[Vec3]
    width: float = DEFAULT_WIDTH
    speed_limit: float = DEFAULT_SPEED_LIMIT
    left_lane_id: Optional[str] = None
    right_lane_id: Optional[str] = None
    next_lane_ids: list[str] = field(default_factory=list)

    @property
    def start(self) -> Vec3:
        return self.centerline[0]

    @property
    def end(self) -> Vec3:
        return self.centerline[-1]

    @property
    def length(self) -> float:
        return polyline_length(self.centerline)

    def closest_point(self, position: Vec3) -> tuple[Vec3, float, float]:
        """Project ``position`` onto the lane centerline.

        Returns ``(point, lateral_distance, arc_length)`` where ``point`` is
        the nearest point on the polyline, ``lateral_distance`` is the planar
        distance from ``position`` to that point, and ``arc_length`` is how
        far along the lane the projection falls (meters from lane start).
        """
        best_point = self.centerline[0]
        best_lat = dist_xz(position, self.centerline[0])
        best_arc = 0.0
        acc = 0.0
        for i in range(len(self.centerline) - 1):
            a = self.centerline[i]
            b = self.centerline[i + 1]
            seg = _project_to_segment(position, a, b)
            point, t, seg_len = seg
            lat = dist_xz(position, point)
            if lat < best_lat:
                best_lat = lat
                best_point = point
                best_arc = acc + t * seg_len
            acc += seg_len
        return best_point, best_lat, best_arc

    def heading_at_arc(self, arc: float) -> float:
        """Unity-style yaw of the lane tangent at ``arc`` (0 = +Z)."""
        remaining = max(0.0, arc)
        fallback = 0.0
        for i in range(len(self.centerline) - 1):
            a, b = self.centerline[i], self.centerline[i + 1]
            dx, dz = b[0] - a[0], b[2] - a[2]
            length = math.hypot(dx, dz)
            if length <= 0.0:
                continue
            fallback = math.degrees(math.atan2(dx, dz)) % 360.0
            if remaining <= length:
                return fallback
            remaining -= length
        return fallback


def _project_to_segment(p: Vec3, a: Vec3, b: Vec3) -> tuple[Vec3, float, float]:
    """Project p onto segment a->b in the xz plane.

    Returns (closest_point, t_clamped_0_1, segment_length). The returned
    point keeps the segment's interpolated y so height stays sane.
    """
    ax, az = a[0], a[2]
    bx, bz = b[0], b[2]
    dx, dz = bx - ax, bz - az
    seg_len_sq = dx * dx + dz * dz
    seg_len = math.sqrt(seg_len_sq)
    if seg_len_sq == 0.0:
        return list(a), 0.0, 0.0
    t = ((p[0] - ax) * dx + (p[2] - az) * dz) / seg_len_sq
    t = max(0.0, min(1.0, t))
    point = [ax + t * dx, a[1] + t * (b[1] - a[1]), az + t * dz]
    return point, t, seg_len


class LaneNetwork:
    """Static lane graph. Implements the planner ``World`` protocol.

    ``is_blocked`` consults an optional set of blocked positions (populated
    from hazard events) — keep that mutation in ``WorldModel``, not here, so
    a plain ``LaneNetwork`` stays a pure search space for unit tests.
    """

    def __init__(self, lanes: Iterable[Lane], name: str = "", scenario: str = ""):
        self.name = name
        self.scenario = scenario
        self.lanes: dict[str, Lane] = {lane.id: lane for lane in lanes}
        self._blocked: list[tuple[Vec3, float]] = []  # (position, radius)

    # ---- construction ----------------------------------------------------- #
    @classmethod
    def from_dict(cls, data: dict) -> "LaneNetwork":
        lanes = []
        for ld in data["lanes"]:
            lanes.append(
                Lane(
                    id=ld["id"],
                    centerline=[list(p) for p in ld["centerline"]],
                    width=ld.get("width", DEFAULT_WIDTH),
                    speed_limit=ld.get("speed_limit", DEFAULT_SPEED_LIMIT),
                    left_lane_id=ld.get("left_lane_id"),
                    right_lane_id=ld.get("right_lane_id"),
                    next_lane_ids=list(ld.get("next_lane_ids", [])),
                )
            )
        return cls(lanes, name=data.get("name", ""), scenario=data.get("scenario", ""))

    @classmethod
    def from_json(cls, path: str | FsPath) -> "LaneNetwork":
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_dict(json.load(f))

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "scenario": self.scenario,
            "lanes": [
                {
                    "id": lane.id,
                    "centerline": lane.centerline,
                    "width": lane.width,
                    "speed_limit": lane.speed_limit,
                    "left_lane_id": lane.left_lane_id,
                    "right_lane_id": lane.right_lane_id,
                    "next_lane_ids": lane.next_lane_ids,
                }
                for lane in self.lanes.values()
            ],
        }

    # ---- World protocol --------------------------------------------------- #
    def neighbors(self, lane_id: str) -> list[str]:
        lane = self.lanes.get(lane_id)
        if lane is None:
            return []
        return list(lane.next_lane_ids)

    def lane_centerline(self, lane_id: str) -> list[Vec3]:
        lane = self.lanes.get(lane_id)
        return [list(p) for p in lane.centerline] if lane else []

    def is_blocked(self, position: Vec3) -> bool:
        for pos, radius in self._blocked:
            if dist_xz(position, pos) <= radius:
                return True
        return False

    # ---- convenience for planners / behavior ------------------------------ #
    def lane(self, lane_id: str) -> Optional[Lane]:
        return self.lanes.get(lane_id)

    def lane_length(self, lane_id: str) -> float:
        lane = self.lanes.get(lane_id)
        return lane.length if lane else 0.0

    def all_lane_ids(self) -> list[str]:
        return list(self.lanes.keys())

    def nearest_lane(self, position: Vec3, heading: float | None = None,
                     candidate_ids: Iterable[str] | None = None) -> Optional[str]:
        """Best matching lane by distance and, when supplied, heading.

        ``candidate_ids`` lets a simulator preserve graph continuity at an
        intersection instead of jumping to an unrelated overlapping lane.
        Calls from planners intentionally omit both optional arguments.
        """
        candidates = (self.lanes.values() if candidate_ids is None else
                      (self.lanes[lid] for lid in candidate_ids
                       if lid in self.lanes))
        best_id, best_score = None, math.inf
        for lane in candidates:
            _, lat, arc = lane.closest_point(position)
            score = lat
            if heading is not None:
                delta = abs((heading - lane.heading_at_arc(arc) + 180.0)
                            % 360.0 - 180.0)
                score += delta / 30.0
            if score < best_score:
                best_score, best_id = score, lane.id
        return best_id

    def block(self, position: Vec3, radius: float = 1.0) -> None:
        self._blocked.append((list(position), radius))

    def clear_blocks(self) -> None:
        self._blocked.clear()


# --------------------------------------------------------------------------- #
# Dynamic world (per-tick)
# --------------------------------------------------------------------------- #
@dataclass
class DynamicVehicle:
    id: str
    position: Vec3
    velocity: Vec3
    heading: float
    current_lane: str
    type: str = "car"
    acceleration: Vec3 = field(default_factory=lambda: [0.0, 0.0, 0.0])
    target_lane: Optional[str] = None
    maneuver: str = "straight"
    behavior_state: Optional[str] = None
    has_goal: bool = False
    goal: Optional[Vec3] = None

    @property
    def speed(self) -> float:
        return math.sqrt(sum(c * c for c in self.velocity))


@dataclass
class DynamicObject:
    id: str
    type: str
    position: Vec3
    velocity: Vec3
    radius: float = 0.4

    @property
    def speed(self) -> float:
        return math.sqrt(sum(c * c for c in self.velocity))


class WorldModel:
    """Holds the lane network + the most recent dynamic snapshot from Unity."""

    def __init__(self, network: LaneNetwork):
        self.network = network
        self.time: float = 0.0
        self.tick: int = -1
        self.scenario: str = network.scenario
        self.vehicles: dict[str, DynamicVehicle] = {}
        self.objects: dict[str, DynamicObject] = {}
        self.events: list[dict] = []

    def update_from_state(self, state: dict) -> None:
        """Ingest a schema-valid StateMessage dict."""
        self.time = state.get("time", self.time)
        self.tick = state.get("tick", self.tick)
        if state.get("scenario"):
            self.scenario = state["scenario"]

        self.vehicles = {}
        for v in state.get("vehicles", []):
            self.vehicles[v["id"]] = DynamicVehicle(
                id=v["id"],
                position=list(v["position"]),
                velocity=list(v["velocity"]),
                heading=v.get("heading", 0.0),
                current_lane=v.get("current_lane", ""),
                type=v.get("type", "car"),
                acceleration=list(v.get("acceleration", [0.0, 0.0, 0.0])),
                target_lane=v.get("target_lane"),
                maneuver=v.get("maneuver", "straight"),
                behavior_state=v.get("behavior_state"),
                has_goal=bool(v.get("has_goal", False)),
                goal=list(v["goal"]) if v.get("goal") is not None else None,
            )

        self.objects = {}
        for o in state.get("objects", []):
            self.objects[o["id"]] = DynamicObject(
                id=o["id"],
                type=o["type"],
                position=list(o["position"]),
                velocity=list(o["velocity"]),
                radius=o.get("radius", 0.4),
            )

        self.events = list(state.get("events", []))
        self._apply_events()

    def _apply_events(self) -> None:
        """Translate hazard events into (a) blocked lane-graph positions so the
        planner reroutes, and (b) static obstacles so collision prediction
        stops vehicles that can't reroute (e.g. a single-lane road)."""
        self.network.clear_blocks()
        for i, ev in enumerate(self.events):
            etype = ev.get("type")
            pos = ev.get("position")
            if pos is None:
                continue
            if etype in ("FallingObject", "VehicleBreakdown", "ConstructionZone"):
                self.network.block(pos, radius=2.0)
            # point hazards are also physical obstacles to predict against
            if etype in ("FallingObject", "VehicleBreakdown"):
                oid = f"_hazard_{etype}_{i}"
                payload = ev.get("payload") or {}
                self.objects[oid] = DynamicObject(
                    id=oid, type="unexpected_obstacle", position=list(pos),
                    velocity=[0.0, 0.0, 0.0], radius=payload.get("radius", 1.0))

    def vehicle(self, vehicle_id: str) -> Optional[DynamicVehicle]:
        return self.vehicles.get(vehicle_id)
