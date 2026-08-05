"""Behavior layer — leader detection and the vehicle finite-state machine.

Sits between collision prediction and the controllers. Two pieces:

  * ``find_leader`` — given an ego vehicle and the others, find the nearest
    vehicle ahead in the *same* lane (by arc length along the centerline) and
    the bumper-to-bumper gap to it. This is what ACC follows.
  * ``next_behavior`` — the FSM transition (plan §12.2). It picks a behavior
    label from goal/leader/conflict/event inputs. Kept as a pure function so
    its decision table is directly testable.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional

from world_model import DynamicVehicle, LaneNetwork

VEHICLE_LENGTH = 4.5  # m, used to convert center spacing to bumper gap

# Behavior labels accepted by the command schema.
LANE_KEEPING = "LaneKeeping"
FOLLOWING = "Following"
LANE_CHANGING = "LaneChanging"
STOPPING = "Stopping"
EMERGENCY_BRAKING = "EmergencyBraking"
WAITING = "WaitingAtIntersection"
ARRIVED = "Arrived"


@dataclass
class Leader:
    vehicle: DynamicVehicle
    gap: float          # bumper-to-bumper distance (m), >= 0
    speed: float        # leader speed (m/s)


def _arc_along_lane(network: LaneNetwork, lane_id: str, position) -> Optional[float]:
    lane = network.lane(lane_id)
    if lane is None:
        return None
    _, _, arc = lane.closest_point(position)
    return arc


def find_leader(ego: DynamicVehicle, others: Iterable[DynamicVehicle],
                network: LaneNetwork, max_range: float = 80.0) -> Optional[Leader]:
    """Nearest vehicle ahead of ego, within ``max_range`` of forward travel.

    Searches the ego's own lane AND downstream lanes reachable via
    ``next_lane_ids`` — so a leader that has already crossed onto the next lane
    segment (long roads, merges, intersection approaches) is still followed.
    """
    if not ego.current_lane:
        return None
    ego_lane = network.lane(ego.current_lane)
    ego_arc = _arc_along_lane(network, ego.current_lane, ego.position)
    if ego_lane is None or ego_arc is None:
        return None

    # bucket the other vehicles by lane for cheap per-lane lookups
    other_vehicles = list(others)
    by_lane: dict[str, list[DynamicVehicle]] = {}
    for other in other_vehicles:
        if other.id == ego.id or not other.current_lane:
            continue
        by_lane.setdefault(other.current_lane, []).append(other)

    best: Optional[Leader] = None

    def consider(other: DynamicVehicle, center_gap: float) -> None:
        nonlocal best
        if center_gap < 0 or center_gap > max_range:
            return
        gap = max(0.0, center_gap - VEHICLE_LENGTH)
        if best is None or gap < best.gap:
            best = Leader(vehicle=other, gap=gap, speed=other.speed)

    # 1) same lane, strictly ahead
    for other in by_lane.get(ego.current_lane, []):
        other_arc = _arc_along_lane(network, other.current_lane, other.position)
        if other_arc is not None and other_arc > ego_arc:
            consider(other, other_arc - ego_arc)

    # 2) downstream lanes: distance = remaining-on-ego-lane + arc-on-that-lane
    ego_remaining = ego_lane.length - ego_arc
    visited = {ego.current_lane}
    frontier: list[tuple[str, float]] = [
        (nxt, ego_remaining) for nxt in ego_lane.next_lane_ids
    ]
    while frontier:
        lane_id, base = frontier.pop()
        if lane_id in visited or base > max_range:
            continue
        visited.add(lane_id)
        lane = network.lane(lane_id)
        if lane is None:
            continue
        for other in by_lane.get(lane_id, []):
            other_arc = _arc_along_lane(network, lane_id, other.position)
            if other_arc is not None:
                consider(other, base + other_arc)
        for nxt in lane.next_lane_ids:
            frontier.append((nxt, base + lane.length))

    # Around a junction, lane classification can change one tick earlier for
    # the follower than for its leader (or briefly select an overlapping
    # connector). The graph-only search then cannot see the leader because it
    # appears to be on a predecessor/unrelated lane. Keep a narrow physical
    # forward-corridor fallback so a transient lane tag cannot disable ACC.
    # Heading agreement rejects perpendicular crossing traffic, which remains
    # the collision predictor's responsibility.
    rad = math.radians(ego.heading)
    forward_x, forward_z = math.sin(rad), math.cos(rad)
    corridor_half_width = max(2.0, ego_lane.width * 0.75)
    fallback_lane_ids = {ego.current_lane}
    fallback_lane_ids.update(
        lane.id for lane in network.lanes.values()
        if ego.current_lane in lane.next_lane_ids)
    for other in other_vehicles:
        if other.id == ego.id or other.current_lane not in fallback_lane_ids:
            continue
        heading_delta = abs((other.heading - ego.heading + 180.0) % 360.0 - 180.0)
        if heading_delta > 45.0:
            continue
        dx = other.position[0] - ego.position[0]
        dz = other.position[2] - ego.position[2]
        longitudinal = dx * forward_x + dz * forward_z
        lateral = abs(dx * forward_z - dz * forward_x)
        if longitudinal > 0.0 and lateral <= corridor_half_width:
            consider(other, longitudinal)
    return best


@dataclass
class BehaviorInputs:
    has_goal: bool
    arrived: bool
    route_found: bool
    leader: Optional[Leader]
    min_ttc: float          # soonest NON-leader conflict TTC (drives Stopping)
    min_ttc_emergency: Optional[float] = None  # soonest of ALL conflicts incl.
                            # leader (drives EmergencyBraking); defaults to min_ttc
    follow_gap_trigger: float = 30.0  # m; closer leader => Following
    emergency_ttc: float = 1.5        # s; below => EmergencyBraking
    caution_ttc: float = 3.0          # s; below => Stopping/slow


def next_behavior(inp: BehaviorInputs) -> str:
    """FSM transition: choose a behavior label from the current situation.

    Priority: emergency (any conflict, incl. an imminent rear-end) > arrival >
    routing > caution-stop (non-leader conflicts only — steady following is not
    a stop) > following > cruising.
    """
    emergency_ttc = inp.min_ttc_emergency if inp.min_ttc_emergency is not None else inp.min_ttc
    if emergency_ttc <= inp.emergency_ttc:
        return EMERGENCY_BRAKING
    if inp.has_goal and inp.arrived:
        return ARRIVED
    if inp.has_goal and not inp.route_found:
        return STOPPING
    if inp.min_ttc <= inp.caution_ttc:
        return STOPPING
    if inp.leader is not None and inp.leader.gap <= inp.follow_gap_trigger:
        return FOLLOWING
    return LANE_KEEPING
