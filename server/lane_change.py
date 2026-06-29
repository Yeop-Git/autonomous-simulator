"""Lane-change gap acceptance with collision prediction (plan §6.2, §12.4).

Central control decides whether a vehicle may move into an adjacent lane. The
test: project the ego onto the target lane, find the would-be leader (ahead)
and follower (behind) there, and accept only if BOTH the forward gap and the
rearward gap exceed speed-dependent safety margins AND no predicted conflict
breaches within the horizon.

Pure: feed it dynamic vehicles + the lane network, get a ``LaneChangeDecision``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional

from collision_predictor import CollisionPredictor
from world_model import DynamicVehicle, LaneNetwork

VEHICLE_LENGTH = 4.5
STANDSTILL_GAP = 4.0
TIME_GAP = 1.0  # s of headway required fore and aft to accept a change


@dataclass
class LaneChangeDecision:
    accept: bool
    target_lane: str
    reason: str = ""
    lead_gap: float = math.inf   # bumper gap to vehicle ahead in target lane
    lag_gap: float = math.inf    # bumper gap to vehicle behind in target lane
    lead_id: Optional[str] = None
    lag_id: Optional[str] = None


def _arc(network: LaneNetwork, lane_id: str, position) -> Optional[float]:
    lane = network.lane(lane_id)
    if lane is None:
        return None
    _, _, arc = lane.closest_point(position)
    return arc


def evaluate(ego: DynamicVehicle, target_lane: str,
             others: Iterable[DynamicVehicle], network: LaneNetwork,
             predictor: Optional[CollisionPredictor] = None) -> LaneChangeDecision:
    """Decide whether ``ego`` may change into ``target_lane``."""
    if network.lane(target_lane) is None:
        return LaneChangeDecision(False, target_lane, reason="no such lane")

    ego_arc = _arc(network, target_lane, ego.position)
    if ego_arc is None:
        return LaneChangeDecision(False, target_lane, reason="ego off target lane")

    lead_gap, lead = math.inf, None
    lag_gap, lag = math.inf, None
    for o in others:
        if o.id == ego.id or o.current_lane != target_lane:
            continue
        o_arc = _arc(network, target_lane, o.position)
        if o_arc is None:
            continue
        delta = o_arc - ego_arc
        gap = abs(delta) - VEHICLE_LENGTH
        if delta >= 0 and gap < lead_gap:
            lead_gap, lead = gap, o
        elif delta < 0 and gap < lag_gap:
            lag_gap, lag = gap, o

    # speed-dependent required gaps
    req_lead = STANDSTILL_GAP + TIME_GAP * ego.speed
    req_lag = STANDSTILL_GAP + TIME_GAP * (lag.speed if lag else 0.0)

    if lead_gap < req_lead:
        return LaneChangeDecision(False, target_lane, "lead gap too small",
                                  lead_gap, lag_gap,
                                  lead.id if lead else None, lag.id if lag else None)
    if lag_gap < req_lag:
        return LaneChangeDecision(False, target_lane, "lag gap too small",
                                  lead_gap, lag_gap,
                                  lead.id if lead else None, lag.id if lag else None)

    # predictive check: would entering the lane breach safety with lead/lag?
    predictor = predictor or CollisionPredictor()
    for other in (lead, lag):
        if other is None:
            continue
        conflict = predictor.pair_conflict(ego, other)
        if conflict is not None and conflict.ttc <= predictor.horizon:
            return LaneChangeDecision(False, target_lane,
                                      f"predicted conflict with {other.id}",
                                      lead_gap, lag_gap,
                                      lead.id if lead else None,
                                      lag.id if lag else None)

    return LaneChangeDecision(True, target_lane, "accepted", lead_gap, lag_gap,
                              lead.id if lead else None, lag.id if lag else None)
