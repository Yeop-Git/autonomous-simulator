"""Highway on-ramp merge reservation (plan §13.3).

The central-control advantage in one scenario: a ramp vehicle wants onto the
mainline. The server knows every mainline vehicle's exact state, so instead of
the ramp car guessing, the server:

  1. computes each vehicle's ETA to the merge point,
  2. finds (or opens) a time gap in the mainline stream big enough to host the
     ramp car,
  3. issues speed targets — nudging the ramp car (and, if needed, the mainline
     follower that would close the gap) — so the merge completes without a
     hard brake.

Pure & testable: positions/speeds in, a ``MergePlan`` of recommended speeds out.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

from world_model import DynamicVehicle

MIN_SPEED = 1.0          # m/s, floor to avoid div-by-zero ETA
REQUIRED_GAP_S = 2.0     # s of time headway the ramp car needs in the stream


@dataclass
class MergePlan:
    ramp_id: str
    feasible: bool
    ramp_target_speed: float
    reason: str = ""
    yield_vehicle: Optional[str] = None      # mainline car asked to open a gap
    yield_target_speed: Optional[float] = None
    slot_eta: float = math.inf               # when the ramp car should arrive


def _eta(position, speed: float, merge_point) -> float:
    d = math.hypot(merge_point[0] - position[0], merge_point[2] - position[2])
    return d / max(speed, MIN_SPEED)


def plan_merge(ramp: DynamicVehicle, mainline: Iterable[DynamicVehicle],
               merge_point, required_gap_s: float = REQUIRED_GAP_S) -> MergePlan:
    """Reserve a slot for ``ramp`` to merge at ``merge_point``."""
    main = list(mainline)
    ramp_eta = _eta(ramp.position, ramp.speed, merge_point)

    # ETAs of mainline cars at the merge point, sorted in time order
    arrivals = sorted(((_eta(m.position, m.speed, merge_point), m) for m in main),
                      key=lambda t: t[0])

    if not arrivals:
        return MergePlan(ramp.id, True, ramp.speed, "clear mainline", slot_eta=ramp_eta)

    # candidate slots: before first, between each pair, after last
    times = [t for t, _ in arrivals]

    # 1) is the ramp ETA already inside a big-enough gap?
    for i in range(len(times) + 1):
        lo = times[i - 1] if i > 0 else -math.inf
        hi = times[i] if i < len(times) else math.inf
        if hi - lo >= required_gap_s and lo + required_gap_s / 2 <= ramp_eta <= hi - required_gap_s / 2:
            return MergePlan(ramp.id, True, ramp.speed, "fits existing gap",
                             slot_eta=ramp_eta)

    # 2) find the nearest naturally-large gap and retime the ramp car into it
    best_slot_time, best_cost = None, math.inf
    for i in range(len(times) + 1):
        lo = times[i - 1] if i > 0 else -math.inf
        hi = times[i] if i < len(times) else math.inf
        if hi - lo < required_gap_s:
            continue
        center = _clamp_slot(lo, hi, ramp_eta, required_gap_s)
        cost = abs(center - ramp_eta)
        if cost < best_cost:
            best_cost, best_slot_time = cost, center

    if best_slot_time is not None:
        target = _speed_for_eta(ramp.position, merge_point, best_slot_time)
        return MergePlan(ramp.id, True, target, "retimed into gap",
                         slot_eta=best_slot_time)

    # 3) no natural gap: ask the mainline car that arrives just after the ramp
    #    to slow and open one behind it.
    after = [(t, m) for t, m in arrivals if t >= ramp_eta]
    if after:
        slot_time = ramp_eta
        follower = after[0][1]
        # follower should arrive required_gap_s after the ramp car
        yield_speed = _speed_for_eta(follower.position, merge_point,
                                     slot_time + required_gap_s)
        return MergePlan(ramp.id, True, ramp.speed, "opened gap via mainline yield",
                         yield_vehicle=follower.id, yield_target_speed=yield_speed,
                         slot_eta=slot_time)

    return MergePlan(ramp.id, False, 0.0, "no feasible slot")


def _clamp_slot(lo: float, hi: float, desired: float, gap: float) -> float:
    lo_ok = (lo + gap / 2) if math.isfinite(lo) else desired
    hi_ok = (hi - gap / 2) if math.isfinite(hi) else desired
    return max(lo_ok, min(hi_ok, desired))


def _speed_for_eta(position, merge_point, eta: float) -> float:
    d = math.hypot(merge_point[0] - position[0], merge_point[2] - position[2])
    return d / eta if eta > 1e-6 else 0.0
