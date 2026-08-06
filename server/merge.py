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
MIN_MERGE_SPEED = 5.0    # m/s, below this the ramp car is crawling, not merging
AT_MERGE_POINT = 1.0     # m, close enough that retiming is meaningless


@dataclass
class MergePlan:
    ramp_id: str
    feasible: bool
    ramp_target_speed: float
    reason: str = ""
    yield_vehicle: Optional[str] = None      # mainline car asked to open a gap
    yield_target_speed: Optional[float] = None
    slot_eta: float = math.inf               # when the ramp car should arrive


def _travel_direction(v: DynamicVehicle) -> tuple[float, float]:
    """Unit heading of travel in the xz plane.

    Velocity is the truth while the vehicle is moving; a stopped car still has
    a heading, and we need one to tell "waiting before the merge" apart from
    "already through it"."""
    if v.speed > MIN_SPEED:
        return v.velocity[0] / v.speed, v.velocity[2] / v.speed
    rad = math.radians(v.heading)
    return math.sin(rad), math.cos(rad)


def _distance_to(position, merge_point) -> float:
    return math.hypot(merge_point[0] - position[0], merge_point[2] - position[2])


def _approach_eta(v: DynamicVehicle, merge_point) -> Optional[float]:
    """Seconds until ``v`` reaches ``merge_point``, or ``None`` if it never will.

    Distance alone is unsigned, so a car that has already cleared the merge
    point looks exactly like one the same distance short of it. Projecting onto
    the direction of travel keeps departed traffic out of the slot arithmetic —
    otherwise the ramp car reserves around phantom arrivals that are, in fact,
    already downstream and irrelevant.
    """
    dx = merge_point[0] - v.position[0]
    dz = merge_point[2] - v.position[2]
    ux, uz = _travel_direction(v)
    ahead = dx * ux + dz * uz
    if ahead < 0.0:
        return None                      # already past the merge point
    return ahead / max(v.speed, MIN_SPEED)


def plan_merge(ramp: DynamicVehicle, mainline: Iterable[DynamicVehicle],
               merge_point, required_gap_s: float = REQUIRED_GAP_S,
               desired_speed: Optional[float] = None,
               min_merge_speed: float = MIN_MERGE_SPEED) -> MergePlan:
    """Reserve a slot for ``ramp`` to merge at ``merge_point``.

    ``desired_speed`` is the speed the ramp car would hold if the mainline were
    empty (normally its lane's speed limit). Every slot is evaluated against
    that, never against the ramp car's *current* speed: planning off the current
    speed makes the reservation self-fulfilling — a car stopped at the ramp head
    is told to keep doing 0 m/s, which keeps its ETA infinite, which keeps the
    plan telling it to hold. Planning off the intended speed breaks that loop.
    """
    if desired_speed is None:
        desired_speed = max(ramp.speed, min_merge_speed)
    desired_speed = max(desired_speed, min_merge_speed)

    ramp_distance = _distance_to(ramp.position, merge_point)
    if ramp_distance <= AT_MERGE_POINT:
        # Already at the join — there is no room left to retime into.
        return MergePlan(ramp.id, True, desired_speed, "at merge point",
                         slot_eta=0.0)
    ramp_eta = ramp_distance / desired_speed

    # ETAs of mainline cars still approaching the merge point, in time order
    arrivals = sorted(
        ((eta, m) for eta, m in ((_approach_eta(m, merge_point), m) for m in mainline)
         if eta is not None),
        key=lambda t: t[0])

    if not arrivals:
        return MergePlan(ramp.id, True, desired_speed, "clear mainline",
                         slot_eta=ramp_eta)

    # candidate slots: before first, between each pair, after last
    times = [t for t, _ in arrivals]

    # 1) is the ramp ETA already inside a big-enough gap?
    for lo, hi in _slots(times):
        if hi - lo >= required_gap_s and lo + required_gap_s / 2 <= ramp_eta <= hi - required_gap_s / 2:
            return MergePlan(ramp.id, True, desired_speed, "fits existing gap",
                             slot_eta=ramp_eta)

    # 2) find the nearest naturally-large gap and retime the ramp car into it.
    #    A slot only counts if the ramp car can actually make it at a speed that
    #    is both legal and not a crawl — "arrive in 90 s" is not a merge plan.
    best_slot_time, best_target, best_cost = None, None, math.inf
    for lo, hi in _slots(times):
        if hi - lo < required_gap_s:
            continue
        center = _clamp_slot(lo, hi, ramp_eta, required_gap_s)
        if center is None or center <= 0.0:
            continue
        target = ramp_distance / center
        if not (min_merge_speed <= target <= desired_speed):
            continue
        cost = abs(center - ramp_eta)
        if cost < best_cost:
            best_cost, best_slot_time, best_target = cost, center, target

    if best_slot_time is not None:
        return MergePlan(ramp.id, True, best_target, "retimed into gap",
                         slot_eta=best_slot_time)

    # 3) no reachable gap: ask the mainline car that arrives just after the ramp
    #    to slow and open one behind it. This is the central-control move — a
    #    ramp car on its own could only wait.
    after = [(t, m) for t, m in arrivals if t >= ramp_eta]
    if after:
        slot_time = ramp_eta
        follower = after[0][1]
        # follower should arrive required_gap_s after the ramp car
        yield_speed = _distance_to(follower.position, merge_point) / (
            slot_time + required_gap_s)
        if yield_speed < follower.speed:
            return MergePlan(ramp.id, True, desired_speed,
                             "opened gap via mainline yield",
                             yield_vehicle=follower.id,
                             yield_target_speed=max(0.0, yield_speed),
                             slot_eta=slot_time)

    return MergePlan(ramp.id, False, 0.0, "no feasible slot")


def _slots(times: list[float]):
    """Candidate (lo, hi) arrival windows: before the first car, between each
    consecutive pair, and after the last."""
    for i in range(len(times) + 1):
        lo = times[i - 1] if i > 0 else -math.inf
        hi = times[i] if i < len(times) else math.inf
        yield lo, hi


def _clamp_slot(lo: float, hi: float, desired: float,
                gap: float) -> Optional[float]:
    """Nearest arrival time to ``desired`` that keeps ``gap/2`` clear of both
    neighbours. An open-ended side imposes no bound — it must not fall back to
    ``desired``, which would silently cancel the bound on the *other* side."""
    lo_ok = (lo + gap / 2) if math.isfinite(lo) else -math.inf
    hi_ok = (hi - gap / 2) if math.isfinite(hi) else math.inf
    if lo_ok > hi_ok:
        return None
    return max(lo_ok, min(hi_ok, desired))
