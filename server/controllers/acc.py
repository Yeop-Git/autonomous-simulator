"""Adaptive Cruise Control — longitudinal policy.

Given the ego's speed, a desired (free-flow) speed, and the gap + speed of a
leader (if any), produce a target speed that keeps a safe time-headway gap.
Pure function of its inputs; no world/Unity state, so it is unit-testable
with synthetic numbers.

Model: constant time-gap spacing policy with a linear feedback law,
clamped by comfortable accel/decel limits. This is the standard "intelligent
driver"-style behaviour without the full IDM nonlinearity — enough to keep
gaps and avoid rear-ends in the central-control demo.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class ACCParams:
    time_gap: float = 2.0          # s, desired headway to leader
    standstill_gap: float = 6.0    # m, min bumper gap at rest
    max_accel: float = 2.0         # m/s^2
    max_decel: float = 4.0         # m/s^2 (comfortable)
    emergency_decel: float = 8.0   # m/s^2 (hard)
    kp_gap: float = 0.4            # gap error -> speed adjust
    kp_speed: float = 0.6          # leader speed matching
    stopped_leader_speed: float = 0.2  # m/s, consider the queue stationary
    stop_gap_tolerance: float = 0.5    # m, settle instead of creeping forever


class ACCController:
    name = "acc"

    def __init__(self, params: ACCParams | None = None):
        self.p = params or ACCParams()

    def desired_gap(self, ego_speed: float) -> float:
        return self.p.standstill_gap + self.p.time_gap * ego_speed

    def safe_speed(self, leader_gap: float, leader_speed: float) -> float:
        """Highest speed from which we can still stop before closing the gap
        to the standstill minimum, under the *equal-braking* assumption.

        Derived from v^2 = v_leader^2 + 2*a*effective_gap. This is the standard
        safe-distance model: it is collision-free as long as BOTH vehicles
        brake at no more than ``max_decel``. It does NOT protect against a
        leader that out-brakes us (e.g. hits a wall and stops instantly) — for
        that, drop the ``leader_speed^2`` term to assume the leader can stop
        immediately. We keep the equal-braking form for realistic flow.
        """
        effective = leader_gap - self.p.standstill_gap
        if effective <= 0.0:
            return 0.0
        return math.sqrt(leader_speed * leader_speed
                         + 2.0 * self.p.max_decel * effective)

    def target_speed(self, ego_speed: float, free_speed: float,
                     leader_gap: float | None, leader_speed: float | None,
                     dt: float = 0.1) -> float:
        """Return a commanded speed (m/s) for the next step.

        ``leader_gap`` is bumper-to-bumper distance (m); ``None`` means no
        leader (free flow). The result is clamped by accel/decel limits.
        """
        if leader_gap is None or leader_speed is None:
            desired = free_speed
        else:
            # Once the protected standstill envelope is entered the desired
            # endpoint is a full stop.  Unity still tracks this command under
            # its configured deceleration limit, so the vehicle stops smoothly
            # instead of having its velocity snapped to zero.
            if (leader_gap <= self.p.standstill_gap
                    or (leader_speed <= self.p.stopped_leader_speed
                        and leader_gap <= (self.p.standstill_gap
                                           + self.p.stop_gap_tolerance))):
                return 0.0
            # kinematic safe speed (collision-free) is the hard ceiling...
            v_safe = self.safe_speed(leader_gap, leader_speed)
            # ...the comfort time-gap term keeps a roomier following distance.
            gap_err = leader_gap - self.desired_gap(ego_speed)
            v_comfort = leader_speed + self.p.kp_gap * gap_err
            desired = min(free_speed, v_safe, max(v_comfort, 0.0))

        desired = max(0.0, desired)
        return self._clamp_rate(ego_speed, desired, leader_gap, dt)

    def _clamp_rate(self, ego_speed: float, desired: float,
                    leader_gap: float | None, dt: float) -> float:
        dv = desired - ego_speed
        if dv >= 0:
            dv = min(dv, self.p.max_accel * dt)
        else:
            limit = self.p.max_decel
            if leader_gap is not None and leader_gap < self.p.standstill_gap:
                limit = self.p.emergency_decel
            dv = max(dv, -limit * dt)
        return max(0.0, ego_speed + dv)
