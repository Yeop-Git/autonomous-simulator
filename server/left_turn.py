"""Per-tick protected-left priority policy.

Unlike a transition-driven FSM, this policy re-evaluates the complete world
snapshot every tick.  ``phase`` is an observable result used to constrain the
path sent to Unity; it does not force the next result.  Only two commitments
survive between ticks: an already-started lateral move and a late-turn abort.
"""
from __future__ import annotations

from dataclasses import dataclass


WAIT_LANE_CHANGE = "wait_lane_change"
START_LANE_CHANGE = "start_lane_change"
CONTINUE_LANE_CHANGE = "continue_lane_change"
APPROACH_STOP_LINE = "approach_stop_line"
WAIT_INTERSECTION = "wait_intersection"
ENTER_INTERSECTION = "enter_intersection"
CROSS_INTERSECTION = "cross_intersection"
ALIGN_EXIT = "align_exit"
COMPLETE = "complete"
ABORT_STRAIGHT = "abort_straight"

SOURCE_LANE = "source_lane"
TARGET_APPROACH = "target_approach"
TURN_CONNECTOR = "turn_connector"
EXIT_LANE = "exit_lane"
OTHER = "other"


@dataclass(frozen=True)
class LeftTurnCommitment:
    lane_change_started: bool = False
    aborted: bool = False


@dataclass(frozen=True)
class LeftTurnInputs:
    location: str
    emergency: bool = False
    lane_change_safe: bool = False
    lane_change_deadline_reached: bool = False
    signal_requires_stop: bool = False
    entry_clear: bool = True
    at_entry_gate: bool = False
    exit_aligned: bool = False


@dataclass(frozen=True)
class LeftTurnDecision:
    action: str
    phase: str
    turn_signal: str = "left"
    stop_now: bool = False
    controlled_signal_stop: bool = False


def evaluate(inp: LeftTurnInputs,
             memory: LeftTurnCommitment = LeftTurnCommitment()
             ) -> tuple[LeftTurnDecision, LeftTurnCommitment]:
    """Return this tick's highest-priority action and next commitment."""
    if memory.aborted:
        return (LeftTurnDecision(ABORT_STRAIGHT, "AbortedStraight", "none"),
                memory)

    if inp.location == EXIT_LANE:
        if inp.exit_aligned:
            return (LeftTurnDecision(COMPLETE, "Completed", "none"),
                    LeftTurnCommitment())
        return (LeftTurnDecision(ALIGN_EXIT, "ExitAligning"),
                LeftTurnCommitment())

    if inp.location == TURN_CONNECTOR:
        return (LeftTurnDecision(
                    CROSS_INTERSECTION, "IntersectionCrossing",
                    stop_now=inp.emergency),
                LeftTurnCommitment())

    if inp.location == TARGET_APPROACH:
        # Safety is re-evaluated every tick; a previous green/clear result does
        # not grant permanent permission to enter.
        if inp.emergency or not inp.entry_clear:
            return (LeftTurnDecision(
                        WAIT_INTERSECTION, "SignalWaiting", stop_now=True),
                    LeftTurnCommitment())
        if inp.signal_requires_stop:
            return (LeftTurnDecision(
                        WAIT_INTERSECTION, "SignalWaiting",
                        controlled_signal_stop=True),
                    LeftTurnCommitment())
        if not inp.at_entry_gate:
            return (LeftTurnDecision(APPROACH_STOP_LINE, "ApproachStopLine"),
                    LeftTurnCommitment())
        return (LeftTurnDecision(ENTER_INTERSECTION, "IntersectionEntry"),
                LeftTurnCommitment())

    if inp.location == SOURCE_LANE:
        if memory.lane_change_started:
            # Do not reverse a partially executed lateral manoeuvre.  A new
            # hazard changes longitudinal speed, not lateral direction.
            return (LeftTurnDecision(
                        CONTINUE_LANE_CHANGE, "LaneChanging",
                        stop_now=inp.emergency),
                    memory)
        if inp.emergency:
            return (LeftTurnDecision(
                        WAIT_LANE_CHANGE, "LaneChangeWaiting", stop_now=True),
                    memory)
        if inp.lane_change_deadline_reached and not inp.lane_change_safe:
            aborted = LeftTurnCommitment(aborted=True)
            return (LeftTurnDecision(
                        ABORT_STRAIGHT, "AbortedStraight", "none"), aborted)
        if inp.lane_change_safe:
            started = LeftTurnCommitment(lane_change_started=True)
            return (LeftTurnDecision(START_LANE_CHANGE, "LaneChanging"),
                    started)
        return (LeftTurnDecision(WAIT_LANE_CHANGE, "LaneChangeWaiting"),
                memory)

    return (LeftTurnDecision(COMPLETE, "Completed", "none"),
            LeftTurnCommitment())
