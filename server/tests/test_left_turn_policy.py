from left_turn import (
    ABORT_STRAIGHT, CONTINUE_LANE_CHANGE, ENTER_INTERSECTION,
    EXIT_LANE, SOURCE_LANE, TARGET_APPROACH, WAIT_INTERSECTION,
    WAIT_LANE_CHANGE, LeftTurnCommitment, LeftTurnInputs, evaluate,
)


def inp(**kwargs):
    values = dict(location=SOURCE_LANE)
    values.update(kwargs)
    return LeftTurnInputs(**values)


def test_priority_rechecks_current_emergency_before_lane_change():
    decision, memory = evaluate(inp(emergency=True, lane_change_safe=True))
    assert decision.action == WAIT_LANE_CHANGE
    assert decision.stop_now
    assert not memory.lane_change_started


def test_started_change_keeps_lateral_commitment_when_hazard_appears():
    memory = LeftTurnCommitment(lane_change_started=True)
    decision, next_memory = evaluate(
        inp(emergency=True, lane_change_safe=False), memory)
    assert decision.action == CONTINUE_LANE_CHANGE
    assert decision.stop_now
    assert next_memory.lane_change_started


def test_late_closed_gap_commits_to_straight_abort():
    decision, memory = evaluate(inp(lane_change_deadline_reached=True))
    assert decision.action == ABORT_STRAIGHT
    assert memory.aborted


def test_green_permission_is_revoked_next_tick_when_exit_blocks():
    clear = inp(location=TARGET_APPROACH, at_entry_gate=True)
    decision, memory = evaluate(clear)
    assert decision.action == ENTER_INTERSECTION

    blocked = inp(location=TARGET_APPROACH, at_entry_gate=True,
                  entry_clear=False)
    decision, _ = evaluate(blocked, memory)
    assert decision.action == WAIT_INTERSECTION
    assert decision.stop_now
