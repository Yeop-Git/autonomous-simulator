"""Unit tests for the sampling-based planners RRT / RRT* (no Unity).

Uses a bare ``LaneNetwork`` purely as a ``World`` that answers ``is_blocked``
(these planners work in continuous free space and ignore the lane graph).
"""
import math

import pytest

from planners import RRTConfig, RRTPlanner, RRTStarPlanner
from planners._rrt_common import collision_free
from world_model import LaneNetwork, dist_xz


def free_world():
    """Empty world: nothing is ever blocked."""
    return LaneNetwork([])


def wall_world(x_center, gap_half=None):
    """A wall of obstacles across +X at z=25. If ``gap_half`` is given, leave a
    passable gap of that half-width centered on x_center; otherwise it's solid
    across a very wide span (unreachable)."""
    net = LaneNetwork([])
    for x in range(-80, 81, 2):
        if gap_half is not None and abs(x - x_center) <= gap_half:
            continue
        net.block([float(x), 0.0, 25.0], radius=1.5)
    return net


def _config(seed=0, **kw):
    return RRTConfig(seed=seed, **kw)


@pytest.mark.parametrize("Planner", [RRTPlanner, RRTStarPlanner])
def test_straight_shot_free_space(Planner):
    planner = Planner(_config())
    path = planner.plan([0.0, 0.0, 0.0], [0.0, 0.0, 40.0], free_world())
    assert path, "expected a path in free space"
    assert path[0] == pytest.approx([0.0, 0.0, 0.0])
    assert dist_xz(path[-1], [0.0, 0.0, 40.0]) < 1e-6
    assert planner.last_cost == pytest.approx(40.0, abs=0.5)


@pytest.mark.parametrize("Planner", [RRTPlanner, RRTStarPlanner])
def test_endpoints_and_collision_free(Planner):
    world = wall_world(x_center=0.0, gap_half=6.0)
    planner = Planner(_config(max_iters=4000))
    start, goal = [0.0, 0.0, 0.0], [0.0, 0.0, 50.0]
    path = planner.plan(start, goal, world)
    assert path, "expected a detour path through the gap"
    assert dist_xz(path[0], start) < 1e-6
    assert dist_xz(path[-1], goal) < 1e-6
    # every leg is collision-free
    for a, b in zip(path, path[1:]):
        assert collision_free(a, b, world, 1.0)


@pytest.mark.parametrize("Planner", [RRTPlanner, RRTStarPlanner])
def test_unreachable_returns_empty(Planner):
    world = wall_world(x_center=0.0, gap_half=None)  # solid wall
    planner = Planner(_config(max_iters=1500, margin=10.0))
    path = planner.plan([0.0, 0.0, 0.0], [0.0, 0.0, 50.0], world)
    assert path == []


@pytest.mark.parametrize("Planner", [RRTPlanner, RRTStarPlanner])
def test_blocked_endpoint_returns_empty(Planner):
    world = free_world()
    world.block([0.0, 0.0, 50.0], radius=3.0)  # goal sits inside an obstacle
    planner = Planner(_config())
    assert planner.plan([0.0, 0.0, 0.0], [0.0, 0.0, 50.0], world) == []


@pytest.mark.parametrize("Planner", [RRTPlanner, RRTStarPlanner])
def test_deterministic_given_seed(Planner):
    world = wall_world(x_center=0.0, gap_half=6.0)
    p1 = Planner(_config(seed=7, max_iters=4000)).plan([0, 0, 0], [0, 0, 50], world)
    p2 = Planner(_config(seed=7, max_iters=4000)).plan([0, 0, 0], [0, 0, 50], world)
    assert p1 == p2


def test_rrt_star_quality_comparable_to_rrt():
    """RRT* refines for the whole budget, so its path cost should be at least
    as good as RRT's up to a small tolerance (both deterministic per seed)."""
    world = wall_world(x_center=0.0, gap_half=8.0)
    cfg = dict(seed=3, max_iters=5000)
    rrt = RRTPlanner(_config(**cfg))
    rrt_star = RRTStarPlanner(_config(**cfg))
    p_rrt = rrt.plan([0, 0, 0], [0, 0, 50], world)
    p_star = rrt_star.plan([0, 0, 0], [0, 0, 50], world)
    assert p_rrt and p_star
    assert rrt_star.last_cost <= rrt.last_cost * 1.05


def test_records_inspection_fields():
    planner = RRTPlanner(_config())
    planner.plan([0, 0, 0], [0, 0, 40], free_world())
    assert planner.last_nodes >= 1
    assert planner.last_cost > 0
    assert planner.name == "rrt"
    assert planner.last_lane_route == []  # RRT ignores the lane graph


def test_conforms_to_planner_interface():
    # The Planner protocol isn't runtime_checkable; assert the callable shape.
    for planner in (RRTPlanner(), RRTStarPlanner()):
        assert callable(getattr(planner, "plan", None))
