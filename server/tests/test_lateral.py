"""Tests for server-side lateral controllers + frenet error geometry.

The tracking tests double as a steering-SIGN check: a vehicle started off the
centerline must converge back to it, not diverge (which is what a sign error
would do).
"""
import math

import pytest

from controllers import lateral
from scenarios import networks


def straight_centerline(length=200.0, step=5.0):
    n = int(length / step)
    return [[0.0, 0.0, i * step] for i in range(n + 1)]


# ---- frenet errors -------------------------------------------------------- #
def test_lateral_sign_left_vs_right():
    cl = straight_centerline()
    # path runs along +Z; a point at +x is to the RIGHT of travel direction
    lat_right, _, _ = lateral.frenet_errors(3.0, 0.0, 0.0, cl)
    lat_left, _, _ = lateral.frenet_errors(-3.0, 0.0, 0.0, cl)
    assert lat_right > 0 > lat_left
    assert abs(lat_right) == pytest.approx(3.0, abs=1e-6)


def test_heading_error_zero_on_aligned():
    cl = straight_centerline()
    _, herr, _ = lateral.frenet_errors(0.0, 50.0, 0.0, cl)
    assert herr == pytest.approx(0.0, abs=1e-6)


def test_heading_error_sign():
    cl = straight_centerline()
    # vehicle yawed +20deg (toward +x) relative to +Z path
    _, herr, _ = lateral.frenet_errors(0.0, 50.0, 20.0, cl)
    assert herr == pytest.approx(-20.0, abs=1e-6)


def test_curvature_zero_on_straight_positive_on_curve():
    cl = straight_centerline()
    _, _, k_straight = lateral.frenet_errors(0.0, 50.0, 0.0, cl)
    assert k_straight == pytest.approx(0.0, abs=1e-6)
    curve = networks.highway_curve(length=200.0, radius=100.0).lane("curve_0").centerline
    _, _, k_curve = lateral.frenet_errors(curve[10][0], curve[10][2], 0.0, curve)
    assert k_curve > 0.0
    # curvature ~ 1/radius
    assert k_curve == pytest.approx(1.0 / 100.0, rel=0.5)


# ---- closed-loop tracking (kinematic bicycle) ----------------------------- #
def _track(ctrl, centerline, start_lateral=2.0, speed=15.0, steps=600, dt=0.05,
           wheel_base=2.7):
    """Drive a bicycle model along the centerline; return (max_tail_err)."""
    # start offset to +x from the first point, heading along +Z
    x = centerline[0][0] + start_lateral
    z = centerline[0][2]
    heading = 0.0
    errs = []
    for _ in range(steps):
        delta = ctrl.steer(x, z, heading, speed, centerline, wheel_base, dt=dt)
        yaw_rate = speed / wheel_base * math.tan(delta)
        heading = (heading + math.degrees(yaw_rate * dt)) % 360.0
        rad = math.radians(heading)
        x += math.sin(rad) * speed * dt
        z += math.cos(rad) * speed * dt
        lat, _, _ = lateral.frenet_errors(x, z, heading, centerline)
        errs.append(abs(lat))
        if z > centerline[-1][2] - 5:
            break
    tail = errs[len(errs) // 3:]  # ignore initial transient
    return max(tail)


@pytest.mark.parametrize("name", ["pure_pursuit", "stanley", "pid"])
def test_controllers_converge_on_straight(name):
    cl = straight_centerline(length=300.0)
    ctrl = lateral.make(name)
    max_tail = _track(ctrl, cl, start_lateral=2.0)
    assert max_tail < 0.5, f"{name} failed to converge, tail err {max_tail:.2f}"


@pytest.mark.parametrize("name", ["pure_pursuit", "stanley"])
def test_controllers_track_curve(name):
    curve = networks.highway_curve(length=200.0, radius=120.0).lane("curve_0").centerline
    ctrl = lateral.make(name)
    max_tail = _track(ctrl, curve, start_lateral=1.0, speed=20.0, steps=800)
    # bounded tracking error on a gentle curve
    assert max_tail < 2.0, f"{name} curve tracking error {max_tail:.2f}"


def test_registry_unknown_raises():
    with pytest.raises(KeyError):
        lateral.make("nope")
