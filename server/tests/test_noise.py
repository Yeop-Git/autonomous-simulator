"""The V2X fidelity modes, and that the controller survives a degraded world.

``noise.py`` had no tests at all. It is the only thing that makes the
"perfect V2X" assumption falsifiable, so what it does needs pinning — and the
decisions downstream of it need to hold up when it is switched on.
"""
import math

import pytest

from central_control import CentralController
from headless_sim import HeadlessSim
from noise import NoiseConfig, NoiseModel
from world_model import LaneNetwork

from test_regression_drive import drive, network


def state(vehicles=(), objects=()):
    return {
        "time": 1.0, "tick": 1, "scenario": "highway",
        "vehicles": [dict(v) for v in vehicles],
        "objects": [dict(o) for o in objects],
        "events": [],
    }


def car(vid, z, speed=10.0):
    return {"id": vid, "type": "car", "position": [0.0, 0.0, z],
            "velocity": [0.0, 0.0, speed], "acceleration": [0.0, 0.0, 0.0],
            "heading": 0.0, "current_lane": "L", "target_lane": None,
            "maneuver": "straight", "has_goal": False, "goal": [0.0, 0.0, 0.0],
            "behavior_state": "LaneKeeping"}


# ---- the transform itself ------------------------------------------------ #
def test_full_mode_is_the_identity():
    original = state([car("a", 10.0)])
    assert NoiseModel(NoiseConfig(mode="full")).apply(original) is original


def test_noisy_mode_jitters_the_plane_but_not_the_height_or_the_original():
    original = state([car("a", 10.0)], [{
        "id": "rock", "type": "static_obstacle", "position": [1.0, 0.7, 20.0],
        "velocity": [0.0, 0.0, 0.0], "radius": 1.0}])
    noisy = NoiseModel(NoiseConfig(mode="noisy", seed=3)).apply(original)

    assert noisy is not original
    assert original["vehicles"][0]["position"] == [0.0, 0.0, 10.0], "mutated the input"
    assert noisy["vehicles"][0]["position"] != [0.0, 0.0, 10.0]
    # the road is flat, so height must survive untouched
    assert noisy["objects"][0]["position"][1] == 0.7


def test_noisy_mode_is_deterministic_for_a_seed():
    original = state([car("a", 10.0), car("b", 40.0)])
    first = NoiseModel(NoiseConfig(mode="noisy", seed=11)).apply(original)
    second = NoiseModel(NoiseConfig(mode="noisy", seed=11)).apply(original)
    assert first["vehicles"][0]["position"] == second["vehicles"][0]["position"]


def test_local_mode_measures_from_a_fixed_site_not_the_fleet():
    """Regression: the fallback reference used to be the fleet centroid.

    That point moves as the vehicles spread out, so the horizon swept over them
    and cars blinked in and out of the world — for *everyone*, including the car
    directly behind them.
    """
    near_and_far = state([car("near", 10.0), car("far", 200.0)])
    model = NoiseModel(NoiseConfig(mode="local", sensing_radius=60.0))
    kept = [v["id"] for v in model.apply(near_and_far)["vehicles"]]
    assert kept == ["near"], "the sensor is at the origin, so only 'near' is in range"

    # ...and moving one car does not change what the sensor can see of the other
    near_and_farther = state([car("near", 10.0), car("far", 400.0)])
    assert [v["id"] for v in model.apply(near_and_farther)["vehicles"]] == ["near"]


def test_local_mode_can_be_sited_or_pinned_to_an_ego():
    far_field = state([car("near", 10.0), car("far", 200.0)])
    sited = NoiseModel(NoiseConfig(mode="local", sensing_radius=60.0,
                                   reference=(0.0, 0.0, 200.0)))
    assert [v["id"] for v in sited.apply(far_field)["vehicles"]] == ["far"]

    from_ego = NoiseModel(NoiseConfig(mode="local", sensing_radius=60.0))
    assert [v["id"] for v in from_ego.apply(far_field, ego_id="far")["vehicles"]] \
        == ["far"]


def test_unknown_mode_is_rejected():
    with pytest.raises(ValueError):
        NoiseModel(NoiseConfig(mode="telepathy")).apply(state())


# ---- and the decisions downstream of it ---------------------------------- #
def test_stopping_for_a_crate_survives_noisy_v2x():
    """Whether an object counts as parked decides if ACC holds a gap to it.

    A bare threshold on the reported speed sits on the noise floor — this mode
    jitters velocity by 0.3 m/s per axis, so a stationary crate reads as moving
    about half the time. The ACC leader flickered in and out with it and the car
    went back to creeping and stopping, ending 2.3 m from the crate.
    """
    net = network("Highway")
    noise = NoiseModel(NoiseConfig(mode="noisy", seed=7))
    sim = HeadlessSim(net, CentralController(net, dt=0.1, noise=noise), dt=0.1,
                      scenario="highway")
    sim.add_vehicle("ego", [3.5, 0.0, 60.0], "hw_l2", speed=22.0,
                    goal=list(net.lane("hw_l2").end))
    sim.add_object("cargo", "unexpected_obstacle", [3.5, 0.0, 150.0],
                   [0.0, 0.0, 0.0], radius=1.25)

    assert not drive(sim, 400)
    gap = 150.0 - sim.vehicles["ego"].position[2]
    assert sim.vehicles["ego"].speed < 0.3, "never came to rest"
    assert 4.0 < gap < 20.0, f"came to rest {gap:.2f} m from the crate"


def test_the_ramp_still_merges_under_noisy_v2x():
    net = network("Highway")
    noise = NoiseModel(NoiseConfig(mode="noisy", seed=5))
    sim = HeadlessSim(net, CentralController(net, dt=0.1, noise=noise), dt=0.1,
                      scenario="highway")
    sim.add_vehicle("ramp", net.lane("hw_ramp").start, "hw_ramp", speed=0.0,
                    goal=list(net.lane("hw_l2").end))
    for i, z in enumerate((95.0, 60.0, 25.0)):
        sim.add_vehicle(f"main_{i}", [3.5, 0.0, z], "hw_l2", speed=25.0,
                        goal=list(net.lane("hw_l2").end))

    assert not drive(sim, 400)
    assert sim.vehicles["ramp"].lane == "hw_l2", "the ramp car never merged"
