"""V2X fidelity / noise modes (plan §11.4).

The project assumes a perfect V2X world, but to study robustness we can
optionally degrade the state the server sees BEFORE it decides. Three modes:

  * ``full``  — perfect information (identity transform).
  * ``noisy`` — additive Gaussian noise on positions and velocities, modelling
    imperfect V2X estimates.
  * ``local`` — each report is dropped beyond a sensing radius of a reference
    point, modelling fall-back to a single local sensor.

``local`` is a *shared* view, not a per-vehicle one: one filtered world is what
the whole server sees. A genuine per-vehicle sensing model would have to filter
once per ego, which the central controller — which decides for every vehicle
from one snapshot — cannot express. Read it as one roadside unit, and keep its
reference point fixed: an earlier version fell back to the centroid of the
fleet, which drifts as vehicles spread out, so cars flickered in and out of view
for *everyone* including the car right behind them. On the Highway scene that
alone was enough to rear-end the mainline at 0.01 m.

Implemented as a pure transform on a schema-shaped StateMessage *dict*, so it
slots in front of ``CentralController.step`` (or the headless sim) without
touching the decision code. Deterministic given a seed.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass


@dataclass
class NoiseConfig:
    mode: str = "full"            # full | noisy | local
    pos_sigma: float = 0.5        # m, stddev of position noise (noisy)
    vel_sigma: float = 0.3        # m/s, stddev of velocity noise (noisy)
    sensing_radius: float = 60.0  # m, visibility radius (local)
    reference: tuple[float, float, float] = (0.0, 0.0, 0.0)  # local sensor site
    seed: int = 0


class NoiseModel:
    def __init__(self, config: NoiseConfig | None = None):
        self.config = config or NoiseConfig()
        self._rng = random.Random(self.config.seed)

    def apply(self, state: dict, ego_id: str | None = None) -> dict:
        """Return a transformed copy of ``state`` per the configured mode."""
        mode = self.config.mode
        if mode == "full":
            return state
        if mode == "noisy":
            return self._noisy(state)
        if mode == "local":
            return self._local(state, ego_id)
        raise ValueError(f"unknown noise mode '{mode}'")

    # ------------------------------------------------------------------ #
    def _noisy(self, state: dict) -> dict:
        out = _shallow_copy_state(state)
        for v in out["vehicles"]:
            v["position"] = self._jitter(v["position"], self.config.pos_sigma)
            v["velocity"] = self._jitter(v["velocity"], self.config.vel_sigma)
        for o in out["objects"]:
            o["position"] = self._jitter(o["position"], self.config.pos_sigma)
            o["velocity"] = self._jitter(o["velocity"], self.config.vel_sigma)
        return out

    def _local(self, state: dict, ego_id: str | None) -> dict:
        out = _shallow_copy_state(state)
        ego = _find_vehicle(out["vehicles"], ego_id) if ego_id else None
        # A named ego sees from where it stands; otherwise the sensor sits at a
        # fixed site. Never the fleet centroid — that moves, and a moving
        # horizon makes vehicles blink in and out of the world.
        center = ego["position"] if ego else list(self.config.reference)
        r = self.config.sensing_radius
        # keep the ego always; filter the rest by planar distance
        out["vehicles"] = [
            v for v in out["vehicles"]
            if (ego is not None and v["id"] == ego["id"]) or _within(v["position"], center, r)
        ]
        out["objects"] = [o for o in out["objects"] if _within(o["position"], center, r)]
        return out

    def _jitter(self, vec, sigma):
        return [vec[0] + self._rng.gauss(0, sigma),
                vec[1],  # leave height untouched (road is flat)
                vec[2] + self._rng.gauss(0, sigma)]


# --------------------------------------------------------------------------- #
def _shallow_copy_state(state: dict) -> dict:
    out = dict(state)
    out["vehicles"] = [dict(v) for v in state.get("vehicles", [])]
    out["objects"] = [dict(o) for o in state.get("objects", [])]
    out["events"] = list(state.get("events", []))
    return out


def _find_vehicle(vehicles, vid):
    for v in vehicles:
        if v.get("id") == vid:
            return v
    return None


def _within(p, center, r) -> bool:
    return math.hypot(p[0] - center[0], p[2] - center[2]) <= r
