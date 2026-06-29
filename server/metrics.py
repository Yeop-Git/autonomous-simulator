"""Metrics over drive logs (plan §6.4 highway, §7.4 urban, §21.2).

Reads the frozen-schema CSV (``logging_csv.COLUMNS``) and computes the summary
indicators the experiments report. Pure analysis — no sim, no Unity — so it
runs the same on a live-captured log or a headless-sim log.

Kept dependency-light: works on a list of ``LogRow``-like dicts OR a pandas
DataFrame. The headless sim can also feed rows directly via ``MetricsAccumulator``
for live metrics without a CSV round-trip.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Optional

HARD_BRAKE_DECEL = 4.0   # m/s^2; speed drop beyond this (per second) = hard brake


@dataclass
class DriveMetrics:
    avg_speed: float = 0.0
    min_ttc: float = math.inf
    max_lateral_error: float = 0.0
    rms_lateral_error: float = 0.0
    lane_departures: int = 0
    hard_brakes: int = 0
    collision_risk_events: int = 0
    arrivals: int = 0
    samples: int = 0

    def as_dict(self) -> dict:
        d = self.__dict__.copy()
        if not math.isfinite(d["min_ttc"]):
            d["min_ttc"] = None
        return d


@dataclass
class _VehTrack:
    last_speed: Optional[float] = None
    last_time: Optional[float] = None
    in_departure: bool = False
    arrived: bool = False


class MetricsAccumulator:
    """Streaming metrics — feed it one logged sample at a time."""

    def __init__(self, lane_width: float = 3.5):
        self.lane_width = lane_width
        self._speed_sum = 0.0
        self._sq_lat_sum = 0.0
        self.m = DriveMetrics()
        self._tracks: dict[str, _VehTrack] = {}

    def add(self, *, time: float, vehicle_id: str, speed: float,
            lateral_error: float = 0.0, ttc: float = math.inf,
            behavior_state: str = "", collision_risk: float = 0.0) -> None:
        tr = self._tracks.setdefault(vehicle_id, _VehTrack())
        self.m.samples += 1
        self._speed_sum += speed
        self._sq_lat_sum += lateral_error * lateral_error
        self.m.max_lateral_error = max(self.m.max_lateral_error, abs(lateral_error))
        if math.isfinite(ttc):
            self.m.min_ttc = min(self.m.min_ttc, ttc)
        if collision_risk > 0.0:
            self.m.collision_risk_events += 1

        # lane departure = crossed half-lane-width (count each excursion once)
        if abs(lateral_error) > self.lane_width / 2.0:
            if not tr.in_departure:
                self.m.lane_departures += 1
                tr.in_departure = True
        else:
            tr.in_departure = False

        # hard brake = large speed drop per second
        if tr.last_speed is not None and tr.last_time is not None:
            dt = time - tr.last_time
            if dt > 0 and (tr.last_speed - speed) / dt > HARD_BRAKE_DECEL:
                self.m.hard_brakes += 1
        tr.last_speed, tr.last_time = speed, time

        if behavior_state == "Arrived" and not tr.arrived:
            tr.arrived = True
            self.m.arrivals += 1

    def result(self) -> DriveMetrics:
        if self.m.samples:
            self.m.avg_speed = self._speed_sum / self.m.samples
            self.m.rms_lateral_error = math.sqrt(self._sq_lat_sum / self.m.samples)
        return self.m


def from_rows(rows: Iterable[dict], lane_width: float = 3.5) -> DriveMetrics:
    """Compute metrics from logged-row dicts (CSV-parsed or in-memory)."""
    acc = MetricsAccumulator(lane_width=lane_width)
    for r in rows:
        ttc = r.get("ttc", "")
        ttc = float(ttc) if ttc not in ("", None) else math.inf
        acc.add(
            time=float(r["time"]), vehicle_id=r["vehicle_id"],
            speed=float(r.get("speed", 0) or 0),
            lateral_error=float(r.get("lateral_error", 0) or 0),
            ttc=ttc, behavior_state=r.get("behavior_state", ""),
            collision_risk=float(r.get("collision_risk", 0) or 0),
        )
    return acc.result()


def from_csv(path: str, lane_width: float = 3.5) -> DriveMetrics:
    import csv
    with open(path, newline="", encoding="utf-8") as f:
        return from_rows(csv.DictReader(f), lane_width=lane_width)
