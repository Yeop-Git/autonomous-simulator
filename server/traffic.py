"""Signalized intersection control (plan §13.1).

A simple fixed-cycle traffic-light model and a manager that answers the only
question the controller cares about: at time ``t``, approaching stop line L,
must this vehicle stop? Pure and time-driven (no Unity); the urban scenario /
experiment advances it.

Cycle order per light: Green -> Yellow -> Red -> (repeat). Phases that should
be complementary across approaches are arranged with ``offset``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

GREEN, YELLOW, RED = "Green", "Yellow", "Red"


@dataclass
class TrafficLight:
    id: str
    stop_line: list  # [x, y, z] position of the stop line on the approach
    approach_heading: float = 0.0  # deg, travel direction through the light
    green_time: float = 12.0
    yellow_time: float = 3.0
    red_time: float = 15.0
    offset: float = 0.0

    @property
    def period(self) -> float:
        return self.green_time + self.yellow_time + self.red_time

    def state(self, t: float) -> str:
        phase = (t + self.offset) % self.period
        if phase < self.green_time:
            return GREEN
        if phase < self.green_time + self.yellow_time:
            return YELLOW
        return RED


class TrafficLightManager:
    def __init__(self):
        self.lights: dict[str, TrafficLight] = {}

    def add(self, light: TrafficLight) -> TrafficLight:
        self.lights[light.id] = light
        return light

    def state(self, light_id: str, t: float) -> str:
        light = self.lights.get(light_id)
        return light.state(t) if light else GREEN

    def should_stop(self, position: list, speed: float, light_id: str, t: float,
                    approach_range: float = 55.0) -> bool:
        """True if a vehicle at ``position`` should halt for ``light_id``.

        Stops on RED (and on YELLOW when far enough to stop comfortably) while
        still upstream of the stop line and within the approach range.
        """
        light = self.lights.get(light_id)
        if light is None:
            return False
        # signed distance to stop line along the approach heading
        rad = math.radians(light.approach_heading)
        fx, fz = math.sin(rad), math.cos(rad)
        dx = light.stop_line[0] - position[0]
        dz = light.stop_line[2] - position[2]
        dist_to_line = dx * fx + dz * fz  # >0 => line is ahead
        if dist_to_line <= 0.0 or dist_to_line > approach_range:
            return False  # past the line or too far to care

        st = light.state(t)
        if st == RED:
            return True
        if st == YELLOW:
            # Use a conservative deceleration so yellow decisions are made
            # early instead of waiting until the painted line is close.
            stop_dist = speed * speed / (2.0 * 2.0)
            return stop_dist <= dist_to_line
        return False
