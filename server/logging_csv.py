"""Fixed-schema CSV logging (plan §21.1).

The column set is FROZEN so the analysis notebooks never break:

    time, vehicle_id, scenario, position_x, position_z, speed, lane_id,
    behavior_state, lateral_error, heading_error, target_speed,
    collision_risk, ttc, event_type

Use ``DriveLogger`` as a context manager; call ``log`` once per vehicle per
logged tick. Missing fields default sensibly (empty / 0 / inf-as-blank).
"""
from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Optional

COLUMNS = [
    "time", "vehicle_id", "scenario", "position_x", "position_z", "speed",
    "lane_id", "behavior_state", "lateral_error", "heading_error",
    "target_speed", "collision_risk", "ttc", "event_type",
]


@dataclass
class LogRow:
    time: float
    vehicle_id: str
    scenario: str = ""
    position_x: float = 0.0
    position_z: float = 0.0
    speed: float = 0.0
    lane_id: str = ""
    behavior_state: str = ""
    lateral_error: float = 0.0
    heading_error: float = 0.0
    target_speed: float = 0.0
    collision_risk: float = 0.0
    ttc: float = math.inf
    event_type: str = ""

    def to_csv(self) -> dict:
        d = asdict(self)
        # blank out non-finite ttc so pandas reads NaN, not the string 'inf'
        if not math.isfinite(d["ttc"]):
            d["ttc"] = ""
        # round floats for compact, stable logs
        for k in ("position_x", "position_z", "speed", "lateral_error",
                  "heading_error", "target_speed", "collision_risk"):
            d[k] = round(d[k], 4)
        return d


class DriveLogger:
    def __init__(self, path: str | Path):
        self.path = Path(path)
        self._fh = None
        self._writer: Optional[csv.DictWriter] = None
        self.rows_written = 0

    def __enter__(self) -> "DriveLogger":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._fh, fieldnames=COLUMNS)
        self._writer.writeheader()
        return self

    def log(self, row: LogRow) -> None:
        assert self._writer is not None, "DriveLogger used outside context"
        self._writer.writerow(row.to_csv())
        self.rows_written += 1

    def __exit__(self, *exc) -> None:
        if self._fh:
            self._fh.close()
            self._fh = None
