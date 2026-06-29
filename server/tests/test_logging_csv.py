"""Tests for the frozen-schema CSV logger."""
import csv
import math

from logging_csv import COLUMNS, DriveLogger, LogRow


def test_header_is_frozen_schema(tmp_path):
    p = tmp_path / "log.csv"
    with DriveLogger(p) as log:
        log.log(LogRow(time=0.0, vehicle_id="car_01"))
    with open(p, newline="", encoding="utf-8") as f:
        header = next(csv.reader(f))
    assert header == COLUMNS


def test_rows_written_and_inf_ttc_blanked(tmp_path):
    p = tmp_path / "log.csv"
    with DriveLogger(p) as log:
        log.log(LogRow(time=0.1, vehicle_id="a", ttc=math.inf, speed=12.3456789))
        log.log(LogRow(time=0.2, vehicle_id="a", ttc=2.5, scenario="highway"))
    assert log.rows_written == 2
    with open(p, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert rows[0]["ttc"] == ""          # inf -> blank
    assert rows[1]["ttc"] == "2.5"
    assert rows[0]["speed"] == "12.3457"  # rounded to 4 dp
    assert rows[1]["scenario"] == "highway"


def test_directory_autocreated(tmp_path):
    p = tmp_path / "nested" / "deep" / "log.csv"
    with DriveLogger(p) as log:
        log.log(LogRow(time=0.0, vehicle_id="x"))
    assert p.exists()
