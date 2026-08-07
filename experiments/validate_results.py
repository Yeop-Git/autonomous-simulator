"""Validate experiment artifacts and write a provenance manifest.

This is a publication gate, not another simulation. It rejects truncated or
internally inconsistent CSVs, independently recomputes the algorithm summary,
and records hashes plus the execution environment in ``results/manifest.json``.

Run after the three experiment runners and ``make_charts.py``::

    python experiments/validate_results.py
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import platform
import statistics
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULTS = Path(__file__).resolve().parent / "results"
EXPECTED_ROWS = {
    "algo_compare_raw.csv": 1170,
    "algo_compare_summary.csv": 27,
    "lka_drive_log.csv": 6546,
    "lka_summary.csv": 12,
    "scene_stats.csv": 5,
}
EXPECTED_CHARTS = {
    "algo_compare.png", "algo_time_vs_vehicles.png", "lka_lateral_error.png"
}
EXPECTED_LKA = {
    (controller, str(speed))
    for controller in ("pure_pursuit", "stanley", "pid")
    for speed in (40, 60, 80, 100)
}
EXPECTED_SCENES = {
    "LKA_Test": (1, 400),
    "Highway (ramp merge)": (4, 400),
    "Urban (8 approaches + pedestrian)": (8, 1200),
    "EmergencyAvoidance (cargo + ambulance)": (2, 500),
    "IntegratedCity (shoulder merge + obstacle)": (3, 500),
}
EXPECTED_PYTEST_IDENTITY_SHA256 = (
    "125d223dfe95b42a927d89f92f1b14d08c3010caa1c18c8e889f1ad0ccd4d3a8"
)
EXPECTED_ALGORITHM_KEYS = {
    (scenario, planner, str(count), str(seed), str(vehicle_id))
    for scenario in ("road_open", "road_detour", "obstacle_field")
    for planner in ("astar", "rrt", "rrt_star")
    for count in (1, 5, 20)
    for seed in range(5)
    for vehicle_id in range(count)
}


def read_csv(name: str) -> list[dict[str, str]]:
    path = RESULTS / name
    if not path.is_file():
        raise AssertionError(f"missing artifact: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    expected = EXPECTED_ROWS[name]
    if len(rows) != expected:
        raise AssertionError(f"{name}: expected {expected} rows, got {len(rows)}")
    return rows


def number(row: dict[str, str], field: str) -> float:
    value = float(row[field])
    if not math.isfinite(value):
        raise AssertionError(f"non-finite {field}: {row}")
    return value


def validate_algorithm() -> list[str]:
    raw = read_csv("algo_compare_raw.csv")
    summary = read_csv("algo_compare_summary.csv")
    key_fields = ("scenario", "planner", "num_vehicles", "seed", "vehicle_id")
    keys = {tuple(row[field] for field in key_fields) for row in raw}
    if len(keys) != len(raw):
        raise AssertionError("algo_compare_raw.csv contains duplicate composite keys")
    if keys != EXPECTED_ALGORITHM_KEYS:
        raise AssertionError("algorithm scenario/planner/batch/seed/query matrix drift")

    groups: dict[tuple[str, str, str], list[dict[str, str]]] = defaultdict(list)
    for row in raw:
        success = int(row["success"])
        if success not in (0, 1):
            raise AssertionError(f"invalid success flag: {row}")
        elapsed = number(row, "plan_time_ms")
        length = number(row, "path_length")
        nodes = number(row, "nodes")
        start_error = number(row, "start_error_m")
        goal_error = number(row, "goal_error_m")
        collision_free = int(row["collision_free"])
        if start_error < 0 or goal_error < 0 or collision_free not in (0, 1):
            raise AssertionError(f"invalid endpoint/collision domain: {row}")
        expected_success = (start_error <= 4.0 and goal_error <= 4.0 and
                            collision_free == 1)
        if bool(success) != expected_success:
            raise AssertionError(f"success/endpoints/collision flags disagree: {row}")
        if elapsed < 0 or nodes < 0 or (success and length <= 0) or (not success and length != 0):
            raise AssertionError(f"invalid algorithm measurement: {row}")
        groups[(row["scenario"], row["planner"], row["num_vehicles"])].append(row)

    by_key = {(r["scenario"], r["planner"], r["num_vehicles"]): r for r in summary}
    if set(groups) != set(by_key):
        raise AssertionError("raw and summary algorithm group keys differ")

    for key, rows in groups.items():
        times = [float(r["plan_time_ms"]) for r in rows]
        successful = [r for r in rows if int(r["success"])]
        lengths = [float(r["path_length"]) for r in successful]
        expected = {
            "runs": float(len(rows)),
            "success_rate": len(successful) / len(rows),
            "mean_time_ms": statistics.fmean(times),
            "std_time_ms": statistics.pstdev(times),
            "total_time_ms": sum(times),
            "mean_path_length": statistics.fmean(lengths) if lengths else 0.0,
            "std_path_length": statistics.pstdev(lengths) if len(lengths) > 1 else 0.0,
            "mean_nodes": statistics.fmean(float(r["nodes"]) for r in rows),
        }
        stored = by_key[key]
        for field, value in expected.items():
            tolerance = 0.050001 if field == "mean_nodes" else 5e-4
            if not math.isclose(float(stored[field]), value, rel_tol=1e-4,
                                abs_tol=tolerance):
                raise AssertionError(f"summary mismatch {key} {field}: {stored[field]} != {value}")
    return ["algorithm row/domain/key checks", "algorithm raw-to-summary recomputation"]


def validate_lka_and_scenes() -> list[str]:
    drive = read_csv("lka_drive_log.csv")
    lka = read_csv("lka_summary.csv")
    scenes = read_csv("scene_stats.csv")
    lka_keys = {(r["controller"], r["speed_kmh"]) for r in lka}
    if lka_keys != EXPECTED_LKA:
        raise AssertionError(f"unexpected LKA conditions: {lka_keys}")
    log_groups: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in drive:
        log_groups[row["vehicle_id"]].append(row)
    for row in lka:
        rms = number(row, "rms_lateral_m")
        maximum = number(row, "max_lateral_m")
        if rms < 0 or maximum < 0 or rms > maximum:
            raise AssertionError(f"negative LKA error: {row}")
        departures = int(row["departures"])
        if departures < 0:
            raise AssertionError(f"negative LKA departure count: {row}")
        vehicle_id = f'{row["controller"]}_{row["speed_kmh"]}'
        samples = log_groups.get(vehicle_id, [])
        if not samples:
            raise AssertionError(f"missing LKA log samples for {vehicle_id}")
        errors = [float(sample["lateral_error"]) for sample in samples]
        recalculated_rms = math.sqrt(statistics.fmean(e * e for e in errors))
        recalculated_max = max(abs(e) for e in errors)
        recalculated_departures = 0
        outside = False
        for error in errors:
            now_outside = abs(error) > 1.75
            if now_outside and not outside:
                recalculated_departures += 1
            outside = now_outside
        if not math.isclose(rms, recalculated_rms, abs_tol=1.5e-4) or not math.isclose(
                maximum, recalculated_max, abs_tol=1.5e-4) or departures != recalculated_departures:
            raise AssertionError(f"LKA raw/summary mismatch for {vehicle_id}")

    scene_keys = {row["scenario"] for row in scenes}
    if scene_keys != set(EXPECTED_SCENES):
        raise AssertionError(f"unexpected scene rows: {scene_keys}")
    for row in scenes:
        expected_vehicles, expected_ticks = EXPECTED_SCENES[row["scenario"]]
        if int(row["vehicles"]) != expected_vehicles or int(row["ticks"]) != expected_ticks:
            raise AssertionError(f"scene size/horizon drift: {row}")
        for field in ("step_p50_ms", "step_p95_ms", "step_max_ms", "mean_speed_mps"):
            if number(row, field) < 0:
                raise AssertionError(f"negative scene metric: {row}")
        if not (float(row["step_p50_ms"]) <= float(row["step_p95_ms"]) <=
                float(row["step_max_ms"])):
            raise AssertionError(f"invalid timing quantile order: {row}")
        for field in ("peak_decel_mps2", "hard_brake_episodes",
                      "max_behavior_changes_10s", "arrivals"):
            if float(row[field]) < 0:
                raise AssertionError(f"negative scene outcome: {row}")
        for field in ("min_same_lane_gap_m", "min_ttc_s"):
            value = row[field]
            if value not in ("n/a", "inf") and float(value) < 0:
                raise AssertionError(f"negative scene distance/TTC: {row}")
    return ["LKA raw-to-summary/domain checks", "scene identity/timing/domain checks"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_value(*args: str) -> str:
    result = subprocess.run(["git", *args], cwd=ROOT, text=True,
                            capture_output=True, check=False)
    return result.stdout.strip() if result.returncode == 0 else "unavailable"


def validate_pytest() -> str:
    path = RESULTS / "pytest.xml"
    if not path.is_file():
        raise AssertionError("missing pytest.xml; run the documented JUnit command")
    root = ET.parse(path).getroot()
    suite = root if root.tag.endswith("testsuite") else root.find("testsuite")
    if suite is None:
        raise AssertionError("pytest.xml has no testsuite element")
    totals = {name: int(suite.attrib.get(name, 0))
              for name in ("tests", "failures", "errors", "skipped")}
    if totals != {"tests": 259, "failures": 0, "errors": 0, "skipped": 2}:
        raise AssertionError(f"unexpected pytest totals: {totals}")
    cases = root.findall(".//testcase")
    identities = {(case.attrib.get("classname", ""), case.attrib.get("name", ""))
                  for case in cases}
    if len(cases) != 259 or len(identities) != 259 or any(
            not classname.startswith("tests.") for classname, _ in identities):
        raise AssertionError("pytest testcase identity set is missing, duplicated, or foreign")
    identity_payload = "\n".join(
        f"{classname}::{name}" for classname, name in sorted(identities)
    ).encode("utf-8")
    actual_identity_hash = hashlib.sha256(identity_payload).hexdigest()
    if actual_identity_hash != EXPECTED_PYTEST_IDENTITY_SHA256:
        raise AssertionError(
            f"pytest exact testcase identity set drift: {actual_identity_hash}"
        )
    return "pytest JUnit exact identities/totals: 257 passed / 2 skipped"


def main() -> None:
    checks = validate_algorithm() + validate_lka_and_scenes() + [validate_pytest()]
    artifacts = {}
    for path in sorted(RESULTS.glob("*.csv")):
        with path.open(newline="", encoding="utf-8") as handle:
            row_count = sum(1 for _ in csv.DictReader(handle))
        artifacts[path.name] = {"rows": row_count, "sha256": sha256(path)}
    chart_paths = sorted((RESULTS / "charts").glob("*.png"))
    if {path.name for path in chart_paths} != EXPECTED_CHARTS:
        raise AssertionError("required chart set is missing or contains unexpected files")
    for path in chart_paths:
        artifacts[f"charts/{path.name}"] = {"bytes": path.stat().st_size,
                                             "sha256": sha256(path)}
    pytest_path = RESULTS / "pytest.xml"
    artifacts[pytest_path.name] = {"bytes": pytest_path.stat().st_size,
                                   "sha256": sha256(pytest_path)}

    source_paths = {
        Path(__file__), ROOT / "experiments" / "run_algorithm_compare.py",
        ROOT / "experiments" / "run_lka_test.py",
        ROOT / "experiments" / "run_scene_stats.py",
        ROOT / "experiments" / "make_charts.py", ROOT / "server" / "requirements.txt",
        *ROOT.joinpath("server").rglob("*.py"),
        *ROOT.joinpath("server", "scenarios").glob("*.json"),
        *ROOT.joinpath("shared", "protocol").glob("*.json"),
    }
    sources = {str(path.relative_to(ROOT)).replace("\\", "/"): sha256(path)
               for path in sorted(source_paths) if path.is_file()}

    status = git_value("status", "--porcelain", "--untracked-files=all")
    manifest = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "git_commit": git_value("rev-parse", "HEAD"),
        "git_worktree_dirty": bool(status and status != "unavailable"),
        "git_status_porcelain": status.splitlines() if status != "unavailable" else [],
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "checks_passed": checks,
        "commands": [
            "python experiments/run_algorithm_compare.py",
            "python experiments/run_lka_test.py",
            "python experiments/run_scene_stats.py",
            "python experiments/make_charts.py",
            "python -m pytest server/tests -q --junitxml=experiments/results/pytest.xml",
            "python experiments/validate_results.py",
        ],
        "experiment_config": {
            "algorithm_seeds": [0, 1, 2, 3, 4],
            "query_batch_sizes": [1, 5, 20],
            "rrt_iterations": 3000,
            "rrt_star_iterations": 1500,
            "endpoint_tolerance_m": 4.0,
            "lka_speeds_kmh": [40, 60, 80, 100],
            "scene_dt_s": 0.1,
        },
        "scope_warning": (
            "Headless model-in-the-loop artifacts. Timing excludes WebSocket, JSON/schema "
            "processing, Unity command application, physics, and rendering."
        ),
        "artifacts": artifacts,
        "source_sha256": sources,
    }
    output = RESULTS / "manifest.json"
    output.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
                      encoding="utf-8")
    print(f"validated {len(artifacts)} artifacts; wrote {output}")


if __name__ == "__main__":
    main()
