"""Read the authored lane graph straight out of a Unity ``.unity`` scene.

The scene is the source of truth for road geometry; ``server/scenarios/*.json``
is a *copy* produced by ``LaneNetworkExporter`` whenever a human remembers to
run it. Nothing has ever forced the two to agree, and a stale export is the
worst kind of failure here: the server plans confidently on a road that is no
longer the one Unity draws.

This module re-implements the exporter's output from the serialized scene, so a
test can diff the two. It is deliberately a small hand-written reader rather
than a YAML parse: Unity scenes are a multi-document stream with custom
``!u!<class> &<fileID>`` tags that standard loaders choke on, and we only need
four component types.

Limits, asserted rather than assumed: waypoint world positions are the sum of
local positions up the parent chain, which is only correct while every ancestor
is unrotated and unscaled. Any violation is reported in ``warnings``.
"""
from __future__ import annotations

import re
from pathlib import Path

# MonoBehaviour script guid of V2X.Road.Lane (unity/Assets/Scripts/Road/Lane.cs.meta)
LANE_SCRIPT_GUID = "4fca05718c0f3e14bbe031fecba180de"

TRANSFORM_CLASS = "4"
MONOBEHAVIOUR_CLASS = "114"

_DOCUMENT = re.compile(r"^--- !u!(\d+) &(\d+)", re.M)
_VEC3 = re.compile(
    r"\{x: *(-?[\d.eE+-]+), *y: *(-?[\d.eE+-]+), *z: *(-?[\d.eE+-]+)")
_QUAT = re.compile(
    r"\{x: *(-?[\d.eE+-]+), *y: *(-?[\d.eE+-]+), *z: *(-?[\d.eE+-]+), "
    r"*w: *(-?[\d.eE+-]+)")
_FILE_ID = re.compile(r"\{fileID: (-?\d+)")


def _documents(text: str):
    marks = list(_DOCUMENT.finditer(text))
    for i, mark in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(text)
        yield mark.group(1), mark.group(2), text[mark.end():end]


def _scalar(body: str, name: str) -> str | None:
    match = re.search(rf"^  {re.escape(name)}: *(.*)$", body, re.M)
    return match.group(1) if match else None


def _reference(body: str, name: str) -> str:
    match = _FILE_ID.search(_scalar(body, name) or "")
    return match.group(1) if match else "0"


def _reference_list(body: str, name: str) -> list[str]:
    """fileIDs of a serialized list field, in order.

    Anchor the header to its own line: ``\\s`` matches newlines, so a ``\\s*$``
    tail would run past the header and start the item scan mid-list.
    """
    header = re.search(rf"^  {re.escape(name)}:[ \t]*(\[\])?[ \t]*$", body, re.M)
    if header is None or header.group(1) == "[]":
        return []
    out: list[str] = []
    for line in body[header.end():].splitlines()[1:]:
        if not line.startswith("  - "):
            break
        match = _FILE_ID.search(line)
        out.append(match.group(1) if match else "0")
    return out


class SceneLaneNetwork:
    """The lane graph an open Unity scene would export."""

    def __init__(self, scene_path: str | Path):
        text = Path(scene_path).read_text(encoding="utf-8", errors="replace")
        self.transforms: dict[str, dict] = {}
        self._lanes: dict[str, dict] = {}
        self.warnings: list[str] = []

        for class_id, file_id, body in _documents(text):
            if class_id == TRANSFORM_CLASS:
                self.transforms[file_id] = self._read_transform(body)
            elif class_id == MONOBEHAVIOUR_CLASS:
                script = re.search(r"m_Script: \{fileID: \d+, guid: (\w+)", body)
                if script is not None and script.group(1) == LANE_SCRIPT_GUID:
                    self._lanes[file_id] = self._read_lane(body)

    @staticmethod
    def _read_transform(body: str) -> dict:
        position = _VEC3.search(_scalar(body, "m_LocalPosition") or "")
        rotation = _QUAT.search(_scalar(body, "m_LocalRotation") or "")
        scale = _VEC3.search(_scalar(body, "m_LocalScale") or "")
        return {
            "position": tuple(float(v) for v in position.groups())
                        if position else (0.0, 0.0, 0.0),
            "rotation": tuple(float(v) for v in rotation.groups())
                        if rotation else (0.0, 0.0, 0.0, 1.0),
            "scale": tuple(float(v) for v in scale.groups())
                     if scale else (1.0, 1.0, 1.0),
            "father": _reference(body, "m_Father"),
        }

    @staticmethod
    def _read_lane(body: str) -> dict:
        return {
            "id": (_scalar(body, "laneId") or "").strip(),
            "width": float(_scalar(body, "width") or 3.5),
            "speed_limit": float(_scalar(body, "speedLimit") or 13.9),
            "waypoints": _reference_list(body, "waypoints"),
            "left": _reference(body, "leftLane"),
            "right": _reference(body, "rightLane"),
            "next": _reference_list(body, "nextLanes"),
        }

    def world_position(self, transform_id: str) -> tuple[float, float, float]:
        x = y = z = 0.0
        visited: set[str] = set()
        current = transform_id
        while current in self.transforms and current not in visited:
            visited.add(current)
            node = self.transforms[current]
            if current != transform_id:
                rotation, scale = node["rotation"], node["scale"]
                if (abs(rotation[0]) + abs(rotation[1]) + abs(rotation[2]) > 1e-6
                        or abs(rotation[3] - 1.0) > 1e-6):
                    self.warnings.append(
                        f"ancestor transform {current} is rotated; waypoint "
                        f"world positions cannot be summed")
                if any(abs(s - 1.0) > 1e-6 for s in scale):
                    self.warnings.append(
                        f"ancestor transform {current} is scaled; waypoint "
                        f"world positions cannot be summed")
            x += node["position"][0]
            y += node["position"][1]
            z += node["position"][2]
            current = node["father"]
        return (x, y, z)

    def lanes(self) -> dict[str, dict]:
        """``{lane_id: lane}`` in the exporter's own shape."""
        by_file_id = self._lanes
        out: dict[str, dict] = {}
        for lane in by_file_id.values():
            out[lane["id"]] = {
                "id": lane["id"],
                "centerline": [list(self.world_position(w))
                               for w in lane["waypoints"]],
                "width": lane["width"],
                "speed_limit": lane["speed_limit"],
                "left_lane_id": by_file_id.get(lane["left"], {}).get("id"),
                "right_lane_id": by_file_id.get(lane["right"], {}).get("id"),
                "next_lane_ids": [by_file_id[n]["id"] for n in lane["next"]
                                  if n in by_file_id],
            }
        return out


def diff_against_export(scene_path: str | Path, export: dict,
                        tolerance: float = 1e-3) -> list[str]:
    """Human-readable differences between an authored scene and its export."""
    scene = SceneLaneNetwork(scene_path)
    authored = scene.lanes()
    exported = {lane["id"]: lane for lane in export["lanes"]}
    problems = sorted(set(scene.warnings))

    for lane_id in sorted(set(authored) - set(exported)):
        problems.append(f"{lane_id}: in the scene but missing from the export")
    for lane_id in sorted(set(exported) - set(authored)):
        problems.append(f"{lane_id}: in the export but not in the scene")

    for lane_id in sorted(set(authored) & set(exported)):
        a, e = authored[lane_id], exported[lane_id]
        for field in ("width", "speed_limit"):
            if abs(a[field] - e.get(field, 0.0)) > 1e-4:
                problems.append(
                    f"{lane_id}.{field}: scene {a[field]} vs export {e.get(field)}")
        for field in ("left_lane_id", "right_lane_id"):
            if (a[field] or None) != (e.get(field) or None):
                problems.append(
                    f"{lane_id}.{field}: scene {a[field]} vs export {e.get(field)}")
        if a["next_lane_ids"] != list(e.get("next_lane_ids", [])):
            problems.append(
                f"{lane_id}.next_lane_ids: scene {a['next_lane_ids']} "
                f"vs export {e.get('next_lane_ids')}")
        if len(a["centerline"]) != len(e["centerline"]):
            problems.append(
                f"{lane_id}.centerline: scene has {len(a['centerline'])} "
                f"waypoints, export has {len(e['centerline'])}")
            continue
        for index, (mine, theirs) in enumerate(
                zip(a["centerline"], e["centerline"])):
            if max(abs(mine[i] - theirs[i]) for i in range(3)) > tolerance:
                problems.append(
                    f"{lane_id}.centerline[{index}]: scene "
                    f"{[round(v, 3) for v in mine]} vs export "
                    f"{[round(v, 3) for v in theirs]}")
    return problems
