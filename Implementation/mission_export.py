"""
QGroundControl / Mission Planner mission export.

Waypoint altitude uses MAV_FRAME_GLOBAL_RELATIVE_ALT (`frame=3`), i.e. metres
above HOME, not above the local ground:

    rel_alt = alt_amsl_m − home_alt_amsl        home row: frame 0, home_alt_amsl (AMSL)

home_alt_amsl defaults to the ground under the first waypoint (alt_amsl_m − agl_m).
Over varying terrain writing agl_m here would fly the wrong height (audit defect #3).
Waypoints without alt_amsl_m (synthetic graphs, no DEM) fall back to agl_m, which
is only correct on flat ground.  The 120 m AGL limit is checked on agl_m.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

from module4_tier2_payload import SelectionResult


QGC_HEADER = "QGC WPL 110"
MAV_FRAME_GLOBAL = 0
MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
DEFAULT_AGL_M = 60.0
MAX_AGL_M = 120.0
BEARING_TOL_DEG = 2.0
ALT_SLOPE_TOL_M = 0.5        # m per DEGREE of lat/lon (≈ exact-match); per-metre 0.5 would merge real climbs


@dataclass
class MissionItem:
    index: int
    current: int
    frame: int
    command: int
    p1: float
    p2: float
    p3: float
    p4: float
    lat: float
    lon: float
    alt: float
    autocontinue: int

    def to_line(self) -> str:
        values = [
            self.index,
            self.current,
            self.frame,
            self.command,
            self.p1,
            self.p2,
            self.p3,
            self.p4,
            self.lat,
            self.lon,
            self.alt,
            self.autocontinue,
        ]
        return "\t".join(_format_value(v) for v in values)


def _format_value(value) -> str:
    if isinstance(value, int):
        return str(value)
    return f"{float(value):.8f}".rstrip("0").rstrip(".")


def _as_float(value, name: str) -> float:
    if value is None:
        raise ValueError(f"Waypoint missing {name}.")
    return float(value)


def waypoints_from_selection(result: SelectionResult) -> list[dict]:
    if result.hold_advisory:
        raise ValueError(f"Cannot export mission while route is on hold: {result.hold_advisory}")
    if not result.path:
        raise ValueError("SelectionResult has no waypoint path to export.")
    return list(result.path)


def _normalise_waypoint(waypoint: dict) -> dict:
    lat = _as_float(waypoint.get("lat"), "lat")
    lon = _as_float(waypoint.get("lon"), "lon")
    agl = waypoint.get("agl_m")
    if agl is None:
        agl = DEFAULT_AGL_M
    agl = float(agl)
    if agl < 0.0:
        raise ValueError(f"Waypoint altitude below ground: {agl:.2f} m.")
    if agl > MAX_AGL_M:
        raise ValueError(f"Waypoint altitude exceeds {MAX_AGL_M:.0f} m AGL: {agl:.2f} m.")
    amsl = waypoint.get("alt_amsl_m")
    return {
        "node": waypoint.get("node"),
        "lat": lat,
        "lon": lon,
        "agl_m": agl,
        "alt_amsl_m": None if amsl is None else float(amsl),
    }


def _bearing_deg(a: dict, b: dict) -> float:
    dy = b["lat"] - a["lat"]
    dx = b["lon"] - a["lon"]
    return (math.degrees(math.atan2(dy, dx)) + 360.0) % 360.0


def _angular_delta(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def _alt_slope(a: dict, b: dict) -> float:
    distance = math.hypot(b["lat"] - a["lat"], b["lon"] - a["lon"])
    if distance <= 1e-12:
        return 0.0
    key = "alt_amsl_m" if a.get("alt_amsl_m") is not None and b.get("alt_amsl_m") is not None else "agl_m"
    return (b[key] - a[key]) / distance


def merge_collinear_waypoints(
    waypoints: Iterable[dict],
    bearing_tol_deg: float = BEARING_TOL_DEG,
    alt_slope_tol_m: float = ALT_SLOPE_TOL_M,
) -> list[dict]:
    """Remove middle points with same bearing and altitude slope."""
    pts = [_normalise_waypoint(wp) for wp in waypoints]
    if len(pts) <= 2:
        return pts

    merged = [pts[0]]
    for idx in range(1, len(pts) - 1):
        prev = merged[-1]
        cur = pts[idx]
        nxt = pts[idx + 1]
        same_bearing = _angular_delta(_bearing_deg(prev, cur), _bearing_deg(cur, nxt)) <= bearing_tol_deg
        same_slope = abs(_alt_slope(prev, cur) - _alt_slope(cur, nxt)) <= alt_slope_tol_m
        if not (same_bearing and same_slope):
            merged.append(cur)
    merged.append(pts[-1])
    return merged


def build_mission_items(
    waypoints: Iterable[dict],
    *,
    merge_collinear: bool = True,
    end_with_rtl: bool = True,
    home_alt_amsl: float | None = None,
) -> list[MissionItem]:
    pts = [_normalise_waypoint(wp) for wp in waypoints]
    if len(pts) < 2:
        raise ValueError("Mission export needs at least two waypoints.")
    if merge_collinear:
        pts = merge_collinear_waypoints(pts)

    home = pts[0]
    if all(p["alt_amsl_m"] is not None for p in pts):
        if home_alt_amsl is None:
            home_alt_amsl = home["alt_amsl_m"] - home["agl_m"]
        home_alt = float(home_alt_amsl)
        rel = [p["alt_amsl_m"] - home_alt for p in pts]
    elif home_alt_amsl is not None:
        raise ValueError("home_alt_amsl given but some waypoints lack alt_amsl_m.")
    else:                                       # no DEM data: flat-ground assumption
        home_alt = home["agl_m"]
        rel = [p["agl_m"] for p in pts]
    items = [
        MissionItem(
            index=0,
            current=1,
            frame=MAV_FRAME_GLOBAL,
            command=MAV_CMD_NAV_WAYPOINT,
            p1=0.0,
            p2=0.0,
            p3=0.0,
            p4=0.0,
            lat=home["lat"],
            lon=home["lon"],
            alt=home_alt,
            autocontinue=1,
        )
    ]

    for seq, waypoint in enumerate(pts[1:], start=1):
        items.append(
            MissionItem(
                index=seq,
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_NAV_WAYPOINT,
                p1=0.0,
                p2=0.0,
                p3=0.0,
                p4=0.0,
                lat=waypoint["lat"],
                lon=waypoint["lon"],
                alt=rel[seq],
                autocontinue=1,
            )
        )

    if end_with_rtl:
        last = pts[-1]
        items.append(
            MissionItem(
                index=len(items),
                current=0,
                frame=MAV_FRAME_GLOBAL_RELATIVE_ALT,
                command=MAV_CMD_NAV_RETURN_TO_LAUNCH,
                p1=0.0,
                p2=0.0,
                p3=0.0,
                p4=0.0,
                lat=last["lat"],
                lon=last["lon"],
                alt=rel[-1],
                autocontinue=1,
            )
        )
    return items


def serialize_qgc_wpl(items: Iterable[MissionItem]) -> str:
    return QGC_HEADER + "\n" + "\n".join(item.to_line() for item in items) + "\n"


def parse_qgc_wpl(text: str) -> list[MissionItem]:
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if not lines or lines[0] != QGC_HEADER:
        raise ValueError("Missing QGC WPL 110 header.")
    items = []
    for line in lines[1:]:
        parts = line.split("\t")
        if len(parts) != 12:
            raise ValueError(f"Mission line must have 12 columns: {line}")
        items.append(
            MissionItem(
                index=int(parts[0]),
                current=int(parts[1]),
                frame=int(parts[2]),
                command=int(parts[3]),
                p1=float(parts[4]),
                p2=float(parts[5]),
                p3=float(parts[6]),
                p4=float(parts[7]),
                lat=float(parts[8]),
                lon=float(parts[9]),
                alt=float(parts[10]),
                autocontinue=int(parts[11]),
            )
        )
    return items


def export_qgc_wpl(
    result: SelectionResult,
    path: str | Path,
    *,
    merge_collinear: bool = True,
    end_with_rtl: bool = True,
    home_alt_amsl: float | None = None,
) -> Path:
    waypoints = waypoints_from_selection(result)
    items = build_mission_items(
        waypoints,
        merge_collinear=merge_collinear,
        end_with_rtl=end_with_rtl,
        home_alt_amsl=home_alt_amsl,
    )
    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(serialize_qgc_wpl(items), encoding="utf-8")
    return out_path
