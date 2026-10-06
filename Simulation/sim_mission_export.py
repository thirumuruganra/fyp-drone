"""
ArduCopter mission for the Gazebo city: QGC `.plan` (Windows QGroundControl demo),
`QGC WPL 110` `.waypoints`, and the item list the SITL runner uploads.

Sequence (no RTL — the return leg is not a validated route):

    0  home            frame 0 (AMSL)  H_home at the depot pad
    1  NAV_TAKEOFF     frame 3         vertical climb at the depot pad to the first node's cruise altitude
    2  DO_CHANGE_SPEED                 conservative cruise speed (limits corner cutting)
    3… NAV_WAYPOINT    frame 3         every node of the Tier 2 route (Tier 1 lattice path)
    n  NAV_WAYPOINT    frame 3         above the destination pad, at the last node's altitude
    n+1 NAV_LAND       frame 3         descend and land on the destination pad

Altitude contract: rel_alt = alt_amsl_m − H_home (frame 3 = above HOME, not ground).
Waypoints without alt_amsl_m are rejected — no 60 m default on a real map.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Sequence

import _paths  # noqa: F401
import config as cfg
import sim_config as scfg
from mission_export import (MAV_FRAME_GLOBAL, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT,
                            MissionItem, serialize_qgc_wpl)

MAV_FRAME_MISSION = 2
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_DO_CHANGE_SPEED = 178

QGC_FIRMWARE_ARDUPILOT = 3
QGC_VEHICLE_QUADROTOR = 2
QGC_ALT_MODE_RELATIVE = 1


def _item(seq, frame, command, lat=0.0, lon=0.0, alt=0.0, p1=0.0, p2=0.0, p3=0.0, p4=0.0):
    return MissionItem(index=seq, current=1 if seq == 0 else 0, frame=frame, command=command,
                       p1=float(p1), p2=float(p2), p3=float(p3), p4=float(p4),
                       lat=float(lat), lon=float(lon), alt=float(alt), autocontinue=1)


def route_points(path: Sequence[dict], *, max_agl_m: float = cfg.H_MAX_AGL) -> list:
    """Validated (lat, lon, alt_amsl_m, agl_m) of the selected route's nodes."""
    pts = []
    for k, wp in enumerate(path):
        for key in ("lat", "lon", "alt_amsl_m", "agl_m"):
            if wp.get(key) is None:
                raise ValueError(f"waypoint {k} (node {wp.get('node')}) has no {key}; "
                                 "select with graph=G so the route carries altitudes")
        if float(wp["agl_m"]) > max_agl_m + 1e-6:
            raise ValueError(f"waypoint {k} is {wp['agl_m']:.1f} m AGL (> {max_agl_m:.0f} m)")
        pts.append({"node": wp.get("node"), "lat": float(wp["lat"]), "lon": float(wp["lon"]),
                    "alt_amsl_m": float(wp["alt_amsl_m"]), "agl_m": float(wp["agl_m"])})
    if len(pts) < 2:
        raise ValueError("route needs at least two nodes")
    return pts


def build_sim_mission(path: Sequence[dict], depot_pad: dict, dest_pad: dict, h_home: float, *,
                      speed_ms: float = scfg.SIM_CRUISE_SPEED_MS) -> list:
    """MissionItem list (seq 0 = home) for one delivery; see module docstring."""
    pts = route_points(path)
    rel = lambda amsl: amsl - h_home                              # noqa: E731
    items = [_item(0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, depot_pad["lat"], depot_pad["lon"], h_home)]
    items.append(_item(1, MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_TAKEOFF,
                       depot_pad["lat"], depot_pad["lon"], rel(pts[0]["alt_amsl_m"])))
    items.append(_item(2, MAV_FRAME_MISSION, MAV_CMD_DO_CHANGE_SPEED, p1=1, p2=speed_ms, p3=-1))
    for p in pts:
        items.append(_item(len(items), MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT,
                           p["lat"], p["lon"], rel(p["alt_amsl_m"])))
    items.append(_item(len(items), MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_WAYPOINT,
                       dest_pad["lat"], dest_pad["lon"], rel(pts[-1]["alt_amsl_m"])))
    items.append(_item(len(items), MAV_FRAME_GLOBAL_RELATIVE_ALT, MAV_CMD_NAV_LAND,
                       dest_pad["lat"], dest_pad["lon"], 0.0))
    for it in items[1:]:
        if it.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT and it.command != MAV_CMD_NAV_LAND and it.alt <= 0:
            raise ValueError(f"mission item {it.index} is not above home ({it.alt:.1f} m)")
    return items


def flight_profile(items: Sequence[MissionItem], h_home: float) -> list:
    """(lat, lon, alt_amsl) of every positional item after home, in flight order (for checks/plots)."""
    out = []
    for it in items[1:]:
        if it.command == MAV_CMD_DO_CHANGE_SPEED:
            continue
        out.append((it.lat, it.lon, it.alt + h_home))
    return out


def to_qgc_plan(items: Sequence[MissionItem], h_home: float, *,
                speed_ms: float = scfg.SIM_CRUISE_SPEED_MS) -> dict:
    """QGroundControl .plan JSON (ArduPilot multirotor, relative altitudes)."""
    home = items[0]
    plan_items = []
    for jump_id, it in enumerate(items[1:], start=1):
        if it.command == MAV_CMD_DO_CHANGE_SPEED:
            params = [it.p1, it.p2, it.p3, 0, 0, 0, 0]
        else:
            params = [it.p1, it.p2, it.p3, None, it.lat, it.lon, it.alt]
        entry = {"type": "SimpleItem", "autoContinue": True, "command": it.command,
                 "doJumpId": jump_id, "frame": it.frame, "params": params}
        if it.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT:
            entry.update({"Altitude": it.alt, "AltitudeMode": QGC_ALT_MODE_RELATIVE,
                          "AMSLAltAboveTerrain": None})
        plan_items.append(entry)
    return {
        "fileType": "Plan", "version": 1, "groundStation": "QGroundControl",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {
            "cruiseSpeed": speed_ms, "hoverSpeed": speed_ms,
            "firmwareType": QGC_FIRMWARE_ARDUPILOT, "vehicleType": QGC_VEHICLE_QUADROTOR,
            "globalPlanAltitudeMode": QGC_ALT_MODE_RELATIVE, "version": 2,
            "plannedHomePosition": [home.lat, home.lon, h_home],
            "items": plan_items,
        },
    }


def items_from_plan(plan: dict) -> list:
    """MissionItem list (with a home row) back from a .plan dict — round-trip check."""
    m = plan["mission"]
    hl = m["plannedHomePosition"]
    items = [_item(0, MAV_FRAME_GLOBAL, MAV_CMD_NAV_WAYPOINT, hl[0], hl[1], hl[2])]
    for e in m["items"]:
        p = [0.0 if v is None else v for v in e["params"]]
        if e["command"] == MAV_CMD_DO_CHANGE_SPEED:
            items.append(_item(len(items), e["frame"], e["command"], p1=p[0], p2=p[1], p3=p[2]))
        else:
            items.append(_item(len(items), e["frame"], e["command"], p[4], p[5], p[6], p[0], p[1], p[2]))
    return items


def write_mission_files(items: Sequence[MissionItem], h_home: float, out_dir: Path, meta: dict) -> dict:
    """mission.plan, mission.waypoints and mission.json (items + identity metadata)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {"plan": out_dir / "mission.plan", "waypoints": out_dir / "mission.waypoints",
             "json": out_dir / "mission.json"}
    paths["plan"].write_text(json.dumps(to_qgc_plan(items, h_home), indent=2), encoding="utf-8")
    paths["waypoints"].write_text(serialize_qgc_wpl(items), encoding="utf-8")
    paths["json"].write_text(json.dumps({**meta, "h_home_amsl_m": h_home,
                                         "items": [asdict(it) for it in items]}, indent=2),
                             encoding="utf-8")
    return paths


def load_mission(mission_dir: Path) -> tuple:
    """(metadata dict, MissionItem list) from mission.json."""
    d = json.loads((Path(mission_dir) / "mission.json").read_text(encoding="utf-8"))
    items = [MissionItem(**it) for it in d.pop("items")]
    return d, items
