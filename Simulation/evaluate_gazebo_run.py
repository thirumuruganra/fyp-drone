"""
Phase 5: planned-versus-flown evaluation of one SITL run.

    python evaluate_gazebo_run.py runs/<run_id>__<time>
    python evaluate_gazebo_run.py runs/*__*            # several runs → runs/summary.csv

Track source: Gazebo ground truth (gz_pose.csv, world frame) when present, else
MAVLink GLOBAL_POSITION_INT (lat/lon → world frame, z = relative altitude).

Metrics (metrics.json, one row per run in runs/summary.csv):
  cross-track error (horizontal, cruise legs only): RMSE / p95 / max
  vertical error vs the planned altitude at the nearest planned point: RMSE / max
  min clearance above any roof the drone flew over (footprint, and footprint + buffer)
  min terrain clearance in cruise, roof/terrain violations (z below roof inside a footprint)
  restricted-zone incursions, landing offset from the destination pad, completion,
  flight time, flown vs planned distance, planner objective vector (proxy units)
Plots: map overlay (buildings by height, Tier 1 front, planned, flown) and an
altitude profile (planned, flown, roof and terrain under the track).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

import _paths
import sim_config as scfg
from sim_region import RegionBundle


def _read_csv(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return {}
    out = {}
    for k in rows[0]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[k]))
            except (TypeError, ValueError):
                vals.append(math.nan)
        out[k] = np.asarray(vals)
    return out


def load_track(run_dir: Path, bundle: RegionBundle, prefer: str = "gz"):
    """(wall_s, sim_s, xyz world, source).  sim_s: simulation / autopilot clock (speed-up safe)."""
    gz = _read_csv(run_dir / "gz_pose.csv")
    if prefer == "gz" and gz and len(gz["x"]) > 10:
        return gz["wall_s"], gz["sim_s"], np.column_stack([gz["x"], gz["y"], gz["z"]]), "gazebo"
    gp = _read_csv(run_dir / "global_position.csv")
    if not gp or len(gp["lat"]) < 2:
        raise SystemExit(f"{run_dir}: no telemetry (gz_pose.csv / global_position.csv)")
    x, y = bundle.world_frame.to_local(gp["lat"], gp["lon"])
    return gp["wall_s"], gp["time_boot_ms"] / 1000.0, np.column_stack([x, y, gp["rel_alt_m"]]), "mavlink"


def mission_seq_at(run_dir: Path, wall_s: np.ndarray) -> np.ndarray:
    """Active mission item for every track sample (−1 before the first MISSION_CURRENT)."""
    ev = []
    path = run_dir / "mission_events.csv"
    if path.exists():
        with open(path, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                if r["event"] == "mission_current" and r["seq"] != "":
                    ev.append((float(r["wall_s"]), int(r["seq"])))
    seq = np.full(len(wall_s), -1)
    for t, s in sorted(ev):
        seq[wall_s >= t] = s
    return seq


def point_segment(p: np.ndarray, a: np.ndarray, b: np.ndarray):
    """Distance from 2-D points p to segment ab and the parameter t of the closest point."""
    d = b - a
    L2 = float(np.dot(d, d))
    t = np.zeros(len(p)) if L2 < 1e-12 else np.clip(((p - a) @ d) / L2, 0.0, 1.0)
    q = a + t[:, None] * d
    return np.linalg.norm(p - q, axis=1), t


def polyline_errors(track: np.ndarray, plan: np.ndarray):
    """Per-sample horizontal distance to the planned polyline and z − planned z there."""
    best = np.full(len(track), np.inf)
    zerr = np.zeros(len(track))
    for k in range(len(plan) - 1):
        dist, t = point_segment(track[:, :2], plan[k, :2], plan[k + 1, :2])
        better = dist < best
        best[better] = dist[better]
        zp = plan[k, 2] + t * (plan[k + 1, 2] - plan[k, 2])
        zerr[better] = track[better, 2] - zp[better]
    return best, zerr


def roof_under(bundle: RegionBundle, xyz: np.ndarray, buffer_m: float):
    """Highest roof AMSL over each sample (footprint buffered by buffer_m); −inf if none."""
    from shapely import STRtree, points
    geoms, roofs = [], []
    for g, row in bundle.local_footprints(bundle.world_frame):
        geoms.append(g.buffer(buffer_m) if buffer_m > 0 else g)
        roofs.append(float(row["roof_amsl_m"]))
    out = np.full(len(xyz), -np.inf)
    if not geoms:
        return out
    tree = STRtree(geoms)
    qi, ti = tree.query(points(xyz[:, 0], xyz[:, 1]), predicate="intersects")
    np.maximum.at(out, qi, np.asarray(roofs)[ti])
    return out


def restricted_hits(bundle: RegionBundle, xyz: np.ndarray) -> int:
    import shapely
    gdf = bundle.layer("restricted")
    if len(gdf) == 0:
        return 0
    frame = bundle.world_frame
    polys = [shapely.transform(g, lambda xy: np.column_stack(frame.to_local(xy[:, 1], xy[:, 0])))
             for g in gdf.geometry if g is not None and g.geom_type in ("Polygon", "MultiPolygon")]
    if not polys:
        return 0
    pts = shapely.points(xyz[:, 0], xyz[:, 1])
    return int(sum(np.count_nonzero(shapely.contains(p, pts)) for p in polys))


def path_length(xyz: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1))) if len(xyz) > 1 else 0.0


def evaluate(run_dir: Path, *, plots: bool = True, prefer: str = "gz") -> dict:
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "mission.json").read_text(encoding="utf-8"))
    manifest = json.loads((run_dir / "run_manifest.json").read_text(encoding="utf-8")) \
        if (run_dir / "run_manifest.json").exists() else {}
    bundle = RegionBundle.load(meta["region_id"])
    if bundle.bundle_hash != meta["bundle_hash"]:
        raise SystemExit(f"bundle {bundle.bundle_hash} != mission bundle {meta['bundle_hash']}")
    wall, sim_t, xyz, source = load_track(run_dir, bundle, prefer)
    plan = np.asarray(meta["planned_world_xyz"], dtype=float)
    n_items = len(meta["items"])
    seq = mission_seq_at(run_dir, wall)

    # cruise = from the first route waypoint (item 3) to the point above the destination pad
    first_wp, above_dest = 3, n_items - 2
    cruise = (seq >= first_wp) & (seq <= above_dest)
    cruise_plan = plan[1:-1]                       # takeoff top … above destination
    if cruise.sum() < 5:                           # no events: altitude-band fallback
        zmin = 0.8 * float(np.min(cruise_plan[:, 2]))
        cruise = xyz[:, 2] >= zmin
    airborne = xyz[:, 2] > 1.0

    xt, zerr = polyline_errors(xyz[cruise], cruise_plan) if cruise.any() else (np.zeros(0), np.zeros(0))
    z_amsl = xyz[:, 2] + bundle.h_home
    ground = bundle.ground(*bundle.world_frame.to_latlon(xyz[:, 0], xyz[:, 1]))
    roof0 = roof_under(bundle, xyz, 0.0)
    roofb = roof_under(bundle, xyz, scfg.SIM_EDGE_BUFFER_M)
    over0 = np.isfinite(roof0) & airborne
    overb = np.isfinite(roofb) & airborne
    violations = int(np.count_nonzero(np.isfinite(roof0) & (z_amsl < roof0)))
    dest = meta["pads"]["dest"]
    dx, dy = bundle.to_world(dest["lat"], dest["lon"])
    land_offset = float(np.hypot(xyz[-1, 0] - dx, xyz[-1, 1] - dy))
    t_air = sim_t[airborne]
    status = manifest.get("status", "unknown")

    m = {
        "run": run_dir.name, "run_id": meta["run_id"], "label": meta.get("label"), "track_source": source,
        "status": status,
        "completed": bool(status == "landed_disarmed" and land_offset <= scfg.SIM_LAND_TOL_M and violations == 0),
        "landing_offset_m": round(land_offset, 2),
        "flight_time_s": round(float(t_air[-1] - t_air[0]), 1) if len(t_air) > 1 else 0.0,
        "flown_distance_m": round(path_length(xyz[airborne]), 1),
        "planned_distance_m": round(path_length(plan), 1),
        "cruise_flown_m": round(path_length(xyz[cruise]), 1),
        "cruise_planned_m": round(path_length(cruise_plan), 1),
        "xtrack_rmse_m": round(float(np.sqrt(np.mean(xt ** 2))), 2) if len(xt) else None,
        "xtrack_p95_m": round(float(np.percentile(xt, 95)), 2) if len(xt) else None,
        "xtrack_max_m": round(float(xt.max()), 2) if len(xt) else None,
        "vert_rmse_m": round(float(np.sqrt(np.mean(zerr ** 2))), 2) if len(zerr) else None,
        "vert_max_abs_m": round(float(np.abs(zerr).max()), 2) if len(zerr) else None,
        "min_roof_clearance_m": round(float(np.min(z_amsl[over0] - roof0[over0])), 2) if over0.any() else None,
        "min_roof_clearance_buffered_m": round(float(np.min(z_amsl[overb] - roofb[overb])), 2) if overb.any() else None,
        "min_terrain_clearance_cruise_m": round(float(np.min(z_amsl[cruise] - ground[cruise])), 2) if cruise.any() else None,
        "roof_violations": violations,
        "restricted_incursions": restricted_hits(bundle, xyz[airborne]),
        "planned_min_roof_clearance_m": meta.get("planned_min_roof_clearance_m"),
        "objective_d_km": meta["selection"]["cost_vector"][0],
        "objective_energy_proxy": meta["selection"]["cost_vector"][1],
        "objective_risk": meta["selection"]["cost_vector"][2],
        "selected_index": meta["selection"]["selected_index"],
        "point_type": meta["selection"]["point_type"],
        "tags": meta["selection"]["tags"],
        "tier2_latency_ms": round(1000 * meta["selection"]["latency_s"], 2),
        "tier1_rebuilt": meta.get("tier1_rebuilt"), "front_size": meta["front"]["size"],
        "front_created_at": meta["front"]["created_at"], "bundle_hash": meta["bundle_hash"],
        "upload_s": manifest.get("upload_s"), "fixture_data": meta.get("fixture_data"),
    }
    (run_dir / "metrics.json").write_text(json.dumps(m, indent=2), encoding="utf-8")
    if plots:
        plot_run(run_dir, bundle, meta, xyz, plan, z_amsl, ground, roofb)
    return m


def plot_run(run_dir, bundle, meta, xyz, plan, z_amsl, ground, roofb):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    fig, ax = plt.subplots(figsize=(9, 9))
    polys, hs = [], []
    for g, row in bundle.local_footprints(bundle.world_frame):
        polys.append(np.asarray(g.exterior.coords))
        hs.append(float(row["height_m"]))
    if polys:
        pc = PolyCollection(polys, array=np.asarray(hs), cmap="Greys", edgecolor="none", alpha=0.8)
        ax.add_collection(pc)
        fig.colorbar(pc, ax=ax, shrink=0.6, label="building height (m)")
    front_file = run_dir / "front_world_xyz.json"
    if front_file.exists():
        for k, r in enumerate(json.loads(front_file.read_text(encoding="utf-8"))["front"]):
            f = np.asarray(r["xyz"])
            ax.plot(f[:, 0], f[:, 1], color="#4a90d9", lw=0.8, alpha=0.5, label="Tier 1 front" if k == 0 else None)
    ax.plot(plan[:, 0], plan[:, 1], color="#d62728", lw=2.5, label="planned (Tier 2 pick)")
    ax.plot(xyz[:, 0], xyz[:, 1], color="black", lw=1.2, label="flown")
    ax.scatter([plan[0, 0], plan[-1, 0]], [plan[0, 1], plan[-1, 1]], c=["#1f77b4", "#ff7f0e"], s=80, zorder=5)
    ax.set_aspect("equal")
    ax.set_xlabel("east (m, from depot)")
    ax.set_ylabel("north (m, from depot)")
    ax.set_title(f"{meta['run_id']}\n{meta.get('label')}  tags {meta['selection']['tags']}", fontsize=9)
    ax.legend(loc="best", fontsize=8)
    fig.savefig(run_dir / "map_overlay.png", dpi=150, bbox_inches="tight")
    plt.close(fig)

    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xyz[:, :2], axis=0), axis=1))])
    fig, ax = plt.subplots(figsize=(11, 4))
    roof_plot = np.where(np.isfinite(roofb), roofb, np.nan)
    ax.fill_between(s, ground, np.where(np.isfinite(roof_plot), roof_plot, ground), color="0.6",
                    alpha=0.6, step="mid", label="roof under track (+buffer)")
    ax.plot(s, ground, color="#8c6d31", lw=1, label="terrain")
    ax.plot(s, z_amsl, color="black", lw=1.2, label="flown")
    _, zerr = polyline_errors(xyz, plan)
    ax.plot(s, z_amsl - zerr, color="#d62728", lw=1.5, ls="--", label="planned (nearest point)")
    ax.set_xlabel("flown horizontal distance (m)")
    ax.set_ylabel("altitude (m AMSL)")
    ax.legend(loc="best", fontsize=8)
    fig.savefig(run_dir / "altitude_profile.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    p = argparse.ArgumentParser(description="Evaluate SITL runs against their planned routes.")
    p.add_argument("run_dirs", nargs="+")
    p.add_argument("--track", choices=["gz", "mavlink"], default="gz")
    p.add_argument("--no-plots", action="store_true")
    a = p.parse_args(argv)
    rows = []
    for rd in a.run_dirs:
        m = evaluate(Path(rd), plots=not a.no_plots, prefer=a.track)
        rows.append(m)
        print(f"[Eval] {m['run']}: completed={m['completed']} xtrack RMSE={m['xtrack_rmse_m']} m "
              f"max={m['xtrack_max_m']} m, min roof clearance={m['min_roof_clearance_m']} m, "
              f"violations={m['roof_violations']}, landing offset={m['landing_offset_m']} m ({m['track_source']})")
    summary = _paths.RUNS_DIR / "summary.csv"
    summary.parent.mkdir(parents=True, exist_ok=True)
    new = not summary.exists()
    with open(summary, "a", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        if new:
            w.writeheader()
        for r in rows:
            w.writerow({k: json.dumps(v) if isinstance(v, (dict, list)) else v for k, v in r.items()})
    print(f"[Eval] summary → {summary}")


if __name__ == "__main__":
    main()
