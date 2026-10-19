"""
Phase 3 orchestrator: region bundle → clearance-checked lattice → Tier 1 front
(cached) → Tier 2 pick → ArduCopter mission + Gazebo route markers.

    python run_sim_mission.py --preset fragile_medical             # dest 0 of the bundle
    python run_sim_mission.py --payload 3 2 2 3 --dest-index 1
    python run_sim_mission.py --all-presets                        # selection proof: 4 picks, ONE front
    python run_sim_mission.py --baseline min-distance              # single-objective baseline

Writes Simulation/missions/<run_id>/:
    mission.plan            load in QGroundControl (Windows) → Upload → Start Mission
    mission.waypoints       QGC WPL 110 (same items)
    mission.json            items + identity (bundle hash, front, tags) for sitl_mission_runner.py
    route_markers.sdf       Tier 1 front (thin blue) + Tier 2 pick (thick red), spawned in Gazebo
    selection_<name>.json   Tier 2 decision record (module4 SelectionResult)

Tier 1 runs only when the cached front is missing or stale; a tag-only change
re-runs Tier 2 only (the log says "Reusing stored front").
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import pickle
import sys
import time
from contextlib import redirect_stdout
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import numpy as np

import _paths
import config as cfg
import sim_config as scfg
from env_data import LocalRasterProvider, WeatherSnapshot, build_layer_indexes
from gazebo_world_builder import route_markers_sdf, world_name
from module1_graph_builder import LatticeAirspaceBuilder
from module4_tier2_payload import PAYLOAD_PRESETS, SelectionResult, TierTwoSelector
from pareto_store import ParetoStore, front_from_record, graph_hash
from route_clearance import ClearanceChecker, NoFeasibleMission, check_polyline, connect_pad
from sim_mission_export import build_sim_mission, flight_profile, write_mission_files
from sim_region import RegionBundle

GRAPH_BUILDER_VERSION = "sim-lattice-1"
BASELINES = {"min-distance": 0, "min-energy": 1, "min-risk": 2}


# ── graph ────────────────────────────────────────────────────────────────────

def graph_key(bundle: RegionBundle, spacing_m: float, weather: WeatherSnapshot, risk_layers: bool) -> str:
    ident = {"bundle": bundle.bundle_hash, "spacing_m": spacing_m, "builder": GRAPH_BUILDER_VERSION,
             "buffer_m": scfg.SIM_EDGE_BUFFER_M, "roof_clear": scfg.SIM_ROOF_CLEARANCE_M,
             "terrain_clear": scfg.SIM_TERRAIN_CLEARANCE_M, "max_agl": cfg.H_MAX_AGL,
             "weather": weather.to_dict(), "risk_layers": risk_layers}
    return hashlib.sha256(json.dumps(ident, sort_keys=True).encode()).hexdigest()[:12]


def build_graph(bundle: RegionBundle, spacing_m: float, weather: WeatherSnapshot, *,
                risk_layers: bool = True, refresh: bool = False):
    """Lattice over the bundle box with hard roof/terrain clearance (cached pickle)."""
    key = graph_key(bundle, spacing_m, weather, risk_layers)
    path = bundle.root / f"graph_{key}.pkl"
    checker = ClearanceChecker.from_bundle(bundle)
    if path.exists() and not refresh:
        with open(path, "rb") as fh:
            G = pickle.load(fh)
        print(f"[Graph] Cached clearance-checked lattice {path.name}: "
              f"{G.number_of_nodes()} nodes, {G.number_of_edges()} edges")
        return G, checker, key
    layers = build_layer_indexes(bundle.planner_layers(), bundle.planner_frame, buildings=False,
                                 risk_layers=risk_layers)
    builder = LatticeAirspaceBuilder(
        center=bundle.center, box_m=bundle.box_m, spacing_m=spacing_m, weather=weather,
        location=f"sim-{bundle.region_id}",
        elevation_providers=[LocalRasterProvider(str(bundle.terrain_path))],
        building_index=bundle.building_index(), edge_filter=checker.edge_filter, **layers)
    G = builder.fetch_graph()
    G.graph.update(bundle_hash=bundle.bundle_hash, region_id=bundle.region_id, graph_key=key,
                   hard_clearance=True, clearance_checked_edges=checker.n_checked,
                   clearance_rejected_edges=checker.n_rejected)
    print(f"[Graph] Hard clearance: {checker.n_rejected} of {checker.n_checked} candidate edges rejected "
          f"(roof +{scfg.SIM_ROOF_CLEARANCE_M:.0f} m, buffer {scfg.SIM_EDGE_BUFFER_M:.0f} m, "
          f"terrain +{scfg.SIM_TERRAIN_CLEARANCE_M:.0f} m)")
    with open(path, "wb") as fh:
        pickle.dump(G, fh)
    return G, checker, key


# ── selection ────────────────────────────────────────────────────────────────

def baseline_selection(selector: TierTwoSelector, objective: str, G) -> SelectionResult:
    """Single-objective extreme of the same front (comparison baseline)."""
    k = BASELINES[objective]
    t0 = time.perf_counter()
    idx = int(np.argmin(selector.costs[:, k]))
    g, path = selector.pareto_front[idx]
    return SelectionResult(
        tags={}, weights={"distance": float(k == 0), "energy": float(k == 1), "risk": float(k == 2)},
        selected_index=idx, cost_vector=[float(x) for x in g],
        path=selector._waypoints_for_path(path, graph=G), normalised_scores=[],
        objective_ranks=selector._objective_ranks(idx), point_type=selector.classify_point(idx),
        latency_s=time.perf_counter() - t0, feasible_count=len(selector.pareto_front), hold_advisory=None)


def xyz_route(bundle: RegionBundle, G, path) -> list:
    return [tuple(float(v) for v in bundle.to_world(G.nodes[n]["lat"], G.nodes[n]["lon"],
                                                   G.nodes[n]["alt_amsl_m"])) for n in path]


def plan_mission(bundle: RegionBundle, G, checker: ClearanceChecker, record: dict, sel: SelectionResult,
                 depot_pad: dict, dest_pad: dict, run_id: str, out_root: Path, extra_meta: dict) -> Path:
    if sel.hold_advisory:
        raise NoFeasibleMission(sel.hold_advisory)
    items = build_sim_mission(sel.path, depot_pad, dest_pad, bundle.h_home)

    # Re-verify the exact flown polyline (pad connectors included) against roofs/terrain.
    prof = flight_profile(items, bundle.h_home)[:-1]          # last item = vertical landing
    pf = bundle.planner_frame
    xy = [tuple(float(v) for v in pf.to_local(la, lo)) for la, lo, _ in prof]
    legs = check_polyline(checker, xy, [z for _, _, z in prof])
    bad = [(k, r.reason) for k, r in enumerate(legs) if not r.ok]
    if bad:
        raise NoFeasibleMission(f"final mission fails clearance on leg(s) {bad}")
    finite = lambda v: None if v is None or not np.isfinite(v) else round(float(v), 3)   # noqa: E731
    min_roof = finite(min((r.min_roof_clearance_m for r in legs), default=None))      # None: no roof crossed
    min_terr = finite(min((r.min_terrain_clearance_m for r in legs), default=None))

    out = out_root / run_id
    out.mkdir(parents=True, exist_ok=True)
    planned_xyz = [tuple(float(v) for v in bundle.to_world(depot_pad["lat"], depot_pad["lon"],
                                                           depot_pad["ground_amsl_m"]))]
    planned_xyz += [tuple(float(v) for v in bundle.to_world(la, lo, z)) for la, lo, z in prof]
    planned_xyz.append(tuple(float(v) for v in bundle.to_world(dest_pad["lat"], dest_pad["lon"],
                                                               dest_pad["ground_amsl_m"])))
    front_xyz = [xyz_route(bundle, G, r["path"]) for r in record["front"]]
    (out / "route_markers.sdf").write_text(route_markers_sdf(planned_xyz, front_xyz), encoding="utf-8")

    meta = {
        "run_id": run_id, "region_id": bundle.region_id, "bundle_hash": bundle.bundle_hash,
        "world_name": world_name(bundle.region_id), "graph_key": G.graph.get("graph_key"),
        "graph_hash": graph_hash(G), "fixture_data": bundle.is_fixture,
        "front": {"location": record["location"], "source": record["source"], "target": record["target"],
                  "created_at": record["created_at"], "size": len(record["front"]),
                  "truncated": record["truncated"], "graph_hash": record["graph_hash"],
                  "identity": record.get("identity")},
        "selection": {k: v for k, v in asdict(sel).items() if k not in ("normalised_scores", "path")},
        "route_nodes": [wp["node"] for wp in sel.path],
        "pads": {"depot": depot_pad, "dest": dest_pad},
        "planned_world_xyz": planned_xyz,
        "planned_latlon_amsl": [[depot_pad["lat"], depot_pad["lon"], depot_pad["ground_amsl_m"]]]
                               + [list(p) for p in prof]
                               + [[dest_pad["lat"], dest_pad["lon"], dest_pad["ground_amsl_m"]]],
        "planned_min_roof_clearance_m": min_roof, "planned_min_terrain_clearance_m": min_terr,
        "cruise_speed_ms": scfg.SIM_CRUISE_SPEED_MS,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        **extra_meta,
    }
    write_mission_files(items, bundle.h_home, out, meta)
    with open(out / "front_world_xyz.json", "w", encoding="utf-8") as fh:
        json.dump({"front": [{"g": r["g"], "xyz": xyz} for r, xyz in zip(record["front"], front_xyz)]}, fh)
    return out


# ── CLI ──────────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--region-id", default=scfg.SIM_REGION_ID)
    g = p.add_mutually_exclusive_group()
    g.add_argument("--preset", choices=sorted(PAYLOAD_PRESETS))
    g.add_argument("--payload", nargs=4, type=int, metavar=("F", "W", "U", "P"))
    g.add_argument("--all-presets", action="store_true", help="one mission per preset from the same front")
    g.add_argument("--baseline", choices=sorted(BASELINES))
    p.add_argument("--dest-index", type=int, default=0, help="destination pad of the bundle (0-based)")
    p.add_argument("--spacing-m", type=float, default=scfg.SIM_SPACING_M)
    p.add_argument("--wind-override", nargs=2, type=float, metavar=("SPEED_KMH", "DIR_FROM_DEG"),
                   help="plan for this wind (the Gazebo world is calm unless wind is added to it)")
    p.add_argument("--no-risk-layers", action="store_true")
    p.add_argument("--refresh", action="store_true", help="rebuild the graph and the Tier 1 front")
    p.add_argument("--max-explored", type=int, default=200_000)
    p.add_argument("--allow-truncated", action="store_true",
                   help="accept a capped (possibly incomplete) front — label results approximate")
    a = p.parse_args(argv)

    bundle = RegionBundle.load(a.region_id)
    if bundle.is_fixture:
        print("[Sim] NOTE: region bundle built from HAND-MADE FIXTURES (smoke test, not real data)")
    weather = WeatherSnapshot.calm(*bundle.center)
    if a.wind_override:
        weather = weather.with_wind(*a.wind_override)
        print("[Sim] WARNING: planning with wind; add matching wind to the Gazebo world before comparing")

    G, checker, key = build_graph(bundle, a.spacing_m, weather, risk_layers=not a.no_risk_layers,
                                  refresh=a.refresh)
    if not 0 <= a.dest_index < len(bundle.pads["destinations"]):
        raise SystemExit(f"--dest-index must be 0..{len(bundle.pads['destinations']) - 1}")
    depot_pad = bundle.pads["depot"]
    dest_pad = bundle.pads["destinations"][a.dest_index]
    try:
        dep = connect_pad(G, depot_pad, checker)
        dst = connect_pad(G, dest_pad, checker)
    except NoFeasibleMission as exc:
        print(f"[Sim] NO FEASIBLE MISSION: {exc}")
        return 2
    if dep.node == dst.node:
        print("[Sim] NO FEASIBLE MISSION: depot and destination connect to the same lattice node")
        return 2
    print(f"[Sim] Depot → node {dep.node} ({dep.distance_m:.0f} m), "
          f"{dest_pad['name']} → node {dst.node} ({dst.distance_m:.0f} m)")

    store = ParetoStore(str(bundle.root / "fronts"))
    identity = {"bundle_hash": bundle.bundle_hash, "graph_key": key, "hard_clearance": True}
    t0 = time.perf_counter()
    record, rebuilt, reasons = store.get_or_build(
        G, dep.node, dst.node, location=f"sim-{bundle.region_id}", weather=weather, refresh=a.refresh,
        max_explored=a.max_explored, extra_identity=identity, valid_hours=scfg.SIM_FRONT_VALID_HOURS)
    tier1_s = time.perf_counter() - t0
    if record["truncated"] and not a.allow_truncated:
        print("[Sim] Tier 1 front is TRUNCATED (search cap hit). Raise --max-explored, or pass "
              "--allow-truncated and label results approximate.")
        return 3
    if not record["front"]:
        print("[Sim] NO FEASIBLE MISSION: empty Pareto front")
        return 2

    selector = TierTwoSelector(front_from_record(record), no_go_reasons=weather.no_go_reasons())
    runs = []
    if a.baseline:
        runs.append((a.baseline, None))
    elif a.payload:
        runs.append(("custom", tuple(a.payload)))
    elif a.all_presets:
        runs += [(n, None) for n in PAYLOAD_PRESETS]
    else:
        runs.append((a.preset or "mixed_cargo", None))

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    extra = {"tier1_rebuilt": rebuilt, "tier1_reasons": reasons, "tier1_wall_s": round(tier1_s, 3),
             "weather": weather.to_dict(), "graph_build_info": {k: G.graph.get(k) for k in (
                 "elevation_provider", "n_buildings", "blocked_nodes", "nfz_nodes", "nfz_edges",
                 "filtered_edges", "clearance_rejected_edges", "clearance_checked_edges")}}
    written = []
    for name, tags in runs:
        run_id = f"{stamp}_{bundle.region_id}_{dest_pad['name']}_{name}"
        k = 1
        while (_paths.MISSIONS_DIR / run_id).exists():            # same second, same label
            k += 1
            run_id = f"{stamp}_{bundle.region_id}_{dest_pad['name']}_{name}_{k}"
        out = _paths.MISSIONS_DIR / run_id
        if a.baseline:
            sel = baseline_selection(selector, a.baseline, G)
            (out).mkdir(parents=True, exist_ok=True)
            (out / f"selection_{name}.json").write_text(json.dumps(asdict(sel), indent=2), encoding="utf-8")
        else:
            t = tags or tuple(PAYLOAD_PRESETS[name][k] for k in "FWUP")
            with redirect_stdout(io.StringIO()):
                sel = selector.select_detailed(*t, graph=G, output_name=name, output_dir=str(out))
        try:
            mdir = plan_mission(bundle, G, checker, record, sel, depot_pad, dest_pad, run_id,
                                _paths.MISSIONS_DIR, {**extra, "label": name})
        except NoFeasibleMission as exc:
            print(f"[Sim] {name}: NO FEASIBLE MISSION — {exc}")
            continue
        written.append(mdir)
        c = sel.cost_vector
        print(f"[Sim] {name:<20s} route #{sel.selected_index} ({sel.point_type}) "
              f"d={c[0]:.3f} km ε={c[1]:.3f} ρ={c[2]:.3f}  Tier 2 {sel.latency_s * 1000:.1f} ms → {mdir}")
    if not written:
        return 2
    print(f"[Sim] Tier 1 {'REBUILT' if rebuilt else 'reused (no NAMOA* run)'} in {tier1_s:.2f} s; "
          f"front size {len(record['front'])}")
    print("[Sim] Next: gazebo/launch_region.sh {0} {1}   then   python sitl_mission_runner.py {1}".format(
        bundle.region_id, written[0]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
