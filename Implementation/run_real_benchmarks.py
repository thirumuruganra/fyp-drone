"""
run_real_benchmarks.py — Module 5 benchmark on lattices of real Chennai areas.
===============================================================================
Same method suite as run_benchmarks.py (NAMOA*, A* per objective, weighted-sum A*
per payload preset, repeated MOPSO), but every graph is a 200 m lattice built from
the cached OSM layers (buildings, restricted zones, water, land use) and DEM of one
area in config.BENCHMARK_AREAS.

Per area:
  * 3 origin-destination pairs (short / medium / long, config.BENCHMARK_OD_BANDS_M)
    x 3 fixed wind cases (config.BENCHMARK_WIND_CASES) on the 6 km box
  * robustness: the calm NAMOA* front re-costed under each wind
  * scaling: farthest pair on the 2 / 4 / 6 km boxes, south-west monsoon wind

Data is fetched once with --prepare (network) and the benchmark itself runs offline
from the caches, so reruns are reproducible.  Areas without buildings or with a flat
(missing) DEM are refused: an empty layer must not pass as real data.

OUTPUT (outputs/benchmarks_real/):
  areas_manifest.json          --prepare: layer counts, DEM provider per area
  cases/<case>.json            one file per case (resume cache, git-ignored)
  real_metrics.csv             case x method front quality
  real_mopso_runs.csv          MOPSO per run, real_mopso_summary.csv mean / std
  real_coverage.csv            Zitzler C(NAMOA*, MOPSO) and back
  real_tier2_evidence.csv      presets, ranks, alignment vs other pickers
  real_tier2_sweep.csv         225 tags per case (path column dropped)
  real_tier2_vs_weighted.csv   Tier 2 pick vs weighted-sum A* re-search
  real_monotonicity.csv        tag-monotonicity violations, per case
  real_robustness.csv          calm front under wind
  real_amortization.csv        cumulative planning time vs missions
  area_profiles.csv            buildings, NFZ, water, risk per area
  real_benchmark.json          every table + key numbers (input of --plots-only)
  real_*.png                   figures

HOW TO RUN:
    python run_real_benchmarks.py --prepare          # fetch + cache OSM / DEM (network)
    python run_real_benchmarks.py --smoke            # one area, 2 km box, tiny MOPSO
    python run_real_benchmarks.py                    # full run, resumes from cases/
    python run_real_benchmarks.py --report-scale     # MOPSO 30 x 100 x 200
    python run_real_benchmarks.py --plots-only       # redraw from real_benchmark.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import networkx as nx
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as cfg
from env_data import LocalFrame, WeatherSnapshot, fetch_osm_layers, fetch_osm_roads
from module1_graph_builder import LatticeAirspaceBuilder, apply_weather
from module4_tier2_payload import PAYLOAD_PRESETS, PRESET_COLOURS, TierTwoSelector
from module5_evaluation import (
    amortization_rows,
    area_profile,
    front_robustness,
    monotonicity_violations,
    risk_component_shares,
    summarize_runs,
    write_csv,
    write_json,
)
from run_ablation import draw_edge_map
from run_benchmarks import (MOPSO, draw_terrain_panel, evaluate_suite, finish_terrain_figure,
                            plot_fronts_2d, plot_fronts_3d, plot_metric_bars, run_method_suite,
                            terrain_figure)
from run_demo_osm import load_environment

SCALING_WIND = "sw_monsoon"
TABLES = ("metrics", "mopso_runs", "coverage", "tier2", "sweep", "tier2_vs_weighted",
          "risk_shares", "timing")
PLOT_METHODS = ("namoa", MOPSO, "astar_distance", "astar_energy", "astar_risk")
PICKERS = ("payload_alignment", "knee_alignment", "crowding_alignment", "fixed_equal_alignment",
           "astar_shortest_alignment")


class AreaDataError(RuntimeError):
    """Cached data for an area is missing or not real (no buildings / flat DEM)."""


# ─────────────────────────────────────────────────────────────────────────────
#  Areas, graphs, origin-destination pairs
# ─────────────────────────────────────────────────────────────────────────────

def select_areas(ids: list[str] | None) -> list[dict]:
    areas = list(cfg.BENCHMARK_AREAS)
    if not ids:
        return areas
    known = {a["id"]: a for a in areas}
    missing = [i for i in ids if i not in known]
    if missing:
        raise ValueError(f"Unknown area id(s) {missing}; known: {sorted(known)}")
    return [known[i] for i in ids]


def area_environment(area: dict, offline: bool, use_fixtures: bool):
    env_args = SimpleNamespace(offline=offline, no_terrain=False, no_buildings=False,
                               no_risk_layers=False, manual_nfz=False)
    return load_environment(env_args, center=tuple(area["center"]), use_fixtures=use_fixtures)


def wind_snapshot(area: dict, wind: str) -> WeatherSnapshot:
    speed, direction = cfg.BENCHMARK_WIND_CASES[wind]
    lat, lon = area["center"]
    return WeatherSnapshot.calm(lat, lon).with_wind(speed, direction)


def build_area_graph(area: dict, box_m: float, spacing_m: float, env, offline: bool) -> nx.DiGraph:
    """Calm lattice of one area; wind cases re-cost a copy with apply_weather()."""
    providers, layer_kwargs = env
    builder = LatticeAirspaceBuilder(
        center=tuple(area["center"]), box_m=box_m, spacing_m=spacing_m,
        weather=wind_snapshot(area, "calm"), offline=offline, location=area["name"],
        elevation_providers=providers, **layer_kwargs)
    return builder.fetch_graph()


def check_real_data(G: nx.DiGraph, area: dict) -> None:
    problems = []
    if int(G.graph.get("n_buildings", 0)) == 0:
        problems.append("no buildings")
    if str(G.graph.get("elevation_provider", "")).startswith(("flat", "none")):
        problems.append(f"no DEM ({G.graph.get('elevation_provider')})")
    if problems:
        raise AreaDataError(f"{area['id']}: {', '.join(problems)} — run --prepare on a "
                            "networked machine (or --allow-fixtures for tests)")


def weather_graphs(G: nx.DiGraph, area: dict, winds) -> dict:
    out = {}
    for wind in winds:
        Gw = G.copy()
        apply_weather(Gw, wind_snapshot(area, wind))
        out[wind] = Gw
    return out


def _largest_scc(G: nx.DiGraph) -> list:
    return sorted(max(nx.strongly_connected_components(G), key=len))


def _xy(G, n):
    return np.array([G.nodes[n]["x_m"], G.nodes[n]["y_m"]], dtype=float)


def pick_od_pairs(G: nx.DiGraph, bands: dict, seed: int, tries: int = 20_000) -> dict:
    """
    One seeded (source, target) per distance band, both in the largest SCC.
    A band with no pair inside the box is skipped.
    """
    nodes = _largest_scc(G)
    rng = np.random.default_rng(seed)
    xy = np.array([_xy(G, n) for n in nodes])
    pairs = {}
    for label, (lo, hi) in bands.items():
        hi = np.inf if hi is None else hi
        for _ in range(tries):
            i, j = rng.integers(0, len(nodes), size=2)
            d = float(np.linalg.norm(xy[i] - xy[j]))
            if i != j and lo <= d <= hi:
                pairs[label] = (nodes[i], nodes[j], d)
                break
        else:
            print(f"[Real] No {label} pair ({lo:.0f}-{hi:.0f} m) in this lattice; skipped")
    return pairs


def farthest_pair(G: nx.DiGraph) -> tuple:
    """Deterministic farthest pair of the largest SCC (corner to corner)."""
    nodes = _largest_scc(G)
    xy = np.array([_xy(G, n) for n in nodes])
    a = int(np.argmin(xy.sum(axis=1)))            # south-west-most
    b = int(np.argmax(np.linalg.norm(xy - xy[a], axis=1)))
    return nodes[a], nodes[b], float(np.linalg.norm(xy[a] - xy[b]))


# ─────────────────────────────────────────────────────────────────────────────
#  One case
# ─────────────────────────────────────────────────────────────────────────────

def case_key(area_id: str, box_m: float, wind: str, od: str) -> str:
    return f"{area_id}_{int(box_m)}m_{wind}_{od}"


def _jsonable_front(front):
    return [[[float(x) for x in g], [int(n) for n in path]] for g, path in front]


def _front_from_json(data):
    return [([float(x) for x in g], [int(n) for n in path]) for g, path in data]


def run_case(G, area, box_m, wind, od, pair, args, cases_dir: Path) -> dict:
    """Run (or load) one case: method suite + every Module 5 table."""
    key = case_key(area["id"], box_m, wind, od)
    path = cases_dir / f"{key}.json"
    if path.exists() and not args.force:
        with path.open() as fh:
            case = json.load(fh)
        if case.get("settings") == suite_settings(args):
            print(f"[Real] {key}: cached")
            return case
        print(f"[Real] {key}: cached with other settings, re-running")

    source, target, od_m = pair
    print(f"\n[Real] {key} | {G.number_of_nodes()} nodes | {source} -> {target} ({od_m:.0f} m)")
    suite = run_method_suite(
        G, source, target, max_explored=args.max_explored,
        mopso_repetitions=args.mopso_repetitions, swarm_size=args.swarm_size,
        iterations=args.iterations, archive_size=args.archive_size, seed=args.seed,
        workers=args.workers)
    labels = {"case": key, "area": area["id"], "box_m": int(box_m), "wind": wind, "od": od,
              "od_m": round(od_m, 1), "nodes": G.number_of_nodes()}
    result = evaluate_suite(suite, labels)
    for row in result["sweep"]:
        row.pop("path", None)

    front = suite["method_routes"]["namoa"]
    picks = {}
    if front:
        selector = TierTwoSelector(front)
        share_routes = []
        for name, tags in PAYLOAD_PRESETS.items():
            selected, *_ = selector.select(tags["F"], tags["W"], tags["U"], tags["P"])
            picks[name] = [int(n) for n in selected[1]]
            share_routes.append((name, selected))
        result["risk_shares"] = [dict(r, **labels) for r in risk_component_shares(G, share_routes)]

    runtimes = suite["runtimes"]
    weighted = [t for m, t in runtimes.items() if m.startswith("astar_weighted_")]
    latencies = [r["selection_latency_ms"] for r in result["sweep"]]
    t_tier2 = float(np.median(latencies)) / 1000.0 if latencies else 0.0
    t_mopso = float(np.mean(suite["mopso_run_times"])) if suite["mopso_run_times"] else 0.0
    timing = dict(labels, t_tier1_s=runtimes["namoa"], t_tier2_s=t_tier2,
                  t_weighted_astar_s=float(np.mean(weighted)) if weighted else 0.0,
                  t_mopso_run_s=t_mopso,
                  breakeven_missions_vs_mopso=(runtimes["namoa"] / (t_mopso - t_tier2)
                                               if t_mopso > t_tier2 else None),
                  front_min_distance=min((g[0] for g, _ in front), default=None),
                  front_min_energy=min((g[1] for g, _ in front), default=None),
                  front_min_risk=min((g[2] for g, _ in front), default=None),
                  truncated=bool(suite["diagnostics"]["namoa"].get("truncated", False)),
                  expanded=suite["diagnostics"]["namoa"].get("expanded", 0))

    case = {
        "key": key,
        "settings": suite_settings(args),
        "labels": labels,
        "source": int(source), "target": int(target),
        "namoa_front": _jsonable_front(front),
        "fronts": {m: [list(map(float, g)) for g, _ in suite["method_routes"][m]]
                   for m in PLOT_METHODS if m in suite["method_routes"]},
        "picks": picks,
        "reference": result["reference"],
        "namoa_diagnostics": suite["diagnostics"]["namoa"],
        "tables": {
            "metrics": result["metrics"],
            "mopso_runs": result["mopso_runs"],
            "coverage": result["coverage"],
            "tier2": result["tier2"],
            "sweep": result["sweep"],
            "tier2_vs_weighted": result["tier2_vs_weighted"],
            "risk_shares": result["risk_shares"],
            "timing": [timing],
        },
    }
    write_json(path, case)
    return case


def suite_settings(args) -> dict:
    return {"mopso_repetitions": args.mopso_repetitions, "swarm_size": args.swarm_size,
            "iterations": args.iterations, "archive_size": args.archive_size,
            "max_explored": args.max_explored, "seed": args.seed, "spacing_m": args.spacing_m}


# ─────────────────────────────────────────────────────────────────────────────
#  Prepare (network)
# ─────────────────────────────────────────────────────────────────────────────

def prepare_areas(args, out_dir: Path) -> dict:
    """Fetch + cache OSM layers, roads and DEM per area; refuse areas without real data."""
    path = out_dir / "areas_manifest.json"
    manifest = {"areas": {}}
    if path.exists():                                         # keep areas prepared earlier
        with path.open() as fh:
            manifest = json.load(fh)
    manifest["prepared_at"] = datetime.now().isoformat(timespec="seconds")
    for area in select_areas(args.areas):
        lat, lon = area["center"]
        print(f"\n[Prepare] {area['name']} ({lat:.4f}, {lon:.4f})")
        entry = {**area, "status": "ok"}
        try:
            layers = fetch_osm_layers(lat, lon, offline=False, fixture_dir=None)
            entry["layer_counts"] = {k: int(len(v)) for k, v in layers.items()}
            try:
                entry["roads"] = int(len(fetch_osm_roads(lat, lon, offline=False)))
            except ImportError:
                entry["roads"] = 0
            env = area_environment(area, offline=False, use_fixtures=False)
            for box_m in sorted(set(args.scaling_boxes) | {args.box_km * 1000.0}):
                G = build_area_graph(area, box_m, args.spacing_m, env, offline=False)
                check_real_data(G, area)
                entry[f"box_{int(box_m)}m"] = area_profile(G)
        except AreaDataError as exc:
            entry["status"] = f"rejected: {exc}"
        except Exception as exc:                              # network, Overpass, DEM
            entry["status"] = f"failed: {type(exc).__name__}: {exc}"
        print(f"[Prepare] {area['id']}: {entry['status']}")
        manifest["areas"][area["id"]] = entry
    write_json(path, manifest)
    return manifest


# ─────────────────────────────────────────────────────────────────────────────
#  Main run
# ─────────────────────────────────────────────────────────────────────────────

def run_area(area, args, cases_dir: Path, tables: dict, extras: dict) -> None:
    env = area_environment(area, offline=True, use_fixtures=args.allow_fixtures)
    box_m = args.box_km * 1000.0
    base = build_area_graph(area, box_m, args.spacing_m, env, offline=True)
    if not args.allow_fixtures:
        check_real_data(base, area)
    extras["profiles"].append({"area": area["id"], "name": area["name"], "box_m": int(box_m),
                               "note": area["note"], **area_profile(base)})

    graphs = weather_graphs(base, area, args.winds)
    pairs = pick_od_pairs(base, args.od_bands, seed=args.seed)
    cases = []
    for od, pair in pairs.items():
        fronts = {}
        for wind, G in graphs.items():
            case = run_case(G, area, box_m, wind, od, pair, args, cases_dir)
            cases.append(case)
            fronts[wind] = _front_from_json(case["namoa_front"])
        calm = fronts.get("calm")
        for wind, front in fronts.items():
            if wind == "calm" or not calm or not front:
                continue
            row = {"area": area["id"], "od": od, "wind": wind, "calm_front_size": len(calm),
                   "wind_front_size": len(front)}
            row.update(front_robustness(graphs[wind], calm, front))
            extras["robustness"].append(row)

    for scale_box in args.scaling_boxes:
        G = base if scale_box == box_m else build_area_graph(area, scale_box, args.spacing_m, env, True)
        G = weather_graphs(G, area, [SCALING_WIND])[SCALING_WIND]
        cases.append(run_case(G, area, scale_box, SCALING_WIND, "far", farthest_pair(G), args, cases_dir))

    for case in cases:
        for name in TABLES:
            tables[name].extend(case["tables"][name])

    map_wind = SCALING_WIND if SCALING_WIND in graphs else next(iter(graphs))
    map_od = "long" if "long" in pairs else next(iter(pairs), None)
    if map_od is not None:
        case = next(c for c in cases if c["key"] == case_key(area["id"], box_m, map_wind, map_od))
        extras["maps"].append((area, graphs[map_wind], case))
    extras["fronts"][area["id"]] = next(
        (c for c in cases if c["labels"]["box_m"] == int(box_m) and c["labels"]["od"] == map_od
         and c["labels"]["wind"] == map_wind), None)


def key_numbers(tables: dict, robustness: list, violations: list) -> dict:
    main = [r for r in tables["metrics"] if r["od"] != "far"]
    by_case: dict = {}
    for r in main:
        by_case.setdefault(r["case"], {})[r["method"]] = r
    runs: dict = {}
    for r in tables["mopso_runs"]:
        runs.setdefault(r["case"], []).append(r)
    namoa_ge_runs = sum(
        1 for case, rows in runs.items() if case in by_case and "namoa" in by_case[case]
        and all(by_case[case]["namoa"]["hypervolume"] >= r["hypervolume"] - 1e-12 for r in rows))
    tier2 = tables["tier2"]
    timing = tables["timing"]
    return {
        "cases": len(by_case),
        "cases_with_runs": len(runs),
        "namoa_truncated_cases": sum(1 for t in timing if t["truncated"]),
        "namoa_hv_ge_every_mopso_run": namoa_ge_runs,
        "namoa_hv_ratio_mean": _mean(r["hv_ratio"] for r in main if r["method"] == "namoa"),
        "mopso_pooled_hv_ratio_mean": _mean(r["hv_ratio"] for r in main if r["method"] == MOPSO),
        "mopso_run_hv_ratio_mean": _mean(r["hv_ratio"] for r in tables["mopso_runs"] if r["od"] != "far"),
        "mopso_run_igd_plus_mean": _mean(r["igd_plus"] for r in tables["mopso_runs"] if r["od"] != "far"),
        "namoa_front_size_median": _median(r["front_size"] for r in main if r["method"] == "namoa"),
        "tier2_non_extreme_share": _mean(r["point_type"] == "non-extreme" for r in tier2),
        "tier2_distinct_routes_mean": _mean(r["unique_routes_across_presets"] for r in tier2),
        "tier2_latency_ms_median": _median(r["selection_latency_ms"] for r in tables["sweep"]),
        "tier2_same_as_weighted_share": _mean(r["same_path"] for r in tables["tier2_vs_weighted"]),
        "monotonicity_violations": len(violations),
        "robustness_still_nondominated_mean": _mean(r["still_nondominated"] for r in robustness),
        "breakeven_missions_vs_mopso_median": _median(
            t["breakeven_missions_vs_mopso"] for t in timing if t["breakeven_missions_vs_mopso"] is not None),
    }


def _mean(values) -> float:
    vals = [float(v) for v in values]
    return float(np.mean(vals)) if vals else float("nan")


def _median(values) -> float:
    vals = [float(v) for v in values]
    return float(np.median(vals)) if vals else float("nan")


def run_real_benchmarks(args):
    out_dir = Path(args.output_dir)
    cases_dir = out_dir / "cases"
    out_dir.mkdir(parents=True, exist_ok=True)
    cases_dir.mkdir(parents=True, exist_ok=True)

    if args.prepare:
        return prepare_areas(args, out_dir)
    if args.plots_only:
        with (out_dir / "real_benchmark.json").open() as fh:
            data = json.load(fh)
        make_plots(data, out_dir, maps=None)
        return data

    tables = {name: [] for name in TABLES}
    extras = {"profiles": [], "robustness": [], "maps": [], "fronts": {}, "failed": {}}
    t0 = time.perf_counter()
    for area in select_areas(args.areas):
        print(f"\n[Real] ══ {area['name']} ({area['note']})")
        try:
            run_area(area, args, cases_dir, tables, extras)
        except AreaDataError as exc:
            print(f"[Real] SKIPPED {exc}")
            extras["failed"][area["id"]] = str(exc)
    if not tables["metrics"]:
        raise SystemExit("[Real] No area ran — run --prepare first (see areas_manifest.json)")

    violations = monotonicity_violations(tables["sweep"], group_keys=("case",))
    mopso_summary = summarize_runs(tables["mopso_runs"], ("method", "case", "area", "box_m", "wind", "od"))
    main_timing = [t for t in tables["timing"] if t["od"] != "far"]
    amortization = amortization_rows(
        _median(t["t_tier1_s"] for t in main_timing), _median(t["t_tier2_s"] for t in main_timing),
        _median(t["t_weighted_astar_s"] for t in main_timing),
        _median(t["t_mopso_run_s"] for t in main_timing), n_max=args.amortization_missions)

    write_csv(out_dir / "real_metrics.csv", tables["metrics"])
    write_csv(out_dir / "real_mopso_runs.csv", tables["mopso_runs"])
    write_csv(out_dir / "real_mopso_summary.csv", mopso_summary)
    write_csv(out_dir / "real_coverage.csv", tables["coverage"])
    write_csv(out_dir / "real_tier2_evidence.csv", tables["tier2"])
    write_csv(out_dir / "real_tier2_sweep.csv", tables["sweep"])
    write_csv(out_dir / "real_tier2_vs_weighted.csv", tables["tier2_vs_weighted"])
    write_csv(out_dir / "real_risk_shares.csv", tables["risk_shares"])
    write_csv(out_dir / "real_timing.csv", tables["timing"])
    write_csv(out_dir / "real_monotonicity.csv", violations)
    write_csv(out_dir / "real_robustness.csv", extras["robustness"])
    write_csv(out_dir / "real_amortization.csv", amortization)
    write_csv(out_dir / "area_profiles.csv", extras["profiles"])

    data = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "settings": suite_settings(args),
        "main_box_m": int(args.box_km * 1000.0),
        "winds": {w: cfg.BENCHMARK_WIND_CASES[w] for w in args.winds},
        "od_bands_m": args.od_bands,
        "failed_areas": extras["failed"],
        "key_numbers": key_numbers(tables, extras["robustness"], violations),
        "tables": {**{k: v for k, v in tables.items() if k != "sweep"},
                   "mopso_summary": mopso_summary, "robustness": extras["robustness"],
                   "amortization": amortization, "profiles": extras["profiles"],
                   "sweep_summary": sweep_summary(tables["sweep"])},
        "fronts": {area: case["fronts"] for area, case in extras["fronts"].items() if case},
    }
    write_json(out_dir / "real_benchmark.json", data)
    make_plots(data, out_dir, maps=extras["maps"])
    print(f"\n[Real] Done in {time.perf_counter() - t0:.0f} s -> {out_dir}")
    for k, v in data["key_numbers"].items():
        print(f"  {k:<40} {v}")
    return data


def sweep_summary(sweep_rows: list[dict]) -> list[dict]:
    """Per case: distinct picks and non-extreme share over the 225-tag sweep."""
    groups: dict = {}
    for r in sweep_rows:
        groups.setdefault(r["case"], []).append(r)
    return [{
        "case": case, "area": rows[0]["area"], "wind": rows[0]["wind"], "od": rows[0]["od"],
        "box_m": rows[0]["box_m"],
        "distinct_picks": len({r["selected_index"] for r in rows}),
        "non_extreme_share": _mean(r["point_type"] == "non-extreme" for r in rows),
        "latency_ms_median": _median(r["selection_latency_ms"] for r in rows),
    } for case, rows in groups.items()]


# ─────────────────────────────────────────────────────────────────────────────
#  Figures
# ─────────────────────────────────────────────────────────────────────────────

def _area_order(data) -> list[str]:
    seen = [p["area"] for p in data["tables"]["profiles"]]
    return seen or list(dict.fromkeys(r["area"] for r in data["tables"]["metrics"]))


def _save(fig, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def make_plots(data: dict, out_dir: Path, maps) -> None:
    t = data["tables"]
    box = int(data.get("main_box_m") or max(int(r["box_m"]) for r in t["metrics"]))
    main = [r for r in t["metrics"] if r["od"] != "far" and int(r["box_m"]) == box]
    plot_area_metrics(main, [r for r in t["mopso_runs"] if r["od"] != "far" and int(r["box_m"]) == box],
                      out_dir)
    plot_scaling(t["metrics"], t["mopso_runs"], out_dir / "real_scaling_runtime.png")
    plot_front_size(t["metrics"], t["profiles"], box, out_dir / "real_front_size.png")
    plot_fronts_2d(data.get("fronts", {}), _area_order(data), out_dir / "real_fronts_2d.png")
    plot_fronts_3d(data.get("fronts", {}), _area_order(data), out_dir / "real_fronts_3d.png",
                   f"Real areas ({box // 1000} km box, sw_monsoon, long pair): Pareto fronts in 3-D")
    timing = [r for r in t["timing"] if r["od"] != "far" and int(r["box_m"]) == box]
    plot_wind(main, timing, t["robustness"], _area_order(data), out_dir / "real_wind_sensitivity.png")
    plot_tier2(t["tier2"], _area_order(data), out_dir / "real_tier2.png")
    plot_amortization(t["amortization"], out_dir / "real_amortization.png")
    if maps:
        plot_maps(maps, out_dir / "real_area_maps.png")
        plot_terrain_maps(maps, out_dir / "real_terrain_maps.png")


def plot_area_metrics(rows, run_rows, out_dir: Path) -> None:
    """Grouped bars per area (mean over OD pairs and winds); MOPSO per-run mean +- std."""
    agg = []
    for area in dict.fromkeys(r["area"] for r in rows):
        for method in PLOT_METHODS:
            sel = [r for r in rows if r["area"] == area and r["method"] == method]
            if sel:
                agg.append({"area": area, "method": method,
                            **{m: _mean(r[m] for r in sel) for m in
                               ("runtime_s", "front_size", "hypervolume", "igd", "igd_plus",
                                "epsilon_add", "spread_delta")}})
    summary = summarize_runs(run_rows, ("area",))
    plot_metric_bars(agg, out_dir, summary, scenario_key="area", prefix="real")


def plot_scaling(metrics, run_rows, out: Path) -> None:
    far = [r for r in metrics if r["od"] == "far"]
    if not far:
        return
    fig, ax = plt.subplots(figsize=(8, 5.5))
    styles = {"namoa": ("o", "#1f77b4"), "astar_distance": ("^", "#2ca02c"),
              "mopso_run": ("s", "#d62728")}
    for method, (marker, colour) in styles.items():
        if method == "mopso_run":
            pts = [(r["nodes"], r["runtime_s"]) for r in run_rows if r["od"] == "far"]
            label = "MOPSO (one run)"
        else:
            pts = [(r["nodes"], r["runtime_s"]) for r in far if r["method"] == method]
            label = method
        if pts:
            x, y = zip(*pts)
            ax.scatter(x, y, marker=marker, color=colour, alpha=0.7, label=label, edgecolors="k",
                       linewidths=0.3)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("Lattice nodes (2 / 4 / 6 km boxes, all areas)")
    ax.set_ylabel("Runtime (s)")
    ax.set_title("Runtime vs graph size, farthest pair, south-west wind", fontweight="bold")
    ax.grid(True, which="both", linestyle="--", alpha=0.35)
    ax.legend()
    _save(fig, out)


def plot_front_size(metrics, profiles, box, out: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    far = [r for r in metrics if r["od"] == "far" and r["method"] == "namoa"]
    if far:
        axes[0].scatter([r["nodes"] for r in far], [r["front_size"] for r in far],
                        color="#1f77b4", edgecolors="k", linewidths=0.3)
    axes[0].set_xlabel("Lattice nodes")
    axes[0].set_ylabel("NAMOA* front size")
    axes[0].set_title("Front size vs graph size (farthest pair)", fontweight="bold")
    prof = {p["area"]: p for p in profiles}
    for area, p in prof.items():
        sizes = [r["front_size"] for r in metrics if r["area"] == area and r["method"] == "namoa"
                 and r["od"] != "far" and int(r["box_m"]) == box]
        if sizes:
            axes[1].errorbar(p["bldg_density_mean"], np.mean(sizes), yerr=np.std(sizes), fmt="o",
                             capsize=3)
            axes[1].annotate(area, (p["bldg_density_mean"], np.mean(sizes)), fontsize=8,
                             xytext=(4, 4), textcoords="offset points")
    axes[1].set_xlabel("Mean building plan-area density per node cell")
    axes[1].set_ylabel("NAMOA* front size (mean +- std, OD x wind)")
    axes[1].set_title("Front size vs urban density", fontweight="bold")
    for ax in axes:
        ax.grid(True, linestyle="--", alpha=0.35)
    _save(fig, out)


def plot_wind(main, timing, robustness, areas, out: Path) -> None:
    winds = list(dict.fromkeys(r["wind"] for r in main))
    fig, axes = plt.subplots(1, 3, figsize=(17, 5))
    x = np.arange(len(areas))
    width = 0.8 / max(1, len(winds))
    for k, wind in enumerate(winds):
        sizes = [_mean(r["front_size"] for r in main if r["area"] == a and r["wind"] == wind
                       and r["method"] == "namoa") for a in areas]
        axes[0].bar(x + (k - (len(winds) - 1) / 2) * width, sizes, width, label=wind)
    axes[0].set_title("NAMOA* front size by wind", fontweight="bold")
    axes[0].set_ylabel("Front size (mean over OD pairs)")

    calm_energy = {(r["area"], r["od"]): r["front_min_energy"] for r in timing if r["wind"] == "calm"}
    for k, wind in enumerate(winds):
        ratios = []
        for a in areas:
            vals = [r["front_min_energy"] / calm_energy[(a, r["od"])] for r in timing
                    if r["area"] == a and r["wind"] == wind and calm_energy.get((a, r["od"]))]
            ratios.append(_mean(vals))
        axes[1].bar(x + (k - (len(winds) - 1) / 2) * width, ratios, width, label=wind)
    axes[1].axhline(1.0, color="k", linewidth=0.8)
    axes[1].set_title("Min front energy relative to calm air", fontweight="bold")

    other = [w for w in winds if w != "calm"]
    grid = np.full((len(areas), len(other)), np.nan)
    for i, a in enumerate(areas):
        for j, w in enumerate(other):
            grid[i, j] = _mean(r["still_nondominated"] for r in robustness
                               if r["area"] == a and r["wind"] == w)
    im = axes[2].imshow(grid, cmap="RdYlGn", vmin=0, vmax=1, aspect="auto")
    axes[2].set_xticks(range(len(other)))
    axes[2].set_xticklabels(other)
    axes[2].set_yticks(range(len(areas)))
    axes[2].set_yticklabels(areas)
    for i in range(len(areas)):
        for j in range(len(other)):
            if np.isfinite(grid[i, j]):
                axes[2].text(j, i, f"{grid[i, j]:.2f}", ha="center", va="center", fontsize=9)
    axes[2].set_title("Calm-front routes still non-dominated under wind", fontweight="bold")
    fig.colorbar(im, ax=axes[2], shrink=0.8)
    for ax in axes[:2]:
        ax.set_xticks(x)
        ax.set_xticklabels(areas, rotation=20, ha="right")
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend(fontsize=8)
    _save(fig, out)


def plot_tier2(tier2, areas, out: Path) -> None:
    presets = list(PAYLOAD_PRESETS)
    fig, axes = plt.subplots(1, 2, figsize=(15, 5))
    grid = np.full((len(areas), len(presets)), np.nan)
    for i, a in enumerate(areas):
        for j, p in enumerate(presets):
            grid[i, j] = _mean(r["point_type"] == "non-extreme" for r in tier2
                               if r["area"] == a and r["preset"] == p)
    im = axes[0].imshow(grid, cmap="Blues", vmin=0, vmax=1, aspect="auto")
    axes[0].set_xticks(range(len(presets)))
    axes[0].set_xticklabels(presets, rotation=20, ha="right")
    axes[0].set_yticks(range(len(areas)))
    axes[0].set_yticklabels(areas)
    for i in range(len(areas)):
        for j in range(len(presets)):
            if np.isfinite(grid[i, j]):
                axes[0].text(j, i, f"{grid[i, j]:.2f}", ha="center", va="center", fontsize=9)
    axes[0].set_title("Share of Tier 2 picks that are non-extreme", fontweight="bold")
    fig.colorbar(im, ax=axes[0], shrink=0.8)

    data = [[float(r[p]) for r in tier2 if p in r] for p in PICKERS]
    names = ["Tier 2", "knee", "crowding", "fixed equal", "shortest"]
    try:
        axes[1].boxplot(data, tick_labels=names)
    except TypeError:                                  # matplotlib < 3.9
        axes[1].boxplot(data, labels=names)
    axes[1].set_ylabel("Payload alignment (rank based, higher better)")
    axes[1].set_title("Tier 2 vs other front pickers (all real cases)", fontweight="bold")
    axes[1].grid(axis="y", linestyle="--", alpha=0.35)
    _save(fig, out)


def plot_amortization(rows, out: Path) -> None:
    if not rows:
        return
    n = [r["missions"] for r in rows]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(n, [r["two_tier_s"] for r in rows], label="Two-tier (Tier 1 once + Tier 2 per mission)")
    ax.plot(n, [r["weighted_astar_s"] for r in rows], label="Weighted-sum A* per mission")
    ax.plot(n, [r["mopso_s"] for r in rows], label="MOPSO per mission (one run)")
    ax.set_yscale("log")
    ax.set_xlabel("Missions on one origin-destination pair")
    ax.set_ylabel("Cumulative planning time (s)")
    ax.set_title("Planning cost amortization (median real case)", fontweight="bold")
    ax.grid(True, which="both", linestyle="--", alpha=0.35)
    ax.legend(fontsize=8)
    _save(fig, out)


def _road_segments(area: dict) -> list:
    try:
        roads = fetch_osm_roads(area["center"][0], area["center"][1], offline=True)
    except Exception:
        return []
    frame = LocalFrame(*area["center"])
    segs = []
    for geom in roads.geometry:
        parts = getattr(geom, "geoms", [geom])
        for part in parts:
            if part.geom_type != "LineString":
                continue
            lon, lat = np.asarray(part.xy)
            x, y = frame.to_local(lat, lon)
            segs.append(np.column_stack([x, y]))
    return segs


def plot_maps(maps, out: Path) -> None:
    from matplotlib.collections import LineCollection
    cols = min(3, len(maps))
    rows = int(np.ceil(len(maps) / cols))
    # compressed layout packs the square panels + colorbar without vertical gaps
    fig, axes = plt.subplots(rows, cols, figsize=(5.6 * cols + 1.4, 5.6 * rows), squeeze=False,
                             layout="compressed")
    for ax in axes.ravel():
        ax.axis("off")
    vmax = max(max(d["rho"] for *_, d in G.edges(data=True)) for _, G, _ in maps)
    lc = None
    for ax, (area, G, case) in zip(axes.ravel(), maps):
        lc = draw_edge_map(ax, G, lambda d: d["rho"], vmax, f"{area['name']} — {case['labels']['wind']}, "
                           f"{case['labels']['od']} pair", case["source"], case["target"])
        lc.set_alpha(0.45)
        roads = _road_segments(area)
        if roads:
            ax.add_collection(LineCollection(roads, colors="#c8a165", linewidths=0.3, alpha=0.6, zorder=1))
        for _g, path in _front_from_json(case["namoa_front"]):
            xy = np.array([_xy(G, n) for n in path if n in G])
            ax.plot(xy[:, 0], xy[:, 1], color="#34495e", linewidth=0.6, alpha=0.3, zorder=3)
        for k, (name, path) in enumerate(case["picks"].items()):   # wide first: overlaps stay visible
            xy = np.array([_xy(G, n) for n in path if n in G])
            ax.plot(xy[:, 0], xy[:, 1], color=PRESET_COLOURS[name], linewidth=5.0 - 1.1 * k,
                    zorder=4 + k, label=name, solid_capstyle="round")
    handles, labels = axes.ravel()[0].get_legend_handles_labels()
    if handles:                                   # one shared legend: per-panel ones hid routes
        fig.legend(handles, labels, loc="upper center", ncol=len(labels), fontsize=10,
                   bbox_to_anchor=(0.45, 0.0), frameon=False)
    if lc is not None:
        fig.colorbar(lc, ax=axes.ravel().tolist(), shrink=0.5, label="edge risk ρ")
    fig.suptitle("Real-area lattices: edge risk, NAMOA* front (grey) and Tier 2 preset picks",
                 fontweight="bold")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


def _layer_polygons(gdf, frame: LocalFrame, bounds) -> list:
    """Exterior rings (local metres) of the polygons inside the lon/lat bounds."""
    if gdf is None or len(gdf) == 0:
        return []
    lon0, lat0, lon1, lat1 = bounds
    rings = []
    for geom in gdf.cx[lon0:lon1, lat0:lat1].geometry:
        for part in getattr(geom, "geoms", [geom]):
            if part.geom_type != "Polygon":
                continue
            lon, lat = np.asarray(part.exterior.xy)
            x, y = frame.to_local(lat, lon)
            rings.append(np.column_stack([x, y]))
    return rings


def plot_terrain_maps(maps, out: Path) -> None:
    """Per area: DEM elevation, OSM buildings / water / restricted zones, roads, lattice nodes and routes."""
    from matplotlib.collections import LineCollection, PolyCollection
    from matplotlib.patches import Patch
    from matplotlib.lines import Line2D
    zs = [d["ground_elev_m"] for _, G, _ in maps for _, d in G.nodes(data=True)]
    levels = np.linspace(np.floor(min(zs)), np.ceil(max(zs)), 21)
    fig, axes = terrain_figure(len(maps))
    terrain = None
    for ax, (area, G, case) in zip(axes.ravel(), maps):
        xy = lambda n, G=G: _xy(G, n)
        terrain = draw_terrain_panel(
            ax, G, xy, lambda n, G=G: G.nodes[n]["ground_elev_m"], levels,
            [path for _g, path in _front_from_json(case["namoa_front"])], case["picks"],
            case["source"], case["target"],
            f"{area['name']} ({G.number_of_nodes()} nodes, front {len(case['namoa_front'])})")
        P = np.array([xy(n) for n in G.nodes()])
        lons = [d["lon"] for _, d in G.nodes(data=True)]
        lats = [d["lat"] for _, d in G.nodes(data=True)]
        pad = 0.003
        bounds = (min(lons) - pad, min(lats) - pad, max(lons) + pad, max(lats) + pad)
        frame = LocalFrame(*area["center"])
        try:
            layers = fetch_osm_layers(area["center"][0], area["center"][1], offline=True)
        except Exception:
            layers = {}
        styles = {"buildings": dict(facecolors="#4d4d4d", edgecolors="none", alpha=0.45, zorder=1),
                  "water": dict(facecolors="#3b8fd9", edgecolors="none", alpha=0.55, zorder=1),
                  "restricted": dict(facecolors="#e74c3c", edgecolors="#c0392b", alpha=0.25,
                                     linewidths=1.0, hatch="//", zorder=1)}
        for name, style in styles.items():
            rings = _layer_polygons(layers.get(name), frame, bounds)
            if rings:
                ax.add_collection(PolyCollection(rings, **style))
        roads = _road_segments(area)
        if roads:
            ax.add_collection(LineCollection(roads, colors="#8c5a2b", linewidths=0.35, alpha=0.55, zorder=1))
        ax.set_xlim(P[:, 0].min() - 150, P[:, 0].max() + 150)   # layers reach past the lattice box
        ax.set_ylim(P[:, 1].min() - 150, P[:, 1].max() + 150)
    finish_terrain_figure(
        fig, axes, terrain, "Real-area airspace lattices: DEM terrain, OSM layers, nodes, "
                            "NAMOA* front and Tier 2 preset picks",
        [Patch(facecolor="#4d4d4d", alpha=0.45, label="buildings"),
         Patch(facecolor="#3b8fd9", alpha=0.55, label="water"),
         Patch(facecolor="#e74c3c", edgecolor="#c0392b", alpha=0.25, hatch="//", label="restricted zone"),
         Line2D([], [], color="#8c5a2b", linewidth=1, label="road (display only)"),
         Line2D([], [], color="#555555", alpha=0.5, label="lattice edge")])
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Module 5 benchmark on real Chennai-area lattices",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--prepare", action="store_true", help="fetch and cache OSM / DEM per area (network)")
    p.add_argument("--plots-only", action="store_true", help="redraw figures from real_benchmark.json")
    p.add_argument("--areas", nargs="+", help="area ids from config.BENCHMARK_AREAS (default all)")
    p.add_argument("--winds", nargs="+", default=list(cfg.BENCHMARK_WIND_CASES),
                   choices=list(cfg.BENCHMARK_WIND_CASES))
    p.add_argument("--box-km", type=float, default=cfg.BOX_M / 1000.0)
    p.add_argument("--spacing-m", type=float, default=cfg.LATTICE_SPACING_M)
    p.add_argument("--scaling-boxes", nargs="*", type=float, default=list(cfg.BENCHMARK_SCALING_BOX_M),
                   help="box sides (m) for the runtime-scaling cases; empty = none")
    p.add_argument("--max-explored", type=int, default=300_000, help="NAMOA* label-expansion cap")
    p.add_argument("--mopso-repetitions", type=int, default=10)
    p.add_argument("--swarm-size", type=int, default=50)
    p.add_argument("--iterations", type=int, default=100)
    p.add_argument("--archive-size", type=int, default=100)
    p.add_argument("--report-scale", action="store_true", help="MOPSO 30 runs x 100 particles x 200 iterations")
    p.add_argument("--smoke", action="store_true", help="one area, 2 km box, two winds, tiny MOPSO")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1),
                   help="parallel processes for the MOPSO repetitions (1 = serial)")
    p.add_argument("--amortization-missions", type=int, default=100)
    p.add_argument("--force", action="store_true", help="re-run cached cases")
    p.add_argument("--allow-fixtures", action="store_true",
                   help="tests only: accept hand-made fixtures / flat ground (NOT real data)")
    p.add_argument("--output-dir", default="outputs/benchmarks_real")
    args = p.parse_args(argv)
    args.od_bands = dict(cfg.BENCHMARK_OD_BANDS_M)
    if args.report_scale:
        args.mopso_repetitions, args.swarm_size, args.iterations = 30, 100, 200
    if args.smoke:
        args.areas = args.areas or [cfg.BENCHMARK_AREAS[0]["id"]]
        args.box_km = 2.0
        args.winds = ["calm", SCALING_WIND]
        args.scaling_boxes = [2000.0, 4000.0]
        args.od_bands = {"short": (800.0, 1500.0), "long": (1800.0, None)}
        args.mopso_repetitions, args.swarm_size, args.iterations, args.archive_size = 2, 8, 5, 30
        if args.output_dir == "outputs/benchmarks_real":
            args.output_dir = "outputs/benchmarks_real_smoke"
    return args


if __name__ == "__main__":
    from env_data import load_dotenv
    load_dotenv()                                   # OPENTOPO_API_KEY etc. from <repo>/.env
    run_real_benchmarks(parse_args())
