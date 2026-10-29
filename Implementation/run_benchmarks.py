"""
Run benchmark suite for Phase I algorithm evaluation (synthetic graphs).

Outputs:
  outputs/benchmarks/benchmark_metrics.csv     one row per scenario x method
  outputs/benchmarks/mopso_runs.csv            MOPSO scored per run (same reference)
  outputs/benchmarks/mopso_run_summary.csv     MOPSO mean / std / min / max per scenario
  outputs/benchmarks/coverage.csv              Zitzler C(A, B), NAMOA* vs MOPSO
  outputs/benchmarks/benchmark_metrics.json
  outputs/benchmarks/tier2_evidence.csv
  outputs/benchmarks/tier2_vs_weighted.csv
  outputs/benchmarks/*.png

The method suite (run_method_suite) is shared with run_real_benchmarks.py.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import os
from pathlib import Path
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from module1_graph_builder import AirspaceGraphBuilder
from module2_tier1_moastar import TierOneEngine
from module3_baselines import (
    mopso_baseline,
    objective_scales_from_extremes,
    single_objective_astar,
    weighted_sum_astar,
)
from module4_tier2_payload import PAYLOAD_PRESETS, PRESET_COLOURS, TierTwoSelector
from module5_evaluation import (
    coverage_rows,
    evaluate_method_set,
    evaluate_runs,
    monotonicity_violations,
    risk_component_shares,
    summarize_runs,
    tier2_evidence,
    tier2_sweep,
    tier2_vs_weighted,
    write_csv,
    write_json,
)


SCENARIOS = [
    {"name": "S1_50_d05", "nodes": 50, "density": 0.05, "seed": 101, "max_explored": 80_000},
    {"name": "S1_50_d20", "nodes": 50, "density": 0.20, "seed": 102, "max_explored": 80_000},
    {"name": "S1_50_d40", "nodes": 50, "density": 0.40, "seed": 103, "max_explored": 80_000},
    {"name": "S2_200_d05", "nodes": 200, "density": 0.05, "seed": 201, "max_explored": 180_000},
    {"name": "S2_200_d20", "nodes": 200, "density": 0.20, "seed": 202, "max_explored": 180_000},
    {"name": "S2_200_d40", "nodes": 200, "density": 0.40, "seed": 203, "max_explored": 180_000},
    {"name": "S3_500_d05", "nodes": 500, "density": 0.05, "seed": 301, "max_explored": 260_000},
    {"name": "S3_500_d20", "nodes": 500, "density": 0.20, "seed": 302, "max_explored": 260_000},
    {"name": "S3_500_d40", "nodes": 500, "density": 0.40, "seed": 303, "max_explored": 260_000},
]

MOPSO = "mopso_pooled"       # all MOPSO runs pooled into one archive (front-quality upper bound)
FRONT_STYLES = {"namoa": ("o", "#1f77b4", 18), MOPSO: ("x", "#d62728", 22),
                "astar_distance": ("^", "#2ca02c", 60), "astar_energy": ("v", "#9467bd", 60),
                "astar_risk": ("D", "#ff7f0e", 50)}
OBJECTIVE_LABELS = ["Distance (km)", "Energy (proxy)", "Risk exposure"]


def build_scenario(n_nodes: int, seed: int, density: float):
    """Synthetic graph whose obstacle discs cover `density` of the area."""
    builder = AirspaceGraphBuilder(seed=seed, obstacle_density=density)
    G = builder._build_synthetic_graph(n_nodes=n_nodes)
    builder.G = G
    nodes = list(G.nodes())
    return G, nodes[0], nodes[-1]


def run_namoa(G, source, target, max_explored: int):
    t0 = time.perf_counter()
    engine = TierOneEngine(G, source, target, max_explored=max_explored)
    routes = engine.run()
    return routes, time.perf_counter() - t0, engine.diagnostics()


def weighted_preset_results(G, source, target, front) -> dict:
    """Weighted-sum A* per payload preset, with the same (priority-sharpened) weights as Tier 2."""
    scales = objective_scales_from_extremes(G, source, target)
    selector = TierTwoSelector(front)
    out = {}
    for preset_name, tags in PAYLOAD_PRESETS.items():
        wr, wd, we = selector._derive_amplified_weights(tags["F"], tags["W"], tags["U"], tags["P"])
        out[preset_name] = weighted_sum_astar(
            G, source, target, {"distance": wd, "energy": we, "risk": wr},
            method_name=f"astar_weighted_{preset_name}", scales=scales)
    return out


def run_method_suite(G, source, target, *, max_explored: int, mopso_repetitions: int,
                     swarm_size: int, iterations: int, archive_size: int, seed: int,
                     workers: int = 1) -> dict:
    """
    NAMOA*, A* per objective, weighted-sum A* per payload preset and repeated MOPSO
    on one source/target pair.

    Returns {"method_routes", "runtimes", "diagnostics", "mopso_runs",
    "mopso_run_times", "weighted_by_preset"}.  method_routes[MOPSO] pools every run.
    workers > 1 runs the MOPSO repetitions in parallel processes (same seeds, same result).
    """
    method_routes: dict = {}
    runtimes: dict = {}
    diagnostics: dict = {}

    namoa_routes, namoa_runtime, namoa_diag = run_namoa(G, source, target, max_explored)
    method_routes["namoa"] = namoa_routes
    runtimes["namoa"] = namoa_runtime
    diagnostics["namoa"] = namoa_diag

    for objective in ("distance", "energy", "risk"):
        result = single_objective_astar(G, source, target, objective)
        method_routes[result.method] = result.routes
        runtimes[result.method] = result.runtime_s
        diagnostics[result.method] = {"expanded": "", "truncated": False}

    weighted_by_preset = {}
    if namoa_routes:
        for preset_name, result in weighted_preset_results(G, source, target, namoa_routes).items():
            method_routes[result.method] = result.routes
            runtimes[result.method] = result.runtime_s
            diagnostics[result.method] = {"expanded": "", "truncated": False}
            weighted_by_preset[preset_name] = result.routes

    kwargs = dict(swarm_size=swarm_size, iterations=iterations, archive_size=archive_size)
    if workers > 1 and mopso_repetitions > 1:
        with ProcessPoolExecutor(max_workers=min(workers, mopso_repetitions)) as pool:
            futures = [pool.submit(mopso_baseline, G, source, target, seed=seed + rep, **kwargs)
                       for rep in range(mopso_repetitions)]
            results = [f.result() for f in futures]
    else:
        results = [mopso_baseline(G, source, target, seed=seed + rep, **kwargs)
                   for rep in range(mopso_repetitions)]
    mopso_runs = [r.routes for r in results]
    mopso_times = [r.runtime_s for r in results]
    method_routes[MOPSO] = [route for run in mopso_runs for route in run]
    runtimes[MOPSO] = float(np.mean(mopso_times)) if mopso_times else 0.0
    diagnostics[MOPSO] = {"expanded": "", "truncated": False}

    return {
        "method_routes": method_routes,
        "runtimes": runtimes,
        "diagnostics": diagnostics,
        "mopso_runs": mopso_runs,
        "mopso_run_times": mopso_times,
        "weighted_by_preset": weighted_by_preset,
    }


def evaluate_suite(suite: dict, labels: dict) -> dict:
    """Every Module 5 table for one suite run; each row is tagged with `labels`."""
    method_routes = suite["method_routes"]
    rows, ref_meta = evaluate_method_set(method_routes, suite["runtimes"])
    for row in rows:
        diag = suite["diagnostics"].get(row["method"], {})
        row.update(labels, expanded=diag.get("expanded", ""), truncated=diag.get("truncated", False))

    run_rows = evaluate_runs(method_routes, {"mopso": suite["mopso_runs"]},
                             {"mopso": suite["mopso_run_times"]})
    for row in run_rows:
        row.update(labels)
    run_summary = summarize_runs(run_rows, ("method",) + tuple(labels))
    coverage = [dict(r, **labels) for r in coverage_rows(method_routes, ("namoa", MOPSO))]

    tier2_rows, sweep_rows, vs_weighted, shares = [], [], [], []
    front = method_routes["namoa"]
    if front:
        tier2_rows = [dict(r, **labels) for r in tier2_evidence(front)]
        sweep_rows = [dict(r, **labels) for r in tier2_sweep(front)]
        vs_weighted = [dict(r, **labels) for r in tier2_vs_weighted(front, suite["weighted_by_preset"])]
    return {
        "metrics": rows,
        "reference": ref_meta,
        "mopso_runs": run_rows,
        "mopso_summary": run_summary,
        "coverage": coverage,
        "tier2": tier2_rows,
        "sweep": sweep_rows,
        "tier2_vs_weighted": vs_weighted,
        "risk_shares": shares,
    }


def preset_share_routes(front, prefix: str) -> list:
    """Front routes plus the four preset picks, labelled for risk_component_shares()."""
    routes = [(f"{prefix}:front_{idx}", route) for idx, route in enumerate(front)]
    selector = TierTwoSelector(front)
    for preset_name, tags in PAYLOAD_PRESETS.items():
        selected, *_ = selector.select(tags["F"], tags["W"], tags["U"], tags["P"])
        routes.append((f"{prefix}:preset_{preset_name}", selected))
    return routes


def plot_metric_bars(rows: list[dict], out_dir: Path, run_summary: list[dict] | None = None,
                     scenario_key: str = "scenario", prefix: str = "benchmark") -> None:
    """Grouped bars per scenario x method; MOPSO per-run mean +- std drawn as its own bar."""
    summary = {r[scenario_key]: r for r in (run_summary or [])}
    methods = sorted({row["method"] for row in rows})
    if summary:
        methods.append("mopso_run_mean")
    scenarios = list(dict.fromkeys(row[scenario_key] for row in rows))
    metrics = [
        ("runtime_s", "Runtime (s)", "runtime"),
        ("front_size", "Front size", "front_size"),
        ("hypervolume", "Hypervolume (higher better)", "hv"),
        ("igd", "IGD (lower better)", "igd"),
        ("igd_plus", "IGD+ (lower better)", "igd_plus"),
        ("epsilon_add", "Additive epsilon (lower better)", "epsilon"),
        ("spread_delta", "Spread delta (lower better)", "spread"),
    ]
    out_dir.mkdir(parents=True, exist_ok=True)

    for metric, title, stem in metrics:
        fig, ax = plt.subplots(figsize=(max(10, 1.3 * len(scenarios)), 5.5))
        x = np.arange(len(scenarios))
        width = 0.8 / max(1, len(methods))
        for idx, method in enumerate(methods):
            vals, errs = [], []
            for scenario in scenarios:
                if method == "mopso_run_mean":
                    s = summary.get(scenario, {})
                    vals.append(s.get(f"{metric}_mean", np.nan))
                    errs.append(s.get(f"{metric}_std", 0.0))
                    continue
                match = next((row for row in rows
                              if row[scenario_key] == scenario and row["method"] == method), None)
                vals.append(match[metric] if match else np.nan)
                errs.append(0.0)
            ax.bar(x + (idx - (len(methods) - 1) / 2) * width, vals, width, label=method,
                   yerr=errs if method == "mopso_run_mean" else None, capsize=2)
        if metric == "runtime_s":
            ax.set_yscale("log")
        ax.set_title(title, fontweight="bold")
        ax.set_xticks(x)
        ax.set_xticklabels(scenarios, rotation=20, ha="right")
        ax.grid(axis="y", linestyle="--", alpha=0.35)
        ax.legend(fontsize=7, ncol=2)
        fig.tight_layout()
        fig.savefig(out_dir / f"{prefix}_{stem}.png", dpi=150, bbox_inches="tight")
        plt.close(fig)


def _save_figure(fig, path: Path) -> None:
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_fronts_2d(fronts: dict, labels: list[str], out: Path) -> None:
    """Pairwise objective projections, one row per scenario/area, methods overlaid."""
    labels = [l for l in labels if l in fronts]
    if not labels:
        return
    fig, axes = plt.subplots(len(labels), 3, figsize=(15, 3.6 * len(labels)), squeeze=False)
    for row, label in enumerate(labels):
        for col, (i, j) in enumerate([(0, 1), (0, 2), (1, 2)]):
            ax = axes[row][col]
            for method, pts in fronts[label].items():
                if not pts or method not in FRONT_STYLES:
                    continue
                arr = np.array(pts)
                marker, colour, size = FRONT_STYLES[method]
                ax.scatter(arr[:, i], arr[:, j], marker=marker, color=colour, s=size, alpha=0.7,
                           label=method)
            ax.set_xlabel(OBJECTIVE_LABELS[i], fontsize=8)
            ax.set_ylabel(OBJECTIVE_LABELS[j], fontsize=8)
            ax.grid(True, linestyle="--", alpha=0.35)
            ax.ticklabel_format(useOffset=False)     # narrow ranges (T. Nagar) got "+5.248" offsets
            if col == 0:
                ax.set_title(label, fontweight="bold", loc="left")
        axes[row][0].legend(fontsize=7)
    fig.tight_layout()
    _save_figure(fig, out)


def plot_fronts_3d(fronts: dict, labels: list[str], out: Path, title: str) -> None:
    """3-D objective space per scenario/area: NAMOA* front vs MOPSO archive vs single-objective A*."""
    labels = [l for l in labels if l in fronts]
    if not labels:
        return
    cols = min(3, len(labels))
    rows = int(np.ceil(len(labels) / cols))
    fig = plt.figure(figsize=(5.4 * cols, 4.8 * rows))
    first = None
    for k, label in enumerate(labels):
        ax = fig.add_subplot(rows, cols, k + 1, projection="3d")
        first = first or ax
        for method, pts in fronts[label].items():
            if not pts or method not in FRONT_STYLES:
                continue
            arr = np.array(pts)
            marker, colour, size = FRONT_STYLES[method]
            ax.scatter(arr[:, 0], arr[:, 1], arr[:, 2], marker=marker, color=colour, s=size,
                       alpha=0.75, depthshade=False, label=method)
        ax.set_xlabel(OBJECTIVE_LABELS[0], fontsize=7, labelpad=2)
        ax.set_ylabel(OBJECTIVE_LABELS[1], fontsize=7, labelpad=2)
        ax.set_zlabel(OBJECTIVE_LABELS[2], fontsize=7, labelpad=6)
        ax.tick_params(labelsize=6, pad=0)
        try:
            ax.set_box_aspect(None, zoom=0.88)        # room for the z label (matplotlib >= 3.7)
        except TypeError:
            pass
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.get_major_formatter().set_useOffset(False)
        ax.view_init(elev=22, azim=-58)
        ax.set_title(label, fontweight="bold", fontsize=10)
    handles, names = first.get_legend_handles_labels()
    fig.legend(handles, names, loc="lower center", ncol=len(names), fontsize=9, frameon=False)
    fig.suptitle(title, fontweight="bold")
    fig.subplots_adjust(bottom=0.06, wspace=0.05, hspace=0.18)
    _save_figure(fig, out)


def draw_terrain_panel(ax, G, xy, elev, levels, front_paths, picks, source, target, title,
                       edge_values=None):
    """
    One airspace-graph panel: ground elevation as filled contours, lattice edges,
    nodes as dots, NAMOA* front routes (grey) and Tier 2 preset picks on top.
    edge_values(d) -> [0, 1] colours edges (synthetic obstacle term); None = plain grey.
    Returns the contour set (for a shared elevation colorbar).
    """
    from matplotlib.collections import LineCollection
    nodes = list(G.nodes())
    P = np.array([xy(n) for n in nodes])
    Z = np.array([elev(n) for n in nodes])
    terrain = ax.tricontourf(P[:, 0], P[:, 1], Z, levels=levels, cmap="terrain", alpha=0.75,
                             extend="both", zorder=0)
    segs = [[xy(u), xy(v)] for u, v in G.edges()]
    if edge_values is None:
        edges = LineCollection(segs, colors="#555555", linewidths=0.25, alpha=0.3, zorder=2)
    else:
        edges = LineCollection(segs, cmap="Reds", linewidths=0.6, alpha=0.7, zorder=2)
        edges.set_array(np.array([edge_values(d) for *_, d in G.edges(data=True)]))
        edges.set_clim(0.0, 1.0)
    ax.add_collection(edges)
    ax.scatter(P[:, 0], P[:, 1], s=3, c="#222222", linewidths=0, zorder=3)
    for path in front_paths:
        pts = np.array([xy(n) for n in path if n in G])
        ax.plot(pts[:, 0], pts[:, 1], color="#34495e", linewidth=0.7, alpha=0.45, zorder=4)
    for k, (name, path) in enumerate(picks.items()):           # wide first: overlaps stay visible
        pts = np.array([xy(n) for n in path if n in G])
        ax.plot(pts[:, 0], pts[:, 1], color=PRESET_COLOURS[name], linewidth=4.5 - 1.0 * k,
                zorder=5 + k, label=name, solid_capstyle="round")
    for node, marker in ((source, "o"), (target, "*")):
        if node in G:
            p = xy(node)
            ax.scatter([p[0]], [p[1]], marker=marker, s=110, c="white", edgecolors="black",
                       linewidths=1.2, zorder=10)
    ax.set_aspect("equal")
    ax.margins(0.04)                                  # keep corner source/target markers whole
    ax.set_title(title, fontweight="bold", fontsize=10)
    ax.set_xticks([])
    ax.set_yticks([])
    return terrain


def terrain_figure(n_panels: int):
    cols = min(3, n_panels)
    rows = int(np.ceil(n_panels / cols))
    # compressed layout packs equal-aspect panels + colorbar without big row gaps
    fig, axes = plt.subplots(rows, cols, figsize=(5.6 * cols + 1.4, 5.6 * rows), squeeze=False,
                             layout="compressed")
    for ax in axes.ravel()[n_panels:]:
        ax.axis("off")
    return fig, axes


def finish_terrain_figure(fig, axes, terrain, title, extra_handles=()):
    """Shared elevation colorbar + one legend (preset picks, front, nodes, extra layers)."""
    from matplotlib.lines import Line2D
    handles, names = axes.ravel()[0].get_legend_handles_labels()
    handles += [Line2D([], [], color="#34495e", alpha=0.6, label="NAMOA* front"),
                Line2D([], [], marker="o", linestyle="", color="#222222", markersize=3,
                       label="lattice node"),
                Line2D([], [], marker="o", linestyle="", markerfacecolor="white",
                       markeredgecolor="black", markersize=8, label="source"),
                Line2D([], [], marker="*", linestyle="", markerfacecolor="white",
                       markeredgecolor="black", markersize=11, label="target"),
                *extra_handles]
    fig.legend(handles=handles, loc="upper center", ncol=min(6, len(handles)), fontsize=9,
               bbox_to_anchor=(0.45, 0.0), frameon=False)          # below the panels, never over them
    if terrain is not None:
        fig.colorbar(terrain, ax=axes.ravel().tolist(), shrink=0.5, label="ground elevation (m)")
    fig.suptitle(title, fontweight="bold")
    return fig


def plot_synthetic_graphs(panels: list[dict], out: Path) -> None:
    """Synthetic graphs: seeded terrain, obstacle term per edge, front routes and preset picks."""
    if not panels:
        return
    from matplotlib.lines import Line2D
    lat0 = np.radians(np.mean([d["y"] for p in panels for _, d in p["G"].nodes(data=True)]))

    def xy_of(G):
        lon0 = min(d["x"] for _, d in G.nodes(data=True))
        la0 = min(d["y"] for _, d in G.nodes(data=True))
        return lambda n: np.array([(G.nodes[n]["x"] - lon0) * 111_320.0 * np.cos(lat0),
                                   (G.nodes[n]["y"] - la0) * 110_574.0])

    zmax = max(d["alt_m"] for p in panels for _, d in p["G"].nodes(data=True))
    levels = np.linspace(0.0, max(zmax, 1.0), 21)
    fig, axes = terrain_figure(len(panels))
    terrain = None
    for ax, p in zip(axes.ravel(), panels):
        G = p["G"]
        terrain = draw_terrain_panel(
            ax, G, xy_of(G), lambda n, G=G: G.nodes[n]["alt_m"], levels,
            [path for _, path in p["front"]], p["picks"], p["source"], p["target"],
            f"{p['name']} ({G.number_of_nodes()} nodes, front {len(p['front'])})",
            edge_values=lambda d: float(d.get("obstacle", 0.0)))
    finish_terrain_figure(
        fig, axes, terrain, "Synthetic graphs: terrain, obstacle term per edge (red), "
                            "NAMOA* front and Tier 2 preset picks",
        [Line2D([], [], color="#c0392b", linewidth=2, label="edge obstacle term")])
    _save_figure(fig, out)


def preset_picks(front) -> dict:
    """Tier 2 path picked for each payload preset."""
    if not front:
        return {}
    selector = TierTwoSelector(front)
    picks = {}
    for name, tags in PAYLOAD_PRESETS.items():
        selected, *_ = selector.select(tags["F"], tags["W"], tags["U"], tags["P"])
        picks[name] = list(selected[1])
    return picks


def run_benchmarks(args):
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    tables = {k: [] for k in ("metrics", "mopso_runs", "mopso_summary", "coverage", "tier2",
                              "sweep", "tier2_vs_weighted", "risk_shares")}
    metadata = {"scenarios": [], "configuration": vars(args)}
    fronts: dict = {}
    panels: list = []

    scenarios = SCENARIOS if args.scenario == "all" else [
        s for s in SCENARIOS if s["name"].startswith(args.scenario)
    ]
    if not scenarios:
        raise ValueError(f"No matching scenario for {args.scenario!r}")

    for scenario in scenarios:
        print(f"\n[Benchmark] {scenario['name']} | nodes={scenario['nodes']} density={scenario['density']}")
        G, source, target = build_scenario(scenario["nodes"], scenario["seed"], scenario["density"])
        suite = run_method_suite(
            G, source, target, max_explored=scenario["max_explored"],
            mopso_repetitions=args.mopso_repetitions, swarm_size=args.swarm_size,
            iterations=args.iterations, archive_size=args.archive_size, seed=scenario["seed"],
            workers=args.workers)
        labels = {"scenario": scenario["name"], "nodes": scenario["nodes"], "density": scenario["density"]}
        result = evaluate_suite(suite, labels)
        if suite["method_routes"]["namoa"]:
            result["risk_shares"] = risk_component_shares(
                G, preset_share_routes(suite["method_routes"]["namoa"], scenario["name"]))
        for key in tables:
            tables[key].extend(result[key])
        front = suite["method_routes"]["namoa"]
        fronts[scenario["name"]] = {m: [list(map(float, g)) for g, _ in routes]
                                    for m, routes in suite["method_routes"].items() if m in FRONT_STYLES}
        panels.append({"name": scenario["name"], "G": G, "front": front, "picks": preset_picks(front),
                       "source": source, "target": target})

        metadata["scenarios"].append({
            **scenario,
            "source": source,
            "target": target,
            "obstacle_density": density_of(G, scenario["density"]),
            "reference": result["reference"],
            "namoa_diagnostics": suite["diagnostics"]["namoa"],
            "method_route_counts_raw": {k: len(v) for k, v in suite["method_routes"].items()},
        })

    violations = monotonicity_violations(tables["sweep"])
    write_csv(out_dir / "benchmark_metrics.csv", tables["metrics"])
    write_csv(out_dir / "mopso_runs.csv", tables["mopso_runs"])
    write_csv(out_dir / "mopso_run_summary.csv", tables["mopso_summary"])
    write_csv(out_dir / "coverage.csv", tables["coverage"])
    write_csv(out_dir / "tier2_evidence.csv", tables["tier2"])
    write_csv(out_dir / "tier2_sweep.csv", tables["sweep"])
    write_csv(out_dir / "tier2_vs_weighted.csv", tables["tier2_vs_weighted"])
    write_csv(out_dir / "risk_component_shares.csv", tables["risk_shares"])
    write_csv(out_dir / "tier2_monotonicity_violations.csv", violations)
    sweep_rows = tables["sweep"]
    write_json(out_dir / "benchmark_metrics.json", {
        "metadata": metadata,
        "metrics": tables["metrics"],
        "mopso_run_summary": tables["mopso_summary"],
        "coverage": tables["coverage"],
        "tier2_evidence": tables["tier2"],
        "tier2_vs_weighted": tables["tier2_vs_weighted"],
        "tier2_sweep_summary": {
            "rows": len(sweep_rows),
            "distinct_selected_points": len({(r["scenario"], r["selected_index"]) for r in sweep_rows}),
            "non_extreme_picks": sum(1 for r in sweep_rows if r["point_type"] == "non-extreme"),
            "monotonicity_violations": len(violations),
            "latency_ms_median": float(np.median([r["selection_latency_ms"] for r in sweep_rows])) if sweep_rows else 0.0,
            "latency_ms_p95": float(np.percentile([r["selection_latency_ms"] for r in sweep_rows], 95)) if sweep_rows else 0.0,
        },
    })
    plot_metric_bars(tables["metrics"], out_dir, tables["mopso_summary"])
    plot_fronts_2d(fronts, list(fronts), out_dir / "benchmark_fronts_2d.png")
    plot_fronts_3d(fronts, list(fronts), out_dir / "benchmark_fronts_3d.png",
                   "Synthetic scenarios: Pareto fronts in 3-D objective space")
    plot_synthetic_graphs(panels, out_dir / "benchmark_graphs.png")

    print(f"\n[Benchmark] Wrote outputs -> {out_dir}")
    return tables["metrics"], tables["tier2"]


def density_of(G, requested: float) -> dict:
    """Requested obstacle density vs. share of edges with a non-zero obstacle term."""
    obstacle = [float(d.get("obstacle", 0.0)) for *_, d in G.edges(data=True)]
    return {
        "requested": requested,
        "edge_share_obstacle_gt_0_1": float(np.mean([o > 0.1 for o in obstacle])) if obstacle else 0.0,
        "edge_obstacle_mean": float(np.mean(obstacle)) if obstacle else 0.0,
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", default="all", help="all, S1, S2, or S3")
    parser.add_argument("--mopso-repetitions", type=int, default=30)
    parser.add_argument("--swarm-size", type=int, default=100)
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--archive-size", type=int, default=100)
    parser.add_argument("--output-dir", default="outputs/benchmarks")
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1),
                        help="parallel processes for the MOPSO repetitions (1 = serial)")
    parser.add_argument("--smoke", action="store_true", help="small deterministic settings for quick local checks")
    args = parser.parse_args(argv)
    if args.smoke:
        args.mopso_repetitions = 2
        args.swarm_size = 12
        args.iterations = 10
        args.archive_size = 30
        if args.scenario == "all":
            args.scenario = "S1"
    return args


if __name__ == "__main__":
    run_benchmarks(parse_args())
