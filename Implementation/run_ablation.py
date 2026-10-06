"""
run_ablation.py — Phase 3 risk-model ablation on the Thiruvanmiyur lattice.
===========================================================================
Builds the lattice four times and runs Tier 1 on the same source/target pair:

    all           every component
    no_wind       calm air (wind override 0 km/h; rain/visibility unchanged)
    no_terrain    flat ground (no DEM)
    no_buildings  no footprints: no clearance lifts, building / comm / turbulence terms

OUTPUT (outputs/ablation/):
  ablation_fronts.png        Pareto fronts of the four variants, three 2-D projections
  edge_risk_maps.png         edge risk ρ of each variant on a common colour scale
  edge_risk_components.png   weighted contribution of every risk component (variant: all)
  ablation_summary.csv/.json front size, objective ranges, mean ρ, runtime per variant

HOW TO RUN:
    python run_ablation.py --offline                 # fixtures, wind 30 km/h from 225°
    python run_ablation.py                           # live wind / DEM / OSM (networked machine)
    python run_ablation.py --offline --box-km 3      # quick check
"""

import argparse
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config as cfg
from env_data import WeatherSnapshot
from module1_graph_builder import LatticeAirspaceBuilder
from module2_tier1_moastar import TierOneEngine
from module5_evaluation import write_csv, write_json
from run_demo_osm import load_environment, pick_distant_pair

VARIANTS = ("all", "no_wind", "no_terrain", "no_buildings")
COLOURS = {"all": "#1f77b4", "no_wind": "#ff7f0e", "no_terrain": "#2ca02c", "no_buildings": "#d62728"}
MARKERS = {"all": "o", "no_wind": "s", "no_terrain": "^", "no_buildings": "D"}


def build_variant(name, args, base_weather, providers, layer_kwargs):
    weather = base_weather.with_wind(0.0, 0.0) if name == "no_wind" else base_weather
    kwargs = dict(layer_kwargs)
    if name == "no_buildings":
        kwargs.pop("building_index", None)
    builder = LatticeAirspaceBuilder(
        box_m=args.box_km * 1000.0, spacing_m=args.spacing_m, weather=weather, offline=args.offline,
        elevation_providers=None if name == "no_terrain" else providers, **kwargs)
    return builder.fetch_graph()


def undirected_edge_values(G, value):
    """One (segment, value) per undirected edge; value = max over both directions."""
    seen = {}
    for u, v, d in G.edges(data=True):
        key = (min(u, v), max(u, v))
        seen[key] = max(seen.get(key, 0.0), value(d))
    segs = [[(G.nodes[u]["x_m"], G.nodes[u]["y_m"]), (G.nodes[v]["x_m"], G.nodes[v]["y_m"])]
            for (u, v) in seen]
    return segs, np.array(list(seen.values()))


def draw_edge_map(ax, G, value, vmax, title, source=None, target=None, cmap="YlOrRd"):
    segs, vals = undirected_edge_values(G, value)
    order = np.argsort(vals)                                  # draw the risky edges last
    lc = LineCollection([segs[i] for i in order], array=vals[order], cmap=cmap,
                        norm=plt.Normalize(0.0, max(vmax, 1e-9)), linewidths=1.6)
    ax.add_collection(lc)
    xs = [d["x_m"] for _, d in G.nodes(data=True)]
    ys = [d["y_m"] for _, d in G.nodes(data=True)]
    ax.set_xlim(min(xs) - 100, max(xs) + 100)
    ax.set_ylim(min(ys) - 100, max(ys) + 100)
    ax.set_aspect("equal")
    for pt, colour in ((source, "#27AE60"), (target, "#2980B9")):
        if pt is not None and pt in G:
            ax.plot(G.nodes[pt]["x_m"], G.nodes[pt]["y_m"], "o", ms=8, color=colour, mec="k", zorder=5)
    ax.set_title(title, fontsize=10, fontweight="bold")
    ax.axis("off")
    return lc


def plot_fronts(fronts, out):
    labels = ["Distance (km)", "Energy (proxy)", "Cumulative risk exposure"]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for ax, (i, j) in zip(axes, [(0, 1), (0, 2), (1, 2)]):
        for name, front in fronts.items():
            pts = np.array([g for g, _ in front])
            ax.scatter(pts[:, i], pts[:, j], s=34, marker=MARKERS[name], color=COLOURS[name],
                       alpha=0.75, edgecolors="k", linewidths=0.3, label=f"{name} ({len(front)})")
        ax.set_xlabel(labels[i])
        ax.set_ylabel(labels[j])
        ax.grid(True, linestyle="--", alpha=0.4)
    axes[0].legend(fontsize=8)
    fig.suptitle("Phase 3 ablation — Pareto fronts, same source/target", fontweight="bold")
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_variant_maps(graphs, source, target, out):
    vmax = max(max(d["rho"] for *_, d in G.edges(data=True)) for G in graphs.values())
    fig, axes = plt.subplots(2, 2, figsize=(13, 12))
    lc = None
    for ax, (name, G) in zip(axes.ravel(), graphs.items()):
        mean_rho = np.mean([d["rho"] for *_, d in G.edges(data=True)])
        lc = draw_edge_map(ax, G, lambda d: d["rho"], vmax, f"{name}   (mean ρ = {mean_rho:.3f})",
                           source, target)
    fig.colorbar(lc, ax=axes.ravel().tolist(), shrink=0.6, label="edge risk ρ")
    fig.suptitle("Edge-risk maps (green = source, blue = target)", fontweight="bold")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_component_maps(G, source, target, out):
    comps = list(cfg.RISK_WEIGHTS)
    cols = 4
    rows = int(np.ceil(len(comps) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4.2 * cols, 4.2 * rows))
    for ax in axes.ravel():
        ax.axis("off")
    for ax, k in zip(axes.ravel(), comps):
        vmax = max(d["risk_components"][k] for *_, d in G.edges(data=True))
        draw_edge_map(ax, G, lambda d, k=k: d["risk_components"][k], vmax,
                      f"{k}  (w = {cfg.RISK_WEIGHTS[k]:g}, max {vmax:.2f})", source, target)
    fig.suptitle("Weighted risk contribution per component (all-components graph)",
                 fontweight="bold")
    fig.tight_layout()
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)


def run_ablation(args):
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    env_args = SimpleNamespace(offline=args.offline, no_terrain=False, no_buildings=False,
                               no_risk_layers=False, manual_nfz=args.manual_nfz or None)
    providers, layer_kwargs = load_environment(env_args)

    probe = LatticeAirspaceBuilder(offline=args.offline,
                                   wind_override=tuple(args.wind_override) if args.wind_override else None)
    base_weather = probe.resolve_weather()
    print(f"[Ablation] Weather: {base_weather.describe()}")

    graphs, fronts, rows = {}, {}, []
    source = target = None
    for name in VARIANTS:
        print(f"\n[Ablation] ── variant: {name}")
        G = build_variant(name, args, base_weather, providers, layer_kwargs)
        if source is None:
            source, target = pick_distant_pair(G, seed=42)
        if source not in G or target not in G:
            raise RuntimeError(f"source/target missing from the {name} graph")
        t0 = time.perf_counter()
        front = TierOneEngine(G, source, target, max_explored=args.max_explored).run()
        runtime = time.perf_counter() - t0
        graphs[name], fronts[name] = G, front
        pts = np.array([g for g, _ in front])
        rho = np.array([d["rho"] for *_, d in G.edges(data=True)])
        rows.append({
            "variant": name, "nodes": G.number_of_nodes(), "edges": G.number_of_edges(),
            "front_size": len(front), "runtime_s": round(runtime, 2),
            "distance_min": pts[:, 0].min(), "distance_max": pts[:, 0].max(),
            "energy_min": pts[:, 1].min(), "energy_max": pts[:, 1].max(),
            "risk_min": pts[:, 2].min(), "risk_max": pts[:, 2].max(),
            "mean_rho": rho.mean(), "max_rho": rho.max(),
        })

    plot_fronts(fronts, out_dir / "ablation_fronts.png")
    plot_variant_maps(graphs, source, target, out_dir / "edge_risk_maps.png")
    plot_component_maps(graphs["all"], source, target, out_dir / "edge_risk_components.png")
    rows = [{k: (round(float(v), 5) if isinstance(v, (float, np.floating)) else v) for k, v in r.items()}
            for r in rows]
    write_csv(out_dir / "ablation_summary.csv", rows)
    write_json(out_dir / "ablation_summary.json", {
        "weather": base_weather.to_dict(), "source": source, "target": target,
        "box_km": args.box_km, "spacing_m": args.spacing_m, "variants": rows})
    print(f"\n[Ablation] Wrote outputs -> {out_dir}")
    for r in rows:
        print(f"  {r['variant']:<13} front={r['front_size']:>4}  mean ρ={r['mean_rho']:.3f}  "
              f"risk {r['risk_min']:.2f}–{r['risk_max']:.2f}  ({r['runtime_s']} s)")
    return rows


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Phase 3 risk-model ablation on the lattice",
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--offline", action="store_true",
                   help="no network: cached data, else hand-made fixtures (not real data)")
    p.add_argument("--wind-override", nargs=2, type=float, metavar=("SPEED_KMH", "DIR_FROM_DEG"),
                   default=[30.0, 225.0], help="wind used by every variant but no_wind (default 30 225)")
    p.add_argument("--box-km", type=float, default=cfg.BOX_M / 1000.0)
    p.add_argument("--spacing-m", type=float, default=cfg.LATTICE_SPACING_M)
    p.add_argument("--max-explored", type=int, default=200_000, help="Tier 1 label-expansion cap")
    p.add_argument("--manual-nfz", action="store_true",
                   help="also apply config.MANUAL_NO_FLY_ZONES (approximate, unverified)")
    p.add_argument("--output-dir", default="outputs/ablation")
    return p.parse_args(argv)


if __name__ == "__main__":
    from env_data import load_dotenv
    load_dotenv()                                   # OPENTOPO_API_KEY etc. from <repo>/.env
    run_ablation(parse_args())
