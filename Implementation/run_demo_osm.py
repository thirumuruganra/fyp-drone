"""
run_demo_osm.py — Phase I Software Simulation  (Thiruvanmiyur lattice variant)
============================================================================
FYP: Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery

Same pipeline as run_demo_syn.py, but Module 1 builds a regular airspace
lattice (200 m spacing, 6 km box) centred on Thiruvanmiyur, Chennai, with
  • LIVE wind from Open-Meteo on every edge (headwind/crosswind energy penalty,
    crosswind + gust risk term),
  • terrain: ground elevation from a DEM (local raster → OpenTopography →
    Open-Meteo → flat),
  • OSM buildings (osmnx, cached as GeoPackage): node altitude clears the
    tallest building in its cell by 15 m, never above 120 m AGL.
  • cached OSM road centre-lines beneath the airspace nodes and flight edges.
The UAV flies the lattice, not the roads.  (The file keeps its historical name.)

Key points
──────────
  • Wind: live Open-Meteo → last cached snapshot → calm.  No API key needed.
  • DEM: free OpenTopography key (env OPENTOPO_API_KEY) is optional; without it
    the keyless Open-Meteo elevation API is used.  $DEM_RASTER_PATH = local GeoTIFF.
  • --offline          no network: cached weather/DEM/OSM, else hand-made
                       FIXTURES in tests/fixtures (NOT real data) / calm wind
  • --no-terrain / --no-buildings   Phase-1 behaviour (flat / no footprints)
  • --wind-override S D  force wind of S km/h blowing FROM D degrees
  • Source and target: the most distant pair in the largest strongly
    connected component of the lattice
  • Outputs go to outputs/ with an "osm_" prefix, next to the synthetic ones

HOW TO RUN:
    pip install -r requirements.txt
    python run_demo_osm.py                        # live wind
    python run_demo_osm.py --offline              # deterministic, no network
    python run_demo_osm.py --wind-override 30 90  # 30 km/h easterly

OUTPUT FILES (all in  outputs/ folder):
  osm_module1_airspace_graph.png      — lattice shaded by altitude, wind in the title
  osm_module2_pareto_front_3d.png     — Pareto front in 3D
  osm_module2_pareto_front_2d.png     — 2D projections
  osm_module4_route_comparison.png    — bar chart per payload
  osm_module4_pareto_selections.png   — Pareto + selection markers
  osm_module4_weight_derivation.png   — stacked bar: bridge weights
  osm_module4_routes_on_graph.png     — flight paths drawn on the lattice
"""

import argparse
import os
import sys
import time
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))

import config as cfg
from env_data import (FIXTURE_DEM, LocalFrame, LocalRasterProvider, build_layer_indexes,
                      default_elevation_providers, fetch_osm_layers, fetch_osm_roads,
                      load_dotenv)
from module1_graph_builder import LatticeAirspaceBuilder
from module2_tier1_moastar  import TierOneEngine
from pareto_store import ParetoStore, front_from_record
from module4_tier2_payload   import (
    TierTwoSelector,
    PAYLOAD_PRESETS,
    validate_payload_tags,
)

os.makedirs("outputs",              exist_ok=True)
os.makedirs("pareto_front_storage", exist_ok=True)

SEED = 42
FIXTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tests", "fixtures")


# ─────────────────────────────────────────────────────────────────────────────
#  Source / target selection
# ─────────────────────────────────────────────────────────────────────────────

def pick_distant_pair(G: nx.DiGraph, seed: int = 42):
    """
    Return (source, target) chosen from the largest strongly-connected
    component (SCC), maximising Euclidean distance between the pair.

    Why the SCC?  Restricting to the largest strongly connected component
    guarantees that NAMOA* can find at least one path from source to target
    (matters once restricted zones remove lattice nodes).  We sample up to 80
    nodes and pick the most distant pair.
    """
    sccs = list(nx.strongly_connected_components(G))
    largest_scc = max(sccs, key=len)
    nodes = list(largest_scc)
    print(f"[Demo] Largest SCC : {len(nodes)} nodes "
          f"(out of {G.number_of_nodes()} total)")

    rng         = np.random.default_rng(seed)
    sample_size = min(80, len(nodes))
    sample      = rng.choice(len(nodes), size=sample_size,
                             replace=False).tolist()
    sample_nodes = [nodes[i] for i in sample]

    best_d2 = -1.0
    src = sample_nodes[0]
    tgt = sample_nodes[-1]

    for i, a in enumerate(sample_nodes):
        ax, ay = G.nodes[a]["x"], G.nodes[a]["y"]
        for b in sample_nodes[i + 1:]:
            bx, by = G.nodes[b]["x"], G.nodes[b]["y"]
            d2 = (ax - bx) ** 2 + (ay - by) ** 2
            if d2 > best_d2:
                best_d2 = d2
                src, tgt = a, b

    return src, tgt


# ─────────────────────────────────────────────────────────────────────────────
#  Step functions
# ─────────────────────────────────────────────────────────────────────────────

def print_banner(step: int, title: str) -> None:
    bar = "─" * 62
    print(f"\n╔{bar}╗")
    print(f"║  STEP {step}: {title:<53}║")
    print(f"╚{bar}╝")


def load_environment(args, center=(cfg.CENTER_LAT, cfg.CENTER_LON), use_fixtures=True):
    """
    (elevation providers, layer kwargs for the builder), honouring --offline and --no-*.
    use_fixtures=False never falls back to the hand-made Thiruvanmiyur fixtures
    (other regions: cached / live data only).
    """
    providers = None
    if not args.no_terrain:
        providers = default_elevation_providers()
        if args.offline and use_fixtures:   # cached DEM first, hand-made fixture DEM last
            providers.append(LocalRasterProvider(os.path.join(FIXTURE_DIR, FIXTURE_DEM)))

    layer_kwargs: dict = {}
    if not (args.no_buildings and args.no_risk_layers):
        try:
            layers = fetch_osm_layers(center[0], center[1], offline=args.offline,
                                      fixture_dir=FIXTURE_DIR if args.offline and use_fixtures else None)
            layer_kwargs = build_layer_indexes(
                layers, LocalFrame(*center),
                buildings=not args.no_buildings, risk_layers=not args.no_risk_layers,
                manual_nfz=args.manual_nfz or None)
        except ImportError as exc:
            print(f"[Demo] OSM layers skipped ({exc}); pip install geopandas shapely osmnx")
    return providers, layer_kwargs


def run_module1_osm(args):
    print_banner(1, "Airspace Graph Builder — Thiruvanmiyur lattice  [Module 1]")
    t0 = time.time()

    override = tuple(args.wind_override) if args.wind_override else None
    providers, layer_kwargs = load_environment(args)
    builder = LatticeAirspaceBuilder(
        box_m=args.box_km * 1000.0, spacing_m=args.spacing_m, seed=SEED,
        offline=args.offline, wind_override=override,
        elevation_providers=providers, **layer_kwargs,
    )
    G = builder.fetch_graph()
    # Roads provide geographic context without changing UAV connectivity/costs.
    try:
        roads = fetch_osm_roads(
            cfg.CENTER_LAT, cfg.CENTER_LON,
            dist_m=args.box_km * 1000.0 / np.sqrt(2.0) + args.spacing_m,
            offline=args.offline,
        )
        G.graph["road_geometries"] = list(roads.geometry)
    except ImportError as exc:
        print(f"[Demo] Road overlay skipped ({exc}); pip install geopandas shapely osmnx")
    builder.save(path="pareto_front_storage/airspace_graph_osm.pkl")

    alts = [d["alt_amsl_m"] for _, d in G.nodes(data=True)]
    grounds = [d["ground_elev_m"] for _, d in G.nodes(data=True)]
    print(f"[Demo]   Elevation   : {G.graph['elevation_provider']}, "
          f"ground {min(grounds):.1f}–{max(grounds):.1f} m")
    print(f"[Demo]   Buildings   : {G.graph['n_buildings']} footprints, "
          f"{G.graph['blocked_nodes']} node(s) blocked "
          f"({G.graph['nfz_nodes']} in restricted zones, rest > {cfg.H_MAX_AGL:.0f} m AGL needed)")
    print(f"[Demo]   Risk layers : {G.graph['n_restricted_zones']} restricted zones "
          f"({G.graph.get('nfz_edges', 0)} edges cut), {G.graph['n_water_zones']} water, "
          f"{G.graph['n_population_features']} exposure features")
    print(f"[Demo]   Weather gate: " + ("NO-GO — " + "; ".join(G.graph['no_go_reasons'])
                                          if G.graph['no_go'] else "GO"))
    print(f"[Demo]   Flight alt. : {min(alts):.1f}–{max(alts):.1f} m AMSL")

    source, target = pick_distant_pair(G, seed=SEED)
    print(f"[Demo]   Source node : {source}")
    print(f"[Demo]   Target node : {target}")

    fig = builder.visualise(
        source=source, target=target,
        title="Module 1 — Airspace Lattice (Thiruvanmiyur, Chennai)"
    )
    fig.savefig("outputs/osm_module1_airspace_graph.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Demo] Module 1 done in {time.time()-t0:.1f}s ✓")
    return G, source, target, builder


def run_module2_osm(G, source, target, args):
    print_banner(2, "Multi-Objective A* — Tier 1 Pareto Front  [Module 2]")
    t0 = time.time()

    # Reuse the stored front while the weather is representative; rebuild on drift,
    # expiry, a changed graph or --refresh (pareto_store.py).
    record, rebuilt, _ = ParetoStore().get_or_build(
        G, source, target, refresh=args.refresh, front_max=args.front_max)
    pareto_front = front_from_record(record)
    engine = TierOneEngine(G, source, target)
    engine.pareto_front = pareto_front
    engine.truncated = record["truncated"]
    engine.print_summary()
    if rebuilt:
        engine.save()

    fig3d = engine.visualise_3d()
    if fig3d:
        fig3d.savefig("outputs/osm_module2_pareto_front_3d.png",
                      dpi=150, bbox_inches="tight")
        plt.close(fig3d)

    fig2d = engine.visualise_2d_projections()
    if fig2d:
        fig2d.savefig("outputs/osm_module2_pareto_front_2d.png",
                      dpi=150, bbox_inches="tight")
        plt.close(fig2d)

    print(f"[Demo] Module 2 done in {time.time()-t0:.1f}s ✓")
    return pareto_front, engine


def run_module4_osm(pareto_front, G, source, target, selection_request=None):
    print_banner(4, "Payload-Aware Route Selection — Tier 2  [Module 4]")
    t0 = time.time()

    selector = TierTwoSelector(pareto_front, no_go_reasons=G.graph.get("no_go_reasons"))
    if selection_request is not None:
        name, tags = selection_request
        result = selector.select_detailed(
            tags["F"], tags["W"], tags["U"], tags["P"],
            graph=G,
            output_name=name,
        )
        if result.hold_advisory:
            print(f"[Demo] {result.hold_advisory}")
        else:
            print(f"[Demo] Selected route index: {result.selected_index}")
            print(f"[Demo] Cost vector: {result.cost_vector}")
            print("[Demo] Detailed selection saved → "
                  f"outputs/tier2/selection_{name}.json/.csv")
        print(f"[Demo] Module 4 done in {time.time()-t0:.1f}s ✓")
        return selector, {name: result}

    results  = selector.run_all_presets()
    if selector.no_go:
        print(f"[Demo] {selector.hold_advisory()} — Tier 2 plots skipped")
        return selector, results

    fig1 = selector.visualise_route_comparison(results)
    fig1.savefig("outputs/osm_module4_route_comparison.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig1)

    fig2 = selector.visualise_pareto_with_selections(results)
    fig2.savefig("outputs/osm_module4_pareto_selections.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig2)

    fig3 = selector.visualise_weight_derivation(results)
    fig3.savefig("outputs/osm_module4_weight_derivation.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig3)

    fig4 = selector.visualise_selected_routes_on_graph(
        G, results, source, target
    )
    fig4.savefig("outputs/osm_module4_routes_on_graph.png",
                 dpi=150, bbox_inches="tight")
    plt.close(fig4)
    print("[Demo] Route-on-graph saved → outputs/osm_module4_routes_on_graph.png")

    print(f"[Demo] Module 4 done in {time.time()-t0:.1f}s ✓")
    return selector, results


# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1],
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    group = p.add_mutually_exclusive_group()
    group.add_argument(
        "--payload",
        nargs=4,
        metavar=("F", "W", "U", "P"),
        help="select one custom payload tag set",
    )
    group.add_argument(
        "--preset",
        choices=sorted(PAYLOAD_PRESETS.keys()),
        help="select one named payload preset",
    )
    p.add_argument("--offline", action="store_true",
                   help="no network: use the cached weather snapshot, else calm")
    p.add_argument("--wind-override", nargs=2, type=float, metavar=("SPEED_KMH", "DIR_FROM_DEG"),
                   help="replace the wind: SPEED km/h blowing FROM DIR degrees (0 = north)")
    p.add_argument("--no-terrain", action="store_true", help="flat ground (skip the DEM)")
    p.add_argument("--no-buildings", action="store_true", help="skip OSM building footprints")
    p.add_argument("--refresh", action="store_true",
                   help="ignore the stored Tier 1 front and rebuild it")
    p.add_argument("--front-max", type=int, default=None, metavar="N",
                   help="thin the Tier 1 front to at most N routes by ε-dominance (default: exact front)")
    p.add_argument("--manual-nfz", action="store_true",
                   help="also apply config.MANUAL_NO_FLY_ZONES (approximate, unverified coordinates)")
    p.add_argument("--no-risk-layers", action="store_true",
                   help="skip restricted-zone, water and land-use/population layers")
    p.add_argument("--box-km", type=float, default=cfg.BOX_M / 1000.0,
                   help="lattice box side in km (default %(default)s)")
    p.add_argument("--spacing-m", type=float, default=cfg.LATTICE_SPACING_M,
                   help="lattice spacing in metres (default %(default)s)")
    return p.parse_args(argv)


def _selection_request_from_args(args):
    if args.payload:
        try:
            parsed = []
            for field, raw in zip(("F", "W", "U", "P"), args.payload):
                try:
                    parsed.append(int(raw))
                except ValueError as exc:
                    raise ValueError(f"{field} must be an integer") from exc
            F, W, U, P = validate_payload_tags(*parsed)
        except ValueError as exc:
            print(f"[ERROR] {exc}")
            sys.exit(2)
        return "custom", {"F": F, "W": W, "U": U, "P": P}
    if args.preset:
        tags = PAYLOAD_PRESETS[args.preset]
        return args.preset, {"F": tags["F"], "W": tags["W"], "U": tags["U"], "P": tags["P"]}
    return None


def main(argv=None):
    load_dotenv()                                   # OPENTOPO_API_KEY etc. from <repo>/.env
    args = parse_args(argv)
    selection_request = _selection_request_from_args(args)
    t_start = time.time()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║   FYP B28 — Payload-Aware Two-Tier UAV Route Planning       ║")
    print("║   Phase I Simulation  |  Thiruvanmiyur lattice + wind + terrain║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print(f"\n  Location : {cfg.LOCATION_NAME}")
    print(f"  Lattice  : {args.box_km:g} km box, {args.spacing_m:g} m spacing")
    mode = ("offline (cache, else hand-made fixtures / calm)" if args.offline
            else "live wind (Open-Meteo), cached/calm fallback")
    print(f"  Weather  : {mode}"
          + (f", override {args.wind_override[0]:g} km/h from {args.wind_override[1]:g}°"
             if args.wind_override else "") + "\n")

    # Step 1 — Build lattice airspace graph
    G, source, target, builder = run_module1_osm(args)

    # Step 2 — Tier 1: Multi-Objective A*
    pareto_front, engine = run_module2_osm(G, source, target, args)

    if not pareto_front:
        print("\n[ERROR] No Pareto-optimal routes found between the chosen nodes.")
        print("  Possible reasons:")
        print("    • The lattice is disconnected for this pair")
        print("    • max_explored budget was too small for this graph size")
        print("  Try: a coarser --spacing-m or a smaller --box-km")
        sys.exit(1)

    # Step 3 — Tier 2: Payload-aware selection
    selector, results = run_module4_osm(
        pareto_front, G, source, target,
        selection_request=selection_request,
    )

    # Final summary
    elapsed = time.time() - t_start
    print("\n" + "═" * 62)
    print("  ALL STEPS COMPLETE  (lattice variant)")
    print(f"  Wind       : {G.graph['weather']['wind_speed_kmh']:.1f} km/h from "
          f"{G.graph['weather']['wind_dir_from_deg']:.0f}°  "
          f"[{G.graph['weather']['source']}]")
    print(f"  Total time : {elapsed:.1f} s")
    print("  Output files in: outputs/  (prefix: osm_)")
    print("    • osm_module1_airspace_graph.png")
    print("    • osm_module2_pareto_front_3d.png")
    print("    • osm_module2_pareto_front_2d.png")
    if selection_request is None:
        print("    • osm_module4_route_comparison.png")
        print("    • osm_module4_pareto_selections.png")
        print("    • osm_module4_weight_derivation.png")
        print("    • osm_module4_routes_on_graph.png")
    else:
        name, _ = selection_request
        print(f"    • tier2/selection_{name}.json")
        print(f"    • tier2/selection_{name}.csv")
    print("═" * 62)


if __name__ == "__main__":
    main()
