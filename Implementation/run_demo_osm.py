"""
run_demo_osm.py — Phase I Software Simulation  (Review 1 Demo — OSM Variant)
============================================================================
FYP: Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery

Identical pipeline to run_demo_syn.py but uses REAL OpenStreetMap road-network
data (via osmnx) instead of the reproducible synthetic grid.

Key differences from run_demo_syn.py
─────────────────────────────────
  • Module 1 downloads a live OSM driving graph for Kalavakkam, Tamil Nadu
  • Node coordinates are kept as lat/lon degrees (no UTM projection) so the
    risk-score obstacle geometry stays in degree scale
  • Parallel OSM edges between the same node pair are deduplicated
    (shortest segment kept)
  • Source and target are chosen as the most geographically distant pair
    within the largest strongly-connected component, guaranteeing reachability
  • All outputs are written to outputs/ with an "osm_" prefix so they live
    alongside (not over) the synthetic outputs

HOW TO RUN:
  ┌─────────────────────────────────────────────────────────────┐
  │  Install dependencies (one-time):                           │
  │    pip install networkx numpy matplotlib osmnx              │
  │                                                             │
  │  Then run:                                                  │
  │    python run_demo_osm.py                                   │
  └─────────────────────────────────────────────────────────────┘

OUTPUT FILES (all in  outputs/ folder):
  osm_module1_airspace_graph.png      — real OSM road network
  osm_module2_pareto_front_3d.png     — Pareto front in 3D
  osm_module2_pareto_front_2d.png     — 2D projections
  osm_module4_route_comparison.png    — bar chart per payload
  osm_module4_pareto_selections.png   — Pareto + selection markers
  osm_module4_weight_derivation.png   — stacked bar: bridge weights
  osm_module4_routes_on_graph.png     — flight paths drawn on OSM map
"""

import os
import sys
import time
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))

# ── osmnx guard ─────────────────────────────────────────────────────────────
try:
    import osmnx as ox
    OSMNX_AVAILABLE = True
except ImportError:
    OSMNX_AVAILABLE = False

from module1_graph_builder import AirspaceGraphBuilder
from module2_tier1_moastar  import TierOneEngine
from module4_tier2_payload   import TierTwoSelector, PAYLOAD_PRESETS, PRESET_COLOURS

os.makedirs("outputs",              exist_ok=True)
os.makedirs("pareto_front_storage", exist_ok=True)

# ── Configuration ────────────────────────────────────────────────────────────
LOCATION = "Thiruvanmiyur, Chennai, Tamil Nadu, India"
DIST_M   = 4000      # OSM fetch radius in metres
SEED     = 42


# ─────────────────────────────────────────────────────────────────────────────
#  OSM-aware graph builder
# ─────────────────────────────────────────────────────────────────────────────

class OSMAirspaceGraphBuilder(AirspaceGraphBuilder):
    """
    Builds the UAV airspace graph from real OpenStreetMap data.

    Changes vs the base class
    ─────────────────────────
    fetch_graph()
        Calls the OSM downloader; never falls back to synthetic.
        Node x/y are lon/lat in degrees (same scale as the synthetic grid),
        which keeps the risk-score obstacle geometry portable.

    _risk_score_scaled()
        Same two-obstacle-cluster model as the base class but with obstacle
        positions and radius derived from the ACTUAL bounding box of the
        downloaded region, so the geometry is correct regardless of location
        or coordinate scale.
    """

    def fetch_graph(self) -> nx.DiGraph:
        """Download OSM driving network and build the airspace graph."""
        if not OSMNX_AVAILABLE:
            print("[Module 1 — OSM] ERROR: osmnx is not installed.")
            print("  Run:  pip install osmnx")
            sys.exit(1)

        print(f"[Module 1 — OSM] Fetching OSM driving network …")
        print(f"[Module 1 — OSM]   Location : {self.location}")
        print(f"[Module 1 — OSM]   Radius   : {self.dist} m")

        G_raw = ox.graph_from_address(
            self.location, dist=self.dist, network_type="drive"
        )
        print(f"[Module 1 — OSM] Raw OSM graph : "
              f"{G_raw.number_of_nodes()} nodes, "
              f"{G_raw.number_of_edges()} edges (multi-edges included)")

        # ── Build our DiGraph with lat/lon as x/y ───────────────────────
        # osmnx raw graph: node data["x"] = longitude, data["y"] = latitude
        G = nx.DiGraph()
        for node, data in G_raw.nodes(data=True):
            lon = float(data.get("x", 0.0))   # longitude → x
            lat = float(data.get("y", 0.0))   # latitude  → y
            G.add_node(node, x=lon, y=lat, lat=lat, lon=lon)

        # Bounding-box span (degrees) — used for scaled risk scores
        all_x  = [d["x"] for _, d in G.nodes(data=True)]
        all_y  = [d["y"] for _, d in G.nodes(data=True)]
        x_min, x_max = min(all_x), max(all_x)
        y_min, y_max = min(all_y), max(all_y)
        span_x = x_max - x_min
        span_y = y_max - y_min

        # ── Deduplicate parallel edges — keep shortest segment ───────────
        # OSM MultiDiGraph can have several edges between the same (u, v).
        # Our DiGraph keeps one; we pick the shortest by length.
        best_edges: dict = {}
        for u, v, data in G_raw.edges(data=True):
            length_m = float(data.get("length", 1.0))
            key = (u, v)
            if key not in best_edges or length_m < best_edges[key]["length"]:
                best_edges[key] = dict(data)
                best_edges[key]["length"] = length_m

        for (u, v), data in best_edges.items():
            d_km = data["length"] / 1000.0
            eps  = self._energy_cost(d_km, data)
            rho  = self._risk_score_scaled(
                u, v, G, x_min, y_min, span_x, span_y
            )
            G.add_edge(u, v,
                       distance=round(d_km, 5),
                       energy=round(eps, 5),
                       risk=round(rho, 5),
                       cost=[round(d_km, 5), round(eps, 5), round(rho, 5)])

        print(f"[Module 1 — OSM] Deduplicated graph : "
              f"{G.number_of_nodes()} nodes, "
              f"{G.number_of_edges()} edges")
        self.G = G
        return G

    # ------------------------------------------------------------------
    def _risk_score_scaled(
        self, u, v, G: nx.DiGraph,
        x_min: float, y_min: float,
        span_x: float, span_y: float
    ) -> float:
        """
        Two-obstacle-cluster risk model scaled to the OSM bounding box.

        Obstacle positions and influence radius are expressed as fractions
        of (span_x, span_y) so the geometry is identical in intent to the
        synthetic grid regardless of actual coordinate values.
        """
        try:
            xm = (G.nodes[u]["x"] + G.nodes[v]["x"]) / 2.0
            ym = (G.nodes[u]["y"] + G.nodes[v]["y"]) / 2.0

            # Obstacle clusters at 1/3 and 2/3 along the bounding-box diagonal
            obstacles = [
                (x_min + span_x * 0.33, y_min + span_y * 0.33),
                (x_min + span_x * 0.67, y_min + span_y * 0.67),
            ]
            r_obs     = min(span_x, span_y) * 0.32   # ~32 % of shorter span
            base_risk = 0.06                           # open sky

            for (ox_, oy_) in obstacles:
                d = float(np.hypot(xm - ox_, ym - oy_))
                if d < r_obs:
                    base_risk = max(base_risk,
                                    0.88 * (1.0 - d / r_obs) + 0.10)

        except (KeyError, TypeError, AttributeError):
            base_risk = 0.25

        rng   = np.random.default_rng(int(abs(hash((u, v)))) % (2**31))
        noise = float(rng.uniform(-0.05, 0.05))
        return round(float(np.clip(base_risk + noise, 0.0, 1.0)), 4)


# ─────────────────────────────────────────────────────────────────────────────
#  Source / target selection
# ─────────────────────────────────────────────────────────────────────────────

def pick_distant_pair(G: nx.DiGraph, seed: int = 42):
    """
    Return (source, target) chosen from the largest strongly-connected
    component (SCC), maximising Euclidean distance between the pair.

    Why the SCC?  An OSM driving graph is often not strongly connected —
    some roads are one-way leading to dead ends.  Restricting to the largest
    SCC guarantees that NAMOA* can find at least one path from source to
    target.  We then sample up to 80 nodes and pick the most distant pair.
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


def run_module1_osm():
    print_banner(1, "Airspace Graph Builder — OSM data  [Module 1]")
    t0 = time.time()

    builder = OSMAirspaceGraphBuilder(location=LOCATION, dist=DIST_M, seed=SEED)
    G = builder.fetch_graph()
    builder.save(path="pareto_front_storage/airspace_graph_osm.pkl")

    source, target = pick_distant_pair(G, seed=SEED)
    print(f"[Demo]   Source node : {source}")
    print(f"[Demo]   Target node : {target}")

    fig = builder.visualise(
        source=source, target=target,
        title="Module 1 — Airspace Graph (OSM, Kalavakkam, Tamil Nadu)"
    )
    fig.savefig("outputs/osm_module1_airspace_graph.png",
                dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Demo] Module 1 done in {time.time()-t0:.1f}s ✓")
    return G, source, target, builder


def run_module2_osm(G, source, target):
    print_banner(2, "Multi-Objective A* — Tier 1 Pareto Front  [Module 2]")
    t0 = time.time()

    engine = TierOneEngine(G, source, target)
    pareto_front = engine.run()
    engine.print_summary()
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


def run_module4_osm(pareto_front, G, source, target):
    print_banner(4, "Payload-Aware Route Selection — Tier 2  [Module 4]")
    t0 = time.time()

    selector = TierTwoSelector(pareto_front)
    results  = selector.run_all_presets()

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

def main():
    if not OSMNX_AVAILABLE:
        print("\n[ERROR] osmnx is not installed.")
        print("  Install it with:  pip install osmnx")
        print("  Then re-run:      python run_demo_osm.py")
        sys.exit(1)

    t_start = time.time()
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║   FYP B28 — Payload-Aware Two-Tier UAV Route Planning       ║")
    print("║   Phase I Simulation  |  Review 1 Demo — OSM Variant        ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print(f"\n  Location : {LOCATION}")
    print(f"  Radius   : {DIST_M} m")
    print(f"  Note     : This demo downloads live OSM data for Thiruvanmiyur (~5–15 s).\n")

    # Step 1 — Build OSM airspace graph
    G, source, target, builder = run_module1_osm()

    # Step 2 — Tier 1: Multi-Objective A*
    pareto_front, engine = run_module2_osm(G, source, target)

    if not pareto_front:
        print("\n[ERROR] No Pareto-optimal routes found between the chosen nodes.")
        print("  Possible reasons:")
        print("    • The OSM graph is too sparse / disconnected for this pair")
        print("    • The source and target are in different components")
        print("    • max_explored budget was too small for this graph size")
        print("  Try: increase DIST_M, or re-run (different SCC sample)")
        sys.exit(1)

    # Step 3 — Tier 2: Payload-aware selection
    selector, results = run_module4_osm(pareto_front, G, source, target)

    # Final summary
    elapsed = time.time() - t_start
    print("\n" + "═" * 62)
    print("  ALL STEPS COMPLETE  (OSM variant)")
    print(f"  Total time : {elapsed:.1f} s")
    print("  Output files in: outputs/  (prefix: osm_)")
    print("    • osm_module1_airspace_graph.png")
    print("    • osm_module2_pareto_front_3d.png")
    print("    • osm_module2_pareto_front_2d.png")
    print("    • osm_module4_route_comparison.png")
    print("    • osm_module4_pareto_selections.png")
    print("    • osm_module4_weight_derivation.png")
    print("    • osm_module4_routes_on_graph.png")
    print("═" * 62)


if __name__ == "__main__":
    main()
