"""
Module 1 — Airspace Graph Builder
==================================
FYP: Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery
Phase I — Software Simulation

What this module does (Section 3.2.2 of the report):
  Extracts a delivery airspace graph G = (V, E) from OpenStreetMap using the
  osmnx Python library.  Each edge e ∈ E carries a 3-objective cost vector:

      c(e) = [d(e),  ε(e),  ρ(e)]

  where:
    d(e) = geodesic distance of the flight segment (km)
    ε(e) = estimated energy consumption  (proxy: d(e) × altitude-change factor)
    ρ(e) = flight risk score ∈ [0, 1]   (obstacle density + no-fly proximity +
                                          wind exposure, simulated for reproducible
                                          benchmarking)

HOW TO RUN:
    python module1_graph_builder.py

OUTPUT:
    pareto_front_storage/airspace_graph.pkl  — serialised NetworkX DiGraph
    outputs/module1_airspace_graph.png       — visualisation
"""

import os
import pickle
import random
from typing import Optional
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use("Agg")          # non-interactive backend — safe on all OS
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

# ---------------------------------------------------------------------------
# Optional: osmnx for real OpenStreetMap data.
# If not installed, the module falls back to a reproducible synthetic graph.
# ---------------------------------------------------------------------------
try:
    import osmnx as ox
    OSMNX_AVAILABLE = True
except ImportError:
    OSMNX_AVAILABLE = False
    print("[Module 1] osmnx not found — will use synthetic graph (install with: pip install osmnx)")


# ─────────────────────────────────────────────────────────────────────────────
#  Main class
# ─────────────────────────────────────────────────────────────────────────────

class AirspaceGraphBuilder:
    """
    Builds the UAV delivery airspace graph.

    Parameters
    ----------
    location : str
        Address string passed to osmnx (e.g. "Thiruvanmiyur, Tamil Nadu, India").
    dist : int
        Radius in metres around `location` to fetch from OSM.
    seed : int
        RNG seed for reproducible synthetic risk scores.
    """

    def __init__(self,
                 location: str = "Thiruvanmiyur, Chennai, Tamil Nadu, India",
                 dist: int = 2000,
                 seed: int = 42):
        self.location = location
        self.dist = dist
        self.seed = seed
        self.G: Optional[nx.DiGraph] = None   # the airspace graph (populated by fetch_graph)

    # ------------------------------------------------------------------
    #  Public API
    # ------------------------------------------------------------------

    def fetch_graph(self) -> nx.DiGraph:
        """
        Build the airspace graph.

        Phase I (software simulation) always uses the reproducible synthetic
        graph so that the risk structure, node ordering, and coordinate system
        are fully controlled.  OSM integration is deferred to Phase II.

        Returns
        -------
        G : nx.DiGraph
            Each edge carries attributes: distance, energy, risk, cost=[d, ε, ρ].
        """
        print("[Module 1] Building reproducible synthetic airspace graph "
              "(Phase I — OSM integration deferred to Phase II).")
        self.G = self._build_synthetic_graph(n_nodes=196)

        print(f"[Module 1] Graph ready: "
              f"{self.G.number_of_nodes()} nodes, "
              f"{self.G.number_of_edges()} edges")
        return self.G

    def save(self, path: str = "pareto_front_storage/airspace_graph.pkl") -> None:
        """Serialise the airspace graph to disk for Tier 1 re-use."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as fh:
            pickle.dump(self.G, fh)
        print(f"[Module 1] Graph saved → {path}")

    @staticmethod
    def load(path: str = "pareto_front_storage/airspace_graph.pkl") -> nx.DiGraph:
        """Load a previously saved airspace graph."""
        with open(path, "rb") as fh:
            return pickle.load(fh)

    def visualise(self,
                  source=None,
                  target=None,
                  route: Optional[list] = None,
                  title: str = "Module 1 — Airspace Graph") -> Figure:
        """
        Draw the airspace graph.

        Parameters
        ----------
        source, target : node IDs to highlight in green / red.
        route          : list of node IDs forming the selected UAV path.
        """
        assert self.G is not None, "Call fetch_graph() first."
        pos = {n: (d["x"], d["y"]) for n, d in self.G.nodes(data=True)}

        fig, ax = plt.subplots(figsize=(11, 9))
        ax.set_facecolor("#F8F8F8")

        # All edges (grey, thin)
        nx.draw_networkx_edges(self.G, pos, ax=ax,
                               edge_color="#C0C0C0", arrows=False,
                               width=0.6, alpha=0.7)

        # All nodes (blue, small)
        nx.draw_networkx_nodes(self.G, pos, ax=ax,
                               node_size=25, node_color="#4A90D9", alpha=0.85)

        # Source / target highlights
        if source is not None:
            nx.draw_networkx_nodes(self.G, pos, nodelist=[source], ax=ax,
                                   node_size=180, node_color="#27AE60")
            ax.annotate("Source", xy=pos[source],
                        xytext=(pos[source][0]+0.001, pos[source][1]+0.001),
                        fontsize=9, color="#27AE60", fontweight="bold")
        if target is not None:
            nx.draw_networkx_nodes(self.G, pos, nodelist=[target], ax=ax,
                                   node_size=180, node_color="#E74C3C")
            ax.annotate("Target", xy=pos[target],
                        xytext=(pos[target][0]+0.001, pos[target][1]+0.001),
                        fontsize=9, color="#E74C3C", fontweight="bold")

        # Selected route (orange, thick)
        if route and len(route) > 1:
            route_edges = list(zip(route[:-1], route[1:]))
            nx.draw_networkx_edges(self.G, pos, edgelist=route_edges, ax=ax,
                                   edge_color="#E67E22", width=3.5,
                                   arrows=True, arrowsize=15,
                                   connectionstyle="arc3,rad=0.1")

        ax.set_title(title +
                     f"\n{self.G.number_of_nodes()} waypoints  |  "
                     f"{self.G.number_of_edges()} flight segments",
                     fontsize=13, fontweight="bold")
        ax.axis("off")
        plt.tight_layout()
        return fig

    # ------------------------------------------------------------------
    #  Private helpers
    # ------------------------------------------------------------------

    def _fetch_from_osm(self) -> nx.DiGraph:
        """Download road network via osmnx and convert to airspace graph."""
        print(f"[Module 1] Fetching OSM data for: {self.location} (dist={self.dist} m)")
        G_raw = ox.graph_from_address(self.location,
                                      dist=self.dist,
                                      network_type="drive")
        G_proj = ox.project_graph(G_raw)   # UTM projection for metric distances

        G = nx.DiGraph()
        for node, data in G_proj.nodes(data=True):
            G.add_node(node,
                       x=data.get("x", 0.0),
                       y=data.get("y", 0.0),
                       lat=data.get("lat", 0.0),
                       lon=data.get("lon", 0.0))

        for u, v, data in G_proj.edges(data=True):
            length_m = data.get("length", 1.0)
            d   = length_m / 1000.0                       # km
            eps = self._energy_cost(d, data)
            rho = self._risk_score(u, v, G=G)
            G.add_edge(u, v,
                       distance=round(d, 5),
                       energy=eps,
                       risk=rho,
                       cost=[round(d, 5), eps, rho])

        print(f"[Module 1] OSM graph: {G.number_of_nodes()} nodes, "
              f"{G.number_of_edges()} edges")
        return G

    def _build_synthetic_graph(self, n_nodes: int = 50) -> nx.DiGraph:
        """
        Generate a reproducible synthetic airspace graph.
        Nodes are laid out on a ~1 km grid with small jitter.
        Edges connect each node to its 8 grid-neighbours.
        """
        print(f"[Module 1] Building synthetic graph with ~{n_nodes} nodes ...")
        rng = np.random.default_rng(self.seed)
        G = nx.DiGraph()
        side = int(np.ceil(np.sqrt(n_nodes)))

        # Node positions (lat/lon-scale: ~0.01° ≈ 1 km)
        positions = {}
        for i in range(n_nodes):
            row, col = divmod(i, side)
            x = col * 0.012 + float(rng.uniform(-0.002, 0.002))
            y = row * 0.012 + float(rng.uniform(-0.002, 0.002))
            G.add_node(i, x=x, y=y, lat=12.70 + y, lon=80.10 + x)
            positions[i] = (x, y)

        # Edges between 8-connected grid neighbours
        for i in range(n_nodes):
            ri, ci = divmod(i, side)
            for j in range(n_nodes):
                if i == j:
                    continue
                rj, cj = divmod(j, side)
                if abs(ri - rj) <= 1 and abs(ci - cj) <= 1:
                    dx = positions[i][0] - positions[j][0]
                    dy = positions[i][1] - positions[j][1]
                    d   = float(np.hypot(dx, dy)) * 111.0   # rough km per degree
                    eps = self._energy_cost(d, {})
                    rho = self._risk_score(i, j, G=G)
                    G.add_edge(i, j,
                               distance=round(d, 5),
                               energy=round(eps, 5),
                               risk=round(rho, 5),
                               cost=[round(d, 5), round(eps, 5), round(rho, 5)])

        print(f"[Module 1] Synthetic graph: {G.number_of_nodes()} nodes, "
              f"{G.number_of_edges()} edges")
        return G

    def _energy_cost(self, distance_km: float, edge_data: dict) -> float:
        """
        Estimate energy consumption (Eq. 3.1 proxy).
        ε(e) = d(e) × (1 + |grade|)
        Grade defaults to a small random value when not provided by OSM.
        """
        grade = abs(edge_data.get("grade", 0.0)) if edge_data else 0.0
        # Add small stochastic altitude variation for synthetic edges
        if grade == 0.0:
            rng = np.random.default_rng(int(distance_km * 1e4) % (2**31))
            grade = float(rng.uniform(0.0, 0.3))
        return round(distance_km * (1.0 + grade), 5)

    def _risk_score(self, u, v, G: nx.DiGraph = None) -> float:
        """
        Compute flight risk ρ(e) ∈ [0, 1]  (Section 3.2.3).

        Two circular HIGH-RISK obstacle clusters are placed at 1/3 and 2/3
        along the main diagonal of the grid (source ↔ target direction).
        The direct / shortest path passes through both clusters — it is fast
        but risky.  Paths that detour above or below the clusters are longer
        but safer, creating genuine distance ↔ risk trade-offs and a rich
        Pareto front with 5–15 non-dominated routes.

        Parameters
        ----------
        G : the graph being built.  Passed explicitly because self.G may
            still be None during graph construction.
        """
        graph = G if G is not None else self.G
        try:
            xm = (graph.nodes[u]["x"] + graph.nodes[v]["x"]) / 2.0
            ym = (graph.nodes[u]["y"] + graph.nodes[v]["y"]) / 2.0
            n    = graph.number_of_nodes()
            side = int(np.ceil(np.sqrt(n))) if n > 0 else 14
            span = (side - 1) * 0.012          # grid width/height in degrees

            # Two obstacle clusters at 1/3 and 2/3 along the main diagonal.
            # The direct shortest path passes through both — short but risky.
            # Detour paths that go above / below are longer but safe.
            r_obs = span * 0.32                # radius ≈ 4.5 grid steps
            obstacles = [
                (span * 0.33, span * 0.33),
                (span * 0.67, span * 0.67),
            ]
            base_risk = 0.06                   # open sky — very low base
            for (ox, oy) in obstacles:
                d = float(np.hypot(xm - ox, ym - oy))
                if d < r_obs:
                    # Risk rises steeply toward the obstacle centre
                    base_risk = max(base_risk,
                                    0.88 * (1.0 - d / r_obs) + 0.10)
        except (KeyError, TypeError, AttributeError):
            base_risk = 0.25

        rng   = np.random.default_rng(int(abs(hash((u, v)))) % (2**31))
        noise = float(rng.uniform(-0.05, 0.05))
        return round(float(np.clip(base_risk + noise, 0.0, 1.0)), 4)


# ─────────────────────────────────────────────────────────────────────────────
#  Script entry-point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    os.makedirs("outputs", exist_ok=True)
    os.makedirs("pareto_front_storage", exist_ok=True)

    builder = AirspaceGraphBuilder(
        location="Thiruvanmiyur, Tamil Nadu, India",
        dist=2000,
        seed=42
    )
    G = builder.fetch_graph()
    builder.save()

    # Pick arbitrary source and target for visualisation
    nodes = list(G.nodes())
    source = nodes[0]
    target = nodes[len(nodes) // 2]

    fig = builder.visualise(source=source, target=target)
    out_path = "outputs/module1_airspace_graph.png"
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"[Module 1] Visualisation saved → {out_path}")
    print("[Module 1] DONE ✓")
