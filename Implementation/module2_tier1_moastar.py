"""
Module 2 — Multi-Objective A* / Dijkstra Engine  (Tier 1)  [OPTIMISED]
========================================================================
FYP: Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery
Phase I — Software Simulation

Implements Algorithm 1 (Section 3.2.5) using the NAMOA* optimisation:

  KEY SPEED FIX — Node-level label dominance pruning:
  Rather than only pruning at the destination, we track, for every
  intermediate node v, the set of non-dominated cost vectors that have
  ever reached v.  Any new partial path arriving at v whose cost is
  already dominated by a previously-seen label at v is discarded
  immediately — before it enters the open set.  This eliminates the
  exponential blowup that occurs with naive MO-A* on dense graphs.

  Additional fixes vs the original:
    • Cycle detection via O(1) set lookup  (was O(n) list scan)
    • Path stored as parent-pointer dict   (no list copy per expansion)
    • Safety cap: stop after max_explored partial paths (configurable)

HOW TO RUN:
    python module2_tier1_moastar.py

OUTPUT:
    pareto_front_storage/pareto_front.pkl
    outputs/module2_pareto_front_3d.png
    outputs/module2_pareto_front_2d.png
"""

import os
import heapq
import pickle
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from collections import defaultdict

from module1_graph_builder import haversine_km


# ─────────────────────────────────────────────────────────────────────────────
#  Pareto dominance helpers  (unchanged — Algorithm 1 logic)
# ─────────────────────────────────────────────────────────────────────────────

def pareto_dominates(g1: list, g2: list) -> bool:
    """True if g1 dominates g2 (≤ all objectives, < at least one)."""
    return (all(a <= b for a, b in zip(g1, g2)) and
            any(a <  b for a, b in zip(g1, g2)))


def weakly_dominates(g1: list, g2: list, tol: float = 1e-9) -> bool:
    """True if g1 ≤ g2 in every objective (within tol): g1 dominates g2 or equals it."""
    return all(a <= b + tol for a, b in zip(g1, g2))


def update_pareto_front(pareto_front: list, g_new: list, path_new: list) -> bool:
    """UpdateParetoFront — Algorithm 1, line 9."""
    for (g_exist, _) in pareto_front:
        if pareto_dominates(g_exist, g_new):
            return False
    pareto_front[:] = [
        (g, p) for (g, p) in pareto_front
        if not pareto_dominates(g_new, g)
    ]
    pareto_front.append((g_new, path_new))
    return True


def heuristic(G: nx.DiGraph, u, t) -> float:
    """Great-circle lower-bound heuristic h(u,t); node x = lon, y = lat."""
    try:
        d = haversine_km(G.nodes[u]["y"], G.nodes[u]["x"],
                         G.nodes[t]["y"], G.nodes[t]["x"])
        return float(np.linalg.norm([d, d, 0.0]))
    except (KeyError, TypeError):
        return 0.0


# ─────────────────────────────────────────────────────────────────────────────
#  Reconstruct path from parent-pointer table
# ─────────────────────────────────────────────────────────────────────────────

def _reconstruct(parents: dict, node) -> list:
    path = []
    while node is not None:
        path.append(node)
        node = parents[node]
    path.reverse()
    return path


# ─────────────────────────────────────────────────────────────────────────────
#  Tier 1 engine  (NAMOA* with node-level label dominance)
# ─────────────────────────────────────────────────────────────────────────────

class TierOneEngine:
    """
    Multi-Objective A*/Dijkstra Engine — Tier 1 Pareto Front Construction.

    Uses NAMOA* node-level label dominance for tractable performance on
    graphs with 50–500 nodes.

    Parameters
    ----------
    G           : nx.DiGraph — airspace graph from Module 1.
    source      : source node ID.
    target      : destination node ID.
    max_explored: safety cap on partial-path expansions (default 200 000).
    """

    def __init__(self, G: nx.DiGraph, source, target, max_explored: int = 200_000):
        self.G            = G
        self.source       = source
        self.target       = target
        self.max_explored = max_explored
        self.pareto_front: list = []

    # ------------------------------------------------------------------
    #  Algorithm 1 + NAMOA* node-label optimisation
    # ------------------------------------------------------------------

    def run(self) -> list:
        """
        Execute NAMOA* and return the Pareto-optimal route set P.

        Returns
        -------
        pareto_front : list of ([d, ε, ρ], path)
        """
        G, s, t = self.G, self.source, self.target
        self.pareto_front = []

        if s not in G or t not in G:
            raise ValueError(f"Source {s} or target {t} not in graph.")
        if not nx.has_path(G, s, t):
            raise ValueError(f"No path from {s} to {t}.")

        # ── Node-level label sets (NAMOA* key data structure) ───────────
        # labels[v] = list of non-dominated g-vectors that reached node v
        labels: dict = defaultdict(list)

        # ── OPEN set ────────────────────────────────────────────────────
        # Each entry: (f_scalar, seq_counter, g_tuple, node, visited_set, parents_dict)
        # seq_counter breaks ties without comparing dicts/sets
        g0   = (0.0, 0.0, 0.0)
        h0   = heuristic(G, s, t)
        seq  = 0
        open_set = [(np.linalg.norm(g0) + h0, seq, g0, s,
                     frozenset([s]),              # visited: O(1) lookup
                     {s: None})]                  # parent pointers

        explored = 0
        labels[s].append(list(g0))

        print(f"[Module 2] NAMOA* | source={s}, target={t} | "
              f"graph={G.number_of_nodes()} nodes, {G.number_of_edges()} edges")

        while open_set:
            if explored >= self.max_explored:
                print(f"[Module 2] Reached max_explored={self.max_explored:,}. "
                      f"Stopping early with {len(self.pareto_front)} routes.")
                break

            f_val, _, g_tuple, u, visited, parents = heapq.heappop(open_set)
            g = list(g_tuple)
            explored += 1

            # ── Prune: dominated by global Pareto front ──────────────
            if any(weakly_dominates(ge, g) for (ge, _) in self.pareto_front):
                continue

            # ── Goal check ──────────────────────────────────────────
            if u == t:
                path = _reconstruct(parents, t)
                update_pareto_front(self.pareto_front, g, path)
                continue

            # ── Expand neighbours ────────────────────────────────────
            for v in G.successors(u):
                if v in visited:                   # O(1) frozenset lookup
                    continue

                edge   = G[u][v]
                c      = [edge["distance"], edge["energy"], edge["risk"]]
                g_new  = [g[i] + c[i] for i in range(3)]

                # ── Node-level label dominance (NAMOA* key check) ────
                # Skip if g_new is dominated by ANY label already at v
                # (equal vectors count as dominated: no duplicate labels)
                if any(weakly_dominates(lv, g_new) for lv in labels[v]):
                    continue

                # Remove labels at v that g_new now dominates
                labels[v] = [lv for lv in labels[v]
                             if not pareto_dominates(g_new, lv)]
                labels[v].append(g_new)

                # Also skip if dominated by global Pareto front
                if any(weakly_dominates(ge, g_new)
                       for (ge, _) in self.pareto_front):
                    continue

                # Build new parent table (share structure, only add one entry)
                new_parents = dict(parents)
                new_parents[v] = u

                h_new = heuristic(G, v, t)
                f_new = float(np.linalg.norm(g_new)) + h_new
                seq  += 1
                heapq.heappush(open_set,
                               (f_new, seq, tuple(g_new),
                                v, visited | {v}, new_parents))

        print(f"[Module 2] Explored {explored:,} partial paths")
        print(f"[Module 2] Pareto front size: {len(self.pareto_front)} routes")
        return self.pareto_front

    # ------------------------------------------------------------------
    #  Persistence
    # ------------------------------------------------------------------

    def save(self, path: str = "pareto_front_storage/pareto_front.pkl") -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "wb") as fh:
            pickle.dump(self.pareto_front, fh)
        print(f"[Module 2] Pareto front saved → {path}")

    @staticmethod
    def load(path: str = "pareto_front_storage/pareto_front.pkl") -> list:
        with open(path, "rb") as fh:
            return pickle.load(fh)

    # ------------------------------------------------------------------
    #  Visualisation
    # ------------------------------------------------------------------

    def visualise_3d(self) -> plt.Figure:
        if not self.pareto_front:
            print("[Module 2] No Pareto front to visualise.")
            return None
        costs = np.array([g for (g, _) in self.pareto_front])
        fig   = plt.figure(figsize=(10, 8))
        ax    = fig.add_subplot(111, projection="3d")
        sc = ax.scatter(costs[:, 0], costs[:, 1], costs[:, 2],
                        c=costs[:, 2], cmap="RdYlGn_r",
                        s=80, edgecolors="k", linewidths=0.5, alpha=0.85)
        ax.set_xlabel("Distance (km)", labelpad=10)
        ax.set_ylabel("Energy (proxy)", labelpad=10)
        ax.set_zlabel("Cumulative Risk", labelpad=10)
        ax.set_title(
            f"Module 2 — Tier 1 Pareto Front\n"
            f"{len(self.pareto_front)} non-dominated routes  (source → target)",
            fontsize=12, fontweight="bold")
        fig.colorbar(sc, ax=ax, shrink=0.45, pad=0.12, label="Cumulative Risk")
        plt.tight_layout()
        return fig

    def visualise_2d_projections(self) -> plt.Figure:
        if not self.pareto_front:
            return None
        costs  = np.array([g for (g, _) in self.pareto_front])
        labels = ["Distance (km)", "Energy (proxy)", "Cumulative Risk"]
        pairs  = [(0, 1), (0, 2), (1, 2)]
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        for ax, (i, j) in zip(axes, pairs):
            ax.scatter(costs[:, i], costs[:, j],
                       c=costs[:, 2], cmap="RdYlGn_r",
                       s=60, edgecolors="k", linewidths=0.4, alpha=0.85)
            ax.set_xlabel(labels[i])
            ax.set_ylabel(labels[j])
            ax.set_title(f"{labels[i]}  vs  {labels[j]}")
            ax.grid(True, linestyle="--", alpha=0.4)
        fig.suptitle("Module 2 — Pareto Front  (2-D Projections)",
                     fontsize=13, fontweight="bold")
        plt.tight_layout()
        return fig

    def print_summary(self) -> None:
        if not self.pareto_front:
            print("[Module 2] Pareto front is empty.")
            return
        costs = np.array([g for (g, _) in self.pareto_front])
        print("\n" + "="*55)
        print("  MODULE 2 — Pareto Front Summary")
        print("="*55)
        print(f"  Routes on front : {len(self.pareto_front)}")
        print(f"  Distance  (km)  : min={costs[:,0].min():.4f}  max={costs[:,0].max():.4f}")
        print(f"  Energy          : min={costs[:,1].min():.4f}  max={costs[:,1].max():.4f}")
        print(f"  Cumulative Risk : min={costs[:,2].min():.4f}  max={costs[:,2].max():.4f}")
        print("="*55)


# ─────────────────────────────────────────────────────────────────────────────
#  Script entry-point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    from module1_graph_builder import AirspaceGraphBuilder

    os.makedirs("outputs", exist_ok=True)
    os.makedirs("pareto_front_storage", exist_ok=True)

    pkl_path = "pareto_front_storage/airspace_graph.pkl"
    if os.path.exists(pkl_path):
        print("[Module 2] Loading pre-built airspace graph ...")
        G = AirspaceGraphBuilder.load(pkl_path)
    else:
        builder = AirspaceGraphBuilder(seed=42)
        G = builder.fetch_graph()
        builder.save()

    nodes  = list(G.nodes())
    source = nodes[0]
    target = nodes[len(nodes) // 2]

    engine = TierOneEngine(G, source, target)
    engine.run()
    engine.print_summary()
    engine.save()

    fig3d = engine.visualise_3d()
    if fig3d:
        fig3d.savefig("outputs/module2_pareto_front_3d.png", dpi=150, bbox_inches="tight")
        plt.close(fig3d)

    fig2d = engine.visualise_2d_projections()
    if fig2d:
        fig2d.savefig("outputs/module2_pareto_front_2d.png", dpi=150, bbox_inches="tight")
        plt.close(fig2d)

    print("[Module 2] DONE ✓")
