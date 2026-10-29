"""
Module 2 — NAMOA* Multi-Objective A*  (Tier 1)
==============================================
FYP: Payload-Aware Two-Tier UAV Route Planning for Last-Mile Delivery

Implements Algorithm 1 (Section 3.2.5) as NAMOA* with label dominance:

  • Label table — every partial path is a label (node, g, parent label id), so a
    path is rebuilt by following parent ids.  Nothing is copied per push, and no
    visited set is needed: a path that revisits a node is weakly dominated by
    its own earlier label (every edge has distance > 0), so cycles die in the
    node-level dominance check.
  • Node-level dominance — a new label at v is dropped if any label already at v
    is ≤ it in every objective (equal vectors count as dominated: no duplicates);
    labels it dominates are marked dead and skipped when popped.
  • Heuristic — h(v, t) = [h, h, 0] with h the great-circle distance to the
    target.  Admissible: distance ≥ d_h and energy ≥ distance ≥ d_h on every edge,
    and risk ≥ 0.  A label is pruned when the front weakly dominates g + h(v, t).
  • Diagnostics — `truncated` (the max_explored cap was hit with live labels
    left, so the front may be incomplete), `expanded`, `generated`, `runtime_s`.

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
import time
import numpy as np
import networkx as nx
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
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


def heuristic_vector(G: nx.DiGraph, u, t) -> tuple:
    """Admissible lower bound [h, h, 0] on the remaining [distance, energy, risk]; h = great-circle km."""
    try:
        d = haversine_km(G.nodes[u]["y"], G.nodes[u]["x"],
                         G.nodes[t]["y"], G.nodes[t]["x"])
        return (d, d, 0.0)
    except (KeyError, TypeError):
        return (0.0, 0.0, 0.0)


# ─────────────────────────────────────────────────────────────────────────────
#  Front thinning (optional)
# ─────────────────────────────────────────────────────────────────────────────

def thin_front(front: list, front_max: int) -> list:
    """
    Cap a Pareto front at front_max routes by ε-dominance, keeping it well distributed.

    Objectives are min-max normalised and cut into ε-boxes; one route (the closest to
    its box's lower corner) survives per box and boxes dominated by another box are
    dropped.  ε is the smallest value (bisection) leaving ≤ front_max − 3 boxes; the
    per-objective extremes (cheapest route in distance, energy, risk) are then added
    back, so the result never exceeds front_max routes and always holds the extremes
    Tier 2 normalises against.  The result is a mutually non-dominated subset.
    """
    if front_max < 3:
        raise ValueError("front_max must be at least 3 (the three objective extremes)")
    if len(front) <= front_max:
        return list(front)
    pts = np.array([g for g, _ in front], dtype=float)
    span = np.where(np.ptp(pts, axis=0) > 0, np.ptp(pts, axis=0), 1.0)
    norm = (pts - pts.min(axis=0)) / span
    extremes = {int(np.argmin(pts[:, k])) for k in range(3)}

    def select(eps: float) -> list:
        boxes = np.floor(norm / eps).astype(int)
        best = {}
        for i, b in enumerate(map(tuple, boxes)):
            corner = np.array(b) * eps
            d = float(np.linalg.norm(norm[i] - corner))
            if b not in best or d < best[b][0]:
                best[b] = (d, i)
        keys = list(best)
        arr = np.array(keys)
        keep = [i for k, (_, i) in best.items()
                if not np.any(np.all(arr <= np.array(k), axis=1) & np.any(arr < np.array(k), axis=1))]
        return sorted(keep)

    target = max(1, front_max - 3)
    lo, hi = 1e-6, 1.0
    chosen = select(hi)
    for _ in range(40):                                  # smallest ε with ≤ target boxes
        mid = (lo + hi) / 2.0
        sel = select(mid)
        if len(sel) <= target:
            hi, chosen = mid, sel
        else:
            lo = mid
    keep = sorted(set(chosen) | extremes)
    return [front[i] for i in keep]


# ─────────────────────────────────────────────────────────────────────────────
#  Tier 1 engine  (NAMOA* with a label table)
# ─────────────────────────────────────────────────────────────────────────────

class TierOneEngine:
    """
    NAMOA* — Tier 1 Pareto front construction (exact unless truncated).

    Parameters
    ----------
    G           : nx.DiGraph — airspace graph from Module 1 (edges carry distance, energy, risk).
    source      : source node ID.
    target      : destination node ID.
    max_explored: safety cap on label expansions (default 200 000).

    After run(): pareto_front (list of ([d, ε, ρ], path), sorted), truncated, expanded,
    generated, runtime_s; diagnostics() bundles them.
    """

    TOL = 1e-9

    def __init__(self, G: nx.DiGraph, source, target, max_explored: int = 200_000):
        self.G            = G
        self.source       = source
        self.target       = target
        self.max_explored = max_explored
        self.pareto_front: list = []
        self.truncated    = False
        self.expanded     = 0
        self.generated    = 0
        self.runtime_s    = 0.0

    def diagnostics(self) -> dict:
        return {"truncated": self.truncated, "expanded": self.expanded,
                "generated": self.generated, "runtime_s": round(self.runtime_s, 4),
                "front_size": len(self.pareto_front), "max_explored": self.max_explored}

    # ------------------------------------------------------------------
    #  NAMOA*
    # ------------------------------------------------------------------

    def run(self) -> list:
        """
        Execute NAMOA* and return the Pareto-optimal route set.

        Returns
        -------
        pareto_front : list of ([d, ε, ρ], path), sorted by cost vector
        """
        G, s, t, tol = self.G, self.source, self.target, self.TOL
        self.pareto_front = []
        self.truncated, self.expanded, self.generated = False, 0, 1

        if s not in G or t not in G:
            raise ValueError(f"Source {s} or target {t} not in graph.")
        if not nx.has_path(G, s, t):
            raise ValueError(f"No path from {s} to {t}.")

        t0 = time.perf_counter()
        if s == t:                                            # trivial route
            self.pareto_front = [([0.0, 0.0, 0.0], [s])]
            self.runtime_s = time.perf_counter() - t0
            print(f"[Module 2] Source equals target ({s}) — trivial front")
            return self.pareto_front
        # Label table: id → (node, g, parent id); dead labels are skipped when popped.
        lab_node, lab_g, lab_parent, lab_dead = [s], [(0.0, 0.0, 0.0)], [-1], [False]
        at_node = defaultdict(list)               # node → ids of live non-dominated labels
        at_node[s].append(0)
        h_cache: dict = {}

        def h(v):
            if v not in h_cache:
                h_cache[v] = np.array(heuristic_vector(G, v, t))
            return h_cache[v]

        front_ids: list = []                      # label ids of the current front
        front_arr = np.empty((0, 3))

        def front_dominates(vec) -> bool:
            return front_arr.shape[0] > 0 and bool(np.any(np.all(front_arr <= vec + tol, axis=1)))

        def f_scalar(gh) -> float:
            return float(np.linalg.norm(gh))

        seq = 0
        open_set = [(f_scalar(h(s)), seq, 0)]

        print(f"[Module 2] NAMOA* | source={s}, target={t} | "
              f"graph={G.number_of_nodes()} nodes, {G.number_of_edges()} edges")

        while open_set:
            if self.expanded >= self.max_explored:
                # Truncated only if some live label could still improve the front.
                self.truncated = any(
                    not lab_dead[lid] and not front_dominates(np.array(lab_g[lid]) + h(lab_node[lid]))
                    for _, _, lid in open_set)
                break

            _, _, lid = heapq.heappop(open_set)
            if lab_dead[lid]:
                continue
            u, g = lab_node[lid], lab_g[lid]
            if front_dominates(np.array(g) + h(u)):          # prune on g + h
                continue
            self.expanded += 1

            for v in G.successors(u):
                edge = G[u][v]
                g_new = (g[0] + edge["distance"], g[1] + edge["energy"], g[2] + edge["risk"])

                # Node-level dominance (equal vectors count as dominated).
                if any(all(a <= b + tol for a, b in zip(lab_g[o], g_new)) for o in at_node[v]):
                    continue
                gh_new = np.array(g_new) + h(v)
                if front_dominates(gh_new):
                    continue

                keep = []
                for o in at_node[v]:                          # kill labels the new one dominates
                    if all(a <= b + tol for a, b in zip(g_new, lab_g[o])):
                        lab_dead[o] = True
                    else:
                        keep.append(o)
                new_id = len(lab_node)
                lab_node.append(v); lab_g.append(g_new); lab_parent.append(lid); lab_dead.append(False)
                self.generated += 1
                keep.append(new_id)
                at_node[v] = keep

                if v == t:                                    # complete route: update the front
                    front_ids = [i for i in front_ids
                                 if not all(a <= b + tol for a, b in zip(g_new, lab_g[i]))] + [new_id]
                    front_arr = np.array([lab_g[i] for i in front_ids])
                else:
                    seq += 1
                    heapq.heappush(open_set, (f_scalar(gh_new), seq, new_id))

        # Rebuild routes from the label table.
        routes = []
        for fid in front_ids:
            path, cur = [], fid
            while cur != -1:
                path.append(lab_node[cur])
                cur = lab_parent[cur]
            routes.append((list(lab_g[fid]), path[::-1]))
        routes.sort(key=lambda r: tuple(r[0]))
        self.pareto_front = routes
        self.runtime_s = time.perf_counter() - t0

        print(f"[Module 2] Expanded {self.expanded:,} labels "
              f"({self.generated:,} generated) in {self.runtime_s:.2f} s")
        if self.truncated:
            print(f"[Module 2] WARNING: reached max_explored={self.max_explored:,} with live labels "
                  f"left — the front may be incomplete ({len(routes)} routes found). "
                  f"Raise max_explored or coarsen the graph.")
        print(f"[Module 2] Pareto front size: {len(routes)} routes"
              + (" (TRUNCATED)" if self.truncated else " (complete)"))
        return routes

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
        print(f"  Routes on front : {len(self.pareto_front)}"
              + ("   (TRUNCATED — may be incomplete)" if self.truncated else ""))
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
